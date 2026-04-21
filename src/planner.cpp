// src/planner.cpp
// Planner thread: drains SLAM bridge queues, feeds EGO-Planner FSM,
// runs the 50 Hz trajectory server, and publishes viz paths + position cmds.
#include "fast_livo/planner.hpp"
#include "ego_planner_time.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <thread>

using namespace std::chrono_literals;

namespace livo {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Planner::Planner(Bridge &bridge, const params::PlannerParams &p)
    : bridge_(bridge), p_(p) {
  ego_planner::FSMParams fsp = makeFSMParams();
  ego_planner::PlannerParams pp = makePlannerParams();

  fsm_.init(fsp, pp);

  // Wire trajectory callback: FSM → pushTrajectory → bridge queue
  fsm_.on_trajectory = [this](double t0, const poly_traj::Trajectory &traj) {
    pushTrajectory(t0, traj);
  };

  // Wire occupancy visualisation callback: GridMap → bridge queue
  fsm_.planner_manager_->grid_map_->on_vis_ = [this](const OccVizCloud &occ) {
    pushOccupancy(occ);
  };

  // Wire visualization on_path callback: PlanningVisualization → bridge queue
  fsm_.setOnPath([this](const std::string &label,
                        const std::vector<Eigen::Vector3d> &pts) {
    pushVizPath(label, pts);
  });

  fprintf(stderr,
          "[Planner] Initialised. max_vel=%.1f, horizon=%.1f, res=%.2f, "
          "mode=%s realworld_experiment=%d\n",
          p_.max_vel, p_.planning_horizon, p_.grid_resolution,
          p_.target_type == 1 ? "PRESET" : "MANUAL",
          (int)p_.realworld_experiment);
}

// ---------------------------------------------------------------------------
// run — blocking thread loop
// ---------------------------------------------------------------------------
void Planner::run() {
  fprintf(stderr, "[Planner] Thread started.\n");

  while (bridge_.running.load()) {
    const double now = ego_now_s();

    // ---- 1. Drain latest pose from SLAM --------------------------------
    // Uses planner_pose_queue (dedicated copy) — pose_queue is consumed by
    // foxglove_streamer and would starve the planner in a race.
    {
      std::unique_lock<std::mutex> lk(bridge_.planner_pose_mtx);
      while (!bridge_.planner_pose_queue.empty()) {
        PoseData pd = bridge_.planner_pose_queue.front();
        bridge_.planner_pose_queue.pop();
        lk.unlock();

        last_odom_pos_ = pd.position;
        has_odom_ = true;

        // Keep grid map's camera_pos_ in sync with actual SLAM odom
        fsm_.planner_manager_->grid_map_->inputOdom(pd.position, pd.rotation);
        fsm_.setOdom(pd.position, pd.velocity);

        static bool first_odom = true;
        if (first_odom) {
          first_odom = false;
          fprintf(stderr, "[Planner] First odom received: (%.2f, %.2f, %.2f)\n",
                  pd.position.x(), pd.position.y(), pd.position.z());
        }

        lk.lock();
      }
    }

    // ---- 2. Drain latest cloud → GridMap inputPointCloud ---------------
    if (has_odom_) {
      std::unique_lock<std::mutex> lk(bridge_.planner_cloud_mtx);
      while (!bridge_.planner_cloud_queue.empty()) {
        CloudData cd = bridge_.planner_cloud_queue.front();
        bridge_.planner_cloud_queue.pop();
        lk.unlock();

        if (cd.cloud && !cd.cloud->empty()) {
          // Filter points closer than min_ray_length to avoid near-field
          // clutter (desk surface, rotor wash, body occlusion) inflating the
          // grid map around the drone origin.
          const double min_ray_sq = p_.min_ray_length * p_.min_ray_length;
          std::vector<Eigen::Vector3d> pts;
          pts.reserve(cd.cloud->size());
          for (const auto &pt : *cd.cloud) {
            Eigen::Vector3d pw(pt.x, pt.y, pt.z);
            if ((pw - last_odom_pos_).squaredNorm() >= min_ray_sq)
              pts.emplace_back(pw);
          }

          // sensor_pos is the actual drone/sensor position from SLAM odom
          fsm_.planner_manager_->grid_map_->inputPointCloud(pts,
                                                            last_odom_pos_);
        }

        lk.lock();
      }
    }

    // ---- 3. Update occupancy every 32 ms (≈30 Hz) ----------------------
    if (now - last_occ_update_s_ >= 0.032) {
      fsm_.planner_manager_->grid_map_->updateOccupancy(now);
      last_occ_update_s_ = now;
    }

    // ---- 4. Fade occupancy every 500 ms ---------------------------------
    if (now - last_fade_s_ >= 0.5) {
      fsm_.planner_manager_->grid_map_->fadeOccupancy(now);
      last_fade_s_ = now;
    }

    // ---- 5. Apply any pending goal / control command from Foxglove --------
    {
      std::queue<livo::Goal> goals;
      {
        std::lock_guard<std::mutex> lk(bridge_.planner_goal_mtx);
        std::swap(goals, bridge_.planner_goal_queue);
      }
      while (!goals.empty()) {
        const auto &g = goals.front();
        if (std::isnan(g.x)) {
          fprintf(stderr, "[Planner] /planner/trigger received\n");
          fsm_.setTrigger();
        } else if (std::isinf(g.x) && g.x < 0.0f) {
          fprintf(stderr, "[Planner] /planner/mandatory_stop received\n");
          fsm_.mandatoryStop();
        } else {
          fprintf(stderr,
                  "[Planner] Goal received (%.2f, %.2f, %.2f), has_odom=%d\n",
                  g.x, g.y, g.z, (int)has_odom_);
          fsm_.setGoal(Eigen::Vector3d(g.x, g.y, g.z));
        }
        goals.pop();
      }
    }

    // ---- 6. Advance FSM --------------------------------------------------
    fsm_.tick(now);

    // ---- 7. Trajectory server (50 Hz position commands) -----------------
    samplePositionCmd(now);

    // ---- 8. Ground height measurement (every 2 s when flying) -----------
    if (p_.enable_ground_height_measurement)
      measureAndPushGroundHeight(now);

    // Run at ~100 Hz
    std::this_thread::sleep_for(10ms);
  }

  fprintf(stderr, "[Planner] Thread exiting.\n");
}

// ---------------------------------------------------------------------------
// setGoal / setTrigger (thread-safe)
// ---------------------------------------------------------------------------
void Planner::setGoal(const Eigen::Vector3d &goal) {
  std::lock_guard<std::mutex> lk(goal_mtx_);
  pending_goal_xyz_ = goal;
  pending_goal_.store(true);
}

void Planner::setTrigger() { fsm_.setTrigger(); }

// ---------------------------------------------------------------------------
// pushTrajectory — sample traj and push TrajectoryData into bridge
// ---------------------------------------------------------------------------
void Planner::pushTrajectory(double start_time,
                             const poly_traj::Trajectory &traj) {
  TrajectoryData td;
  td.timestamp = ego_now_s();
  td.start_time = start_time;
  td.duration = traj.getTotalDuration();

  constexpr double sample_dt = 0.1; // 10 Hz samples
  for (double t = 0.0; t <= td.duration + 1e-6; t += sample_dt) {
    double tc = std::min(t, td.duration);
    td.waypoints.push_back(traj.getPos(tc));
    td.times.push_back(tc);
  }

  std::lock_guard<std::mutex> lk(bridge_.planner_traj_mtx);
  while (bridge_.planner_traj_queue.size() >= kPlannerTrajQueueMax)
    bridge_.planner_traj_queue.pop();
  bridge_.planner_traj_queue.push(std::move(td));
}

// ---------------------------------------------------------------------------
// pushOccupancy
// ---------------------------------------------------------------------------
void Planner::pushOccupancy(const OccVizCloud &occ) {
  OccupancyViz ov;
  ov.timestamp = occ.timestamp;
  ov.occupied_voxels = occ.occupied;

  std::lock_guard<std::mutex> lk(bridge_.planner_occ_mtx);
  while (bridge_.planner_occ_queue.size() >= kPlannerOccQueueMax)
    bridge_.planner_occ_queue.pop();
  bridge_.planner_occ_queue.push(std::move(ov));
}

// ---------------------------------------------------------------------------
// pushVizPath — push a labeled visualization path into bridge
// ---------------------------------------------------------------------------
void Planner::pushVizPath(const std::string &label,
                          const std::vector<Eigen::Vector3d> &pts) {
  PlannerPath pp;
  pp.timestamp = ego_now_s();
  pp.label = label;
  pp.waypoints = pts;

  std::lock_guard<std::mutex> lk(bridge_.planner_viz_mtx);
  while (bridge_.planner_viz_queue.size() >= kPlannerVizQueueMax)
    bridge_.planner_viz_queue.pop();
  bridge_.planner_viz_queue.push(std::move(pp));
}

// ---------------------------------------------------------------------------
// pushPositionCmd
// ---------------------------------------------------------------------------
void Planner::pushPositionCmd(const PositionCmd &cmd) {
  std::lock_guard<std::mutex> lk(bridge_.planner_cmd_mtx);
  while (bridge_.planner_cmd_queue.size() >= kPlannerCmdQueueMax)
    bridge_.planner_cmd_queue.pop();
  bridge_.planner_cmd_queue.push(cmd);
}

// ---------------------------------------------------------------------------
// samplePositionCmd — trajectory server at 50 Hz
// Exact port of ROS1 traj_server::cmdCallback + calculate_yaw.
// ---------------------------------------------------------------------------
void Planner::samplePositionCmd(double now_s) {
  constexpr double CMD_DT = 0.02; // 50 Hz
  if (now_s - last_cmd_s_ < CMD_DT)
    return;
  const double dt = now_s - last_cmd_s_;
  last_cmd_s_ = now_s;

  const ego_planner::LocalTrajData *info =
      fsm_.planner_manager_ ? &fsm_.planner_manager_->traj_.local_traj
                            : nullptr;
  if (!info || info->traj_id <= 0)
    return;

  double t_cur = now_s - info->start_time;
  if (t_cur < 0.0 || t_cur > info->duration + 0.5)
    return;
  t_cur = std::min(t_cur, info->duration);

  Eigen::Vector3d pos = info->traj.getPos(t_cur);
  Eigen::Vector3d vel = info->traj.getVel(t_cur);
  Eigen::Vector3d acc = info->traj.getAcc(t_cur);
  Eigen::Vector3d jer = info->traj.getJer(t_cur);

  std::pair<double, double> yy = calculateYaw(t_cur, info->traj, dt);

  PositionCmd cmd;
  cmd.timestamp = now_s;
  cmd.position = pos;
  cmd.velocity = vel;
  cmd.acceleration = acc;
  cmd.jerk = jer;
  cmd.yaw = yy.first;
  cmd.yaw_dot = yy.second;
  cmd.traj_id = info->traj_id;

  pushPositionCmd(cmd);
}

// ---------------------------------------------------------------------------
// calculateYaw — exact port of traj_server::calculate_yaw from ROS1
// Returns {yaw_rad, yaw_dot_rad_per_s}
// ---------------------------------------------------------------------------
std::pair<double, double>
Planner::calculateYaw(double t_cur, const poly_traj::Trajectory &traj,
                      double dt) {
  constexpr double YAW_DOT_MAX = 2.0 * M_PI;
  constexpr double YAW_DOT_DOT_MAX = 5.0 * M_PI;

  const double dur = traj.getTotalDuration();
  Eigen::Vector3d pos = traj.getPos(t_cur);
  Eigen::Vector3d dir = (t_cur + p_.time_forward <= dur)
                            ? traj.getPos(t_cur + p_.time_forward) - pos
                            : traj.getPos(dur) - pos;

  double yaw_temp = dir.norm() > 0.1 ? std::atan2(dir(1), dir(0)) : last_yaw_;

  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
    d_yaw -= 2.0 * M_PI;
  if (d_yaw <= -M_PI)
    d_yaw += 2.0 * M_PI;

  const double YDM = d_yaw >= 0.0 ? YAW_DOT_MAX : -YAW_DOT_MAX;
  const double YDDM = d_yaw >= 0.0 ? YAW_DOT_DOT_MAX : -YAW_DOT_DOT_MAX;
  double d_yaw_max;
  if (std::fabs(last_yaw_dot_ + dt * YDDM) <= std::fabs(YDM))
    d_yaw_max = last_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
  else {
    double t1 = (YDM - last_yaw_dot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yaw_dot_) / 2.0;
  }

  if (std::fabs(d_yaw) > std::fabs(d_yaw_max))
    d_yaw = d_yaw_max;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2.0 * M_PI;
  if (yaw < -M_PI)
    yaw += 2.0 * M_PI;
  double yaw_dot = d_yaw / std::max(dt, 1e-6);

  last_yaw_ = yaw;
  last_yaw_dot_ = yaw_dot;
  return {yaw, yaw_dot};
}

// ---------------------------------------------------------------------------
// measureAndPushGroundHeight — port of EGOReplanFSM::measureGroundHeight
// Scans occupancy grid downward along the forward trajectory to find ground.
// Pushes a single-point viz path with label "ground_height".
// ---------------------------------------------------------------------------
void Planner::measureAndPushGroundHeight(double now_s) {
  static double last_gh_s = 0.0;
  if (now_s - last_gh_s < 2.0)
    return;

  const ego_planner::LocalTrajData *traj =
      fsm_.planner_manager_ ? &fsm_.planner_manager_->traj_.local_traj
                            : nullptr;
  if (!traj || traj->pts_chk.size() < 3)
    return;

  auto map = fsm_.planner_manager_->grid_map_;
  if (!map)
    return;

  double forward_t = 2.0 / fsm_.planner_manager_->pp_.max_vel_;
  double traj_t = (now_s - traj->start_time) + forward_t;
  if (traj_t > traj->duration)
    return;

  Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);
  const double reso = map->getResolution();

  for (;;) {
    int ret = map->getOccupancy(forward_p);
    if (ret == -1)
      return; // out of map
    if (ret == 1) {
      last_gh_s = now_s;
      // Push as viz path so foxglove_streamer can expose it
      std::vector<Eigen::Vector3d> pts = {forward_p};
      pushVizPath("ground_height", pts);
      return;
    }
    forward_p(2) -= reso;
    if (forward_p(2) < -20.0)
      return; // safety guard
  }
}

// ---------------------------------------------------------------------------
// makeFSMParams / makePlannerParams
// ---------------------------------------------------------------------------
ego_planner::FSMParams Planner::makeFSMParams() const {
  ego_planner::FSMParams f;
  f.replan_thresh = p_.replan_thresh;
  f.planning_horizon = p_.planning_horizon;
  f.emergency_time = p_.emergency_time;
  f.enable_fail_safe = p_.enable_fail_safe;
  f.target_type = p_.target_type;
  f.realworld_experiment = p_.realworld_experiment;
  f.waypoint_num = p_.waypoint_num;
  f.waypoints = p_.waypoints;
  return f;
}

ego_planner::PlannerParams Planner::makePlannerParams() const {
  ego_planner::PlannerParams pp;
  pp.max_vel = p_.max_vel;
  pp.max_acc = p_.max_acc;
  pp.feasibility_tolerance = p_.feasibility_tolerance;
  pp.polyTraj_piece_length = p_.polyTraj_piece_length;
  pp.planning_horizon = p_.planning_horizon;
  pp.use_multitopology_trajs = p_.use_multitopology_trajs;
  pp.drone_id = p_.drone_id;

  auto &opt = pp.optimizer;
  opt.cps_num_prePiece = p_.cps_num_prePiece;
  opt.max_vel = p_.max_vel;
  opt.max_acc = p_.max_acc;
  opt.max_jer = p_.max_jer;
  opt.wei_obs = p_.wei_obs;
  opt.wei_obs_soft = p_.wei_obs_soft;
  opt.wei_feas = p_.wei_feas;
  opt.wei_sqrvar = p_.wei_sqrvar;
  opt.wei_time = p_.wei_time;
  opt.obs_clearance = p_.obs_clearance;
  opt.obs_clearance_soft = p_.obs_clearance_soft;

  auto &gm = pp.grid_map;
  gm.resolution_ = p_.grid_resolution;
  gm.local_update_range3d_ = {p_.local_range_xy, p_.local_range_xy,
                              p_.local_range_z};
  gm.obstacles_inflation_ = p_.obstacles_inflation;
  gm.p_hit_ = p_.p_hit;
  gm.p_miss_ = p_.p_miss;
  gm.p_min_ = p_.p_min;
  gm.p_max_ = p_.p_max;
  gm.p_occ_ = p_.p_occ;
  gm.fading_time_ = p_.fading_time;
  gm.min_ray_length_ = p_.min_ray_length;
  gm.odom_depth_timeout_ = p_.odom_depth_timeout;
  gm.enable_virtual_walll_ = p_.enable_virtual_wall;
  gm.virtual_ceil_ = p_.virtual_ceil;
  gm.virtual_ground_ = p_.virtual_ground;

  return pp;
}

} // namespace livo
