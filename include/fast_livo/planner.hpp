// include/fast_livo/planner.hpp
// Wrapper that connects SLAM's Bridge queues to EGO-Planner's FSM + GridMap.
// Lives in its own thread started by main().
#pragma once

#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"

#include "plan_env/grid_map.h"
#include "plan_manage/ego_replan_fsm.h"

#include <atomic>
#include <cmath>
#include <thread>
#include <utility>

namespace livo {

class Planner {
public:
  Planner(Bridge &bridge, const params::PlannerParams &p);

  // Blocking loop — designed to run in its own std::thread.
  void run();

  // Thread-safe goal setter (can be called from any thread).
  void setGoal(const Eigen::Vector3d &goal);

  // Arm the planner (PRESET_TARGET / realworld_experiment mode).
  void setTrigger();

private:
  Bridge &bridge_;
  params::PlannerParams p_;

  ego_planner::EGOReplanFSM fsm_;

  double last_occ_update_s_ = 0.0; // time of last updateOccupancy()
  double last_fade_s_ = 0.0;       // time of last fadeOccupancy()
  double last_safety_s_ = 0.0;     // time of last checkCollision pass

  // Sensor position tracking (used as grid map camera origin)
  Eigen::Vector3d last_odom_pos_{0.0, 0.0, 0.0};
  bool has_odom_ = false;

  // Trajectory server yaw state
  double last_yaw_ = 0.0;
  double last_yaw_dot_ = 0.0;
  double last_cmd_s_ = 0.0; // time of last position command

  std::atomic<bool> pending_goal_{false};
  Eigen::Vector3d pending_goal_xyz_{0, 0, 0};
  std::mutex goal_mtx_;

  // Build ego_planner PlannerParams from our flat params::PlannerParams.
  ego_planner::PlannerParams makePlannerParams() const;
  ego_planner::FSMParams makeFSMParams() const;

  // Push new trajectory / occupancy / viz / cmd data into bridge queues.
  void pushTrajectory(double start_time, const poly_traj::Trajectory &traj);
  void pushOccupancy(const OccVizCloud &occ);
  void pushVizPath(const std::string &label,
                   const std::vector<Eigen::Vector3d> &pts);
  void pushPositionCmd(const PositionCmd &cmd);

  // Sample the active trajectory and output a position command at 50 Hz.
  void samplePositionCmd(double now_s);

  // Compute yaw and yaw_dot from trajectory look-ahead (exact port of
  // traj_server::calculate_yaw from ROS1).
  std::pair<double, double> calculateYaw(double t_cur,
                                         const poly_traj::Trajectory &traj,
                                         double dt);

  // Scan occupancy grid downward along trajectory to find ground height.
  // Pushes result into bridge planner_viz_queue (label "ground_height").
  void measureAndPushGroundHeight(double now_s);
};

} // namespace livo
