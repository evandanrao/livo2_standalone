#include "plan_manage/ego_replan_fsm.h"
#include "ego_planner_time.h"

#include <cstdio>
#include <iostream>

namespace ego_planner {

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
void EGOReplanFSM::init(const FSMParams &fsp, const PlannerParams &pp) {
  exec_state_ = INIT;
  have_target_ = false;
  have_odom_ = false;
  have_new_target_ = false;
  flag_escape_emergency_ = true;
  mandatory_stop_ = false;

  replan_thresh_ = fsp.replan_thresh;
  planning_horizen_ = fsp.planning_horizon;
  emergency_time_ = fsp.emergency_time;
  enable_fail_safe_ = fsp.enable_fail_safe;

  // Preset target mode
  target_type_ = fsp.target_type;
  waypoint_num_ = fsp.waypoint_num;
  wpt_id_ = 0;
  wps_.resize(fsp.waypoints.size());
  for (size_t i = 0; i < fsp.waypoints.size(); ++i)
    wps_[i] = Eigen::Vector3d(fsp.waypoints[i][0], fsp.waypoints[i][1],
                              fsp.waypoints[i][2]);
  // In realworld_experiment mode the drone waits for setTrigger();
  // otherwise it proceeds as soon as a goal arrives.
  realworld_experiment_ = fsp.realworld_experiment;
  have_trigger_ = !fsp.realworld_experiment;

  /* initialize planner */
  visualization_.reset(new PlanningVisualization);
  planner_manager_.reset(new EGOPlannerManager);
  planner_manager_->initPlanModules(pp, visualization_);

  no_replan_thresh_ = 0.5 * emergency_time_ * planner_manager_->pp_.max_vel_;
}

// ---------------------------------------------------------------------------
// setOdom / setGoal / mandatoryStop
// ---------------------------------------------------------------------------
void EGOReplanFSM::setOdom(const Eigen::Vector3d &pos,
                           const Eigen::Vector3d &vel) {
  odom_pos_ = pos;
  odom_vel_ = vel;
  have_odom_ = true;
}

void EGOReplanFSM::setGoal(const Eigen::Vector3d &xyz) {
  planNextWaypoint(xyz);
}

void EGOReplanFSM::setTrigger() {
  have_trigger_ = true;
  fprintf(stderr, "[FSM] Trigger received.\n");
  // PRESET_TARGET: arm first waypoint when triggered
  if (target_type_ == 1 && waypoint_num_ > 0 && !have_target_) {
    wpt_id_ = 0;
    planNextWaypoint(wps_[0]);
  }
}

void EGOReplanFSM::setOnPath(
    std::function<void(const std::string &, const vector<Eigen::Vector3d> &)>
        cb) {
  if (visualization_)
    visualization_->on_path = cb;
}

void EGOReplanFSM::mandatoryStop() {
  mandatory_stop_ = true;
  fprintf(stderr, "[FSM] Mandatory stop received.\n");
  changeFSMExecState(EMERGENCY_STOP, "MandatoryStop");
  enable_fail_safe_ = false;
}

// ---------------------------------------------------------------------------
// tick — main entry point called from Planner::run() loop
// ---------------------------------------------------------------------------
bool EGOReplanFSM::tick(double now_s) {
  now_s_ = now_s;
  checkCollision();
  execFSM();

  // Return true whenever we're in EXEC_TRAJ (plan exists and is running)
  return (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ);
}

// ---------------------------------------------------------------------------
// execFSM — replaces execFSMCallback (10ms timer)
// ---------------------------------------------------------------------------
void EGOReplanFSM::execFSM() {
  static int fsm_num = 0;
  if (++fsm_num == 500) {
    fsm_num = 0;
    printFSMExecState();
  }

  switch (exec_state_) {
  case INIT: {
    if (!have_odom_)
      return;
    changeFSMExecState(WAIT_TARGET, "FSM");
    break;
  }

  case WAIT_TARGET: {
    if (!have_target_)
      return;
    // In realworld_experiment mode a manual /planner/trigger is required.
    // Otherwise (desktop / simulation) proceed as soon as a goal arrives.
    if (!have_trigger_ && realworld_experiment_)
      return;
    have_trigger_ = true; // ensure armed for next time
    changeFSMExecState(SEQUENTIAL_START, "FSM");
    break;
  }

  case SEQUENTIAL_START: {
    // single-drone: drone_id <= -1 always satisfies condition
    if (planner_manager_->pp_.drone_id <= 0) {
      bool success = planFromGlobalTraj(10);
      if (success)
        changeFSMExecState(EXEC_TRAJ, "FSM");
      else {
        fprintf(stderr,
                "[FSM] Failed to generate first trajectory, retrying.\n");
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
    }
    break;
  }

  case GEN_NEW_TRAJ: {
    if (planFromGlobalTraj(10)) {
      changeFSMExecState(EXEC_TRAJ, "FSM");
      flag_escape_emergency_ = true;
    } else {
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }
    break;
  }

  case REPLAN_TRAJ: {
    if (planFromLocalTraj(1))
      changeFSMExecState(EXEC_TRAJ, "FSM");
    else
      changeFSMExecState(REPLAN_TRAJ, "FSM");
    break;
  }

  case EXEC_TRAJ: {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = now_s_ - info->start_time;
    t_cur = std::min(info->duration, t_cur);
    Eigen::Vector3d pos = info->traj.getPos(t_cur);
    bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);

    const PtsChk_t *chk_ptr = &planner_manager_->traj_.local_traj.pts_chk;
    bool close_to_current_traj_end =
        (chk_ptr->size() >= 1 && chk_ptr->back().size() >= 1)
            ? chk_ptr->back().back().first - t_cur < emergency_time_
            : false;

    if (mondifyInCollisionFinalGoal()) {
      // goal moved, FSM will replan
    } else if (target_type_ == 1 /*PRESET*/ && wpt_id_ < waypoint_num_ - 1 &&
               (final_goal_ - pos).norm() < no_replan_thresh_) {
      // Advance to the next preset waypoint while close to current
      wpt_id_++;
      planNextWaypoint(wps_[wpt_id_]);
    } else if ((t_cur > info->duration - 1e-2) && touch_the_goal) {
      // reached final goal
      have_target_ = false;
      // Only require re-trigger in realworld_experiment mode; otherwise stay
      // armed.
      have_trigger_ = !realworld_experiment_;
      if (target_type_ == 1 /*PRESET*/ && waypoint_num_ > 0) {
        // Loop back to first waypoint; wait for trigger to restart
        wpt_id_ = 0;
        planNextWaypoint(wps_[0]);
      }
      fprintf(stderr, "[FSM] Navigation goal reached.\n");
      changeFSMExecState(WAIT_TARGET, "FSM");
    } else if (t_cur > replan_thresh_ ||
               (!touch_the_goal && close_to_current_traj_end)) {
      changeFSMExecState(REPLAN_TRAJ, "FSM");
    }
    break;
  }

  case EMERGENCY_STOP: {
    if (flag_escape_emergency_) {
      callEmergencyStop(odom_pos_);
    } else {
      if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }
    flag_escape_emergency_ = false;
    break;
  }
  }
}

// ---------------------------------------------------------------------------
// checkCollision — replaces checkCollisionCallback (50ms timer)
// ---------------------------------------------------------------------------
void EGOReplanFSM::checkCollision() {
  if (exec_state_ == WAIT_TARGET || exec_state_ == INIT)
    return;

  LocalTrajData *info = &planner_manager_->traj_.local_traj;
  if (info->traj_id <= 0)
    return;

  auto map = planner_manager_->grid_map_;
  const double t_cur = now_s_ - info->start_time;

  /* Lost sensor data */
  if (map->getOdomDepthTimeout()) {
    fprintf(stderr, "[FSM] Sensor depth timeout! EMERGENCY_STOP\n");
    enable_fail_safe_ = false;
    changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    return;
  }

  /* Check trajectory collision */
  PtsChk_t pts_chk = info->pts_chk;
  double t_temp = t_cur;
  int i_start = info->traj.locatePieceIdx(t_temp);
  if (i_start >= (int)pts_chk.size())
    return;

  size_t j_start = 0;
  for (; i_start < (int)pts_chk.size(); ++i_start) {
    for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start) {
      if (pts_chk[i_start][j_start].first > t_cur)
        goto find_ij_start;
    }
  }
find_ij_start:;

  const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
  size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;

  for (size_t i = i_start; i < i_end; ++i) {
    for (size_t j = j_start; j < pts_chk[i].size(); ++j) {
      double t = pts_chk[i][j].first;
      Eigen::Vector3d p = pts_chk[i][j].second;

      bool dangerous = map->getInflateOccupancy(p);

      if (dangerous) {
        if (planFromLocalTraj()) {
          fprintf(stderr,
                  "[FSM] Replanned on collision detection at t=%.2f/%.2f.\n", t,
                  info->duration);
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        } else {
          if (t - t_cur < emergency_time_) {
            fprintf(stderr, "[FSM] EMERGENCY_STOP at t=%.2f.\n", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          } else {
            fprintf(stderr, "[FSM] Collision ahead, replanning.\n");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
      }
    }
    j_start = 0;
  }
}

// ---------------------------------------------------------------------------
// changeFSMExecState
// ---------------------------------------------------------------------------
void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state,
                                      const string &caller) {
  if (new_state == exec_state_)
    continously_called_times_++;
  else
    continously_called_times_ = 1;

  static const char *state_str[] = {
      "INIT",      "WAIT_TARGET",    "GEN_NEW_TRAJ",    "REPLAN_TRAJ",
      "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
  int pre_s = (int)exec_state_;
  exec_state_ = new_state;
  fprintf(stderr, "[FSM|%s] Drone %d: %s -> %s\n", caller.c_str(),
          planner_manager_->pp_.drone_id, state_str[pre_s],
          state_str[(int)new_state]);
}

void EGOReplanFSM::printFSMExecState() {
  static const char *state_str[] = {
      "INIT",      "WAIT_TARGET",    "GEN_NEW_TRAJ",    "REPLAN_TRAJ",
      "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
  fprintf(stderr, "\r[FSM] state=%s drone=%d%s%s%s%s\n",
          state_str[(int)exec_state_], planner_manager_->pp_.drone_id,
          !have_odom_ ? " [no_odom]" : "", !have_target_ ? " [no_target]" : "",
          !have_trigger_ ? " [no_trigger]" : "",
          mandatory_stop_ ? " [MANDATORY_STOP]" : "");
}

std::pair<int, EGOReplanFSM::FSM_EXEC_STATE>
EGOReplanFSM::timesOfConsecutiveStateCalls() {
  return {continously_called_times_, exec_state_};
}

// ---------------------------------------------------------------------------
// callEmergencyStop
// ---------------------------------------------------------------------------
bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos) {
  planner_manager_->EmergencyStop(stop_pos);
  notifyNewTrajectory();
  return true;
}

// ---------------------------------------------------------------------------
// callReboundReplan
// ---------------------------------------------------------------------------
bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init,
                                     bool flag_randomPolyTraj) {
  planner_manager_->getLocalTarget(planning_horizen_, start_pt_, final_goal_,
                                   local_target_pt_, local_target_vel_,
                                   touch_goal_);

  bool plan_success = planner_manager_->reboundReplan(
      start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
      (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj,
      touch_goal_);

  have_new_target_ = false;

  if (plan_success)
    notifyNewTrajectory();

  return plan_success;
}

// ---------------------------------------------------------------------------
// planFromGlobalTraj / planFromLocalTraj
// ---------------------------------------------------------------------------
bool EGOReplanFSM::planFromGlobalTraj(int trial_times) {
  start_pt_ = odom_pos_;
  start_vel_ = odom_vel_;
  start_acc_.setZero();

  bool flag_random = (timesOfConsecutiveStateCalls().first != 1);

  for (int i = 0; i < trial_times; i++) {
    if (callReboundReplan(true, flag_random))
      return true;
  }
  return false;
}

bool EGOReplanFSM::planFromLocalTraj(int trial_times) {
  LocalTrajData *info = &planner_manager_->traj_.local_traj;
  double t_cur = now_s_ - info->start_time;

  start_pt_ = info->traj.getPos(t_cur);
  start_vel_ = info->traj.getVel(t_cur);
  start_acc_ = info->traj.getAcc(t_cur);

  if (callReboundReplan(false, false))
    return true;
  if (callReboundReplan(true, false))
    return true;
  for (int i = 0; i < trial_times; i++)
    if (callReboundReplan(true, true))
      return true;
  return false;
}

// ---------------------------------------------------------------------------
// planNextWaypoint
// ---------------------------------------------------------------------------
bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d &next_wp) {
  std::vector<Eigen::Vector3d> one_pt_wps = {next_wp};
  bool success = planner_manager_->planGlobalTrajWaypoints(
      odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), one_pt_wps,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  if (success) {
    final_goal_ = next_wp;
    have_target_ = true;
    have_new_target_ = true;

    /* Display global path via visualization stub's callback */
    constexpr double step_t = 0.1;
    int i_end =
        (int)std::floor(planner_manager_->traj_.global_traj.duration / step_t);
    vector<Eigen::Vector3d> global_traj(i_end);
    for (int i = 0; i < i_end; i++)
      global_traj[i] =
          planner_manager_->traj_.global_traj.traj.getPos(i * step_t);

    visualization_->displayGlobalPathList(global_traj, 0.1, 0);

    if (exec_state_ != WAIT_TARGET && exec_state_ != INIT)
      changeFSMExecState(REPLAN_TRAJ, "TRIG");
  } else {
    fprintf(stderr, "[FSM] Unable to generate global trajectory to goal.\n");
  }

  return success;
}

// ---------------------------------------------------------------------------
// mondifyInCollisionFinalGoal
// ---------------------------------------------------------------------------
bool EGOReplanFSM::mondifyInCollisionFinalGoal() {
  if (!planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
    return false;

  Eigen::Vector3d orig_goal = final_goal_;
  double t_step = planner_manager_->grid_map_->getResolution() /
                  planner_manager_->pp_.max_vel_;

  for (double t = planner_manager_->traj_.global_traj.duration; t > 0;
       t -= t_step) {
    Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.getPos(t);
    if (!planner_manager_->grid_map_->getInflateOccupancy(pt)) {
      if (planNextWaypoint(pt)) {
        fprintf(stderr,
                "[FSM] In-collision goal (%.2f,%.2f,%.2f) moved to "
                "(%.2f,%.2f,%.2f).\n",
                orig_goal(0), orig_goal(1), orig_goal(2), final_goal_(0),
                final_goal_(1), final_goal_(2));
        return true;
      }
    }
    if (t <= t_step)
      fprintf(stderr,
              "[FSM] No collision-free point found on global trajectory.\n");
  }
  return false;
}

// ---------------------------------------------------------------------------
// notifyNewTrajectory — fire on_trajectory callback
// ---------------------------------------------------------------------------
void EGOReplanFSM::notifyNewTrajectory() {
  if (!on_trajectory)
    return;
  const LocalTrajData *ltd = &planner_manager_->traj_.local_traj;
  if (ltd->traj_id <= 0)
    return;
  on_trajectory(ltd->start_time, ltd->traj);
}

} // namespace ego_planner
