#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <functional>
#include <iostream>
#include <vector>

#include "plan_manage/planner_manager.h"
#include "traj_utils/planning_visualization.h"

using std::string;
using std::vector;

namespace ego_planner {

struct FSMParams {
  double replan_thresh = 1.0;    // distance-to-end threshold for replanning
  double planning_horizon = 6.0; // metres ahead to plan
  double emergency_time = 1.0;   // seconds: emergency stop window
  bool enable_fail_safe = true;  // attempt recovery instead of staying stopped

  // Preset waypoint mode
  int target_type = 0; // 0 = MANUAL_TARGET, 1 = PRESET_TARGET
  bool realworld_experiment =
      false; // if true, require setTrigger() before first move
  int waypoint_num = 0;
  std::vector<std::array<double, 3>> waypoints; // [x,y,z] list
};

class EGOReplanFSM {
public:
  enum FSM_EXEC_STATE {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    SEQUENTIAL_START
  };

  EGOReplanFSM() {}
  ~EGOReplanFSM() {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void init(const FSMParams &fsp, const PlannerParams &pp);

  // Public access to the planner manager (used by Planner wrapper to reach
  // grid_map and trajectory data without extra accessor boilerplate).
  EGOPlannerManager::Ptr planner_manager_;

  // Feed latest odometry.  Called before tick().
  void setOdom(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel);

  // Set a new navigation goal (replaces waypointCallback).
  void setGoal(const Eigen::Vector3d &xyz);

  // Arm the planner (PRESET_TARGET mode or realworld_experiment=true).
  void setTrigger();

  // Wire the visualization on_path callback (call after init()).
  void setOnPath(
      std::function<void(const std::string &, const vector<Eigen::Vector3d> &)>
          cb);

  // Advance the FSM by one step.  now_s is wall-clock seconds (steady_clock).
  // Returns true if a new trajectory was generated this tick.
  bool tick(double now_s);

  // Mandatory stop (e.g. operator override).
  void mandatoryStop();

  FSM_EXEC_STATE getState() const { return exec_state_; }

  // Access planned trajectory for sampling.
  const LocalTrajData *getLocalTraj() const {
    return planner_manager_ ? &planner_manager_->traj_.local_traj : nullptr;
  }

  // Called whenever a new local trajectory is successfully computed.
  // Signature: (start_time_s, reference to trajectory)
  std::function<void(double, const poly_traj::Trajectory &)> on_trajectory;

private:
  /* planning modules */
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  double no_replan_thresh_;
  double replan_thresh_;
  double planning_horizen_;
  double emergency_time_;
  bool enable_fail_safe_;

  /* flags */
  bool have_target_{false};
  bool have_odom_{false};
  bool have_new_target_{false};
  bool touch_goal_{false};
  bool mandatory_stop_{false};
  bool flag_escape_emergency_{true};
  bool have_trigger_{false}; // armed for PRESET_TARGET / realworld_experiment
  bool realworld_experiment_{
      false}; // if true, require trigger before each move

  FSM_EXEC_STATE exec_state_{INIT};
  int continously_called_times_{0};

  /* preset target state */
  int target_type_{0}; // 0 = MANUAL, 1 = PRESET
  int waypoint_num_{0};
  int wpt_id_{0};
  std::vector<Eigen::Vector3d> wps_; // loaded waypoints

  /* state */
  double now_s_{0.0}; // cached from tick() argument
  Eigen::Vector3d start_pt_, start_vel_, start_acc_;
  Eigen::Vector3d final_goal_;
  Eigen::Vector3d local_target_pt_, local_target_vel_;
  Eigen::Vector3d odom_pos_, odom_vel_;

  /* internal methods */
  void execFSM();
  void checkCollision();
  void changeFSMExecState(FSM_EXEC_STATE new_state, const string &caller);
  void printFSMExecState();
  std::pair<int, FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

  bool callEmergencyStop(Eigen::Vector3d stop_pos);
  bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
  bool planFromGlobalTraj(int trial_times = 1);
  bool planFromLocalTraj(int trial_times = 1);
  bool planNextWaypoint(const Eigen::Vector3d &next_wp);
  bool mondifyInCollisionFinalGoal();

  void notifyNewTrajectory();
};

} // namespace ego_planner

#endif
