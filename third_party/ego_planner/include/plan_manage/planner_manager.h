#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include "optimizer/poly_traj_optimizer.h"
#include "optimizer/poly_traj_utils.hpp"
#include "plan_env/grid_map.h"
#include "traj_utils/plan_container.hpp"
#include "traj_utils/planning_visualization.h"

namespace ego_planner {

struct PlannerParams {
  double max_vel = 3.0;
  double max_acc = 4.0;
  double feasibility_tolerance = 0.0;
  double polyTraj_piece_length = 1.5;
  double planning_horizon = 6.0;
  bool use_multitopology_trajs = false;
  int drone_id = -1; // <= -1: single drone; >= 0: swarm
  PolyTrajOptimizer::OptimizerParams optimizer;
  GridMapParams grid_map;
};

class EGOPlannerManager {
  // SECTION stable
public:
  EGOPlannerManager();
  ~EGOPlannerManager();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* main planning interface */
  void initPlanModules(const PlannerParams &p,
                       PlanningVisualization::Ptr vis = nullptr);
  bool computeInitState(const Eigen::Vector3d &start_pt,
                        const Eigen::Vector3d &start_vel,
                        const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &local_target_pt,
                        const Eigen::Vector3d &local_target_vel,
                        const bool flag_polyInit,
                        const bool flag_randomPolyTraj, const double &ts,
                        poly_traj::MinJerkOpt &initMJO);
  bool reboundReplan(const Eigen::Vector3d &start_pt,
                     const Eigen::Vector3d &start_vel,
                     const Eigen::Vector3d &start_acc,
                     const Eigen::Vector3d &end_pt,
                     const Eigen::Vector3d &end_vel, const bool flag_polyInit,
                     const bool flag_randomPolyTraj, const bool touch_goal);
  bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &start_acc,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               const Eigen::Vector3d &end_vel,
                               const Eigen::Vector3d &end_acc);
  void getLocalTarget(const double planning_horizen,
                      const Eigen::Vector3d &start_pt,
                      const Eigen::Vector3d &global_end_pt,
                      Eigen::Vector3d &local_target_pos,
                      Eigen::Vector3d &local_target_vel, bool &touch_goal);
  bool EmergencyStop(Eigen::Vector3d stop_pos);
  bool checkCollision(int drone_id);
  bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt,
                           const bool touch_goal);
  inline double getSwarmClearance(void) {
    return ploy_traj_opt_->get_swarm_clearance_();
  }
  inline int getCpsNumPrePiece(void) {
    return ploy_traj_opt_->get_cps_num_prePiece_();
  }

  PlanParameters pp_;
  GridMap::Ptr grid_map_;
  TrajContainer traj_;

private:
  PlanningVisualization::Ptr visualization_;
  PolyTrajOptimizer::Ptr ploy_traj_opt_;
  int continous_failures_count_{0};

public:
  typedef std::unique_ptr<EGOPlannerManager> Ptr;
  // !SECTION
};

} // namespace ego_planner

#endif
