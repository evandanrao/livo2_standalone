#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

// De-ROS'd visualization stub.
// All display calls are routed to an optional std::function callback so the
// Planner wrapper can forward data to Foxglove without any ROS dependency.

#include <Eigen/Eigen>
#include <functional>
#include <string>
#include <vector>

using std::vector;

namespace ego_planner {
class PlanningVisualization {
public:
  // Called with (label, waypoints) when a trajectory or path is available.
  // The Planner wrapper sets this to push data into the Bridge.
  std::function<void(const std::string &, const vector<Eigen::Vector3d> &)>
      on_path;

  PlanningVisualization() {}
  ~PlanningVisualization() {}

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d /*color*/,
                        const double /*scale*/, int /*id*/) {
    if (on_path) {
      vector<Eigen::Vector3d> pts = {goal_point};
      on_path("goal", pts);
    }
  }

  void displayGlobalPathList(vector<Eigen::Vector3d> global_pts,
                             const double /*scale*/, int /*id*/) {
    if (on_path)
      on_path("global", global_pts);
  }

  void displayInitPathList(vector<Eigen::Vector3d> init_pts,
                           const double /*scale*/, int /*id*/) {
    if (on_path && !init_pts.empty())
      on_path("init", init_pts);
  }

  void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs,
                                const double /*scale*/) {
    if (on_path && !init_trajs.empty())
      on_path("init", init_trajs[0]);
  }

  void
  displayMultiOptimalPathList(vector<vector<Eigen::Vector3d>> optimal_trajs,
                              const double /*scale*/) {
    if (on_path && !optimal_trajs.empty())
      on_path("optimal", optimal_trajs[0]);
  }

  void displayOptimalList(Eigen::MatrixXd optimal_pts, int /*id*/) {
    if (on_path) {
      vector<Eigen::Vector3d> pts;
      pts.reserve(optimal_pts.cols());
      for (int i = 0; i < optimal_pts.cols(); ++i)
        pts.push_back(optimal_pts.col(i));
      if (!pts.empty())
        on_path("optimal", pts);
    }
  }

  void displayFailedList(Eigen::MatrixXd failed_pts, int /*id*/) {
    if (on_path) {
      vector<Eigen::Vector3d> pts;
      pts.reserve(failed_pts.cols());
      for (int i = 0; i < failed_pts.cols(); ++i)
        pts.push_back(failed_pts.col(i));
      if (!pts.empty())
        on_path("failed", pts);
    }
  }

  void
  displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths,
                   int /*id*/) {
    if (on_path && !a_star_paths.empty()) {
      vector<Eigen::Vector3d> all;
      for (auto &seg : a_star_paths)
        for (auto &pt : seg)
          all.push_back(pt);
      if (!all.empty())
        on_path("astar", all);
    }
  }

  void displayIntermediatePt(std::string /*type*/, Eigen::MatrixXd & /*pts*/,
                             int /*id*/, Eigen::Vector4d /*color*/) {}

  void displayIntermediateGrad(std::string /*type*/, Eigen::MatrixXd & /*pts*/,
                               Eigen::MatrixXd & /*grad*/, int /*id*/,
                               Eigen::Vector4d /*color*/) {}
};
} // namespace ego_planner
#endif
