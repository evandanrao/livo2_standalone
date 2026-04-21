// standalones/livo2/include/fast_livo/bridge.hpp
// Inter-thread data exchange hub. All cross-thread transfers use mutex + queue.
// The Bridge is constructed in main() and passed by reference into every
// thread.
#pragma once
#include "b_types.hpp"
#include <atomic>
#include <mutex>
#include <queue>

namespace livo {

// Maximum queue capacities to prevent unbounded memory growth
static constexpr size_t kImuQueueMax = 2000;   // 400 Hz × 5 s
static constexpr size_t kCloudQueueMax = 10;   // 10 Hz LiDAR
static constexpr size_t kImageQueueMax = 30;   // 30 fps camera
static constexpr size_t kVizCloudQueueMax = 5; // downsampled XYZI cloud
static constexpr size_t kVizImgQueueMax = 5;   // feature-overlay image

// New in Stage 5 — additional visualisation channels
static constexpr size_t kVizRgbCloudQueueMax = 3; // LIVO VIO RGB frames
static constexpr size_t kImuPropQueueMax = 200;   // ~1 s at 200 Hz
static constexpr size_t kEffectCloudQueueMax = 5; // matched feature points
static constexpr size_t kVoxelMapQueueMax = 3;    // voxel plane centroids
static constexpr size_t kVisualMapQueueMax = 5;   // visual tracking sub-map
static constexpr size_t kGlobalMapQueueMax = 2;   // accumulated downsampled map
static constexpr size_t kPlannerTrajQueueMax = 4; // planner trajectory
static constexpr size_t kPlannerOccQueueMax = 2;  // planner occupancy grid
static constexpr size_t kPlannerGoalQueueMax = 4; // inbound goal commands
static constexpr size_t kPlannerCloudQueueMax =
    10; // planner-dedicated cloud feed
static constexpr size_t kPlannerVizQueueMax = 20; // viz paths (all labels)
static constexpr size_t kPlannerCmdQueueMax = 10; // 50 Hz position commands

struct Bridge {
  // LiDAR driver → SLAM core
  std::mutex imu_mtx;
  std::queue<ImuData> imu_queue;

  std::mutex cloud_mtx;
  std::queue<CloudData> cloud_queue;

  // Camera driver → SLAM core
  std::mutex img_mtx;
  std::queue<ImageData> img_queue;

  // SLAM core → Foxglove streamer
  std::mutex pose_mtx;
  std::queue<PoseData> pose_queue;

  // SLAM core → Foxglove streamer (downsampled map cloud)
  std::mutex viz_cloud_mtx;
  std::queue<CloudData> viz_cloud_queue;

  // SLAM core → Foxglove streamer (feature-overlay image)
  std::mutex viz_img_mtx;
  std::queue<ImageData> viz_img_queue;

  // ---- New Stage 5 channels -----------------------------------------------

  // RGB-coloured point cloud, produced only in LIVO+VIO mode (img_en=1).
  std::mutex viz_rgb_cloud_mtx;
  std::queue<RgbCloudData> viz_rgb_cloud_queue;

  // IMU-propagated pose at ~200 Hz — gives smooth high-rate odometry in
  // Foxglove.
  std::mutex imu_prop_mtx;
  std::queue<PoseData> imu_prop_queue;

  // Matched LiDAR-to-plane feature points used in each EKF update.
  std::mutex effect_cloud_mtx;
  std::queue<CloudData> effect_cloud_queue;

  // Voxel plane centroids from the incremental voxel map.
  std::mutex voxel_map_mtx;
  std::queue<CloudData> voxel_map_queue;

  // Visual tracking sub-map (3D positions of VIO map points, img_en=1 only).
  std::mutex visual_map_mtx;
  std::queue<CloudData> visual_map_queue;

  // /livo2/global_map — downsampled accumulated world-frame point cloud
  std::mutex global_map_mtx;
  std::queue<CloudData> global_map_queue;

  // ---- Planner visualisation channels -------------------------------------
  // EGO-Planner → Foxglove: sampled trajectory waypoints
  std::mutex planner_traj_mtx;
  std::queue<TrajectoryData> planner_traj_queue;

  // EGO-Planner → Foxglove: inflated occupancy voxels
  std::mutex planner_occ_mtx;
  std::queue<OccupancyViz> planner_occ_queue;

  // External → Planner: goal commands from Foxglove WS or UDP
  std::mutex planner_goal_mtx;
  std::queue<Goal> planner_goal_queue;

  // SLAM core → Planner: dedicated odom feed (separate from pose_queue which
  // is consumed by foxglove_streamer — same race as planner_cloud_queue).
  std::mutex planner_pose_mtx;
  std::queue<PoseData> planner_pose_queue;

  // SLAM core → Planner: dedicated cloud feed (separate from viz_cloud_queue
  // to avoid a 3-consumer race with foxglove_streamer and lidar_publisher).
  std::mutex planner_cloud_mtx;
  std::queue<CloudData> planner_cloud_queue;

  // EGO-Planner → Foxglove: visualization paths (goal, init, optimal,
  // failed, astar, global).
  std::mutex planner_viz_mtx;
  std::queue<PlannerPath> planner_viz_queue;

  // Traj server → Foxglove: 50 Hz position / velocity / yaw commands.
  std::mutex planner_cmd_mtx;
  std::queue<PositionCmd> planner_cmd_queue;

  // -------------------------------------------------------------------------

  // Shutdown flag — set by SIGINT handler in main
  std::atomic<bool> running{true};
};

} // namespace livo
