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

  // -------------------------------------------------------------------------

  // Shutdown flag — set by SIGINT handler in main
  std::atomic<bool> running{true};
};

} // namespace livo
