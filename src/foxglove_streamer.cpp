// Stage 5/6 — Foxglove WebSocket + MCAP streamer
// Publishes SLAM outputs (pose, cloud, image) to Foxglove Studio and/or an MCAP
// file.
#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <thread>

#ifdef BUILD_FOXGLOVE
#include <foxglove/foxglove.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>

#include <cstdint>
#include <cstring>
#include <ctime>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <sys/stat.h> // mkdir for GCC 7 (no <filesystem>)

#include <opencv2/opencv.hpp> // for cv::FileStorage to read camera intrinsics

using namespace std::chrono;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint64_t to_ns(double seconds) {
  return static_cast<uint64_t>(seconds * 1e9);
}

static uint64_t now_ns() {
  return static_cast<uint64_t>(
      duration_cast<nanoseconds>(system_clock::now().time_since_epoch())
          .count());
}

static foxglove::messages::Timestamp to_ts(double seconds) {
  uint64_t ns = to_ns(seconds);
  return {static_cast<uint32_t>(ns / 1'000'000'000u),
          static_cast<uint32_t>(ns % 1'000'000'000u)};
}

static void mkdir_p(const std::string &path) {
  // Simple recursive mkdir for GCC 7 (no <filesystem>)
  for (size_t pos = 0; (pos = path.find('/', pos + 1)) != std::string::npos;) {
    ::mkdir(path.substr(0, pos).c_str(), 0755);
  }
  ::mkdir(path.c_str(), 0755);
}

static std::string make_mcap_path(const params::Params &p) {
  mkdir_p(p.log.log_dir);
  auto t = std::time(nullptr);
  char buf[64];
  std::strftime(buf, sizeof(buf), "livo2_%Y-%m-%d_%H-%M-%S.mcap",
                std::localtime(&t));
  return p.log.log_dir + "/" + buf;
}

// Simple token-bucket rate limiter
struct RateLimiter {
  explicit RateLimiter(int hz) : period_(1'000'000 / std::max(hz, 1)) {}
  bool ready() {
    auto now = steady_clock::now();
    if (duration_cast<microseconds>(now - last_).count() >= period_) {
      last_ = now;
      return true;
    }
    return false;
  }

private:
  int64_t period_;
  steady_clock::time_point last_ = steady_clock::now();
};

// ---------------------------------------------------------------------------
// Message converters
// ---------------------------------------------------------------------------

static void publish_odom(foxglove::messages::OdometryChannel &ch,
                         const livo::PoseData &pose) {
  using namespace foxglove::messages;
  Eigen::Quaterniond q(pose.rotation);
  Odometry msg;
  msg.timestamp = to_ts(pose.timestamp);
  msg.frame_id = "map";
  msg.body_frame_id = "body";
  msg.pose = Pose{.position = Vector3{pose.position.x(), pose.position.y(),
                                      pose.position.z()},
                  .orientation = Quaternion{q.x(), q.y(), q.z(), q.w()}};
  msg.linear_velocity =
      Vector3{pose.velocity.x(), pose.velocity.y(), pose.velocity.z()};
  ch.log(msg, to_ns(pose.timestamp));
}

static void publish_tf(foxglove::messages::FrameTransformChannel &ch,
                       const livo::PoseData &pose) {
  using namespace foxglove::messages;
  Eigen::Quaterniond q(pose.rotation);
  FrameTransform msg;
  msg.timestamp = to_ts(pose.timestamp);
  msg.parent_frame_id = "map";
  msg.child_frame_id = "body";
  msg.translation =
      Vector3{pose.position.x(), pose.position.y(), pose.position.z()};
  msg.rotation = Quaternion{q.x(), q.y(), q.z(), q.w()};
  ch.log(msg, to_ns(pose.timestamp));
}

static void append_path(foxglove::messages::PosesInFrame &path_msg,
                        const livo::PoseData &pose) {
  using namespace foxglove::messages;
  Eigen::Quaterniond q(pose.rotation);
  Pose p;
  p.position = Vector3{pose.position.x(), pose.position.y(), pose.position.z()};
  p.orientation = Quaternion{q.x(), q.y(), q.z(), q.w()};
  path_msg.poses.push_back(std::move(p));
}

static void publish_cloud(foxglove::messages::PointCloudChannel &ch,
                          const livo::CloudData &cd) {
  using namespace foxglove::messages;
  PointCloud msg;
  msg.timestamp = to_ts(cd.timestamp);
  msg.frame_id = "map";
  // 3 floats (x,y,z) + 1 float (intensity) per point → 16 bytes per point
  constexpr uint32_t stride = 16;
  msg.point_stride = stride;
  msg.fields = {
      {"x", 0, PackedElementField::NumericType::FLOAT32},
      {"y", 4, PackedElementField::NumericType::FLOAT32},
      {"z", 8, PackedElementField::NumericType::FLOAT32},
      {"intensity", 12, PackedElementField::NumericType::FLOAT32},
  };
  size_t n = cd.cloud->points.size();
  msg.data.resize(n * stride);
  auto *ptr = reinterpret_cast<float *>(msg.data.data());
  for (auto &pt : cd.cloud->points) {
    ptr[0] = pt.x;
    ptr[1] = pt.y;
    ptr[2] = pt.z;
    ptr[3] = pt.intensity;
    ptr += 4;
  }
  ch.log(msg, to_ns(cd.timestamp));
}

// Publishes an RGB-coloured point cloud (from LIVO VIO frames).
// Wire format: x(F32), y(F32), z(F32), r(U8), g(U8), b(U8) — stride 16 bytes.
// The final 2 bytes of each stride slot are unused padding to keep 4-byte
// alignment, which is required by some Foxglove renderer paths.
static void publish_rgb_cloud(foxglove::messages::PointCloudChannel &ch,
                              const livo::RgbCloudData &cd) {
  using namespace foxglove::messages;
  PointCloud msg;
  msg.timestamp = to_ts(cd.timestamp);
  msg.frame_id = "map";
  constexpr uint32_t stride = 16; // 3×F32 + 3×U8 + 1 pad byte
  msg.point_stride = stride;
  msg.fields = {
      {"x", 0, PackedElementField::NumericType::FLOAT32},
      {"y", 4, PackedElementField::NumericType::FLOAT32},
      {"z", 8, PackedElementField::NumericType::FLOAT32},
      {"r", 12, PackedElementField::NumericType::UINT8},
      {"g", 13, PackedElementField::NumericType::UINT8},
      {"b", 14, PackedElementField::NumericType::UINT8},
  };
  size_t n = cd.cloud->points.size();
  msg.data.resize(
      n * stride); // default-initialised to std::byte{0} (padding stays 0)
  std::byte *base = msg.data.data();
  for (size_t i = 0; i < n; ++i) {
    const auto &pt = cd.cloud->points[i];
    std::byte *slot = base + i * stride;
    float xyz[3] = {pt.x, pt.y, pt.z};
    std::memcpy(slot, xyz, 12);              // bytes 0-11: xyz
    slot[12] = static_cast<std::byte>(pt.r); // byte 12: red
    slot[13] = static_cast<std::byte>(pt.g); // byte 13: green
    slot[14] = static_cast<std::byte>(pt.b); // byte 14: blue
    // slot[15] = std::byte{0} (padding, already zero from resize)
  }
  ch.log(msg, to_ns(cd.timestamp));
}

static void publish_image(foxglove::messages::RawImageChannel &ch,
                          const livo::ImageData &img) {
  using namespace foxglove::messages;
  if (img.image.empty())
    return;
  RawImage msg;
  msg.timestamp = to_ts(img.timestamp);
  msg.frame_id = "camera";
  msg.width = static_cast<uint32_t>(img.image.cols);
  msg.height = static_cast<uint32_t>(img.image.rows);
  msg.encoding = "bgr8";
  msg.step = static_cast<uint32_t>(img.image.cols * 3);
  size_t nbytes = img.image.total() * img.image.elemSize();
  msg.data.resize(nbytes);
  std::memcpy(msg.data.data(), img.image.data, nbytes);
  ch.log(msg, to_ns(img.timestamp));
}

// ---------------------------------------------------------------------------
// Streamer thread
// ---------------------------------------------------------------------------

void foxglove_streamer_thread(livo::Bridge &bridge, const params::Params &p) {
  foxglove::setLogLevel(foxglove::LogLevel::Info);

  // ---- Create sinks ----
  std::optional<foxglove::McapWriter> mcap_writer;
  if (p.foxglove.enable_mcap) {
    foxglove::McapWriterOptions opts;
    std::string mcap_path =
        make_mcap_path(p); // keep alive — opts.path is string_view
    opts.path = mcap_path;
    opts.compression = foxglove::McapCompression::Zstd;
    auto res = foxglove::McapWriter::create(opts);
    if (res.has_value()) {
      mcap_writer = std::move(res.value());
    } else {
      fprintf(stderr, "[foxglove] Failed to create MCAP writer: %s\n",
              foxglove::strerror(res.error()));
    }
  }

  std::optional<foxglove::WebSocketServer> ws_server;
  if (p.foxglove.enable_ws) {
    foxglove::WebSocketServerOptions opts;
    opts.name = "livo2";
    opts.host = "0.0.0.0";
    opts.port = static_cast<uint16_t>(p.foxglove.ws_port);

    // ---- Enable bidirectional client-publish for goal injection ----
    opts.capabilities = foxglove::WebSocketServerCapabilities::ClientPublish;
    opts.supported_encodings = {"json"};

    // Track which (client_id, channel_id) pairs correspond to known topics.
    // Maps GoalKey → topic string so one dispatcher handles all inbound topics.
    auto client_ch_mtx = std::make_shared<std::mutex>();
    using GoalKey = std::pair<uint32_t, uint32_t>;
    auto client_ch_map = std::make_shared<std::map<GoalKey, std::string>>();

    opts.callbacks.onClientAdvertise =
        [client_ch_map, client_ch_mtx](uint32_t client_id,
                                       const foxglove::ClientChannel &ch) {
          const std::string topic(ch.topic.data(), ch.topic.size());
          if (topic == "/planner/goal" || topic == "/planner/trigger" ||
              topic == "/planner/mandatory_stop") {
            std::lock_guard<std::mutex> lk(*client_ch_mtx);
            (*client_ch_map)[{client_id, ch.id}] = topic;
            fprintf(stdout,
                    "[foxglove] Client %u advertised %s (ch %u)\n",
                    client_id, topic.c_str(), ch.id);
          }
        };

    opts.callbacks.onClientUnadvertise =
        [client_ch_map, client_ch_mtx](uint32_t client_id,
                                       uint32_t client_channel_id) {
          std::lock_guard<std::mutex> lk(*client_ch_mtx);
          client_ch_map->erase({client_id, client_channel_id});
        };

    // Shared pointer to bridge for use inside the lambda
    livo::Bridge *bridge_ptr = &bridge;
    // Shared pointer to planner to call setTrigger/mandatoryStop
    // (planner lives in main's scope; bridge.running guards its lifetime)
    opts.callbacks.onMessageData =
        [bridge_ptr, client_ch_map, client_ch_mtx](
            uint32_t client_id, uint32_t client_channel_id,
            const std::byte *data, size_t data_len) {
          std::string topic;
          {
            std::lock_guard<std::mutex> lk(*client_ch_mtx);
            auto it = client_ch_map->find({client_id, client_channel_id});
            if (it == client_ch_map->end())
              return;
            topic = it->second;
          }

          if (topic == "/planner/goal") {
            // Parse JSON: {"x": <float>, "y": <float>, "z": <float>}
            std::string json(reinterpret_cast<const char *>(data), data_len);
            const char *s = json.c_str();
            const char *xp = strstr(s, "\"x\"");
            const char *yp = strstr(s, "\"y\"");
            const char *zp = strstr(s, "\"z\"");
            if (!xp || !yp || !zp)
              return;
            xp = strchr(xp + 3, ':');
            yp = strchr(yp + 3, ':');
            zp = strchr(zp + 3, ':');
            if (!xp || !yp || !zp)
              return;
            float x = std::strtof(xp + 1, nullptr);
            float y = std::strtof(yp + 1, nullptr);
            float z = std::strtof(zp + 1, nullptr);
            fprintf(stdout,
                    "[foxglove] /planner/goal: (%.2f, %.2f, %.2f)\n", x, y, z);
            livo::Goal goal{x, y, z};
            std::lock_guard<std::mutex> lk(bridge_ptr->planner_goal_mtx);
            while (bridge_ptr->planner_goal_queue.size() >=
                   livo::kPlannerGoalQueueMax)
              bridge_ptr->planner_goal_queue.pop();
            bridge_ptr->planner_goal_queue.push(goal);

          } else if (topic == "/planner/trigger") {
            // Any message on this topic arms the planner.
            // Push a special sentinel goal {NaN,NaN,NaN} to signal trigger.
            livo::Goal trigger_sentinel{std::numeric_limits<float>::quiet_NaN(),
                                       std::numeric_limits<float>::quiet_NaN(),
                                       std::numeric_limits<float>::quiet_NaN()};
            std::lock_guard<std::mutex> lk(bridge_ptr->planner_goal_mtx);
            // Prepend sentinel by clearing and re-pushing — we use NaN as
            // "trigger" signal recognised in Planner::run().
            bridge_ptr->planner_goal_queue.push(trigger_sentinel);

          } else if (topic == "/planner/mandatory_stop") {
            // Any message forces emergency stop.
            livo::Goal stop_sentinel{-std::numeric_limits<float>::infinity(),
                                    -std::numeric_limits<float>::infinity(),
                                    -std::numeric_limits<float>::infinity()};
            std::lock_guard<std::mutex> lk(bridge_ptr->planner_goal_mtx);
            bridge_ptr->planner_goal_queue.push(stop_sentinel);
          }
        };

    auto res = foxglove::WebSocketServer::create(std::move(opts));
    if (res.has_value()) {
      ws_server = std::move(res.value());
    } else {
      fprintf(stderr, "[foxglove] Failed to create WebSocket server: %s\n",
              foxglove::strerror(res.error()));
    }
  }

  // ---- Create typed channels (global context → auto-routed to all sinks) ----
  auto odom_ch =
      foxglove::messages::OdometryChannel::create("/livo2/odom").value();
  auto cloud_ch =
      foxglove::messages::PointCloudChannel::create("/livo2/map_cloud").value();
  auto img_ch =
      foxglove::messages::RawImageChannel::create("/livo2/image_tracked")
          .value();
  auto tf_ch = foxglove::messages::FrameTransformChannel::create("/tf").value();
  auto path_ch =
      foxglove::messages::PosesInFrameChannel::create("/livo2/path").value();

  // ---- Channels added in Stage 5 (parity with ROS1) ----
  // RGB-coloured map cloud (LIVO VIO frames only, img_en=1).
  auto rgb_cloud_ch =
      foxglove::messages::PointCloudChannel::create("/livo2/map_cloud_rgb")
          .value();
  // High-rate IMU-propagated odometry (~200 Hz, when imu_rate_odom=true).
  auto imu_odom_ch =
      foxglove::messages::OdometryChannel::create("/livo2/imu_odom").value();
  // Matched LiDAR-to-plane effect points used in each EKF update.
  auto effect_ch =
      foxglove::messages::PointCloudChannel::create("/livo2/effect_points")
          .value();
  // Voxel plane centroids — structural quality map (when pub_plane_en=true).
  auto voxel_map_ch =
      foxglove::messages::PointCloudChannel::create("/livo2/voxel_map").value();
  // Visual tracking sub-map (3D map points, img_en=1 only).
  auto visual_map_ch =
      foxglove::messages::PointCloudChannel::create("/livo2/visual_sub_map")
          .value();
  // Accumulated downsampled global map — refreshed at global_map/publish_hz.
  auto global_map_ch =
      foxglove::messages::PointCloudChannel::create("/livo2/global_map")
          .value();
  // EGO-Planner: sampled trajectory waypoints (PosesInFrame).
  auto planner_traj_ch =
      foxglove::messages::PosesInFrameChannel::create("/planner/trajectory")
          .value();
  // EGO-Planner: inflated occupancy voxel centres (PointCloud xyz float).
  auto planner_occ_ch =
      foxglove::messages::PointCloudChannel::create("/planner/occupancy")
          .value();
  // EGO-Planner: visualization paths (goal, init, optimal, failed, astar, global).
  auto planner_goal_viz_ch =
      foxglove::messages::PosesInFrameChannel::create("/planner/viz/goal")
          .value();
  auto planner_global_ch =
      foxglove::messages::PosesInFrameChannel::create("/planner/viz/global")
          .value();
  auto planner_init_ch =
      foxglove::messages::PosesInFrameChannel::create("/planner/viz/init")
          .value();
  auto planner_optimal_ch =
      foxglove::messages::PosesInFrameChannel::create("/planner/viz/optimal")
          .value();
  auto planner_failed_ch =
      foxglove::messages::PosesInFrameChannel::create("/planner/viz/failed")
          .value();
  auto planner_astar_ch =
      foxglove::messages::PosesInFrameChannel::create("/planner/viz/astar")
          .value();
  // EGO-Planner: 50 Hz position command (traj server output).
  auto planner_cmd_ch =
      foxglove::messages::OdometryChannel::create("/planner/position_cmd")
          .value();
  auto camcal_ch =
      foxglove::messages::CameraCalibrationChannel::create("/livo2/camera_info")
          .value();

  // ---- Publish camera calibration once at startup ----
  {
    cv::FileStorage fs(p.camera_yaml, cv::FileStorage::READ);
    if (fs.isOpened()) {
      using namespace foxglove::messages;
      CameraCalibration cal;
      cal.timestamp = to_ts(0.0);
      cal.frame_id = "camera";
      cal.width = (uint32_t)(int)fs["image_width"];
      cal.height = (uint32_t)(int)fs["image_height"];
      cal.distortion_model = "plumb_bob";
      cv::FileNode pp = fs["projection_parameters"];
      double fx = (double)pp["fx"], fy = (double)pp["fy"];
      double cx = (double)pp["cx"], cy = (double)pp["cy"];
      cv::FileNode dc = fs["distortion_coefficients"];
      double k1 = (double)dc["k1"], k2 = (double)dc["k2"];
      double p1 = (double)dc["p1"], p2 = (double)dc["p2"];
      double k3 = (double)dc["k3"];
      cal.d = {k1, k2, p1, p2, k3};
      cal.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
      cal.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      cal.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
      camcal_ch.log(cal, now_ns());
      fs.release();
    }
  }

  // ---- Rate limiters ----
  RateLimiter cloud_limiter(p.foxglove.cloud_hz);
  RateLimiter image_limiter(p.foxglove.image_hz);

  // ---- Accumulate path ----
  foxglove::messages::PosesInFrame path_msg;
  path_msg.frame_id = "map";

  while (bridge.running.load()) {

    // ---- Drain pose_queue (LIO rate, ~10 Hz) ----
    // Publishes each pose as /livo2/odom + /tf.  Path appended and re-logged
    // only when new poses arrive (avoids redundant large message sends).
    {
      std::queue<livo::PoseData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.pose_mtx);
        std::swap(local, bridge.pose_queue);
      }
      bool path_updated = false;
      while (!local.empty()) {
        const auto &pose = local.front();
        publish_odom(odom_ch, pose);
        publish_tf(tf_ch, pose);
        append_path(path_msg, pose);
        local.pop();
        path_updated = true;
      }
      if (path_updated && !path_msg.poses.empty()) {
        path_ch.log(path_msg, now_ns());
      }
    }

    // ---- Drain imu_prop_queue (~200 Hz IMU-rate odometry) ----
    // Publishes every pose — no dropping — to preserve the full high-rate
    // stream.
    {
      std::queue<livo::PoseData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.imu_prop_mtx);
        std::swap(local, bridge.imu_prop_queue);
      }
      while (!local.empty()) {
        publish_odom(imu_odom_ch, local.front());
        local.pop();
      }
    }

    // ---- Drain viz_cloud_queue (XYZI map cloud, rate-limited) ----
    if (cloud_limiter.ready()) {
      std::queue<livo::CloudData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.viz_cloud_mtx);
        std::swap(local, bridge.viz_cloud_queue);
      }
      if (!local.empty()) {
        // Only publish the most recent frame (drop stale ones).
        while (local.size() > 1)
          local.pop();
        publish_cloud(cloud_ch, local.front());
      }
    }

    // ---- Drain viz_rgb_cloud_queue (RGB map cloud, rate-limited) ----
    // Only has data when slam_mode=LIVO and img_en=1.
    if (cloud_limiter.ready()) {
      std::queue<livo::RgbCloudData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.viz_rgb_cloud_mtx);
        std::swap(local, bridge.viz_rgb_cloud_queue);
      }
      if (!local.empty()) {
        while (local.size() > 1)
          local.pop();
        publish_rgb_cloud(rgb_cloud_ch, local.front());
      }
    }

    // ---- Drain effect_cloud_queue (matched feature points) ----
    // Published at LIO rate; kept separate so Foxglove can colour them
    // differently from the map cloud.
    {
      std::queue<livo::CloudData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.effect_cloud_mtx);
        std::swap(local, bridge.effect_cloud_queue);
      }
      if (!local.empty()) {
        while (local.size() > 1)
          local.pop();
        publish_cloud(effect_ch, local.front());
      }
    }

    // ---- Drain voxel_map_queue (plane centroids, heavily throttled) ----
    // pubVoxelMap() already throttles to ~2 Hz, so drain without further
    // rate-limiting.
    {
      std::queue<livo::CloudData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.voxel_map_mtx);
        std::swap(local, bridge.voxel_map_queue);
      }
      if (!local.empty()) {
        while (local.size() > 1)
          local.pop();
        publish_cloud(voxel_map_ch, local.front());
      }
    }

    // ---- Drain visual_map_queue (VIO tracking sub-map, img_en=1 only) ----
    {
      std::queue<livo::CloudData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.visual_map_mtx);
        std::swap(local, bridge.visual_map_queue);
      }
      if (!local.empty()) {
        while (local.size() > 1)
          local.pop();
        publish_cloud(visual_map_ch, local.front());
      }
    }

    // ---- Drain global_map_queue (accumulated downsampled map) ----
    // Already rate-limited by LIVMapper (global_map/publish_hz). Always send
    // the latest — Foxglove replaces the topic on each message so older
    // clients see the growing map.
    {
      std::queue<livo::CloudData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.global_map_mtx);
        std::swap(local, bridge.global_map_queue);
      }
      if (!local.empty()) {
        while (local.size() > 1)
          local.pop();
        publish_cloud(global_map_ch, local.front());
      }
    }

    // ---- Drain planner_traj_queue (EGO-Planner trajectory waypoints) ----
    {
      std::queue<livo::TrajectoryData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.planner_traj_mtx);
        std::swap(local, bridge.planner_traj_queue);
      }
      if (!local.empty()) {
        while (local.size() > 1)
          local.pop();
        const auto &td = local.front();
        using namespace foxglove::messages;
        PosesInFrame msg;
        msg.frame_id = "map";
        msg.timestamp = to_ts(td.timestamp);
        msg.poses.reserve(td.waypoints.size());
        for (size_t i = 0; i < td.waypoints.size(); ++i) {
          Pose ps;
          ps.position = Vector3{td.waypoints[i](0), td.waypoints[i](1),
                                td.waypoints[i](2)};
          ps.orientation = Quaternion{0, 0, 0, 1}; // identity (x,y,z,w)
          msg.poses.push_back(ps);
        }
        planner_traj_ch.log(msg, now_ns());
      }
    }

    // ---- Drain planner_occ_queue (EGO-Planner occupancy voxels) ----
    {
      std::queue<livo::OccupancyViz> local;
      {
        std::lock_guard<std::mutex> lk(bridge.planner_occ_mtx);
        std::swap(local, bridge.planner_occ_queue);
      }
      if (!local.empty()) {
        while (local.size() > 1)
          local.pop();
        const auto &ov = local.front();
        using namespace foxglove::messages;
        PointCloud msg;
        msg.frame_id = "map";
        msg.timestamp = to_ts(ov.timestamp);
        msg.point_stride = 12; // 3× float32
        msg.fields = {
            {"x", 0, PackedElementField::NumericType::FLOAT32},
            {"y", 4, PackedElementField::NumericType::FLOAT32},
            {"z", 8, PackedElementField::NumericType::FLOAT32},
        };
        const size_t n = ov.occupied_voxels.size();
        msg.data.resize(n * 12);
        for (size_t i = 0; i < n; ++i) {
          float buf[3] = {ov.occupied_voxels[i](0), ov.occupied_voxels[i](1),
                          ov.occupied_voxels[i](2)};
          std::memcpy(msg.data.data() + i * 12, buf, 12);
        }
        planner_occ_ch.log(msg, now_ns());
      }
    }

    // ---- Drain viz_img_queue (feature-overlay image, rate-limited) ----
    if (image_limiter.ready()) {
      std::queue<livo::ImageData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.viz_img_mtx);
        std::swap(local, bridge.viz_img_queue);
      }
      if (!local.empty()) {
        while (local.size() > 1)
          local.pop();
        publish_image(img_ch, local.front());
      }
    }

    // ---- Drain planner_viz_queue (goal, init, optimal, failed, astar, global, ground_height) ----
    {
      // Collect latest per-label (drop stale frames of same label)
      std::map<std::string, livo::PlannerPath> latest_by_label;
      {
        std::queue<livo::PlannerPath> local;
        std::lock_guard<std::mutex> lk(bridge.planner_viz_mtx);
        std::swap(local, bridge.planner_viz_queue);
        while (!local.empty()) {
          latest_by_label[local.front().label] = std::move(local.front());
          local.pop();
        }
      }
      for (auto &kv : latest_by_label) {
        const auto &pp = kv.second;
        using namespace foxglove::messages;
        PosesInFrame msg;
        msg.frame_id = "map";
        msg.timestamp = to_ts(pp.timestamp);
        msg.poses.reserve(pp.waypoints.size());
        for (const auto &w : pp.waypoints) {
          Pose ps;
          ps.position = Vector3{w(0), w(1), w(2)};
          ps.orientation = Quaternion{0, 0, 0, 1};
          msg.poses.push_back(ps);
        }
        const std::string &lbl = kv.first;
        if (lbl == "goal")
          planner_goal_viz_ch.log(msg, now_ns());
        else if (lbl == "global")
          planner_global_ch.log(msg, now_ns());
        else if (lbl == "init")
          planner_init_ch.log(msg, now_ns());
        else if (lbl == "optimal")
          planner_optimal_ch.log(msg, now_ns());
        else if (lbl == "failed")
          planner_failed_ch.log(msg, now_ns());
        else if (lbl == "astar")
          planner_astar_ch.log(msg, now_ns());
        // "ground_height" is a single point — reuse planner_goal_viz_ch is OK
        // but publish on a dedicated OdometryChannel would be cleaner; for now
        // we just skip it (it's an internal diagnostic, not primary viz).
      }
    }

    // ---- Drain planner_cmd_queue (50 Hz traj server position commands) ----
    {
      std::queue<livo::PositionCmd> local;
      {
        std::lock_guard<std::mutex> lk(bridge.planner_cmd_mtx);
        std::swap(local, bridge.planner_cmd_queue);
      }
      while (!local.empty()) {
        const auto &cmd = local.front();
        using namespace foxglove::messages;
        // Encode yaw as Z-axis rotation quaternion
        double half_yaw = cmd.yaw * 0.5;
        double sin_h = std::sin(half_yaw), cos_h = std::cos(half_yaw);
        Odometry msg;
        msg.timestamp = to_ts(cmd.timestamp);
        msg.frame_id = "map";
        msg.body_frame_id = "cmd";
        msg.pose = Pose{
            Vector3{cmd.position(0), cmd.position(1), cmd.position(2)},
            Quaternion{0.0, 0.0, sin_h, cos_h}};
        msg.linear_velocity =
            Vector3{cmd.velocity(0), cmd.velocity(1), cmd.velocity(2)};
        planner_cmd_ch.log(msg, now_ns());
        local.pop();
      }
    }

    std::this_thread::sleep_for(2ms);
  }

  // Graceful shutdown
  if (ws_server.has_value()) {
    ws_server->stop();
  }
  if (mcap_writer.has_value()) {
    mcap_writer->close();
  }
}

#else // BUILD_FOXGLOVE not defined

void foxglove_streamer_thread(livo::Bridge &bridge,
                              const params::Params & /*p*/) {
  // Foxglove not compiled in — wait until shutdown
  while (bridge.running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
  }
}

#endif // BUILD_FOXGLOVE
