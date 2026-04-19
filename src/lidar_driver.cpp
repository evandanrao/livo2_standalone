// standalones/livo2/src/lidar_driver.cpp
// Stage 1 — Hesai SDK direct integration (replaces HesaiLidar_ROS_2.0 driver).
// Registers point cloud + IMU callbacks, converts to livo::{CloudData,ImuData},
// and pushes into bridge queues.
//
// Mirrors the SDK setup in standalones/lio_lc laserMapping.cpp (Stage 1
// section). IMU units from SDK: accel in g → convert to m/s²; gyro in deg/s →
// rad/s.

#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"

#include <HesaiLidar_SDK_2.0/driver/hesai_lidar_sdk.hpp>
#include <pcl/point_types.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

using namespace hesai::lidar;

static constexpr double G_TO_MS2 = 9.80665;
static constexpr double DEG_TO_RAD = M_PI / 180.0;

void lidar_driver_thread(livo::Bridge &bridge, const params::Params &p) {
  // ── Configure SDK ────────────────────────────────────────────────────────
  DriverParam sdk_param;
  sdk_param.input_param.source_type = DATA_FROM_LIDAR;
  sdk_param.input_param.device_ip_address = p.lidar.device_ip;
  sdk_param.input_param.udp_port = static_cast<uint16_t>(p.lidar.udp_port);
  sdk_param.input_param.ptc_port = static_cast<uint16_t>(p.lidar.ptc_port);
  sdk_param.input_param.correction_file_path = p.lidar.correction_file;
  sdk_param.input_param.firetimes_path = p.lidar.firetimes_file;
  sdk_param.input_param.send_point_cloud_ros = false;
  // Despite the ROS-like name, this flag controls whether IMU packets are
  // emitted by the SDK callback path we consume in standalone mode.
  sdk_param.input_param.send_imu_ros = true;
  sdk_param.input_param.send_packet_ros = false;
  sdk_param.decoder_param.enable_parser_thread = true;
  sdk_param.decoder_param.thread_num = p.lidar.thread_num;

  HesaiLidarSdk<LidarPointXYZIRT> sdk;

  // ── Point cloud callback ─────────────────────────────────────────────────
  sdk.RegRecvCallback([&bridge,
                       &p](const LidarDecodedFrame<LidarPointXYZIRT> &frame) {
    if (frame.points_num == 0)
      return;

    livo::CloudData data;
    data.timestamp = frame.frame_start_timestamp;
    data.cloud = PointCloudXYZI::Ptr(new PointCloudXYZI());
    data.cloud->reserve(frame.points_num);

    double time_head = frame.frame_start_timestamp;

    for (int i = 0; i < frame.points_num; i++) {
      const auto &sp = frame.points[i];
      // Filter out-of-range scans and NaN/zero points
      if (sp.ring >= static_cast<uint16_t>(p.lidar.n_scans))
        continue;
      float range_sq = sp.x * sp.x + sp.y * sp.y + sp.z * sp.z;
      if (range_sq < static_cast<float>(p.lidar.blind_sqr))
        continue;
      if (std::isnan(sp.x) || std::isnan(sp.y) || std::isnan(sp.z))
        continue;

      pcl::PointXYZINormal pt;
      pt.x = sp.x;
      pt.y = sp.y;
      pt.z = sp.z;
      pt.intensity = static_cast<float>(sp.intensity);
      pt.normal_x = 0.f;
      pt.normal_y = 0.f;
      pt.normal_z = 0.f;
      // curvature = per-point time offset in ms from frame start (for motion
      // undistortion)
      pt.curvature = static_cast<float>((sp.timestamp - time_head) * 1000.0);
      data.cloud->push_back(pt);
    }

    // Sort by time (JT128 delivers ring-first; undistortion needs time order)
    std::sort(data.cloud->begin(), data.cloud->end(),
              [](const pcl::PointXYZINormal &a, const pcl::PointXYZINormal &b) {
                return a.curvature < b.curvature;
              });

    std::lock_guard<std::mutex> lk(bridge.cloud_mtx);
    if (bridge.cloud_queue.size() < livo::kCloudQueueMax)
      bridge.cloud_queue.push(std::move(data));
  });

  // ── IMU callback ─────────────────────────────────────────────────────────
  sdk.RegRecvCallback([&bridge, &p](const LidarImuData &imu) {
    // Apply time offset (lidar→IMU) same as lio_lc
    double ts = imu.timestamp - p.imu.time_offset;

    livo::ImuData data;
    data.timestamp = ts;
    // SDK delivers accel in g — convert to m/s²
    data.accel = {imu.imu_accel_x * G_TO_MS2, imu.imu_accel_y * G_TO_MS2,
                  imu.imu_accel_z * G_TO_MS2};
    // SDK delivers gyro in deg/s — convert to rad/s
    data.gyro = {imu.imu_ang_vel_x * DEG_TO_RAD, imu.imu_ang_vel_y * DEG_TO_RAD,
                 imu.imu_ang_vel_z * DEG_TO_RAD};

    std::lock_guard<std::mutex> lk(bridge.imu_mtx);
    if (bridge.imu_queue.size() < livo::kImuQueueMax)
      bridge.imu_queue.push(data);
  });

  // ── Start SDK ────────────────────────────────────────────────────────────
  if (!sdk.Init(sdk_param)) {
    fprintf(
        stderr,
        "[lidar_driver] Hesai SDK Init failed — check device_ip and network\n");
    bridge.running.store(false);
    return;
  }
  sdk.Start();
  fprintf(stderr, "[lidar_driver] Hesai SDK started (ip=%s udp=%d)\n",
          p.lidar.device_ip.c_str(), p.lidar.udp_port);

  // ── Poll shutdown flag ───────────────────────────────────────────────────
  while (bridge.running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  sdk.Stop();
  fprintf(stderr, "[lidar_driver] stopped\n");
}
