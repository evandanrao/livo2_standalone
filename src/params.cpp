// Stage 4: YAML parameter loading for standalones/livo2
// Uses OpenCV FileStorage (already a dependency via FAST_LIVO2).
#include "fast_livo/params.h"
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>

namespace params {

// --------------------------------------------------------------------------
// Helper: read a scalar with a default value if the key is absent.
// --------------------------------------------------------------------------
template <typename T>
static T readOr(const cv::FileNode &node, const std::string &key, T def) {
  cv::FileNode n = node[key];
  if (n.empty())
    return def;
  T val;
  n >> val;
  return val;
}

static std::vector<double> readVecD(const cv::FileNode &node,
                                    const std::string &key,
                                    const std::vector<double> &def) {
  cv::FileNode n = node[key];
  if (n.empty() || !n.isSeq())
    return def;
  std::vector<double> out;
  for (auto it = n.begin(); it != n.end(); ++it)
    out.push_back(static_cast<double>(*it));
  return out;
}

static std::vector<int> readVecI(const cv::FileNode &node,
                                 const std::string &key,
                                 const std::vector<int> &def) {
  cv::FileNode n = node[key];
  if (n.empty() || !n.isSeq())
    return def;
  std::vector<int> out;
  for (auto it = n.begin(); it != n.end(); ++it)
    out.push_back(static_cast<int>(*it));
  return out;
}

// --------------------------------------------------------------------------
// Main load function
// --------------------------------------------------------------------------
Params load(const std::string &livo2_yaml, const std::string &camera_yaml) {
  Params p;
  p.livo2_yaml = livo2_yaml;
  p.camera_yaml = camera_yaml;

  // ---- livo2.yaml ----
  cv::FileStorage fs(livo2_yaml, cv::FileStorage::READ);
  if (!fs.isOpened())
    throw std::runtime_error("Cannot open livo2 config: " + livo2_yaml);

  // lidar:
  {
    cv::FileNode n = fs["lidar"];
    if (!n.empty()) {
      p.lidar.device_ip =
          readOr<std::string>(n, "device_ip", p.lidar.device_ip);
      p.lidar.ptc_port = readOr<int>(n, "ptc_port", p.lidar.ptc_port);
      p.lidar.udp_port = readOr<int>(n, "udp_port", p.lidar.udp_port);
      p.lidar.thread_num = readOr<int>(n, "thread_num", p.lidar.thread_num);
      p.lidar.correction_file =
          readOr<std::string>(n, "correction_file", p.lidar.correction_file);
      p.lidar.firetimes_file =
          readOr<std::string>(n, "firetimes_file", p.lidar.firetimes_file);
      p.lidar.n_scans = readOr<int>(n, "scan_line", p.lidar.n_scans);
      p.lidar.blind = readOr<double>(n, "blind", p.lidar.blind);
      p.lidar.blind_sqr = p.lidar.blind * p.lidar.blind;
    }
  }

  // time_offset:
  {
    cv::FileNode n = fs["time_offset"];
    if (!n.empty()) {
      p.imu.time_offset =
          readOr<double>(n, "imu_time_offset", p.imu.time_offset);
      p.lidar.time_offset =
          readOr<double>(n, "lidar_time_offset", p.lidar.time_offset);
      p.slam.img_time_offset =
          readOr<double>(n, "img_time_offset", p.slam.img_time_offset);
      p.slam.exposure_time_init =
          readOr<double>(n, "exposure_time_init", p.slam.exposure_time_init);
    }
  }

  // common:
  {
    cv::FileNode n = fs["common"];
    if (!n.empty()) {
      p.slam.img_en = readOr<int>(n, "img_en", p.slam.img_en);
      p.slam.lidar_en = readOr<int>(n, "lidar_en", p.slam.lidar_en);
    }
  }

  // preprocess:
  {
    cv::FileNode n = fs["preprocess"];
    if (!n.empty()) {
      p.slam.hilti_en = readOr<bool>(n, "hilti_en", p.slam.hilti_en);
      p.slam.point_filter_num =
          readOr<int>(n, "point_filter_num", p.slam.point_filter_num);
      p.slam.filter_size_surf =
          readOr<double>(n, "filter_size_surf", p.slam.filter_size_surf);
      p.lidar.n_scans = readOr<int>(n, "scan_line", p.lidar.n_scans);
      p.lidar.blind = readOr<double>(n, "blind", p.lidar.blind);
      p.lidar.blind_sqr = p.lidar.blind * p.lidar.blind;
      // feature_extract_enabled: false = use all points (faster); true =
      // edge/plane features only
      p.slam.feature_extract_enabled = readOr<bool>(
          n, "feature_extract_enabled", p.slam.feature_extract_enabled);
      // Note: lidar_type is intentionally not read. The Hesai SDK handles
      // packet decoding; preprocess.cpp does not branch on lidar_type in
      // standalone mode.
    }
  }

  // vio:
  {
    cv::FileNode n = fs["vio"];
    if (!n.empty()) {
      p.slam.max_iterations =
          readOr<int>(n, "max_iterations", p.slam.max_iterations);
      p.slam.outlier_threshold =
          readOr<double>(n, "outlier_threshold", p.slam.outlier_threshold);
      p.slam.img_point_cov =
          readOr<double>(n, "img_point_cov", p.slam.img_point_cov);
      p.slam.patch_size = readOr<int>(n, "patch_size", p.slam.patch_size);
      p.slam.patch_pyrimid_level =
          readOr<int>(n, "patch_pyrimid_level", p.slam.patch_pyrimid_level);
      p.slam.normal_en = readOr<bool>(n, "normal_en", p.slam.normal_en);
      p.slam.raycast_en = readOr<bool>(n, "raycast_en", p.slam.raycast_en);
      p.slam.inverse_composition_en = readOr<bool>(
          n, "inverse_composition_en", p.slam.inverse_composition_en);
      p.slam.exposure_estimate_en =
          readOr<bool>(n, "exposure_estimate_en", p.slam.exposure_estimate_en);
      p.slam.inv_expo_cov =
          readOr<double>(n, "inv_expo_cov", p.slam.inv_expo_cov);
      // VIO grid search parameters (control patch-tracking density)
      p.slam.grid_size = readOr<int>(n, "grid_size", p.slam.grid_size);
      p.slam.grid_n_height =
          readOr<int>(n, "grid_n_height", p.slam.grid_n_height);
    }
  }

  // imu:
  {
    cv::FileNode n = fs["imu"];
    if (!n.empty()) {
      p.slam.imu_en = readOr<bool>(n, "imu_en", p.slam.imu_en);
      p.imu.imu_int_frame =
          readOr<int>(n, "imu_int_frame", p.imu.imu_int_frame);
      p.imu.acc_cov = readOr<double>(n, "acc_cov", p.imu.acc_cov);
      p.imu.gyr_cov = readOr<double>(n, "gyr_cov", p.imu.gyr_cov);
      p.imu.ba_cov = readOr<double>(n, "b_acc_cov", p.imu.ba_cov);
      p.imu.bg_cov = readOr<double>(n, "b_gyr_cov", p.imu.bg_cov);
      // Estimation flags — disable to reduce CPU on constrained hardware
      p.slam.gravity_est_en =
          readOr<bool>(n, "gravity_est_en", p.slam.gravity_est_en);
      p.slam.ba_bg_est_en =
          readOr<bool>(n, "ba_bg_est_en", p.slam.ba_bg_est_en);
    }
  }

  // lio:
  {
    cv::FileNode n = fs["lio"];
    if (!n.empty()) {
      p.slam.max_iterations =
          readOr<int>(n, "max_iterations", p.slam.max_iterations);
      p.slam.dept_err = readOr<double>(n, "dept_err", p.slam.dept_err);
      p.slam.beam_err = readOr<double>(n, "beam_err", p.slam.beam_err);
      p.slam.min_eigen_value =
          readOr<double>(n, "min_eigen_value", p.slam.min_eigen_value);
      p.slam.voxel_size = readOr<double>(n, "voxel_size", p.slam.voxel_size);
      p.slam.max_layer = readOr<int>(n, "max_layer", p.slam.max_layer);
      p.slam.max_points_num =
          readOr<int>(n, "max_points_num", p.slam.max_points_num);
      p.slam.layer_init_num =
          readVecI(n, "layer_init_num", p.slam.layer_init_num);
    }
  }

  // local_map:
  {
    cv::FileNode n = fs["local_map"];
    if (!n.empty()) {
      p.slam.map_sliding_en =
          readOr<bool>(n, "map_sliding_en", p.slam.map_sliding_en);
      p.slam.half_map_size =
          readOr<double>(n, "half_map_size", p.slam.half_map_size);
      p.slam.sliding_thresh =
          readOr<double>(n, "sliding_thresh", p.slam.sliding_thresh);
    }
  }

  // uav:
  {
    cv::FileNode n = fs["uav"];
    if (!n.empty()) {
      p.slam.gravity_align_en =
          readOr<bool>(n, "gravity_align_en", p.slam.gravity_align_en);
      // imu_rate_odom enables high-rate (~200 Hz) IMU-propagated odometry
      // published on /livo2/imu_odom in addition to the 10 Hz LIO odom.
      p.slam.imu_prop_enable =
          readOr<bool>(n, "imu_rate_odom", p.slam.imu_prop_enable);
    }
  }

  // publish:
  {
    cv::FileNode n = fs["publish"];
    if (!n.empty()) {
      p.slam.dense_map_en =
          readOr<bool>(n, "dense_map_en", p.slam.dense_map_en);
      p.slam.pub_effect_point_en =
          readOr<bool>(n, "pub_effect_point_en", p.slam.pub_effect_point_en);
      // pub_plane_en publishes voxel plane centroids on /livo2/voxel_map.
      p.slam.pub_plane_map_en =
          readOr<bool>(n, "pub_plane_en", p.slam.pub_plane_map_en);
      p.slam.pub_scan_num = readOr<int>(n, "pub_scan_num", p.slam.pub_scan_num);
      p.slam.blind_rgb_points =
          readOr<double>(n, "blind_rgb_points", p.slam.blind_rgb_points);
    }
  }

  // global_map:
  {
    cv::FileNode n = fs["global_map"];
    if (!n.empty()) {
      p.slam.global_map_en =
          readOr<bool>(n, "enable", p.slam.global_map_en);
      p.slam.global_map_filter_size =
          readOr<double>(n, "filter_size", p.slam.global_map_filter_size);
      p.slam.global_map_publish_hz =
          readOr<double>(n, "publish_hz", p.slam.global_map_publish_hz);
    }
  }

  // debug:
  {
    cv::FileNode n = fs["debug"];
    if (!n.empty()) {
      p.slam.verbose_en = readOr<bool>(n, "verbose_en", p.slam.verbose_en);
      // plot_time: skip per-frame timing print until this many seconds have
      // elapsed (negative = always print when verbose_en=true)
      p.slam.plot_time = readOr<double>(n, "plot_time", p.slam.plot_time);
      // frame_cnt: number of frames used in the IMU propagation warm-up window
      p.slam.frame_cnt = readOr<int>(n, "frame_cnt", p.slam.frame_cnt);
    }
  }

  // evo:
  {
    cv::FileNode n = fs["evo"];
    if (!n.empty()) {
      p.slam.seq_name = readOr<std::string>(n, "seq_name", p.slam.seq_name);
      p.slam.pose_output_en =
          readOr<bool>(n, "pose_output_en", p.slam.pose_output_en);
    }
  }

  // pcd_save:
  {
    cv::FileNode n = fs["pcd_save"];
    if (!n.empty()) {
      p.slam.pcd_save_en = readOr<bool>(n, "pcd_save_en", p.slam.pcd_save_en);
      // Key names match ROS1 YAML convention (interval/type, not
      // pcd_save_interval/pcd_save_type)
      p.slam.pcd_save_interval =
          readOr<int>(n, "interval", p.slam.pcd_save_interval);
      p.slam.pcd_save_type = readOr<int>(n, "type", p.slam.pcd_save_type);
      p.slam.filter_size_pcd =
          readOr<double>(n, "filter_size_pcd", p.slam.filter_size_pcd);
      p.slam.colmap_output_en =
          readOr<bool>(n, "colmap_output_en", p.slam.colmap_output_en);
    }
  }

  // image_save: (separate section, matching ROS1 YAML convention)
  {
    cv::FileNode n = fs["image_save"];
    if (!n.empty()) {
      p.slam.img_save_en = readOr<bool>(n, "img_save_en", p.slam.img_save_en);
      p.slam.img_save_interval =
          readOr<int>(n, "interval", p.slam.img_save_interval);
    }
  }

  // extrin_calib:
  {
    cv::FileNode n = fs["extrin_calib"];
    if (!n.empty()) {
      p.slam.extrin_T = readVecD(n, "extrinsic_T", p.slam.extrin_T);
      p.slam.extrin_R = readVecD(n, "extrinsic_R", p.slam.extrin_R);
      p.slam.camera_extrin_T = readVecD(n, "Pcl", p.slam.camera_extrin_T);
      p.slam.camera_extrin_R = readVecD(n, "Rcl", p.slam.camera_extrin_R);
    }
  }

  // log: and foxglove: sections (optional)
  {
    cv::FileNode n = fs["log"];
    if (!n.empty()) {
      p.log.log_dir = readOr<std::string>(n, "log_dir", p.log.log_dir);
      p.log.enable_mcap = readOr<bool>(n, "enable_mcap", p.log.enable_mcap);
      p.log.mcap_max_mb = readOr<int>(n, "mcap_max_mb", p.log.mcap_max_mb);
    }
  }
  {
    cv::FileNode n = fs["foxglove"];
    if (!n.empty()) {
      p.foxglove.ws_port = readOr<int>(n, "ws_port", p.foxglove.ws_port);
      p.foxglove.cloud_hz = readOr<int>(n, "cloud_hz", p.foxglove.cloud_hz);
      p.foxglove.image_hz = readOr<int>(n, "image_hz", p.foxglove.image_hz);
      p.foxglove.enable_ws = readOr<bool>(n, "enable_ws", p.foxglove.enable_ws);
      p.foxglove.enable_mcap =
          readOr<bool>(n, "enable_mcap", p.foxglove.enable_mcap);
    }
  }

  fs.release();

  // ---- camera.yaml ----
  p.slam.camera_yaml = camera_yaml;
  // Note: intrinsics are read directly by livo::loadCameraFromYaml() in
  // camera_loader.cpp.

  return p;
}

} // namespace params
