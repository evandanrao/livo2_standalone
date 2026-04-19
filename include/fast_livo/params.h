// standalones/livo2/include/fast_livo/params.h
// Stage 4 — All runtime configuration.
// Sub-structs are expanded as needed by each stage.
// load() is fully implemented in Stage 4; for now it just stores file paths.
#pragma once
#include <string>
#include <vector>

namespace params {

struct LidarParams {
  std::string device_ip = "192.168.4.201";
  int udp_port = 2368;
  int ptc_port = 9347;
  int thread_num = 2;
  std::string correction_file; // path to .csv
  std::string firetimes_file;  // path to .csv
  int n_scans = 128;           // JT128: use all rings
  double blind = 0.01;         // minimum range, metres
  double blind_sqr = 0.0001;   // blind^2, pre-computed
  double time_offset = 0.0;    // lidar_time_offset (seconds)
};

struct ImuParams {
  double time_offset = 0.0; // imu_time_offset (seconds)
  double acc_cov = 1.0;     // accelerometer noise covariance
  double gyr_cov = 1.0;     // gyroscope noise covariance
  double ba_cov = 0.0001;
  double bg_cov = 0.0001;
  int imu_int_frame = 3; // frames used for IMU init
};

struct SlamParams {
  // Feature / enable flags
  int img_en = 1;
  int lidar_en = 1;
  bool imu_en = false;
  bool imu_prop_enable = false;
  bool gravity_align_en = false;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  // VIO
  bool normal_en = true;
  bool inverse_composition_en = false;
  int max_iterations = 5;
  double img_point_cov = 100.0;
  bool raycast_en = false;
  bool exposure_estimate_en = true;
  double inv_expo_cov = 0.2;
  int grid_size = 5;
  int grid_n_height = 17;
  int patch_pyrimid_level = 3;
  int patch_size = 8;
  double outlier_threshold = 1000.0;
  // Time offsets
  double exposure_time_init = 0.0;
  double img_time_offset = 0.0;
  // Preprocess
  double filter_size_surf = 0.5;
  bool hilti_en = false;
  int point_filter_num = 3;
  bool feature_extract_enabled = false;
  // Output
  int pcd_save_interval = -1;
  bool pcd_save_en = false;
  int pcd_save_type = 0;
  bool img_save_en = false;
  int img_save_interval = 1;
  bool colmap_output_en = false;
  double filter_size_pcd = 0.5;
  // EVO / logging
  std::string seq_name = "01";
  bool pose_output_en = false;
  // Calibration (flat vectors: T=[x,y,z], R=[3x3 row-major -> 9 elements])
  std::vector<double> extrin_T = {0.0, 0.0, 0.0};
  std::vector<double> extrin_R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::vector<double> camera_extrin_T = {0.0, 0.0, 0.0};
  std::vector<double> camera_extrin_R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  // Debug
  double plot_time = -10.0;
  int frame_cnt = 6;
  // Publish / visualisation
  double blind_rgb_points = 0.01;
  int pub_scan_num = 1;
  bool pub_effect_point_en = false;
  bool pub_plane_map_en = false;   // publish voxel plane centroids channel
  bool dense_map_en = false;
  bool verbose_en = false;
  // Camera YAML path (used in initializeComponents)
  std::string camera_yaml;
  // Voxel map / LIO params (from lio: section)
  double voxel_size = 0.4;
  int max_layer = 2;
  int max_points_num = 100;
  std::vector<int> layer_init_num = {5, 5, 5, 5, 5};
  double dept_err = 0.02;
  double beam_err = 0.05;
  double min_eigen_value = 0.0001;
  // Local map / sliding (from local_map: section)
  bool map_sliding_en = true;
  double half_map_size = 50.0;
  double sliding_thresh = 1.0;
};

struct Params {
  LidarParams lidar;
  ImuParams imu;
  SlamParams slam;
  std::string livo2_yaml;
  std::string camera_yaml; // top-level shortcut (also in slam.camera_yaml)
  // Logging
  struct LogParams {
    std::string log_dir = "/media/internal_logs/livo2";
    bool enable_mcap = true;
    int mcap_max_mb = 2048; // rotate at this many MB
  } log;
  // Foxglove streaming
  struct FoxgloveParams {
    int ws_port = 8765;
    int cloud_hz = 5;
    int image_hz = 15;
    bool enable_ws = true;
    bool enable_mcap = true;
  } foxglove;
};

// Load from YAML files. Throws std::runtime_error on missing required fields.
// Fully implemented in Stage 4; stub returns default-constructed Params.
Params load(const std::string &livo2_yaml, const std::string &camera_yaml);

} // namespace params
