// standalones/livo2/include/fast_livo/params.h
// Stage 4 — All runtime configuration.
// Sub-structs are expanded as needed by each stage.
// load() is fully implemented in Stage 4; for now it just stores file paths.
#pragma once
#include <array>
#include <string>
#include <vector>

namespace params {

struct LidarParams {
  std::string device_ip = "192.168.4.201";
  int udp_port = 2368;
  int ptc_port = 9347;
  int thread_num = 2;
  std::string correction_file; // path to .csv (empty = download from lidar)
  std::string firetimes_file = // JT128 bundled CSV; overridable via yaml
#ifdef HESAI_FIRETIMES_FILE
      HESAI_FIRETIMES_FILE
#else
      ""
#endif
      ;
  int n_scans = 128;         // JT128: use all rings
  double blind = 0.01;       // minimum range, metres
  double blind_sqr = 0.0001; // blind^2, pre-computed
  double time_offset = 0.0;  // lidar_time_offset (seconds)
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
  std::string pcd_dir =
      "/media/internal_logs/pcd"; // directory for saved .pcd files
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
  bool pub_plane_map_en = false; // publish voxel plane centroids channel
  // Global map accumulation channel (/livo2/global_map)
  bool global_map_en = false;
  double global_map_filter_size = 0.3; // voxel grid leaf size (metres)
  double global_map_publish_hz = 0.2;  // publish rate (Hz); 0.2 = every 5 s
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

struct PlannerParams {
  // FSM
  double replan_thresh = 1.0;
  double planning_horizon = 6.0;
  double emergency_time = 1.0;
  bool enable_fail_safe = true;

  // Manager / trajectory
  double max_vel = 3.0;
  double max_acc = 4.0;
  double feasibility_tolerance = 0.0;
  double polyTraj_piece_length = 1.5;
  bool use_multitopology_trajs = false;
  int drone_id = -1;

  // Optimizer
  int cps_num_prePiece = 3;
  double max_jer = 9.0;
  double wei_obs = 10000.0;
  double wei_obs_soft = 3000.0;
  double wei_feas = 10000.0;
  double wei_sqrvar = 10.0;
  double wei_time = 10.0;
  double obs_clearance = 0.3;
  double obs_clearance_soft = 1.5;

  // Grid map
  double grid_resolution = 0.1;
  double local_range_xy = 5.5;
  double local_range_z = 3.0;
  double obstacles_inflation = 0.3;
  double p_hit = 0.70;
  double p_miss = 0.35;
  double p_min = 0.12;
  double p_max = 0.97;
  double p_occ = 0.80;
  double fading_time = 1000.0;
  double min_ray_length = 0.1;
  double odom_depth_timeout = 1.0;
  bool enable_virtual_wall = false;
  double virtual_ceil = 1.0;
  double virtual_ground = -0.1;

  // Preset waypoint flight mode
  int target_type = 0; // 0 = MANUAL_TARGET, 1 = PRESET_TARGET
  bool realworld_experiment =
      false; // if true, wait for trigger before first move
  int waypoint_num = 0;
  std::vector<std::array<double, 3>> waypoints; // list of [x,y,z] waypoints

  // Trajectory server yaw control
  double time_forward = 1.0; // seconds look-ahead for yaw computation
  bool enable_ground_height_measurement =
      false; // scan occupancy grid downward to find floor
};

struct Params {
  LidarParams lidar;
  ImuParams imu;
  SlamParams slam;
  PlannerParams planner;
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
  // UDP lidar publisher
  struct PublisherParams {
    std::string broadcast_addr =
        "192.168.168.255"; // drone subnet; use 255.255.255.255 for desktop
    int broadcast_port = 8892;
  } publisher;
};

// Load from YAML files. Throws std::runtime_error on missing required fields.
// Fully implemented in Stage 4; stub returns default-constructed Params.
Params load(const std::string &livo2_yaml, const std::string &camera_yaml);

} // namespace params
