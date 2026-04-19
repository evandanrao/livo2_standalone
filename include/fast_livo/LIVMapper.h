/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "bridge.hpp"
#include "camera_loader.hpp"
#include "params.h"
#include "preprocess.h"
#include "vio.h"
// Stage 3.5: cv_bridge, image_transport, nav_msgs removed
// Stage 3.8: vikit/camera_loader.h removed — replaced by
// livo::loadCameraFromYaml

class LIVMapper {
public:
  // Stage 3: NodeHandle removed. Bridge provides inter-thread data exchange.
  LIVMapper(livo::Bridge &bridge, const params::Params &p);
  ~LIVMapper();
  void initializeComponents(const params::Params &p);
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();

  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr,
                     V3D angvel_avr);
  void
  imu_prop_iteration(); // Stage 3: replaces imu_prop_callback(ros::TimerEvent)
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                      const PointCloudXYZI::Ptr &input_cloud,
                      PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  // Stage 3: ROS subscriber callbacks replaced by direct push methods
  void push_cloud(const livo::CloudData &cloud);
  void push_imu(const livo::ImuData &imu);
  void push_image(const livo::ImageData &img);
  // Stage 3.4/3.5/3.6: publish_* now write to bridge queues (no ros::Publisher
  // args)
  void publish_img_rgb(VIOManagerPtr vio_manager);
  void publish_frame_world(VIOManagerPtr vio_manager);
  void publish_visual_sub_map();
  void publish_effect_world(const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry();
  void readParameters(const params::Params &p);
  template <typename T> void set_posestamp(T &out);
  template <typename T>
  void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi,
                        Eigen::Matrix<T, 3, 1> &po);
  template <typename T>
  Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  // Stage 3: getImageFromMsg replaced — images arrive as cv::Mat from
  // camera_driver cv::Mat getImageFromMsg(...) removed

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;

  string root_dir;
  string pcd_dir_;   // log_dir + "/" (directory portion, for interval saves)
  string pcd_stem_;  // full path prefix for PCD files, e.g. /media/.../livo2_2026-04-19_15-00-00
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0,
         last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false,
       pub_effect_point_en = false, pose_output_en = false,
       ros_driver_fix_en = false, hilti_en = false;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  // Stage 3.1: imu_buffer is now deque<livo::ImuData>
  bool new_imu = false, state_update_flg = false, imu_prop_enable = true,
       ekf_finish_once = false;
  deque<livo::ImuData> prop_imu_buffer;
  livo::ImuData newest_imu;
  double latest_ekf_time;
  // Stage 3.4: nav_msgs::Odometry imu_prop_odom removed
  // Stage 3: pubImuPropOdom removed
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false,
       ba_bg_est_en = true;
  bool dense_map_en = false;
  bool verbose_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  // Stage 3.1: imu_buffer changed from deque<sensor_msgs::Imu::ConstPtr> to
  // deque<livo::ImuData>
  deque<livo::ImuData> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  // Global map accumulation for /livo2/global_map
  PointCloudXYZI::Ptr global_map_accum_;  // raw accumulated buffer
  double global_map_filter_size_ = 0.3;   // voxel leaf size (metres)
  double global_map_publish_interval_ = 5.0; // seconds between publishes
  double global_map_last_pub_time_ = 0.0;
  bool global_map_en_ = false;

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup state_propagat;

  // Stage 3.4: nav_msgs::Path path, nav_msgs::Odometry odomAftMapped removed
  // Stage 3.4: geometry_msgs::Quaternion geoQuat, geometry_msgs::PoseStamped
  // msg_body_pose removed Stage 3.4: pose is published as livo::PoseData to
  // bridge->pose_queue
  Eigen::Quaterniond state_quat_; // cached quaternion from latest EKF state

  livo::Bridge *bridge_; // set in constructor; used by publish_* methods

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;
  std::shared_ptr<vk::AbstractCamera>
      cam_holder_; // owns lifetime; cam raw ptr set in initializeComponents

  // Stage 3: all ros::Publisher, ros::Subscriber, ros::Timer,
  // image_transport::Publisher removed Stage 5: Foxglove channels will be added
  // here

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif