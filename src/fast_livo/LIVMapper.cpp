/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <thread>

LIVMapper::LIVMapper(livo::Bridge &bridge, const params::Params &p)
    : bridge_(&bridge), extT(0, 0, 0), extR(M3D::Identity()) {
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(p);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(p, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents(p);
  // Stage 3.4: path.header.stamp = ros::Time::now() removed
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(const params::Params &p) {
  // Stage 3.4: all nh.param<>() calls replaced by p.slam.* / p.lidar.* / etc.
  // Values are read from YAML in Stage 4. For now fall through to defaults.
  img_en = p.slam.img_en;
  lidar_en = p.slam.lidar_en;
  imu_en = p.slam.imu_en;
  imu_prop_enable = p.slam.imu_prop_enable;
  gravity_align_en = p.slam.gravity_align_en;
  gravity_est_en = p.slam.gravity_est_en;
  ba_bg_est_en = p.slam.ba_bg_est_en;
  normal_en = p.slam.normal_en;
  inverse_composition_en = p.slam.inverse_composition_en;
  max_iterations = p.slam.max_iterations;
  IMG_POINT_COV = p.slam.img_point_cov;
  raycast_en = p.slam.raycast_en;
  exposure_estimate_en = p.slam.exposure_estimate_en;
  inv_expo_cov = p.slam.inv_expo_cov;
  grid_size = p.slam.grid_size;
  grid_n_height = p.slam.grid_n_height;
  patch_pyrimid_level = p.slam.patch_pyrimid_level;
  patch_size = p.slam.patch_size;
  outlier_threshold = p.slam.outlier_threshold;
  exposure_time_init = p.slam.exposure_time_init;
  img_time_offset = p.slam.img_time_offset;
  imu_time_offset = p.imu.time_offset;
  lidar_time_offset = p.lidar.time_offset;
  seq_name = p.slam.seq_name;
  pose_output_en = p.slam.pose_output_en;
  gyr_cov = p.imu.gyr_cov;
  acc_cov = p.imu.acc_cov;
  imu_int_frame = p.imu.imu_int_frame;
  p_pre->blind = p.lidar.blind;
  p_pre->N_SCANS = p.lidar.n_scans;
  p_pre->point_filter_num = p.slam.point_filter_num;
  p_pre->feature_enabled = p.slam.feature_extract_enabled;
  filter_size_surf_min = p.slam.filter_size_surf;
  hilti_en = p.slam.hilti_en;
  pcd_save_interval = p.slam.pcd_save_interval;
  pcd_save_en = p.slam.pcd_save_en;
  pcd_save_type = p.slam.pcd_save_type;
  img_save_en = p.slam.img_save_en;
  img_save_interval = p.slam.img_save_interval;
  colmap_output_en = p.slam.colmap_output_en;
  filter_size_pcd = p.slam.filter_size_pcd;
  extrinT = p.slam.extrin_T;
  extrinR = p.slam.extrin_R;
  cameraextrinT = p.slam.camera_extrin_T;
  cameraextrinR = p.slam.camera_extrin_R;
  plot_time = p.slam.plot_time;
  frame_cnt = p.slam.frame_cnt;
  blind_rgb_points = p.slam.blind_rgb_points;
  pub_scan_num = p.slam.pub_scan_num;
  pub_effect_point_en = p.slam.pub_effect_point_en;
  dense_map_en = p.slam.dense_map_en;
  verbose_en = p.slam.verbose_en;
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
  // ros_driver_fix_en not applicable (standalone has no ROS driver bug)
  ros_driver_fix_en = false;
}

void LIVMapper::initializeComponents(const params::Params &p) {
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);
  voxelmap_manager->verbose_en = verbose_en;

  // Stage 3.5: replaced vk::camera_loader::loadFromNh (ROS) with
  // livo::loadCameraFromYaml
  if (img_en && !p.slam.camera_yaml.empty()) {
    cam_holder_ = livo::loadCameraFromYaml(p.slam.camera_yaml);
    if (!cam_holder_)
      throw std::runtime_error("Camera model not correctly specified.");
    vio_manager->cam = cam_holder_.get();
  }

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  if (img_en)
    vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(p.imu.bg_cov, p.imu.bg_cov, p.imu.bg_cov));
  p_imu->set_acc_bias_cov(V3D(p.imu.ba_cov, p.imu.ba_cov, p.imu.ba_cov));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en)
    p_imu->disable_imu();
  if (!gravity_est_en)
    p_imu->disable_gravity_est();
  if (!ba_bg_est_en)
    p_imu->disable_bias_est();
  if (!exposure_estimate_en)
    p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() {
  if (pcd_save_en && colmap_output_en) {
    const std::string folderPath =
        std::string(ROOT_DIR) + "/scripts/colmap_output.sh";

    std::string chmodCommand = "chmod +x " + folderPath;

    int chmodRet = system(chmodCommand.c_str());
    if (chmodRet != 0) {
      std::cerr << "Failed to set execute permissions for the script."
                << std::endl;
      return;
    }

    int executionRet = system(folderPath.c_str());
    if (executionRet != 0) {
      std::cerr << "Failed to execute the script." << std::endl;
      return;
    }
  }
  if (colmap_output_en)
    fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt",
                     std::ios::out);
  if (pcd_save_en)
    fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt",
                        std::ios::out);
  if (img_save_en)
    fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt",
                         std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::handleFirstFrame() {
  if (!is_first_frame) {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() {
  if (!p_imu->imu_need_init && !gravity_align_finished) {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() {
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en)
    gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() <<
  // std::endl; std::cout << "[ Mapping ] predict cov: " <<
  // _state.cov.diagonal().transpose() << std::endl; std::cout << "[ Mapping ]
  // predict sta: " << state_propagat.pos_end.transpose() <<
  // state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() {
  switch (LidarMeasures.lio_vio_flg) {
  case VIO:
    handleVIO();
    break;
  case LIO:
  case LO:
    handleLIO();
    break;
  }
}

void LIVMapper::handleVIO() {
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20)
           << LidarMeasures.last_lio_update_time - _first_lidar_time << " "
           << euler_cur.transpose() * 57.3 << " " << _state.pos_end.transpose()
           << " " << _state.vel_end.transpose() << " "
           << _state.bias_g.transpose() << " " << _state.bias_a.transpose()
           << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;

  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }

  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size()
            << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) -
           plot_time) < (frame_cnt / 2 * 0.1)) {
    vio_manager->plot_flag = true;
  } else {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(
      LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_,
      LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++)
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(vio_manager);
  publish_img_rgb(vio_manager);
  publish_visual_sub_map(); // no-op when visual_sub_map is empty (img_en=0)

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20)
           << LidarMeasures.last_lio_update_time - _first_lidar_time << " "
           << euler_cur.transpose() * 57.3 << " " << _state.pos_end.transpose()
           << " " << _state.vel_end.transpose() << " "
           << _state.bias_g.transpose() << " " << _state.bias_a.transpose()
           << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " "
           << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() {
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time
           << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose()
           << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " "
           << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;

  if (feats_undistort->empty() || (feats_undistort == nullptr)) {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);

  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body,
                 feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;

  if (!lidar_map_inited) {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt",
                   std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open())
        fprintf(stderr, "[LIVMapper] open evo file failed\n");
    } else {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt",
                   std::ios::app);
      if (!evoFile.is_open())
        fprintf(stderr, "[LIVMapper] open evo file failed\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0]
            << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
            << std::endl;
  }

  // Stage 3.4: geoQuat = tf::createQuaternionMsgFromRollPitchYaw removed
  // state_quat_ is set inside publish_odometry
  publish_odometry();

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x,
        world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) *
              (-point_crossmat).transpose() +
          _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  if (verbose_en)
    std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;

  double t4 = omp_get_wtime();

  if (voxelmap_manager->config_setting_.map_sliding_en) {
    voxelmap_manager->mapSliding();
  }

  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort
                                                     : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                        &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  // In LIVO mode, VIO handler owns publication to avoid publishing empty clouds
  // on LIO cycles
  if (slam_mode_ != LIVO)
    publish_frame_world(vio_manager);
  if (pub_effect_point_en)
    publish_effect_world(voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) {
    // pubVoxelMap() is throttled internally (~2 Hz). Push result to bridge
    // so the streamer can send it on /livo2/voxel_map.
    PointCloudXYZI::Ptr plane_cloud = voxelmap_manager->pubVoxelMap();
    if (plane_cloud) {
      livo::CloudData cd;
      cd.timestamp = LidarMeasures.last_lio_update_time;
      cd.cloud     = plane_cloud;
      std::lock_guard<std::mutex> lk(bridge_->voxel_map_mtx);
      if (bridge_->voxel_map_queue.size() < livo::kVoxelMapQueueMax)
        bridge_->voxel_map_queue.push(std::move(cd));
    }
  }
  // Stage 3.7: publish_path / publish_mavros removed (Stage 5 replaces with
  // Foxglove)

  frame_num++;
  aver_time_consu =
      aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) /
  // frame_num; aver_time_map_inre = aver_time_map_inre * (frame_num - 1) /
  // frame_num + (t4 - t3) / frame_num; aver_time_solve = aver_time_solve *
  // (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) /
  // frame_num + solve_const_H_time / frame_num; printf("[ mapping time ]: per
  // scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP:
  // %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total:
  //         %0.6f \n", t_prop - t0, t1 - t_prop, match_time, solve_time, t3 -
  //         t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time,
  //         aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map
  // incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map
  //         incre: %0.6f secs, total: %0.6f secs.\033[0m\n", t2 - t1, t4 - t3,
  //         t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  if (verbose_en) {
    printf("\033[1;34m+--------------------------------------------------------"
           "-----+\033[0m\n");
    printf("\033[1;34m|                         LIO Mapping Time               "
           "     |\033[0m\n");
    printf("\033[1;34m+--------------------------------------------------------"
           "-----+\033[0m\n");
    printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage",
           "Time (secs)");
    printf("\033[1;34m+--------------------------------------------------------"
           "-----+\033[0m\n");
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
    printf("\033[1;34m+--------------------------------------------------------"
           "-----+\033[0m\n");
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time",
           t4 - t0);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time",
           aver_time_consu);
    printf("\033[1;34m+--------------------------------------------------------"
           "-----+\033[0m\n");
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20)
           << LidarMeasures.last_lio_update_time - _first_lidar_time << " "
           << euler_cur.transpose() * 57.3 << " " << _state.pos_end.transpose()
           << " " << _state.vel_end.transpose() << " "
           << _state.bias_g.transpose() << " " << _state.bias_a.transpose()
           << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " "
           << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() {
  if (pcd_save_en &&
      (pcl_wait_save->points.size() > 0 ||
       pcl_wait_save_intensity->points.size() > 0) &&
      pcd_save_interval < 0) {
    std::string raw_points_dir =
        std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir =
        std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd,
                               filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);

      pcd_writer.writeBinary(raw_points_dir,
                             *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: " << pcl_wait_save->points.size()
                << RESET << std::endl;

      pcd_writer.writeBinary(
          downsampled_points_dir,
          *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: "
                << downsampled_points_dir
                << " with point count after filtering: "
                << downsampled_cloud->points.size() << RESET << std::endl;

      if (colmap_output_en) {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) {
          const auto &point = downsampled_cloud->points[i];
          fout_points << i << " " << std::fixed << std::setprecision(6)
                      << point.x << " " << point.y << " " << point.z << " "
                      << static_cast<int>(point.r) << " "
                      << static_cast<int>(point.g) << " "
                      << static_cast<int>(point.b) << " " << 0 << std::endl;
        }
      }
    } else {
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: "
                << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run() {
  // Stage 3.6: ros::Rate / ros::ok() / ros::spinOnce() replaced
  // Spin at 5000 Hz (200 µs per iteration) — same cadence as original
  // ros::Rate(5000)
  static constexpr std::chrono::microseconds kLoopPeriod{200};
  while (bridge_->running.load()) {
    // Drain bridge queues into internal buffers (replaces ros::spinOnce())
    {
      std::lock_guard<std::mutex> lk(bridge_->cloud_mtx);
      while (!bridge_->cloud_queue.empty()) {
        push_cloud(bridge_->cloud_queue.front());
        bridge_->cloud_queue.pop();
      }
    }
    {
      std::lock_guard<std::mutex> lk(bridge_->imu_mtx);
      while (!bridge_->imu_queue.empty()) {
        push_imu(bridge_->imu_queue.front());
        bridge_->imu_queue.pop();
      }
    }
    {
      std::lock_guard<std::mutex> lk(bridge_->img_mtx);
      while (!bridge_->img_queue.empty()) {
        push_image(bridge_->img_queue.front());
        bridge_->img_queue.pop();
      }
    }

    if (!sync_packages(LidarMeasures)) {
      std::this_thread::sleep_for(kLoopPeriod);
      continue;
    }
    handleFirstFrame();
    processImu();
    stateEstimationAndMapping();
  }
  try {
    savePCD();
  } catch (const std::exception &e) {
    fprintf(stderr, "[LIVMapper] savePCD failed: %s\n", e.what());
  }
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt,
                              V3D acc_avr, V3D angvel_avr) {
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr +
                V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1],
                    imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end +
                           imu_prop_state.vel_end * dt +
                           0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_iteration() {
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) {
    return;
  }
  mtx_buffer_imu_prop.lock();
  new_imu = false;
  if (imu_prop_enable && !prop_imu_buffer.empty()) {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg) {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() &&
              prop_imu_buffer.front().timestamp < latest_ekf_time)) {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (size_t i = 0; i < prop_imu_buffer.size(); i++) {
        double t_from_lidar_end_time =
            prop_imu_buffer[i].timestamp - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        V3D acc_imu(prop_imu_buffer[i].accel.x(), prop_imu_buffer[i].accel.y(),
                    prop_imu_buffer[i].accel.z());
        V3D omg_imu(prop_imu_buffer[i].gyro.x(), prop_imu_buffer[i].gyro.y(),
                    prop_imu_buffer[i].gyro.z());
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    } else {
      V3D acc_imu(newest_imu.accel.x(), newest_imu.accel.y(),
                  newest_imu.accel.z());
      V3D omg_imu(newest_imu.gyro.x(), newest_imu.gyro.y(),
                  newest_imu.gyro.z());
      double t_from_lidar_end_time = newest_imu.timestamp - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    // Push the propagated pose to the high-rate odom queue.
    // This is separate from pose_queue (~10 Hz LIO) and gives ~200 Hz smooth
    // odometry on the /livo2/imu_odom Foxglove channel.
    livo::PoseData pd;
    pd.timestamp = newest_imu.timestamp;
    pd.rotation  = imu_propagate.rot_end;
    pd.position  = imu_propagate.pos_end;
    pd.velocity  = imu_propagate.vel_end;
    {
      std::lock_guard<std::mutex> lk(bridge_->imu_prop_mtx);
      if (bridge_->imu_prop_queue.size() < livo::kImuPropQueueMax)
        bridge_->imu_prop_queue.push(pd);
    }
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot,
                               const Eigen::Vector3d t,
                               const PointCloudXYZI::Ptr &input_cloud,
                               PointCloudXYZI::Ptr &trans_cloud) {
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++) {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po) {
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T>
void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi,
                                 Matrix<T, 3, 1> &po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T>
Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi) {
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi,
                                    PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi,
                                       PointType *const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void LIVMapper::push_cloud(const livo::CloudData &cloud) {
  // Stage 3.6: replaces standard_pcl_cbk(sensor_msgs::PointCloud2::ConstPtr).
  // lidar_driver.cpp has already done ring filter, blind filter, curvature
  // encoding, sort. preprocess() applies point_filter_num decimation.
  if (!lidar_en)
    return;
  mtx_buffer.lock();

  double cur_head_time = cloud.timestamp + lidar_time_offset;
  if (cur_head_time < last_timestamp_lidar) {
    fprintf(stderr, "[LIVMapper] lidar loop back, clear buffer\n");
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
  }
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(cloud.cloud, ptr);
  // Drop oldest frame if buffer is deep to prevent lag building up
  while (lid_raw_data_buffer.size() >= 3) {
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
  }
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::push_imu(const livo::ImuData &imu_in) {
  // Stage 3.6: replaces imu_cbk(sensor_msgs::Imu::ConstPtr).
  // IMU data arrives already in SI units (m/s², rad/s) from lidar_driver.cpp.
  if (!imu_en)
    return;

  if (last_timestamp_lidar < 0.0)
    return;

  double timestamp = imu_in.timestamp + imu_time_offset;

  if (fabs(last_timestamp_lidar - timestamp) > 0.5) {
    fprintf(stderr, "[LIVMapper] IMU and LiDAR not synced! delta: %lf\n",
            last_timestamp_lidar - timestamp);
  }

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu) {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    fprintf(stderr, "[LIVMapper] imu loop back, offset: %lf\n",
            last_timestamp_imu - timestamp);
    return;
  }

  last_timestamp_imu = timestamp;

  livo::ImuData adjusted = imu_in;
  adjusted.timestamp = timestamp;
  imu_buffer.push_back(adjusted);

  mtx_buffer.unlock();
  if (imu_prop_enable) {
    mtx_buffer_imu_prop.lock();
    if (!p_imu->imu_need_init) {
      prop_imu_buffer.push_back(adjusted);
    }
    newest_imu = adjusted;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

void LIVMapper::push_image(const livo::ImageData &img_in) {
  // Stage 3.6: replaces img_cbk(sensor_msgs::ImageConstPtr).
  // Image arrives as cv::Mat from camera_driver.
  if (!img_en)
    return;

  if (hilti_en) {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0)
      return;
  }

  double msg_header_time = img_in.timestamp + img_time_offset;
  if (fabs(msg_header_time - last_timestamp_img) < 0.001)
    return;
  if (last_timestamp_lidar < 0)
    return;

  if (msg_header_time < last_timestamp_img) {
    fprintf(stderr, "[LIVMapper] image loop back.\n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time;

  if (img_time_correct - last_timestamp_img < 0.02) {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  img_buffer.push_back(img_in.image.clone());
  img_time_buffer.push_back(img_time_correct);

  last_timestamp_img = img_time_correct;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas) {
  if (lid_raw_data_buffer.empty() && lidar_en)
    return false;
  if (img_buffer.empty() && img_en)
    return false;
  if (imu_buffer.empty() && imu_en)
    return false;

  switch (slam_mode_) {
  case ONLY_LIO: {
    if (meas.last_lio_update_time < 0.0)
      meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed) {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1)
        return false;

      meas.lidar_frame_beg_time =
          lid_header_time_buffer.front(); // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time +
                                  meas.lidar->points.back().curvature /
                                      double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true; // flag
    }

    if (imu_en &&
        last_timestamp_imu <
            meas.lidar_frame_end_time) { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty()) {
      if (imu_buffer.front().timestamp > meas.lidar_frame_end_time)
        break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg =
        LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO: {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg) {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO: {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0)
        meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time =
          lid_header_time_buffer.back() +
          lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = imu_buffer.back().timestamp;

      if (img_capture_time < meas.last_lio_update_time + 0.00001) {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        fprintf(stderr, "[LIVMapper] Throw one image frame!\n");
        return false;
      }

      if (img_capture_time > lid_newest_time ||
          img_capture_time > imu_newest_time) {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty()) {
        if (imu_buffer.front().timestamp > m.lio_time)
          break;

        if (imu_buffer.front().timestamp > meas.last_lio_update_time)
          m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // (imu_buffer.front().timestamp)
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty()) {
        if (lid_header_time_buffer.front() > img_capture_time)
          break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++) {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms) {
            pt.curvature +=
                (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          } else {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO: {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = imu_buffer.front().timestamp;

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = imu_buffer.front().timestamp;
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   (imu_buffer.front().timestamp)
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed =
          false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default: {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO: {
    if (!lidar_pushed) {
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())
        return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time =
          lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time +
                                  meas.lidar->points.back().curvature /
                                      double(1000); // calc lidar scan end time
      lidar_pushed = true;
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg =
        LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default: {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  fprintf(stderr, "[LIVMapper] sync_packages: unexpected state\n");
}

void LIVMapper::publish_img_rgb(VIOManagerPtr vio_manager) {
  // Stage 3.7: cv_bridge / image_transport removed.
  // Push to bridge viz queue for Stage 5 Foxglove streaming.
  cv::Mat img_rgb = vio_manager->img_cp;
  if (img_rgb.empty())
    return;
  livo::ImageData out;
  out.timestamp = LidarMeasures.last_lio_update_time;
  out.image = img_rgb.clone();
  std::lock_guard<std::mutex> lk(bridge_->viz_img_mtx);
  if (bridge_->viz_img_queue.size() < livo::kVizImgQueueMax)
    bridge_->viz_img_queue.push(std::move(out));
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(VIOManagerPtr vio_manager) {
  if (pcl_w_wait_pub->empty())
    return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO) {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if (pub_num >= pub_scan_num) {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++) {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y,
                pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w));
        if (pf[2] < 0)
          continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo;
          // pointRGB.b = pixel[0] * inv_expo; if (pointRGB.r > 255) pointRGB.r
          // = 255; else if (pointRGB.r < 0) pointRGB.r = 0; if (pointRGB.g >
          // 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0; if
          // (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0)
          // pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points)
            laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  // Push RGB-coloured cloud when LIVO+VIO produced colour for this frame.
  // Only published when img_en=1 and pub_scan_num accumulation is complete.
  // Streamer sends this on /livo2/map_cloud_rgb.
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO &&
      !laserCloudWorldRGB->empty()) {
    livo::RgbCloudData rcd;
    rcd.timestamp = LidarMeasures.last_lio_update_time;
    rcd.cloud     = laserCloudWorldRGB; // moves ownership of this-frame alloc
    std::lock_guard<std::mutex> lk(bridge_->viz_rgb_cloud_mtx);
    if (bridge_->viz_rgb_cloud_queue.size() < livo::kVizRgbCloudQueueMax)
      bridge_->viz_rgb_cloud_queue.push(std::move(rcd));
  }

  // Always push an XYZI copy for the existing /livo2/map_cloud channel.
  // This keeps the basic map visible in all modes (LO / LIO / LIVO).
  {
    PointCloudXYZI::Ptr outCloud(new PointCloudXYZI(*pcl_w_wait_pub));
    livo::CloudData cd;
    cd.timestamp = LidarMeasures.last_lio_update_time;
    cd.cloud     = outCloud;
    std::lock_guard<std::mutex> lk(bridge_->viz_cloud_mtx);
    if (bridge_->viz_cloud_queue.size() < livo::kVizCloudQueueMax)
      bridge_->viz_cloud_queue.push(std::move(cd));
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en) {
    static int scan_wait_num = 0;

    switch (pcd_save_type) {
    case 0: /** world frame **/
      if (slam_mode_ == LIVO) {
        *pcl_wait_save += *laserCloudWorldRGB;
      } else {
        *pcl_wait_save_intensity += *pcl_w_wait_pub;
      }
      if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        scan_wait_num++;
      break;

    case 1: /** body frame **/
      if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++) {
          RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                                 &laserCloudBody->points[i]);
        }
        *pcl_wait_save_intensity += *laserCloudBody;
        scan_wait_num++;
        cout << "save body frame points: "
             << pcl_wait_save_intensity->points.size() << endl;
      }
      pcd_save_interval = 1;

      break;

    default:
      pcd_save_interval = 1;
      scan_wait_num++;
      break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) &&
        pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") +
                            ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0) {
        pcd_writer.writeBinary(
            all_points_dir,
            *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir,
                             // *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if (pcl_wait_save_intensity->points.size() > 0) {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }

    if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos << LidarMeasures.measures.back().lio_time << " "
                     << _state.pos_end[0] << " " << _state.pos_end[1] << " "
                     << _state.pos_end[2] << " " << q.x() << " " << q.y() << " "
                     << q.z() << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO) {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval) {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() +
                  string(".png"),
              vio_manager->img_rgb);

      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " "
                      << _state.pos_end[0] << " " << _state.pos_end[1] << " "
                      << _state.pos_end[2] << " " << q.x() << " " << q.y()
                      << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if (LidarMeasures.lio_vio_flg == VIO)
    PointCloudXYZI().swap(*pcl_wait_pub);
  else if (laserCloudWorldRGB->size() > 0)
    PointCloudXYZI().swap(*pcl_wait_pub);
  if (LidarMeasures.lio_vio_flg == VIO)
    PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map() {
  // Publishes the visual tracking sub-map: 3D positions of VIO map points
  // visible in the current frame.  Streamer sends on /livo2/visual_sub_map.
  //
  // Note: visual_sub_map is populated from vio_manager->visual_sub_map_cur
  // in the VIO handler (currently commented out pending camera availability).
  // When img_en=1 and that block is uncommented this function will stream live.
  if (!visual_sub_map || visual_sub_map->empty()) return;

  livo::CloudData cd;
  cd.timestamp = LidarMeasures.last_lio_update_time;
  cd.cloud     = visual_sub_map;
  std::lock_guard<std::mutex> lk(bridge_->visual_map_mtx);
  if (bridge_->visual_map_queue.size() < livo::kVisualMapQueueMax)
    bridge_->visual_map_queue.push(std::move(cd));
}

void LIVMapper::publish_effect_world(
    const std::vector<PointToPlane> &ptpl_list) {
  if (ptpl_list.empty()) return;

  // Build a cloud of matched LiDAR-to-plane correspondence points (world frame).
  // Each point is a scan point that successfully matched a voxel plane in this
  // EKF update step.  Streamer sends on /livo2/effect_points.
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  cloud->reserve(ptpl_list.size());
  for (const PointToPlane &pp : ptpl_list) {
    pcl::PointXYZINormal pt;
    pt.x         = static_cast<float>(pp.point_w_[0]);
    pt.y         = static_cast<float>(pp.point_w_[1]);
    pt.z         = static_cast<float>(pp.point_w_[2]);
    pt.intensity = 1.0f;
    cloud->push_back(pt);
  }

  livo::CloudData cd;
  cd.timestamp = LidarMeasures.last_lio_update_time;
  cd.cloud     = cloud;
  std::lock_guard<std::mutex> lk(bridge_->effect_cloud_mtx);
  if (bridge_->effect_cloud_queue.size() < livo::kEffectCloudQueueMax)
    bridge_->effect_cloud_queue.push(std::move(cd));
}

template <typename T> void LIVMapper::set_posestamp(T &out) {
  // Stage 3.7: geoQuat removed; compute from state directly.
  state_quat_ = Eigen::Quaterniond(_state.rot_end);
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = state_quat_.x();
  out.orientation.y = state_quat_.y();
  out.orientation.z = state_quat_.z();
  out.orientation.w = state_quat_.w();
}

void LIVMapper::publish_odometry() {
  // Stage 3.7: nav_msgs::Odometry + TF broadcaster removed.
  // Push pose to bridge->pose_queue for Stage 5 Foxglove / MCAP.
  state_quat_ = Eigen::Quaterniond(_state.rot_end);
  livo::PoseData pd;
  pd.timestamp = LidarMeasures.last_lio_update_time;
  pd.rotation = _state.rot_end;
  pd.position = _state.pos_end;
  pd.velocity = _state.vel_end;
  std::lock_guard<std::mutex> lk(bridge_->pose_mtx);
  if (bridge_->pose_queue.size() < 100)
    bridge_->pose_queue.push(pd);
}

// Stage 3.7: publish_mavros and publish_path removed (Stage 5 replaces with
// Foxglove).