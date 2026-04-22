#include "plan_env/grid_map.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>

using namespace std;

static inline double ego_now_s() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void GridMap::initMap(const GridMapParams &gmp) {
  mp_.resolution_ = gmp.resolution_;
  mp_.local_update_range3d_ = gmp.local_update_range3d_;
  mp_.obstacles_inflation_ = gmp.obstacles_inflation_;
  mp_.enable_virtual_walll_ = gmp.enable_virtual_walll_;
  mp_.virtual_ceil_ = gmp.virtual_ceil_;
  mp_.virtual_ground_ = gmp.virtual_ground_;
  mp_.frame_id_ = gmp.frame_id_;
  mp_.p_hit_ = gmp.p_hit_;
  mp_.p_miss_ = gmp.p_miss_;
  mp_.p_min_ = gmp.p_min_;
  mp_.p_max_ = gmp.p_max_;
  mp_.p_occ_ = gmp.p_occ_;
  mp_.fading_time_ = gmp.fading_time_;
  mp_.min_ray_length_ = gmp.min_ray_length_;
  mp_.show_occ_time_ = gmp.show_occ_time_;
  mp_.odom_depth_timeout_ = gmp.odom_depth_timeout_;

  // Inflation grid
  mp_.inf_grid_ =
      (int)ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
  if (mp_.inf_grid_ > 4) {
    mp_.inf_grid_ = 4;
    mp_.resolution_ = mp_.obstacles_inflation_ / mp_.inf_grid_;
    fprintf(stderr,
            "[GridMap] Inflation too big; resolution auto-enlarged to %f.\n",
            mp_.resolution_);
  }

  mp_.resolution_inv_ = 1.0 / mp_.resolution_;
  mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.resolution_inv_)
                                  .array()
                                  .ceil()
                                  .cast<int>();
  mp_.local_update_range3d_ =
      mp_.local_update_range3i_.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_size3i_ = 2 * mp_.local_update_range3i_;
  md_.ringbuffer_inf_size3i_ =
      md_.ringbuffer_size3i_ +
      Eigen::Vector3i(2 * mp_.inf_grid_, 2 * mp_.inf_grid_, 2 * mp_.inf_grid_);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);

  // Allocate buffers
  Eigen::Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
  int buffer_size =
      map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
  int buffer_inf_size = (map_voxel_num3i(0) + 2 * mp_.inf_grid_) *
                        (map_voxel_num3i(1) + 2 * mp_.inf_grid_) *
                        (map_voxel_num3i(2) + 2 * mp_.inf_grid_);

  md_.ringbuffer_origin3i_ = Eigen::Vector3i(0, 0, 0);
  md_.ringbuffer_inf_origin3i_ = Eigen::Vector3i(0, 0, 0);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_);
  md_.occupancy_buffer_inflate_ = vector<uint16_t>(buffer_inf_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);
  md_.cache_voxel_ =
      vector<Eigen::Vector3i>(buffer_size, Eigen::Vector3i(0, 0, 0));

  md_.raycast_num_ = 0;
  md_.proj_points_cnt_ = 0;
  md_.cache_voxel_cnt_ = 0;

  md_.occ_need_update_ = false;
  md_.has_odom_ = false;
  md_.last_occ_update_time_s_ = 0.0;
  md_.flag_have_ever_received_cloud_ = false;
  md_.flag_depth_odom_timeout_ = false;

  mp_.have_initialized_ = true;
  fprintf(stderr,
          "[GridMap] Initialised. resolution=%.3f, range=[%.1f,%.1f,%.1f], "
          "inf=%d\n",
          mp_.resolution_, mp_.local_update_range3d_(0),
          mp_.local_update_range3d_(1), mp_.local_update_range3d_(2),
          mp_.inf_grid_);
}

// ---------------------------------------------------------------------------
// inputPointCloud: replaces cloudCallback() + odomCallback() combo.
// pts are already in world frame. sensor_pos is the LiDAR/camera origin.
// ---------------------------------------------------------------------------
void GridMap::inputPointCloud(const std::vector<Eigen::Vector3d> &pts,
                              const Eigen::Vector3d &sensor_pos) {
  if (pts.empty())
    return;
  if (!std::isfinite(sensor_pos(0)) || !std::isfinite(sensor_pos(1)) ||
      !std::isfinite(sensor_pos(2)))
    return;

  if (!mp_.have_initialized_)
    return;
  if (md_.ringbuffer_lowbound3d_.norm() < 1e-10)
    initMapBoundary(); // lazy first call

  // Update ray origin so raycastProcess() marks free space correctly.
  md_.camera_pos_ = sensor_pos;
  md_.has_odom_ = true;

  // Populate proj_points_ for the raycast pass in updateOccupancy().
  // Previously this function stamped occupied voxels directly to clamp_max,
  // bypassing raycastProcess entirely — meaning free space was never marked
  // and the map only ever grew denser. By feeding proj_points_ instead,
  // raycastProcess traces a ray from sensor_pos through each point, marking
  // all voxels along the ray as free (miss) and the endpoint as occupied.
  if ((int)md_.proj_points_.size() < (int)pts.size())
    md_.proj_points_.resize(pts.size());
  md_.proj_points_cnt_ = 0;
  for (const auto &p3d : pts) {
    if (p3d.array().isNaN().sum())
      continue;
    md_.proj_points_[md_.proj_points_cnt_++] = p3d;
  }

  md_.occ_need_update_ = true;
  md_.flag_have_ever_received_cloud_ = true;
}

// ---------------------------------------------------------------------------
// inputOdom: replaces odomCallback()
// ---------------------------------------------------------------------------
void GridMap::inputOdom(const Eigen::Vector3d &pos,
                        const Eigen::Matrix3d &rot) {
  md_.camera_pos_ = pos;
  md_.camera_r_m_ = rot;
  md_.has_odom_ = true;
}

// ---------------------------------------------------------------------------
// updateOccupancy: replaces updateOccupancyCallback()
// now_s is the current wall-clock time in seconds.
// ---------------------------------------------------------------------------
void GridMap::updateOccupancy(double now_s) {
  if (!md_.has_odom_)
    return;
  if (!mp_.have_initialized_)
    return;

  // Timeout check
  if (md_.last_occ_update_time_s_ < 1.0)
    md_.last_occ_update_time_s_ = now_s;

  if (!md_.occ_need_update_) {
    if (md_.flag_have_ever_received_cloud_ &&
        (now_s - md_.last_occ_update_time_s_) > mp_.odom_depth_timeout_) {
      fprintf(stderr, "[GridMap] odom or cloud lost! timeout=%.2fs\n",
              mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }

  // Update occupancy with raycasting
  double t1 = ego_now_s();

  moveRingBuffer();
  double t2 = ego_now_s();

  // For point-cloud input mode we skip projectDepthImage — cloud already
  // accumulated. We rerun raycast on accumulated md_.proj_points_ if any.
  if (md_.proj_points_cnt_ > 0) {
    raycastProcess();
    double t3 = ego_now_s();

    clearAndInflateLocalMap();
    double t4 = ego_now_s();

    if (mp_.show_occ_time_) {
      printf("[GridMap] moveRingBuf=%.3fms raycast=%.3fms inflate=%.3fms\n",
             (t2 - t1) * 1e3, (t3 - t2) * 1e3, (t4 - t3) * 1e3);
    }

    md_.proj_points_cnt_ = 0;
  }

  md_.occ_need_update_ = false;
  md_.last_occ_update_time_s_ = now_s;
  md_.flag_depth_odom_timeout_ =
      false; // cloud arrived — clear any stale timeout

  // Fire visualisation callback (replaces visCallback 0.125s timer)
  if (on_vis_ && (now_s - last_vis_time_s_) >= 0.125) {
    last_vis_time_s_ = now_s;
    OccVizCloud viz;
    viz.timestamp = now_s;
    collectOccupiedVoxels(viz.occupied);
    on_vis_(viz);
  }
}

// ---------------------------------------------------------------------------
// fadeOccupancy: replaces fadingCallback() — call at ~2 Hz
// ---------------------------------------------------------------------------
void GridMap::fadeOccupancy(double now_s) {
  if (!mp_.have_initialized_)
    return;
  if (mp_.fading_time_ <= 0.0)
    return;
  if ((now_s - last_fade_time_s_) < 0.5)
    return;
  last_fade_time_s_ = now_s;

  const double reduce =
      (mp_.clamp_max_log_ - mp_.min_occupancy_log_) / (mp_.fading_time_ * 2.0);
  const double low_thres = mp_.clamp_min_log_ + reduce;

  for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i) {
    if (md_.occupancy_buffer_[i] > low_thres) {
      bool obs_flag = md_.occupancy_buffer_[i] >= mp_.min_occupancy_log_;
      md_.occupancy_buffer_[i] -= reduce;
      if (obs_flag && md_.occupancy_buffer_[i] < mp_.min_occupancy_log_) {
        Eigen::Vector3i idx = BufIdx2GlobalIdx(i);
        int inf_buf_idx = globalIdx2InfBufIdx(idx);
        changeInfBuf(false, inf_buf_idx, idx);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// collectOccupiedVoxels: harvest inflated-occupied voxel centres for display
// (replaces publishMapInflate headed-direction filter — simpler: full volume)
// ---------------------------------------------------------------------------
void GridMap::collectOccupiedVoxels(std::vector<Eigen::Vector3f> &out) {
  out.clear();
  if (!mp_.have_initialized_)
    return;

  double lbz =
      mp_.enable_virtual_walll_
          ? std::max(md_.ringbuffer_inf_lowbound3d_(2), mp_.virtual_ground_)
          : md_.ringbuffer_inf_lowbound3d_(2);
  double ubz =
      mp_.enable_virtual_walll_
          ? std::min(md_.ringbuffer_inf_upbound3d_(2), mp_.virtual_ceil_)
          : md_.ringbuffer_inf_upbound3d_(2);

  const double res = mp_.resolution_;
  for (double x = md_.ringbuffer_inf_lowbound3d_(0) + res / 2;
       x < md_.ringbuffer_inf_upbound3d_(0); x += res)
    for (double y = md_.ringbuffer_inf_lowbound3d_(1) + res / 2;
         y < md_.ringbuffer_inf_upbound3d_(1); y += res)
      for (double z = lbz + res / 2; z < ubz; z += res) {
        Eigen::Vector3d pos(x, y, z);
        if (!isInInfBuf(pos))
          continue;
        if (md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(
                pos2GlobalIdx(pos))])
          out.emplace_back((float)x, (float)y, (float)z);
      }
}

// ---------------------------------------------------------------------------
// moveRingBuffer
// ---------------------------------------------------------------------------
void GridMap::moveRingBuffer() {
  if (!mp_.have_initialized_)
    initMapBoundary();

  Eigen::Vector3i center_new = pos2GlobalIdx(md_.camera_pos_);
  Eigen::Vector3i lb_new = center_new - mp_.local_update_range3i_;
  Eigen::Vector3d lbd_new = lb_new.cast<double>() * mp_.resolution_;
  Eigen::Vector3i ub_new = center_new + mp_.local_update_range3i_;
  Eigen::Vector3d ubd_new = ub_new.cast<double>() * mp_.resolution_;
  ub_new -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf3d = inf3i.array().cast<double>() * mp_.resolution_;
  Eigen::Vector3i ilb_new = lb_new - inf3i;
  Eigen::Vector3d ilbd_new = lbd_new - inf3d;
  Eigen::Vector3i iub_new = ub_new + inf3i;
  Eigen::Vector3d iubd_new = ubd_new + inf3d;

  if (center_new(0) < md_.center_last3i_(0))
    clearBuffer(0, ub_new(0));
  if (center_new(0) > md_.center_last3i_(0))
    clearBuffer(1, lb_new(0));
  if (center_new(1) < md_.center_last3i_(1))
    clearBuffer(2, ub_new(1));
  if (center_new(1) > md_.center_last3i_(1))
    clearBuffer(3, lb_new(1));
  if (center_new(2) < md_.center_last3i_(2))
    clearBuffer(4, ub_new(2));
  if (center_new(2) > md_.center_last3i_(2))
    clearBuffer(5, lb_new(2));

  for (int i = 0; i < 3; ++i) {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
      md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
    while (md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i))
      md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
    while (md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i))
      md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
    while (md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i))
      md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
  }

  md_.center_last3i_ = center_new;
  md_.ringbuffer_lowbound3i_ = lb_new;
  md_.ringbuffer_lowbound3d_ = lbd_new;
  md_.ringbuffer_upbound3i_ = ub_new;
  md_.ringbuffer_upbound3d_ = ubd_new;
  md_.ringbuffer_inf_lowbound3i_ = ilb_new;
  md_.ringbuffer_inf_lowbound3d_ = ilbd_new;
  md_.ringbuffer_inf_upbound3i_ = iub_new;
  md_.ringbuffer_inf_upbound3d_ = iubd_new;
}

// ---------------------------------------------------------------------------
// raycastProcess
// ---------------------------------------------------------------------------
void GridMap::raycastProcess() {
  md_.cache_voxel_cnt_ = 0;
  md_.raycast_num_ += 1;

  RayCaster raycaster;
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt_; ++i) {
    int vox_idx;
    pt_w = md_.proj_points_[i];

    if (!isInBuf(pt_w)) {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);
      vox_idx = setCacheOccupancy(pt_w, 0);
    } else {
      vox_idx = setCacheOccupancy(pt_w, 1);
    }

    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
        continue;
      else
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
    }

    raycaster.setInput(pt_w / mp_.resolution_,
                       md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp =
          (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * mp_.resolution_;
      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
          break;
        else
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
      }
    }
  }

  for (int i = 0; i < md_.cache_voxel_cnt_; ++i) {
    int idx_ctns = globalIdx2BufIdx(md_.cache_voxel_[i]);

    double log_odds_update =
        md_.count_hit_[idx_ctns] >=
                md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns]
            ? mp_.prob_hit_log_
            : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 &&
        md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
      continue;
    else if (log_odds_update <= 0 &&
             md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
      continue;

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update,
                          mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

// ---------------------------------------------------------------------------
// clearAndInflateLocalMap
// ---------------------------------------------------------------------------
void GridMap::clearAndInflateLocalMap() {
  for (int i = 0; i < md_.cache_voxel_cnt_; ++i) {
    Eigen::Vector3i idx = md_.cache_voxel_[i];
    int buf_id = globalIdx2BufIdx(idx);
    int inf_buf_id = globalIdx2InfBufIdx(idx);

    if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG &&
        md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_) {
      changeInfBuf(true, inf_buf_id, idx);
    }

    if (md_.occupancy_buffer_inflate_[inf_buf_id] >= GRID_MAP_OBS_FLAG &&
        md_.occupancy_buffer_[buf_id] < mp_.min_occupancy_log_) {
      changeInfBuf(false, inf_buf_id, idx);
    }
  }
}

// ---------------------------------------------------------------------------
// initMapBoundary
// ---------------------------------------------------------------------------
void GridMap::initMapBoundary() {
  md_.center_last3i_ = pos2GlobalIdx(md_.camera_pos_);

  md_.ringbuffer_lowbound3i_ = md_.center_last3i_ - mp_.local_update_range3i_;
  md_.ringbuffer_lowbound3d_ =
      md_.ringbuffer_lowbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ = md_.center_last3i_ + mp_.local_update_range3i_;
  md_.ringbuffer_upbound3d_ =
      md_.ringbuffer_upbound3i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound3i_ -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf3d = inf3i.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_inf_lowbound3i_ = md_.ringbuffer_lowbound3i_ - inf3i;
  md_.ringbuffer_inf_lowbound3d_ = md_.ringbuffer_lowbound3d_ - inf3d;
  md_.ringbuffer_inf_upbound3i_ = md_.ringbuffer_upbound3i_ + inf3i;
  md_.ringbuffer_inf_upbound3d_ = md_.ringbuffer_upbound3d_ + inf3d;

  for (int i = 0; i < 3; ++i) {
    while (md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
      md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
    while (md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i))
      md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
    while (md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i))
      md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
    while (md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i))
      md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
  }
}

// ---------------------------------------------------------------------------
// clearBuffer
// ---------------------------------------------------------------------------
void GridMap::clearBuffer(char casein, int bound) {
  for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0));
       x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.ringbuffer_lowbound3i_(1));
         y <= (casein == 3 ? bound : md_.ringbuffer_upbound3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.ringbuffer_lowbound3i_(2));
           z <= (casein == 5 ? bound : md_.ringbuffer_upbound3i_(2)); ++z) {
        Eigen::Vector3i id_global(x, y, z);
        int id_buf = globalIdx2BufIdx(id_global);
        int id_buf_inf = globalIdx2InfBufIdx(id_global);

        md_.count_hit_[id_buf] = 0;
        md_.count_hit_and_miss_[id_buf] = 0;
        md_.flag_traverse_[id_buf] = md_.raycast_num_;
        md_.flag_rayend_[id_buf] = md_.raycast_num_;
        md_.occupancy_buffer_[id_buf] = mp_.clamp_min_log_;

        if (md_.occupancy_buffer_inflate_[id_buf_inf] > GRID_MAP_OBS_FLAG)
          changeInfBuf(false, id_buf_inf, id_global);
      }
}

// ---------------------------------------------------------------------------
// closetPointInMap
// ---------------------------------------------------------------------------
Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt,
                                          const Eigen::Vector3d &camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = md_.ringbuffer_upbound3d_ - camera_pt;
  Eigen::Vector3d min_tc = md_.ringbuffer_lowbound3d_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}
