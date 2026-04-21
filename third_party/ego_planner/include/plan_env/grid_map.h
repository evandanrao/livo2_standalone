#ifndef _GRID_MAP_
#define _GRID_MAP_

// De-ROS'd GridMap for livo2_standalone.
// Input data is pushed directly via inputPointCloud() / inputOdom().
// updateOccupancy() must be called periodically from the planner thread loop.
// Visualisation is delivered via the on_vis_ callback (optional).

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <queue>
#include <random>
#include <tuple>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767
#define GRID_MAP_NEW_PLATFORM_TEST false

using namespace std;

// ---- Parameter struct (replaces ros::NodeHandle param reads) ----------------
struct GridMapParams {
  bool have_initialized_ = false;

  double resolution_ = 0.1;
  Eigen::Vector3d local_update_range3d_ = Eigen::Vector3d(5.5, 5.5, 3.0);
  double obstacles_inflation_ = 0.3;
  bool enable_virtual_walll_ = false;
  double virtual_ceil_ = 1.0;
  double virtual_ground_ = -0.1;
  std::string frame_id_ = "world";

  double p_hit_ = 0.70;
  double p_miss_ = 0.35;
  double p_min_ = 0.12;
  double p_max_ = 0.97;
  double p_occ_ = 0.80;
  double fading_time_ = 1000.0;
  double min_ray_length_ = 0.1;

  bool show_occ_time_ = false;
  double odom_depth_timeout_ = 1.0;
};

// ---- Internal parameter cache (computed from GridMapParams) -----------------
struct MappingParameters {
  bool have_initialized_ = false;

  Eigen::Vector3d local_update_range3d_;
  Eigen::Vector3i local_update_range3i_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  int inf_grid_;
  std::string frame_id_;

  bool enable_virtual_walll_;
  double virtual_ceil_, virtual_ground_;

  double odom_depth_timeout_;

  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;
  double min_ray_length_;
  double fading_time_;

  bool show_occ_time_;
};

// ---- Internal mapping data --------------------------------------------------
struct MappingData {
  Eigen::Vector3i center_last3i_;
  Eigen::Vector3i ringbuffer_origin3i_;
  Eigen::Vector3i ringbuffer_lowbound3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_;
  Eigen::Vector3i ringbuffer_upbound3i_;
  Eigen::Vector3d ringbuffer_upbound3d_;
  Eigen::Vector3i ringbuffer_size3i_;
  Eigen::Vector3i ringbuffer_inf_origin3i_;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_;
  Eigen::Vector3d ringbuffer_inf_upbound3d_;
  Eigen::Vector3i ringbuffer_inf_upbound3i_;
  Eigen::Vector3i ringbuffer_inf_size3i_;

  std::vector<double> occupancy_buffer_;
  std::vector<uint16_t> occupancy_buffer_inflate_;

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;

  // Lifecycle flags
  bool occ_need_update_, local_updated_;
  bool has_odom_;
  double last_occ_update_time_s_;
  bool flag_depth_odom_timeout_;
  bool flag_have_ever_received_cloud_;

  // Projected point cloud buffer
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt_;

  // Raycasting buffers
  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;

  vector<Eigen::Vector3i> cache_voxel_;
  int cache_voxel_cnt_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ---- Visualisation payload --------------------------------------------------
struct OccVizCloud {
  double timestamp;                      // seconds
  std::vector<Eigen::Vector3f> occupied; // inflated occupied voxel centres
};

// ---- GridMap ----------------------------------------------------------------
class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  // Initialise from parameter struct (replaces initMap(ros::NodeHandle&)).
  void initMap(const GridMapParams &gmp);

  // Push a point cloud (in world frame) with the sensor origin.
  // Replaces cloudCallback + odomCallback in the ROS version.
  void inputPointCloud(const std::vector<Eigen::Vector3d> &pts,
                       const Eigen::Vector3d &sensor_pos);

  // Push SLAM odometry (world-frame position + rotation).
  void inputOdom(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot);

  // Run one occupancy update cycle (replaces 32ms occ_timer_ callback).
  void updateOccupancy(double now_s);

  // Run one fading cycle (replaces 0.5s fading_timer_ callback).
  void fadeOccupancy(double now_s);

  // Collect all inflated-occupied voxel centres for visualisation.
  void collectOccupiedVoxels(std::vector<Eigen::Vector3f> &out);

  // Optional visualisation callback — called at end of updateOccupancy().
  std::function<void(const OccVizCloud &)> on_vis_;

  // Query interface (used by AStar / PolyTrajOptimizer)
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getInflateOccupancy(Eigen::Vector3d pos);
  inline double getResolution();
  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  enum { INVALID_IDX = -10000 };

  // Ring-buffer indexing (same as original)
  inline Eigen::Vector3d globalIdx2Pos(const Eigen::Vector3i &id);
  inline Eigen::Vector3i pos2GlobalIdx(const Eigen::Vector3d &pos);
  inline int globalIdx2BufIdx(const Eigen::Vector3i &id);
  inline int globalIdx2InfBufIdx(const Eigen::Vector3i &id);
  inline Eigen::Vector3i BufIdx2GlobalIdx(size_t address);
  inline Eigen::Vector3i infBufIdx2GlobalIdx(size_t address);
  inline bool isInBuf(const Eigen::Vector3d &pos);
  inline bool isInBuf(const Eigen::Vector3i &idx);
  inline bool isInInfBuf(const Eigen::Vector3d &pos);
  inline bool isInInfBuf(const Eigen::Vector3i &idx);

  // Core algorithm steps (same math as original)
  void moveRingBuffer();
  void raycastProcess();
  void clearAndInflateLocalMap();
  void initMapBoundary();
  void clearBuffer(char casein, int bound);

  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt,
                                   const Eigen::Vector3d &camera_pt);

  inline void changeInfBuf(const bool dir, const int inf_buf_idx,
                           const Eigen::Vector3i global_idx);
  inline int setCacheOccupancy(Eigen::Vector3d pos, int occ);

  // Timing for timeout check
  double last_vis_time_s_ = 0.0;
  double last_fade_time_s_ = 0.0;
};

/* ============================== inline implementations =====================
 */

inline int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id = pos2GlobalIdx(pos);
  int idx_ctns = globalIdx2BufIdx(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_[md_.cache_voxel_cnt_++] = id;
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

inline void GridMap::changeInfBuf(const bool dir, const int inf_buf_idx,
                                  const Eigen::Vector3i global_idx) {
  int inf_grid = mp_.inf_grid_;
  if (dir)
    md_.occupancy_buffer_inflate_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
  else
    md_.occupancy_buffer_inflate_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;

  for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
    for (int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
      for (int z_inf = -inf_grid; z_inf <= inf_grid; ++z_inf) {
        Eigen::Vector3i id_inf(global_idx +
                               Eigen::Vector3i(x_inf, y_inf, z_inf));
        int id_inf_buf = globalIdx2InfBufIdx(id_inf);
        if (dir)
          ++md_.occupancy_buffer_inflate_[id_inf_buf];
        else {
          --md_.occupancy_buffer_inflate_[id_inf_buf];
          if (md_.occupancy_buffer_inflate_[id_inf_buf] > 65000) {
            fprintf(
                stderr,
                "[GridMap] Negative nearby-obstacle count – resetting map.\n");
            fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(),
                 mp_.clamp_min_log_);
            fill(md_.occupancy_buffer_inflate_.begin(),
                 md_.occupancy_buffer_inflate_.end(), 0);
          }
        }
      }
}

inline int GridMap::globalIdx2BufIdx(const Eigen::Vector3i &id) {
  int x_buffer =
      (id(0) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(0);
  int y_buffer =
      (id(1) - md_.ringbuffer_origin3i_(1)) % md_.ringbuffer_size3i_(1);
  int z_buffer =
      (id(2) - md_.ringbuffer_origin3i_(2)) % md_.ringbuffer_size3i_(2);
  if (x_buffer < 0)
    x_buffer += md_.ringbuffer_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.ringbuffer_size3i_(1);
  if (z_buffer < 0)
    z_buffer += md_.ringbuffer_size3i_(2);
  return md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1) * z_buffer +
         md_.ringbuffer_size3i_(0) * y_buffer + x_buffer;
}

inline int GridMap::globalIdx2InfBufIdx(const Eigen::Vector3i &id) {
  int x_buffer =
      (id(0) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(0);
  int y_buffer =
      (id(1) - md_.ringbuffer_inf_origin3i_(1)) % md_.ringbuffer_inf_size3i_(1);
  int z_buffer =
      (id(2) - md_.ringbuffer_inf_origin3i_(2)) % md_.ringbuffer_inf_size3i_(2);
  if (x_buffer < 0)
    x_buffer += md_.ringbuffer_inf_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.ringbuffer_inf_size3i_(1);
  if (z_buffer < 0)
    z_buffer += md_.ringbuffer_inf_size3i_(2);
  return md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1) *
             z_buffer +
         md_.ringbuffer_inf_size3i_(0) * y_buffer + x_buffer;
}

inline Eigen::Vector3i GridMap::BufIdx2GlobalIdx(size_t address) {
  const int xy = md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1);
  int z = address / xy;
  address %= xy;
  int y = address / md_.ringbuffer_size3i_(0);
  int x = address % md_.ringbuffer_size3i_(0);

  int xg = x + md_.ringbuffer_origin3i_(0);
  if (xg > md_.ringbuffer_upbound3i_(0))
    xg -= md_.ringbuffer_size3i_(0);
  int yg = y + md_.ringbuffer_origin3i_(1);
  if (yg > md_.ringbuffer_upbound3i_(1))
    yg -= md_.ringbuffer_size3i_(1);
  int zg = z + md_.ringbuffer_origin3i_(2);
  if (zg > md_.ringbuffer_upbound3i_(2))
    zg -= md_.ringbuffer_size3i_(2);
  return Eigen::Vector3i(xg, yg, zg);
}

inline Eigen::Vector3i GridMap::infBufIdx2GlobalIdx(size_t address) {
  const int xy = md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1);
  int z = address / xy;
  address %= xy;
  int y = address / md_.ringbuffer_inf_size3i_(0);
  int x = address % md_.ringbuffer_inf_size3i_(0);

  int xg = x + md_.ringbuffer_inf_origin3i_(0);
  if (xg > md_.ringbuffer_inf_upbound3i_(0))
    xg -= md_.ringbuffer_inf_size3i_(0);
  int yg = y + md_.ringbuffer_inf_origin3i_(1);
  if (yg > md_.ringbuffer_inf_upbound3i_(1))
    yg -= md_.ringbuffer_inf_size3i_(1);
  int zg = z + md_.ringbuffer_inf_origin3i_(2);
  if (zg > md_.ringbuffer_inf_upbound3i_(2))
    zg -= md_.ringbuffer_inf_size3i_(2);
  return Eigen::Vector3i(xg, yg, zg);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInBuf(pos))
    return 0;
  if (mp_.enable_virtual_walll_ &&
      (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;
  return md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(pos))] >
                 mp_.min_occupancy_log_
             ? 1
             : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!isInInfBuf(pos))
    return 0;
  if (mp_.enable_virtual_walll_ &&
      (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;
  return int(
      md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
}

inline bool GridMap::isInBuf(const Eigen::Vector3d &pos) {
  return !(pos(0) < md_.ringbuffer_lowbound3d_(0) ||
           pos(1) < md_.ringbuffer_lowbound3d_(1) ||
           pos(2) < md_.ringbuffer_lowbound3d_(2) ||
           pos(0) > md_.ringbuffer_upbound3d_(0) ||
           pos(1) > md_.ringbuffer_upbound3d_(1) ||
           pos(2) > md_.ringbuffer_upbound3d_(2));
}

inline bool GridMap::isInBuf(const Eigen::Vector3i &idx) {
  return !(idx(0) < md_.ringbuffer_lowbound3i_(0) ||
           idx(1) < md_.ringbuffer_lowbound3i_(1) ||
           idx(2) < md_.ringbuffer_lowbound3i_(2) ||
           idx(0) > md_.ringbuffer_upbound3i_(0) ||
           idx(1) > md_.ringbuffer_upbound3i_(1) ||
           idx(2) > md_.ringbuffer_upbound3i_(2));
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3d &pos) {
  return !(pos(0) < md_.ringbuffer_inf_lowbound3d_(0) ||
           pos(1) < md_.ringbuffer_inf_lowbound3d_(1) ||
           pos(2) < md_.ringbuffer_inf_lowbound3d_(2) ||
           pos(0) > md_.ringbuffer_inf_upbound3d_(0) ||
           pos(1) > md_.ringbuffer_inf_upbound3d_(1) ||
           pos(2) > md_.ringbuffer_inf_upbound3d_(2));
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3i &idx) {
  return !(idx(0) < md_.ringbuffer_inf_lowbound3i_(0) ||
           idx(1) < md_.ringbuffer_inf_lowbound3i_(1) ||
           idx(2) < md_.ringbuffer_inf_lowbound3i_(2) ||
           idx(0) > md_.ringbuffer_inf_upbound3i_(0) ||
           idx(1) > md_.ringbuffer_inf_upbound3i_(1) ||
           idx(2) > md_.ringbuffer_inf_upbound3i_(2));
}

inline Eigen::Vector3d GridMap::globalIdx2Pos(const Eigen::Vector3i &id) {
  return Eigen::Vector3d((id(0) + 0.5) * mp_.resolution_,
                         (id(1) + 0.5) * mp_.resolution_,
                         (id(2) + 0.5) * mp_.resolution_);
}

inline Eigen::Vector3i GridMap::pos2GlobalIdx(const Eigen::Vector3d &pos) {
  return (pos * mp_.resolution_inv_).array().floor().cast<int>();
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
