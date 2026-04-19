/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "b_types.hpp"
#include "common_lib.h"

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

enum LiDARFeature {
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};
enum Surround { Prev, Next };
enum E_jump { Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind };

struct orgtype {
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  LiDARFeature ftype;
  orgtype() {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

class Preprocess {
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  // Stage 3.3: process() now accepts pre-filtered PointCloudXYZI from
  // lidar_driver (SDK frame is converted there; no ROS PointCloud2
  // deserialization)
  void process(const PointCloudXYZI::Ptr &cloud_in,
               PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128];
  vector<orgtype> typess[128];
  int lidar_type, point_filter_num, N_SCANS;

  double blind, blind_sqr;
  bool feature_enabled, given_offset_time;
  // Stage 3.3: ros::Publisher pub_full/surf/corn removed

private:
  // Stage 3.3: hesai_jt128_handler removed (processing now in lidar_driver.cpp)
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  // Stage 3.3: pub_func removed (no ROS publisher)
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i,
                  uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur,
                   uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i,
                       Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_