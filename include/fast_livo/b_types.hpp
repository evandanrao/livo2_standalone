// standalones/livo2/include/fast_livo/b_types.hpp
// Plain C++ message structs — the universal data contract between all threads.
// No ROS, no sensor_msgs, no std_msgs anywhere in this file.
#pragma once
#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Match FAST_LIVO2 internal point type (PointType = pcl::PointXYZINormal).
// curvature field stores per-point time offset in ms from scan start.
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZINormal>;

// RGB-coloured point cloud produced by LIVO VIO frames (camera + LiDAR fusion).
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;

namespace livo {

struct ImuData {
  double timestamp;      // seconds (monotonic, already SI-converted)
  Eigen::Vector3d accel; // m/s², body frame
  Eigen::Vector3d gyro;  // rad/s, body frame
};

struct CloudData {
  double timestamp;          // seconds, frame start time
  PointCloudXYZI::Ptr cloud; // curvature = per-point offset ms from scan start
};

// RGB-coloured cloud produced by LIVO VIO frames.  Each point is coloured by
// projecting it into the current camera image.  Only produced when img_en=1.
struct RgbCloudData {
  double timestamp;            // seconds, frame time
  PointCloudXYZRGB::Ptr cloud; // points coloured by camera image
};

struct ImageData {
  double timestamp; // seconds
  cv::Mat image;    // CV_8UC3 BGR, camera-native resolution
};

struct PoseData {
  double timestamp;
  Eigen::Matrix3d rotation; // body-in-world (SO3)
  Eigen::Vector3d position; // body-in-world, metres
  Eigen::Vector3d velocity; // body-in-world, m/s
};

} // namespace livo
