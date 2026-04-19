#pragma once

#include <string>
#include <ros/ros.h>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>

namespace vk {
namespace camera_loader {

/// Load camera from a ROS NodeHandle whose namespace contains the cam_* params.
inline bool loadFromNh(ros::NodeHandle& nh, vk::AbstractCamera*& cam)
{
  std::string cam_model;
  nh.param<std::string>("cam_model", cam_model, "");

  if (cam_model == "Pinhole")
  {
    int w = 640, h = 480;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    double d0 = 0.0, d1 = 0.0, d2 = 0.0, d3 = 0.0;
    nh.param("cam_width",  w,  w);
    nh.param("cam_height", h,  h);
    nh.param("cam_fx", fx, fx);
    nh.param("cam_fy", fy, fy);
    nh.param("cam_cx", cx, cx);
    nh.param("cam_cy", cy, cy);
    nh.param("cam_d0", d0, d0);
    nh.param("cam_d1", d1, d1);
    nh.param("cam_d2", d2, d2);
    nh.param("cam_d3", d3, d3);
    cam = new vk::PinholeCamera(w, h, fx, fy, cx, cy, d0, d1, d2, d3);
    return true;
  }
  else if (cam_model == "ATAN")
  {
    int w = 640, h = 480;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0, d0 = 0.0;
    nh.param("cam_width",  w,  w);
    nh.param("cam_height", h,  h);
    nh.param("cam_fx", fx, fx);
    nh.param("cam_fy", fy, fy);
    nh.param("cam_cx", cx, cx);
    nh.param("cam_cy", cy, cy);
    nh.param("cam_d0", d0, d0);
    cam = new vk::ATANCamera(w, h, fx, fy, cx, cy, d0);
    return true;
  }

  cam = nullptr;
  return false;
}

} // namespace camera_loader
} // namespace vk
