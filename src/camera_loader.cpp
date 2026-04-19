// standalones/livo2/src/camera_loader.cpp
// Loads a vk::PinholeCamera from a camera.yaml read via OpenCV FileStorage.
//
// Expected YAML keys (must start with %YAML:1.0):
//   cam_model:  "Pinhole"
//   cam_width:  <int>       image width  (before any scale)
//   cam_height: <int>       image height (before any scale)
//   scale:      <double>    optional, default 1.0 — multiplied into w/h and
//   fx/fy/cx/cy cam_fx, cam_fy, cam_cx, cam_cy: intrinsics (at full resolution)
//   cam_d0..cam_d4: distortion coefficients (plumb_bob / radtan), default 0.0

#include "fast_livo/camera_loader.hpp"
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <vikit/pinhole_camera.h>

namespace livo {

std::shared_ptr<vk::AbstractCamera>
loadCameraFromYaml(const std::string &yaml_path) {
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened())
    throw std::runtime_error("camera_loader: cannot open " + yaml_path);

  std::string model;
  fs["cam_model"] >> model;
  if (model != "Pinhole")
    throw std::runtime_error("camera_loader: unsupported cam_model '" + model +
                             "' (only 'Pinhole' is supported)");

  double scale = 1.0;
  if (!fs["scale"].empty())
    fs["scale"] >> scale;

  int w_raw = 0, h_raw = 0;
  fs["cam_width"] >> w_raw;
  fs["cam_height"] >> h_raw;
  if (w_raw <= 0 || h_raw <= 0)
    throw std::runtime_error(
        "camera_loader: cam_width / cam_height missing or zero");

  double fx, fy, cx, cy;
  fs["cam_fx"] >> fx;
  fs["cam_fy"] >> fy;
  fs["cam_cx"] >> cx;
  fs["cam_cy"] >> cy;

  double d0 = 0, d1 = 0, d2 = 0, d3 = 0, d4 = 0;
  if (!fs["cam_d0"].empty())
    fs["cam_d0"] >> d0;
  if (!fs["cam_d1"].empty())
    fs["cam_d1"] >> d1;
  if (!fs["cam_d2"].empty())
    fs["cam_d2"] >> d2;
  if (!fs["cam_d3"].empty())
    fs["cam_d3"] >> d3;
  if (!fs["cam_d4"].empty())
    fs["cam_d4"] >> d4;

  // Apply scale to resolution and intrinsics
  double w = w_raw * scale;
  double h = h_raw * scale;
  fx *= scale;
  fy *= scale;
  cx *= scale;
  cy *= scale;

  return std::make_shared<vk::PinholeCamera>(w, h, fx, fy, cx, cy, d0, d1, d2,
                                             d3, d4);
}

} // namespace livo
