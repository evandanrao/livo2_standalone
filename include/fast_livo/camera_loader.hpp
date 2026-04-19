// standalones/livo2/include/fast_livo/camera_loader.hpp
// YAML-based camera model loader — replaces vk::camera_loader::loadFromNh
// (ROS). Reads camera.yaml and constructs a vk::PinholeCamera or
// vk::ATANCamera.
#pragma once
#include <memory>
#include <string>
#include <vikit/abstract_camera.h>

namespace livo {

// Load a camera model from a YAML file.
// Supports "Pinhole" + "plumb_bob" distortion (k1–k5).
// Returns shared_ptr compatible with vio_manager->cam.
// Throws std::runtime_error on missing or invalid fields.
std::shared_ptr<vk::AbstractCamera>
loadCameraFromYaml(const std::string &yaml_path);

} // namespace livo
