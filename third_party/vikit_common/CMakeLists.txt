cmake_minimum_required(VERSION 3.5)
project(vikit_common)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# Sophus is vendored in third_party/sophus/ — header-only, no find_package needed

add_library(vikit_common STATIC
  src/atan_camera.cpp
  src/omni_camera.cpp
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/pinhole_camera.cpp
  src/homography.cpp
  src/img_align.cpp
)

target_include_directories(vikit_common PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  # Sophus vendored headers — resolved from parent CMakeLists via SOPHUS_INCLUDE_DIR
  ${SOPHUS_INCLUDE_DIR}
)

target_link_libraries(vikit_common
  ${OpenCV_LIBS}
)
