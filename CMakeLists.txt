# CMakeLists.txt — standalones/livo2 (hesai_livo2)
# ROS-free standalone FAST-LIVO2 + Hesai SDK + GStreamer camera + Foxglove.
# Build: cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j12

cmake_minimum_required(VERSION 3.22)  # foxglove-sdk requires 3.22; Jetson has 3.26.4
project(hesai_livo2 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----- CPU count flags (mirrors FAST-LIO2 setup) ----------------------------
include(ProcessorCount)
ProcessorCount(N)
if(N GREATER 4)
  add_definitions(-DMP_EN -DMP_PROC_NUM=3)
elseif(N GREATER 3)
  add_definitions(-DMP_EN -DMP_PROC_NUM=2)
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# ----- Core dependencies -----------------------------------------------------
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# GStreamer (for camera_driver.cpp)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP   REQUIRED gstreamer-app-1.0)

# Sophus — vendored header-only, no find_package needed
# Code uses #include <sophus/se3.h>, so parent dir is the include root.
# se3.h itself uses #include "so3.hpp" (relative), so sophus/ dir also needed.
set(SOPHUS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party;${CMAKE_CURRENT_SOURCE_DIR}/third_party/sophus")

# ----- Hesai SDK (same pattern as standalones/lio_lc) ------------------------
set(HESAI_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/HesaiLidar_SDK_2.0")
set(DISENABLE_TEST_CC ON CACHE BOOL "Disable Hesai SDK test targets" FORCE)
set(FIND_CUDA         ON  CACHE BOOL "Find CUDA for Hesai SDK"       FORCE)
# Jetson NX = Volta = sm_72; set before SDK subdirectory so NVCC picks it up
find_package(CUDA)
if(CUDA_FOUND)
  set(CMAKE_CUDA_ARCHITECTURES 72)
  set(CUDA_NVCC_FLAGS -arch=sm_72;-O3;)
  # nvcc may not be in PATH (common on Jetson); set explicitly from find_package result
  if(NOT CMAKE_CUDA_COMPILER AND CUDA_NVCC_EXECUTABLE)
    set(CMAKE_CUDA_COMPILER "${CUDA_NVCC_EXECUTABLE}" CACHE FILEPATH "nvcc path" FORCE)
  endif()
  message(STATUS "CUDA ${CUDA_VERSION} found — Hesai SDK GPU parser enabled (sm_72), nvcc: ${CMAKE_CUDA_COMPILER}")
else()
  message(STATUS "CUDA not found — Hesai SDK will use CPU parser")
endif()
add_subdirectory(${HESAI_SDK_DIR} hesai_sdk_build EXCLUDE_FROM_ALL)

# ----- vikit_common (camera models, no ROS) ----------------------------------
# Pass SOPHUS_INCLUDE_DIR into vikit_common's scope
set(SOPHUS_INCLUDE_DIR "${SOPHUS_INCLUDE_DIR}" CACHE PATH "Sophus headers")
add_subdirectory(third_party/vikit_common)

# ----- Foxglove C++ SDK ------------------------------------------------------
# Requires Rust + Corrosion on the build machine.
# For cross-compile / Jetson deploy: set FOXGLOVE_PREBUILT_LIB_DIR.
# For Stage 3 build/test (no foxglove yet): -DBUILD_FOXGLOVE=OFF
# See PLAN.md "foxglove-sdk Build" section.
option(BUILD_FOXGLOVE "Build and link the Foxglove SDK" ON)
if(BUILD_FOXGLOVE)
  set(FOXGLOVE_SDK_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/foxglove-sdk/cpp")
  set(FOXGLOVE_SDK_C_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/third_party/foxglove-sdk/c")

  if(DEFINED FOXGLOVE_PREBUILT_LIB_DIR)
    # Prebuilt mode: libfoxglove.so + libfoxglove_cpp.a are cross-compiled on dev machine.
    # No Rust, no FetchContent, no C++ standard issues on the target machine.
    get_filename_component(FOXGLOVE_PREBUILT_LIB_DIR "${FOXGLOVE_PREBUILT_LIB_DIR}" ABSOLUTE)

    add_library(foxglove-shared SHARED IMPORTED GLOBAL)
    set_target_properties(foxglove-shared PROPERTIES
        IMPORTED_LOCATION "${FOXGLOVE_PREBUILT_LIB_DIR}/libfoxglove.so")

    add_library(foxglove_cpp_shared STATIC IMPORTED GLOBAL)
    set_target_properties(foxglove_cpp_shared PROPERTIES
        IMPORTED_LOCATION "${FOXGLOVE_PREBUILT_LIB_DIR}/libfoxglove_cpp.a"
        INTERFACE_INCLUDE_DIRECTORIES
            "${FOXGLOVE_SDK_CPP_DIR}/foxglove/include;${FOXGLOVE_SDK_C_DIR}/include"
        INTERFACE_LINK_LIBRARIES "foxglove-shared;pthread;dl;m")
  else()
    add_subdirectory(third_party/foxglove-sdk/cpp foxglove_sdk_build)
  endif()
endif()

# ----- Algorithm sources (FAST_LIVO2 core — cleaned in Stage 3) --------------
set(FAST_LIVO_SRCS
    src/fast_livo/LIVMapper.cpp
    src/fast_livo/IMU_Processing.cpp
    src/fast_livo/preprocess.cpp
    src/fast_livo/voxel_map.cpp
    src/fast_livo/vio.cpp
    src/fast_livo/frame.cpp
    src/fast_livo/visual_point.cpp
)

# ----- EGO-Planner static library -------------------------------------------
set(EGO_PLANNER_SRCS
    third_party/ego_planner/src/dyn_a_star.cpp
    third_party/ego_planner/src/raycast.cpp
    third_party/ego_planner/src/grid_map.cpp
    third_party/ego_planner/src/poly_traj_optimizer.cpp
    third_party/ego_planner/src/planner_manager.cpp
    third_party/ego_planner/src/ego_replan_fsm.cpp
    src/planner.cpp
)
add_library(ego_planner_core STATIC ${EGO_PLANNER_SRCS})
target_include_directories(ego_planner_core PUBLIC
    third_party/ego_planner/include
    include/
    include/fast_livo/
    ${SOPHUS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(ego_planner_core PUBLIC cxx_std_17)

# ----- Main target -----------------------------------------------------------
add_executable(hesai_livo2
    src/main.cpp
    src/bridge.cpp
    src/params.cpp
    src/lidar_driver.cpp
    src/camera_driver.cpp
    src/foxglove_streamer.cpp
    src/camera_loader.cpp
    src/lidar_publisher.cpp
    src/goal_listener.cpp
    ${FAST_LIVO_SRCS}
)

target_include_directories(hesai_livo2 PRIVATE
    # Our own headers
    include/
    include/fast_livo/
    # Sophus (vendored header-only)
    ${SOPHUS_INCLUDE_DIR}
    # Eigen, PCL, OpenCV
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    # GStreamer
    ${GSTREAMER_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    # vikit_common
    third_party/vikit_common/include
    # Hesai SDK
    ${HESAI_SDK_DIR}/libhesai
    ${HESAI_SDK_DIR}/driver
    # CUDA runtime headers (needed when USE_CUDA — hesai_lidar_sdk.hpp includes udp_parser_gpu.h)
    $<$<BOOL:${CUDA_FOUND}>:${CUDA_INCLUDE_DIRS}>
)
if(BUILD_FOXGLOVE)
  target_include_directories(hesai_livo2 PRIVATE
    # Foxglove SDK
    third_party/foxglove-sdk/cpp/foxglove/include
    third_party/foxglove-sdk/c/include
  )
endif()

# PCL 1.8 on Ubuntu 18.04 has deprecated warnings — suppress by marking as SYSTEM
target_include_directories(hesai_livo2 SYSTEM PRIVATE
    ${PCL_INCLUDE_DIRS}
)

# GStreamer library search path (needed when GStreamer is not in standard system paths,
# e.g. Jetson local install at ~/gst/).  pkg_check_modules populates LIBRARY_DIRS.
target_link_directories(hesai_livo2 PRIVATE
    ${GSTREAMER_LIBRARY_DIRS}
    ${GST_APP_LIBRARY_DIRS}
)

target_link_libraries(hesai_livo2 PRIVATE
    # Hesai SDK
    hesai_sdk_lib
    # Foxglove WebSocket + MCAP (single shared lib)
    $<$<BOOL:${BUILD_FOXGLOVE}>:foxglove_cpp_shared>
    # Camera models
    vikit_common
    # Point cloud
    ${PCL_LIBRARIES}
    # Vision
    ${OpenCV_LIBS}
    # GStreamer
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES}
    # Eigen (header-only; link target sets -I correctly)
    Eigen3::Eigen
    # Threading
    pthread
    # EGO-Planner
    ego_planner_core
)
# CUDA runtime — link libcudart so GPU parser symbols resolve at link time.
# genex can't expand CUDA_LIBRARIES list reliably; plain if() is safer.
if(CUDA_FOUND)
    target_link_libraries(hesai_livo2 PRIVATE ${CUDA_CUDART_LIBRARY})
endif()

# ROOT_DIR: Log/ and scripts/ paths relative to project source directory
# Matches the original catkin package ROOT_DIR convention.
#
# HESAI_FIRETIMES_FILE: bundled JT128 firetime correction CSV (same lidar model
# used by lio_lc standalone). Used as compile-time default; overridable via
# livo2.yaml lidar/firetimes_file key.
set(HESAI_FIRETIMES_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/HesaiLidar_SDK_2.0/correction/firetime_correction/JT128_Firetime Correction File.csv")
target_compile_definitions(hesai_livo2 PRIVATE
    ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
    HESAI_FIRETIMES_FILE="${HESAI_FIRETIMES_FILE}"
    $<$<BOOL:${BUILD_FOXGLOVE}>:BUILD_FOXGLOVE>
)
