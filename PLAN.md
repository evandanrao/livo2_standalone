# standalones/livo2 — Standalone FAST-LIVO2 Architectural Plan

> **Status:** Plan finalized. All open questions resolved. Ready for Stage 0 implementation.  
> **Reference:** `standalones/lio_lc/` is the established pattern for all architecture decisions.  
> **Source:** `ros1/livo2/src/` (branch `jetson_nx` for FAST_LIVO2 and HesaiLidar, `main` for camera_stream_node)

---

## Overview

Convert the ros1/livo2 FAST-LIVO2 stack into a fully ROS-free standalone binary (`hesai_livo2`) that:

- Receives LiDAR point clouds and IMU directly from Hesai SDK (no ROS driver)  
- Receives FPV camera frames directly from GStreamer shmsrc (no image_transport)  
- Runs FAST-LIVO2 SLAM with no ROS dependency whatsoever  
- Streams live state to Foxglove Studio via the official C++ Foxglove SDK WebSocket server  
- Logs pose, point cloud, and images to `/media/internal_logs/livo2/` (MCAP format)  
- Reads all runtime configuration from `config/livo2.yaml` and `config/camera.yaml`

---

## Directory Layout (final state)

```
standalones/livo2/
├── CMakeLists.txt
├── config/
│   ├── livo2.yaml          # SLAM + lidar + IMU params (mirrors params.yaml in lio_lc)
│   └── camera.yaml         # Camera intrinsics + extrinsics (Tcl, Rcl, distortion)
├── include/
│   └── fast_livo/          # Standalone-adapted headers (replaces ros1 include/)
│       ├── b_types.hpp     # Plain C++ message structs (ImuData, PoseData, CloudData, ImageData)
│       ├── bridge.hpp      # Bridge struct: inter-thread data exchange
│       ├── params.h        # Typed param namespace + params::load(yaml_path)
│       ├── camera_loader.hpp  # YAML-based camera model loader (replaces vikit_ros loadFromNh)
│       ├── common_lib.h    # Cleaned: no sensor_msgs, no tf headers
│       ├── IMU_Processing.h   # Cleaned: ImuData instead of sensor_msgs::ImuConstPtr
│       ├── LIVMapper.h     # Cleaned: no ros::Publisher/Subscriber/Timer members
│       ├── preprocess.h    # Cleaned: no pcl_conversions
│       └── ...             # All other FAST_LIVO2 headers, adapted
├── src/
│   ├── main.cpp            # Entry point: signal handler, load params, start threads
│   ├── bridge.cpp          # Bridge implementation
│   ├── lidar_driver.cpp    # Hesai SDK thread: RegRecvCallback → bridge queues
│   ├── camera_driver.cpp   # GStreamer shmsrc thread: appsink → bridge queue
│   ├── foxglove_streamer.cpp  # Foxglove WebSocket + MCAP sink, channels, publish loop
│   ├── camera_loader.cpp   # YAML camera model loader implementation
│   └── fast_livo/          # Cleaned FAST_LIVO2 algorithm sources (copied + adapted)
│       ├── LIVMapper.cpp
│       ├── IMU_Processing.cpp
│       ├── preprocess.cpp
│       ├── voxel_map.cpp
│       ├── vio.cpp
│       ├── frame.cpp
│       └── visual_point.cpp
└── third_party/
    ├── HesaiLidar_SDK_2.0/ # Git submodule or copy from standalones/lio_lc/
    ├── FAST-LIO/           # Pure algorithm, keep as-is
    ├── vikit_common/       # Keep camera model math; remove vikit_ros entirely
    └── foxglove-sdk/       # Official Foxglove C++ SDK (git submodule)
```

---

## Staged Implementation Plan

### Stage 0 — Scaffold the Directory

**Goal:** Create the directory skeleton and copy source files into place so subsequent stages have a clear workspace.

**Step 0.1 — Create directory tree**

```
mkdir -p standalones/livo2/{config,include/fast_livo,src/fast_livo,third_party}
```

**Step 0.2 — Copy algorithm sources from ros1/livo2**

Copy the following verbatim (not modified yet):

| Source (ros1/livo2) | Destination (standalones/livo2) |
|---|---|
| `src/FAST_LIVO2/src/*.cpp` | `src/fast_livo/*.cpp` |
| `src/FAST_LIVO2/include/*.h` | `include/fast_livo/*.h` |
| `src/FAST_LIVO2/third_party/vikit_common/` | `third_party/vikit_common/` |
| `src/FAST_LIVO2/third_party/sophus/` | `third_party/sophus/` (if present) |
| `src/camera_stream_node/src/camera_stream_node.cpp` | `src/camera_driver.cpp` (will be reworked) |
| `src/FAST_LIVO2/config/hesai_jt128.yaml` | `config/livo2.yaml` (starting template) |
| `src/FAST_LIVO2/config/camera_pinhole.yaml` | `config/camera.yaml` (starting template) |

**Step 0.3 — Add third_party submodules**

```
# Reuse lio_lc's Hesai SDK (same platform)
cp -r standalones/lio_lc/third_party/HesaiLidar_SDK_2.0 standalones/livo2/third_party/

# Foxglove SDK — git submodule at v0.x (pinned)
cd standalones/livo2/third_party
git submodule add https://github.com/foxglove/foxglove-sdk foxglove-sdk
```

**Step 0.4 — Stub out CMakeLists.txt and new source files**

Create empty placeholder files: `src/main.cpp`, `src/bridge.cpp`, `src/bridge.hpp` (in include/), `src/lidar_driver.cpp`, `src/foxglove_streamer.cpp`, `src/camera_loader.cpp`, `include/fast_livo/b_types.hpp`, `include/fast_livo/params.h`.

At this point nothing compiles — that is intentional. Each subsequent stage brings it closer.

---

### Stage 1 — Hesai SDK Direct Integration (replace HesaiLidar_ROS_2.0)

**Goal:** Delete the ROS driver entirely. Use `HesaiLidarSdk<LidarPointXYZICRT>` directly, feeding raw point cloud and IMU frames into the bridge.

**Why this first:** The Hesai SDK integration is already proven in `standalones/lio_lc/`. The same `DriverParam` configuration used there applies here. This stage is the lowest-risk starting point and establishes the data pipeline before touching the SLAM core.

#### 1.1 — `include/fast_livo/b_types.hpp`

Define plain C++ structs as the universal data contract between all threads. No ROS types anywhere.

```cpp
// standalones/livo2/include/fast_livo/b_types.hpp
#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>
#include <memory>

namespace livo {

struct ImuData {
    double timestamp;           // seconds
    Eigen::Vector3d accel;      // m/s^2, body frame
    Eigen::Vector3d gyro;       // rad/s, body frame
};

struct CloudData {
    double timestamp;           // seconds, time of last point in scan
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

struct ImageData {
    double timestamp;           // seconds
    cv::Mat frame;              // CV_8UC3, BGR, camera-native resolution
};

struct PoseData {
    double timestamp;
    Eigen::Matrix3d rotation;   // body-in-world
    Eigen::Vector3d position;   // body-in-world, meters
    Eigen::Vector3d velocity;   // body-in-world, m/s
};

}  // namespace livo
```

**Rationale:** Mirrors `b_types.hpp` in lio_lc exactly. No smart pointer overhead for IMU (copied by value into queue, same as lio_lc's `BridgeImu`). CloudData uses `shared_ptr` to avoid copying large point clouds. ImageData copies the `cv::Mat` header (reference counting — cheap).

#### 1.2 — `include/fast_livo/bridge.hpp`

```cpp
// standalones/livo2/include/fast_livo/bridge.hpp
#pragma once
#include "b_types.hpp"
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>

namespace livo {

// Maximum queue capacities to prevent unbounded memory growth
static constexpr size_t kImuQueueMax   = 2000;
static constexpr size_t kCloudQueueMax = 10;
static constexpr size_t kImageQueueMax = 10;

struct Bridge {
    // LiDAR driver → SLAM core
    std::mutex              imu_mtx;
    std::queue<ImuData>     imu_queue;

    std::mutex              cloud_mtx;
    std::queue<CloudData>   cloud_queue;

    // Camera driver → SLAM core
    std::mutex              img_mtx;
    std::queue<ImageData>   img_queue;

    // SLAM core → Foxglove streamer
    std::mutex              pose_mtx;
    std::queue<PoseData>    pose_queue;

    // SLAM core → Foxglove streamer (downsampled map cloud)
    std::mutex              viz_cloud_mtx;
    std::queue<CloudData>   viz_cloud_queue;

    // SLAM core → Foxglove streamer (current frame image with tracked features)
    std::mutex              viz_img_mtx;
    std::queue<ImageData>   viz_img_queue;

    // Shutdown flag — set by SIGINT handler in main
    std::atomic<bool>       running{true};
};

}  // namespace livo
```

**Rationale:** Separates concerns cleanly. The SLAM core writes `pose_queue`, `viz_cloud_queue`, `viz_img_queue`. The Foxglove streamer reads them. All cross-thread transfers use mutex + queue, same as lio_lc.

#### 1.3 — `src/lidar_driver.cpp`

```cpp
// STRUCTURE (not final code — describes what this file must do)

void lidar_driver_thread(Bridge& bridge, const params::LidarParams& p) {
    HesaiLidarSdk<LidarPointXYZICRT> sdk;
    DriverParam param;

    // Configure from params (same pattern as lio_lc test.cc)
    param.input_param.source_type          = DATA_FROM_LIDAR;
    param.input_param.device_ip_address    = p.device_ip;   // e.g. "192.168.4.201"
    param.input_param.ptc_port             = p.ptc_port;    // e.g. 9347
    param.input_param.udp_port             = p.udp_port;    // e.g. 2368
    param.input_param.correction_file_path = p.correction_file;
    param.input_param.firetimes_path       = p.firetimes_file;

    // Point cloud callback → CloudData → bridge.cloud_queue
    sdk.RegRecvCallback([&bridge](const LidarDecodedFrame<LidarPointXYZICRT>& frame) {
        CloudData data;
        data.timestamp = frame.points[frame.points_num - 1].timestamp;
        data.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        // Convert LidarPointXYZICRT → pcl::PointXYZI
        for (int i = 0; i < frame.points_num; i++) {
            pcl::PointXYZI p;
            p.x = frame.points[i].x;
            p.y = frame.points[i].y;
            p.z = frame.points[i].z;
            p.intensity = frame.points[i].intensity;
            data.cloud->push_back(p);
        }
        std::lock_guard<std::mutex> lk(bridge.cloud_mtx);
        if (bridge.cloud_queue.size() < kCloudQueueMax)
            bridge.cloud_queue.push(std::move(data));
    });

    // IMU callback → ImuData → bridge.imu_queue
    sdk.RegRecvCallback([&bridge](const LidarImuData& imu) {
        ImuData data;
        data.timestamp = imu.timestamp;
        data.accel     = {imu.imu_accel_x, imu.imu_accel_y, imu.imu_accel_z};
        data.gyro      = {imu.imu_ang_vel_x, imu.imu_ang_vel_y, imu.imu_ang_vel_z};
        std::lock_guard<std::mutex> lk(bridge.imu_mtx);
        if (bridge.imu_queue.size() < kImuQueueMax)
            bridge.imu_queue.push(data);
    });

    sdk.Start(param);

    // Poll shutdown flag
    while (bridge.running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    sdk.Stop();
}
```

**Key notes:**
- `LidarPointXYZICRT` is the Hesai SDK point type (x, y, z, intensity, confidence, ring, timestamp) — identical to what lio_lc uses.
- `LidarImuData` fields: `imu_accel_x/y/z`, `imu_ang_vel_x/y/z`, `timestamp` (double, seconds) — confirmed from `libhesai/lidar_types.h`. There are no arrays; each axis is a named `double` field.
- Both callbacks are registered before `sdk.Start()`.
- No ROS anywhere. No `std::thread` needed — the SDK manages its own threads internally.

---

### Stage 2 — Camera Driver (replace camera_stream_node ROS node)

**Goal:** Extract the GStreamer pipeline from `camera_stream_node.cpp`, remove all ROS, and feed frames directly into `bridge.img_queue`.

**Why this is easy:** The existing `camera_stream_node.cpp` is ~240 lines where only ~15 lines touch ROS. The GStreamer pipeline, socket discovery, format conversion — all stay identical.

#### 2.1 — `src/camera_driver.cpp`

The following ROS lines are removed:
```cpp
// REMOVE:
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
ros::init(...)
ros::NodeHandle nh;
image_transport::ImageTransport it(nh);
image_transport::Publisher pub = it.advertise("/fpv_cam/image_raw", 1);
ros::spinOnce();
ros::ok()
```

The following ROS lines are replaced:
```cpp
// OLD (ROS):
sensor_msgs::ImagePtr msg = cv_bridge::CvImage(...).toImageMsg();
pub.publish(msg);

// NEW (standalone):
ImageData data;
data.timestamp = get_monotonic_seconds();  // or from GStreamer buffer PTS
data.frame     = bgr_mat.clone();          // cv::Mat clone (reference counted)
std::lock_guard<std::mutex> lk(bridge.img_mtx);
if (bridge.img_queue.size() < kImageQueueMax)
    bridge.img_queue.push(std::move(data));
```

Everything else stays: `gst_init()`, socket glob for `/tmp/fpv_cam*`, `shmsrc → videoconvert → appsink`, `GstSample` extraction, `cv::Mat` wrapping from `GstMapInfo`. 

**Thread structure:**
```cpp
void camera_driver_thread(Bridge& bridge, const params::CameraParams& p) {
    // [GStreamer init and pipeline creation — unchanged from camera_stream_node.cpp]
    // appsink callback → push ImageData to bridge.img_queue
    // Poll bridge.running, cleanup on exit
}
```

**Timestamp:** Use `clock_gettime(CLOCK_MONOTONIC)` converted to seconds. FAST-LIVO2's visual-inertial sync works on monotonic time. In the ROS version, `ros::Time::now()` is also monotonic.

---

### Stage 3 — Strip All ROS From FAST_LIVO2 Core

This is the largest stage and has the most sub-stages. Each sub-stage targets one ROS dependency surface in isolation. The rule is: **compile after each sub-stage**.

#### 3.1 — Replace `sensor_msgs::Imu` with `livo::ImuData` throughout

**Files affected:**
- `include/fast_livo/common_lib.h`
- `include/fast_livo/IMU_Processing.h`
- `src/fast_livo/IMU_Processing.cpp`
- `include/fast_livo/LIVMapper.h`
- `src/fast_livo/LIVMapper.cpp`

**What changes:**

In `common_lib.h`:
```cpp
// REMOVE:
#include <sensor_msgs/Imu.h>
// The MeasureGroup struct:
struct MeasureGroup {
    deque<sensor_msgs::Imu::ConstPtr> imu;   // OLD
    ...
};

// REPLACE WITH:
#include "b_types.hpp"
struct MeasureGroup {
    deque<livo::ImuData> imu;                 // NEW — plain struct, copied by value
    ...
};
```

In `IMU_Processing.h`:
```cpp
// REMOVE:
#include <sensor_msgs/Imu.h>
// Also remove tf include:
#include <tf/transform_broadcaster.h>

// All uses of sensor_msgs::ImuConstPtr:
void Process(const MeasureGroup& meas, ...)  // interface unchanged — MeasureGroup already fixed
```

In `LIVMapper.cpp` (subscriber callback `imu_cbk`):
```cpp
// OLD:
void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in) {
    imu_buffer.push_back(msg_in);
    last_timestamp_imu = msg_in->header.stamp.toSec();
}

// NEW — method called directly from bridge consumer:
void LIVMapper::push_imu(const livo::ImuData& imu) {
    imu_buffer.push_back(imu);
    last_timestamp_imu = imu.timestamp;
}
// Note: imu_buffer changes type from deque<sensor_msgs::Imu::ConstPtr>
//       to deque<livo::ImuData> in LIVMapper.h
```

In `common_lib.h`, all subsequent accesses to `imu->header.stamp.toSec()` change to `imu.timestamp`, `imu->linear_acceleration.x` changes to `imu.accel.x()`, etc.

**Full field mapping:**
| `sensor_msgs::Imu` field | `livo::ImuData` field |
|---|---|
| `header.stamp.toSec()` | `timestamp` |
| `linear_acceleration.x/y/z` | `accel.x()/y()/z()` |
| `angular_velocity.x/y/z` | `gyro.x()/y()/z()` |

#### 3.2 — Remove `tf/transform_broadcaster.h` and TF broadcasting

FAST-LIVO2 publishes TF transforms (`map → body`, `body → camera`) via `tf::TransformBroadcaster broadcaster`. In the standalone, this is replaced by Foxglove `FrameTransformChannel` (Stage 5). For Stage 3.2, simply remove the TF broadcast calls (they are not needed for the SLAM algorithm itself).

**Files affected:** `include/fast_livo/common_lib.h`, `src/fast_livo/LIVMapper.cpp`

```cpp
// REMOVE from common_lib.h:
#include <tf/transform_broadcaster.h>

// REMOVE from LIVMapper.h:
tf::TransformBroadcaster broadcaster;

// REMOVE from LIVMapper.cpp (in publish functions):
broadcaster.sendTransform(tf::StampedTransform(...));
```

Leave a `// TODO Stage 5: FrameTransformChannel` comment in place of each removed broadcast.

#### 3.3 — Replace `pcl_conversions` in `preprocess.h`

`preprocess.h` uses `pcl_conversions::toPCL()` and `pcl::fromROSMsg()` to convert `sensor_msgs::PointCloud2` to PCL. In the standalone, the input arrives as `livo::CloudData` (already PCL), so this conversion layer disappears entirely.

**Files affected:** `include/fast_livo/preprocess.h`, `src/fast_livo/preprocess.cpp`, `src/fast_livo/LIVMapper.cpp`

```cpp
// REMOVE:
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

// In Preprocess::process():
// OLD: takes sensor_msgs::PointCloud2::ConstPtr
//      calls pcl::fromROSMsg(*msg, *pl_full)
// NEW: takes livo::CloudData (already a pcl::PointCloud<pcl::PointXYZI>::Ptr)
//      directly assigns: *pl_full = *cloud_data.cloud;

// Update the signature in preprocess.h:
void process(const livo::CloudData& cloud, PointCloudXYZI::Ptr& pcl_out);
// (was: void process(const livox_ros_driver::CustomMsg::ConstPtr& msg, PointCloudXYZI::Ptr& pcl_out))
// Note: the Hesai driver already outputs cartesian XYZI, so Livox-specific scan unpack disappears
```

**Note:** The existing ros1 FAST_LIVO2 already has a `process()` path for standard `pcl::PointXYZI` (the `point_filter` path in `preprocess.cpp`). This path is kept; only the `sensor_msgs::PointCloud2` deserialization is removed.

#### 3.4 — Replace `nav_msgs` and `geometry_msgs` outputs

FAST-LIVO2 publishes `nav_msgs::Odometry`, `nav_msgs::Path`, `geometry_msgs::PoseStamped`. These are the outputs of the SLAM algorithm. In the standalone, they are replaced by:
- Writing `livo::PoseData` into `bridge.pose_queue` (consumed by Stage 5 Foxglove streamer)
- Appending to an in-memory `std::vector<livo::PoseData>` path buffer (consumed by Stage 5 path channel)

**Files affected:** `include/fast_livo/LIVMapper.h`, `src/fast_livo/LIVMapper.cpp`

```cpp
// REMOVE from LIVMapper.h:
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
ros::Publisher pub_odom_aft_mapped;
ros::Publisher pub_path;
nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;

// ADD to LIVMapper.h:
#include "bridge.hpp"
Bridge* bridge_;     // set in constructor/init

// In LIVMapper.cpp — where pose is published after state estimation:
// OLD:
odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
odomAftMapped.pose.pose.position.x = pos_end[0]; ...
pub_odom_aft_mapped.publish(odomAftMapped);

// NEW:
livo::PoseData pose;
pose.timestamp = lidar_end_time;
pose.position  = pos_end;
pose.rotation  = rot_end.toRotationMatrix();
pose.velocity  = vel_end;
{
    std::lock_guard<std::mutex> lk(bridge_->pose_mtx);
    bridge_->pose_queue.push(pose);
}
```

#### 3.5 — Replace `image_transport` and `cv_bridge` outputs

FAST-LIVO2 publishes debug images via `image_transport::Publisher`:
- `/fast_livo/image_with_features` (visual feature overlay)
- `/fast_livo/image_depth` (depth map)

In the standalone: write `livo::ImageData` into `bridge.viz_img_queue`. The Foxglove streamer (Stage 5) publishes these over `RawImageChannel`.

**Files affected:** `include/fast_livo/LIVMapper.h`, `src/fast_livo/LIVMapper.cpp`

```cpp
// REMOVE:
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
image_transport::Publisher img_pub_vis;

// REPLACE publish calls:
// OLD:
cv_bridge::CvImage cv_image;
cv_image.image = img_tracked;
img_pub_vis.publish(cv_image.toImageMsg());

// NEW:
livo::ImageData viz_img;
viz_img.timestamp = lidar_end_time;
viz_img.frame = img_tracked.clone();
{
    std::lock_guard<std::mutex> lk(bridge_->viz_img_mtx);
    bridge_->viz_img_queue.push(std::move(viz_img));
}
```

#### 3.6 — Replace `visualization_msgs::MarkerArray` (map cloud publisher)

Currently FAST-LIVO2 publishes the voxel map as `sensor_msgs::PointCloud2`. In standalone: downsample and write `livo::CloudData` to `bridge.viz_cloud_queue`.

**Files affected:** `src/fast_livo/LIVMapper.cpp` (the `publish_map()` or equivalent function)

```cpp
// REMOVE:
ros::Publisher pub_map;
sensor_msgs::PointCloud2 map_cloud_msg;
pcl::toROSMsg(*map_cloud, map_cloud_msg);
pub_map.publish(map_cloud_msg);

// REPLACE:
livo::CloudData viz_cloud;
viz_cloud.timestamp = lidar_end_time;
viz_cloud.cloud = map_cloud;  // already PCL — no conversion
{
    std::lock_guard<std::mutex> lk(bridge_->viz_cloud_mtx);
    if (bridge_->viz_cloud_queue.size() < kCloudQueueMax)
        bridge_->viz_cloud_queue.push(std::move(viz_cloud));
}
```

#### 3.7 — Replace `ros::NodeHandle` parameter loading

All `nh.param<>()` calls in `LIVMapper::readParameters()` are replaced by reads from the `params::` namespace (Stage 3.8 defines this). The function signature changes from:

```cpp
// OLD:
void LIVMapper::readParameters(ros::NodeHandle& nh) { ... }

// NEW:
void LIVMapper::readParameters(const params::SlamParams& p) { ... }
```

Every `nh.param<T>("name", var, default)` becomes a direct field assignment from struct `p`.

#### 3.8 — Camera model loader (replace `vk::camera_loader::loadFromNh`)

This is the trickiest ROS dependency in FAST-LIVO2. `initializeComponents()` calls:

```cpp
vk::camera_loader::loadFromNh(nh_cam, vio_manager->cam);
```

This reads `camera_model`, `distortion_model`, `fx`, `fy`, `cx`, `cy`, `k1..k5`, `image_width`, `image_height` from ROS parameter server.

**Solution:** Write `camera_loader.hpp/.cpp` that reads the same fields from `config/camera.yaml` via `cv::FileStorage` (same mechanism as params.h).

```cpp
// include/fast_livo/camera_loader.hpp
#pragma once
#include <vk_camera/abstract_camera.hpp>  // existing vikit_common header
#include <string>

namespace livo {

// Loads camera model from a YAML file.
// Supports "Pinhole" models with "plumb_bob" distortion (matches existing calibration).
// Returns a pointer compatible with vio_manager->cam.
std::shared_ptr<vk::AbstractCamera> loadCameraFromYaml(const std::string& yaml_path);

}  // namespace livo
```

```cpp
// src/camera_loader.cpp — implementation
// Use cv::FileStorage to read:
//   camera_model, distortion_model, image_width, image_height
//   projection_parameters: [fx, fy, cx, cy]
//   distortion_coefficients: [k1, k2, p1, p2, k3]
// Then construct vk::PinholeCamera(...) or vk::EquidistCamera(...) directly
// (same constructors that vikit_ros's loadFromNh uses internally)
```

**Reference:** `vikit_ros/src/camera_loader.cpp` (the existing ros1 version) is the specification for what YAML keys to read. Copy that logic, remove the `ros::NodeHandle` call, replace with `cv::FileStorage`.

**Config structure for `camera.yaml`:**
```yaml
camera_model: Pinhole
distortion_model: plumb_bob
image_width: 1920
image_height: 1080
projection_parameters:
  fx: 1672.85
  fy: 1668.33
  cx: 955.12
  cy: 541.34
distortion_coefficients:
  k1: -0.234
  k2:  0.071
  p1:  0.0
  p2:  0.0
  k3: -0.012
# Extrinsics (LiDAR-to-camera transform)
Rcl: [1,0,0, 0,0.7071,0.7071, 0,-0.7071,0.7071]
Pcl: [0.0, 0.02, 0.50]
```

#### 3.9 — Replace `ros::Timer` with a thread loop

FAST-LIVO2 uses a `ros::Timer` to trigger `stateEstimationAndMapping()` at a fixed rate. In standalone, replace with a `std::thread` + `std::this_thread::sleep_for`:

```cpp
// OLD:
ros::Timer timer = nh.createTimer(ros::Duration(0.005), &LIVMapper::run, this);

// NEW (in main.cpp):
std::thread slam_thread([&mapper, &bridge]() {
    while (bridge.running.load()) {
        mapper.run();  // processes whatever is in the queues
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
});
```

`LIVMapper::run()` is refactored to:
1. Drain `bridge.imu_queue` → internal `imu_buffer`
2. Drain `bridge.cloud_queue` → trigger scan processing
3. Drain `bridge.img_queue` → trigger visual update
4. Run `stateEstimationAndMapping()` if buffers are ready

This preserves the same timing semantics as the ROS timer at 200Hz.

#### 3.10 — Clean up `main.cpp` / remove `ros::init`

```cpp
// OLD ros1 main.cpp:
ros::init(argc, argv, "fast_livo");
ros::NodeHandle nh("~");
LIVMapper mapper(nh);
ros::spin();

// NEW standalone main.cpp:
// (see Stage 6 for full structure)
```

After Stage 3.10, the binary should compile with zero ROS headers. Validate with:
```bash
grep -r "#include <ros/" standalones/livo2/src/ standalones/livo2/include/
# Should return empty
```

---

### Stage 4 — YAML Config System

**Goal:** All runtime parameters in `config/livo2.yaml` and `config/camera.yaml`, loaded via `params::load()`, mirroring lio_lc's `params.h` pattern exactly.

#### 4.1 — `include/fast_livo/params.h`

```cpp
// standalones/livo2/include/fast_livo/params.h
#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>

namespace params {

// --- Lidar driver settings ---
struct LidarParams {
    std::string device_ip     = "192.168.4.201";
    int         ptc_port      = 9347;
    int         udp_port      = 2368;
    std::string correction_file;   // path to .csv
    std::string firetimes_file;    // path to .csv
};

// --- IMU settings ---
struct ImuParams {
    double acc_cov   = 0.1;
    double gyr_cov   = 0.1;
    double ba_cov    = 0.001;
    double bg_cov    = 0.001;
    bool   time_sync_en = false;
};

// --- SLAM core settings ---
struct SlamParams {
    bool   img_en          = true;
    bool   lidar_en        = true;
    int    point_filter_num = 3;
    double filter_size_surf = 0.3;
    double filter_size_map  = 0.3;
    double half_map_size    = 50.0;
    double sliding_thresh   = 1.0;
    double cube_side_length = 50.0;
    int    max_iterations   = 4;
    // ... (all nh.param<> fields from LIVMapper::readParameters)
};

// --- Camera settings (from camera.yaml) ---
struct CameraParams {
    std::string model             = "Pinhole";
    std::string distortion_model  = "plumb_bob";
    int    image_width  = 1920;
    int    image_height = 1080;
    double fx = 1672.85, fy = 1668.33;
    double cx = 955.12,  cy = 541.34;
    double k1=0, k2=0, p1=0, p2=0, k3=0;
    Eigen::Matrix3d Rcl;   // LiDAR-to-camera rotation
    Eigen::Vector3d Pcl;   // LiDAR-to-camera translation
};

// --- Logging settings ---
struct LogParams {
    std::string log_dir     = "/media/internal_logs/livo2";
    bool        enable_mcap = true;
    int         mcap_max_mb = 2048;  // rotate at 2 GB
};

// --- Foxglove streamer settings ---
struct FoxgloveParams {
    int  ws_port        = 8765;
    int  cloud_hz       = 5;    // max point cloud publish rate to WS
    int  image_hz       = 15;   // max image publish rate to WS
    bool enable_ws      = true;
    bool enable_mcap    = true;
};

// Aggregate
struct Params {
    LidarParams   lidar;
    ImuParams     imu;
    SlamParams    slam;
    CameraParams  camera;
    LogParams     log;
    FoxgloveParams foxglove;
};

// Load from YAML files. Throws std::runtime_error on missing required fields.
Params load(const std::string& livo2_yaml, const std::string& camera_yaml);

}  // namespace params
```

#### 4.2 — `config/livo2.yaml` (canonical keys, populated from ros1 hesai_jt128.yaml)

```yaml
# standalones/livo2/config/livo2.yaml

lidar:
  device_ip: "192.168.4.201"
  ptc_port: 9347
  udp_port: 2368
  correction_file: "/opt/hesai/correction.csv"
  firetimes_file: "/opt/hesai/firetimes.csv"

imu:
  acc_cov: 0.1
  gyr_cov: 0.1
  ba_cov: 0.001
  bg_cov: 0.001
  time_sync_en: false

slam:
  img_en: true
  lidar_en: true
  point_filter_num: 3
  filter_size_surf: 0.3
  filter_size_map: 0.3
  half_map_size: 50.0
  sliding_thresh: 1.0
  cube_side_length: 50.0
  max_iterations: 4
  # ... complete list from readParameters()

log:
  log_dir: "/media/internal_logs/livo2"
  enable_mcap: true
  mcap_max_mb: 2048

foxglove:
  ws_port: 8765
  cloud_hz: 5
  image_hz: 15
  enable_ws: true
  enable_mcap: true
```

#### 4.3 — `config/camera.yaml` (from ros1 camera_pinhole.yaml)

```yaml
camera_model: Pinhole
distortion_model: plumb_bob
image_width: 1920
image_height: 1080
projection_parameters:
  fx: 1672.85
  fy: 1668.33
  cx: 955.12
  cy: 541.34
distortion_coefficients:
  k1: -0.234
  k2:  0.071
  p1:  0.0
  p2:  0.0
  k3: -0.012
extrinsics:
  Rcl: [1,0,0, 0,0.7071,0.7071, 0,-0.7071,0.7071]
  Pcl: [0.0, 0.02, 0.50]
```

---

### Stage 5 — Logging to /media/internal_logs

**Goal:** Write MCAP logs identical in structure to what `lio_lc` writes (binary log files, one per session, rotated by size).

#### 5.1 — Log directory and session file

```cpp
// In main.cpp, after params::load():
std::filesystem::create_directories(p.log.log_dir);
std::string session = timestamp_string();  // e.g. "2026-04-14_22-33-14"
std::string mcap_path = p.log.log_dir + "/livo2_" + session + ".mcap";
```

#### 5.2 — MCAP writer via Foxglove SDK

The Foxglove SDK's `McapWriter` and `WebSocketServer` share the same channel API. Channels registered once are automatically available to both sinks. This is the key architectural insight.

```cpp
// foxglove_streamer.cpp — setup (see Stage 6 for full structure)
foxglove::WebSocketServerOptions ws_opts;
ws_opts.port = p.foxglove.ws_port;
auto ws_server = foxglove::WebSocketServer::create(ws_opts);

foxglove::McapWriterOptions mcap_opts;
auto mcap_writer = foxglove::McapWriter::create(mcap_path, mcap_opts);

// Channels — auto-registered to both WS and MCAP sinks:
auto odom_ch    = foxglove::messages::OdometryChannel::create("/livo2/odom").value();
auto cloud_ch   = foxglove::messages::PointCloudChannel::create("/livo2/map_cloud").value();
auto img_ch     = foxglove::messages::RawImageChannel::create("/livo2/image").value();
auto tf_ch      = foxglove::messages::FrameTransformChannel::create("/tf").value();
auto path_ch    = foxglove::messages::PosesInFrameChannel::create("/livo2/path").value();
auto cam_cal_ch = foxglove::messages::CameraCalibrationChannel::create("/livo2/camera_info").value();
```

**MCAP rotation:** When file size exceeds `mcap_max_mb`, close the current writer and open a new one. This is identical to what lio_lc does for its binary log rotation.

---

### Stage 6 — Foxglove WebSocket Streaming

**Goal:** Stream all SLAM outputs to Foxglove Studio in real time over WebSocket on port 8765. No ROS, no rosbridge. Uses the official `foxglove-sdk` C++ library.

#### 6.1 — `src/foxglove_streamer.cpp` — streamer thread

```cpp
// STRUCTURE of foxglove_streamer_thread(Bridge& bridge, const params::Params& p):

void foxglove_streamer_thread(Bridge& bridge, const params::Params& p) {
    // --- Setup sinks ---
    auto ws = foxglove::WebSocketServer::create({.port = p.foxglove.ws_port});
    auto mcap = p.foxglove.enable_mcap
              ? foxglove::McapWriter::create(make_mcap_path(p), {})
              : std::nullopt;

    // --- Create typed channels (registered to both WS and MCAP) ---
    auto odom_ch  = foxglove::messages::OdometryChannel::create("/livo2/odom").value();
    auto cloud_ch = foxglove::messages::PointCloudChannel::create("/livo2/map_cloud").value();
    auto img_ch   = foxglove::messages::RawImageChannel::create("/livo2/image_tracked").value();
    auto tf_ch    = foxglove::messages::FrameTransformChannel::create("/tf").value();
    auto path_ch  = foxglove::messages::PosesInFrameChannel::create("/livo2/path").value();

    // --- Rate limiters ---
    RateLimiter cloud_limiter(p.foxglove.cloud_hz);
    RateLimiter image_limiter(p.foxglove.image_hz);

    // --- Path accumulation buffer ---
    foxglove::messages::PosesInFrame path_msg;
    path_msg.frame_id = "map";

    while (bridge.running.load()) {
        // Drain pose_queue → publish Odometry + TF + accumulate path
        {
            std::queue<livo::PoseData> local;
            { std::lock_guard lk(bridge.pose_mtx); std::swap(local, bridge.pose_queue); }
            while (!local.empty()) {
                auto& pose = local.front();
                publish_odom(odom_ch, pose);
                publish_tf(tf_ch, pose, "map", "body");
                append_path(path_msg, pose);
                local.pop();
            }
            if (!path_msg.poses.empty())
                path_ch.log(path_msg, now_ns());
        }

        // Drain viz_cloud_queue → publish PointCloud (rate-limited)
        if (cloud_limiter.ready()) {
            std::queue<livo::CloudData> local;
            { std::lock_guard lk(bridge.viz_cloud_mtx); std::swap(local, bridge.viz_cloud_queue); }
            if (!local.empty()) {
                publish_cloud(cloud_ch, local.back());  // only latest frame
            }
        }

        // Drain viz_img_queue → publish RawImage (rate-limited)
        if (image_limiter.ready()) {
            std::queue<livo::ImageData> local;
            { std::lock_guard lk(bridge.viz_img_mtx); std::swap(local, bridge.viz_img_queue); }
            if (!local.empty()) {
                publish_raw_image(img_ch, local.back());
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
```

#### 6.2 — Odometry publish helper

```cpp
void publish_odom(foxglove::messages::OdometryChannel& ch, const livo::PoseData& pose) {
    foxglove::messages::Odometry msg;
    msg.timestamp = {.sec  = (uint32_t)pose.timestamp,
                     .nsec = (uint32_t)((pose.timestamp - (uint32_t)pose.timestamp) * 1e9)};
    msg.frame_id      = "map";
    msg.body_frame_id = "body";
    auto q = Eigen::Quaterniond(pose.rotation);
    msg.pose = {
        .position    = {pose.position.x(), pose.position.y(), pose.position.z()},
        .orientation = {q.x(), q.y(), q.z(), q.w()}
    };
    msg.linear_velocity = {pose.velocity.x(), pose.velocity.y(), pose.velocity.z()};
    ch.log(msg, (uint64_t)(pose.timestamp * 1e9));
}
```

#### 6.3 — PointCloud publish helper

```cpp
void publish_cloud(foxglove::messages::PointCloudChannel& ch, const livo::CloudData& data) {
    foxglove::messages::PointCloud msg;
    msg.frame_id     = "map";
    msg.timestamp    = to_foxglove_ts(data.timestamp);
    msg.point_stride = 12;  // 3 × float32 = 12 bytes per point (x,y,z)

    // Field descriptors
    msg.fields = {
        {.name = "x", .offset = 0,  .type = foxglove::messages::PackedElementField::NumericType::FLOAT32},
        {.name = "y", .offset = 4,  .type = foxglove::messages::PackedElementField::NumericType::FLOAT32},
        {.name = "z", .offset = 8,  .type = foxglove::messages::PackedElementField::NumericType::FLOAT32},
    };

    // Pack point data
    const auto& pts = data.cloud->points;
    msg.data.resize(pts.size() * 12);
    for (size_t i = 0; i < pts.size(); i++) {
        float xyz[3] = {pts[i].x, pts[i].y, pts[i].z};
        std::memcpy(msg.data.data() + i * 12, xyz, 12);
    }
    ch.log(msg, (uint64_t)(data.timestamp * 1e9));
}
```

#### 6.4 — RawImage publish helper

```cpp
void publish_raw_image(foxglove::messages::RawImageChannel& ch, const livo::ImageData& data) {
    foxglove::messages::RawImage msg;
    msg.frame_id  = "camera";
    msg.timestamp = to_foxglove_ts(data.timestamp);
    msg.width     = data.frame.cols;
    msg.height    = data.frame.rows;
    msg.encoding  = "bgr8";
    msg.step      = data.frame.cols * 3;
    msg.data.resize(msg.step * msg.height);
    std::memcpy(msg.data.data(), data.frame.data, msg.data.size());
    ch.log(msg, (uint64_t)(data.timestamp * 1e9));
}
```

#### 6.5 — CameraCalibration — publish once on startup

```cpp
void publish_camera_calibration(foxglove::messages::CameraCalibrationChannel& ch,
                                const params::CameraParams& cam) {
    foxglove::messages::CameraCalibration msg;
    msg.frame_id = "camera";
    msg.width    = cam.image_width;
    msg.height   = cam.image_height;
    msg.distortion_model = "plumb_bob";
    msg.d = {cam.k1, cam.k2, cam.p1, cam.p2, cam.k3};
    msg.k = {cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1};
    msg.r = {1,0,0, 0,1,0, 0,0,1};  // identity (monocular)
    msg.p = {cam.fx, 0, cam.cx, 0, 0, cam.fy, cam.cy, 0, 0, 0, 1, 0};
    ch.log(msg, now_ns());
}
```

---

### Stage 7 — `main.cpp` and Thread Orchestration

**Goal:** Wire everything together. Mirror `standalones/lio_lc/src/main.cpp` structure exactly.

```cpp
// standalones/livo2/src/main.cpp — structure

#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"
#include "fast_livo/LIVMapper.h"
#include <csignal>
#include <thread>
#include <atomic>

// Forward declarations
void lidar_driver_thread(livo::Bridge& bridge, const params::Params& p);
void camera_driver_thread(livo::Bridge& bridge, const params::Params& p);
void foxglove_streamer_thread(livo::Bridge& bridge, const params::Params& p);

livo::Bridge g_bridge;

void sigint_handler(int) {
    g_bridge.running.store(false);
}

int main(int argc, char** argv) {
    // Parse args
    std::string livo2_yaml  = (argc > 1) ? argv[1] : "config/livo2.yaml";
    std::string camera_yaml = (argc > 2) ? argv[2] : "config/camera.yaml";

    // Load config
    auto p = params::load(livo2_yaml, camera_yaml);

    // SIGINT → clean shutdown
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    // Load camera model
    auto cam = livo::loadCameraFromYaml(camera_yaml);

    // Initialize SLAM core
    LIVMapper mapper;
    mapper.init(p, &g_bridge, cam);

    // Start threads
    std::thread t_lidar  ([&]{ lidar_driver_thread(g_bridge, p); });
    std::thread t_camera ([&]{ camera_driver_thread(g_bridge, p); });
    std::thread t_slam   ([&]{
        while (g_bridge.running.load()) {
            mapper.run();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
    std::thread t_fox    ([&]{ foxglove_streamer_thread(g_bridge, p); });

    // Wait for shutdown
    t_slam.join();
    t_lidar.join();
    t_camera.join();
    t_fox.join();

    return 0;
}
```

---

### Stage 8 — CMakeLists.txt

**Goal:** Plain CMake, no catkin, no ament-cmake, no ROS. Pure C++ (FAST_LIVO2 has zero CUDA files — confirmed).

```cmake
cmake_minimum_required(VERSION 3.22)   # foxglove-sdk requires 3.22; Jetson has 3.26.4
project(hesai_livo2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# ─── Dependencies ─────────────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# Hesai SDK (same as lio_lc)
add_subdirectory(third_party/HesaiLidar_SDK_2.0 hesai_sdk_build)

# Foxglove C++ SDK
# NOTE: the SDK has a Rust core compiled via Corrosion.
# Option A (recommended for Jetson): cross-compile on dev machine, deploy prebuilt:
#   cmake .. -DFOXGLOVE_PREBUILT_LIB_DIR=/path/to/prebuilt/aarch64
# Option B: install Rust + Corrosion on the Jetson, then add_subdirectory works directly.
add_subdirectory(third_party/foxglove-sdk/cpp foxglove_sdk_build)

# vikit_common (camera models — no vikit_ros)
add_subdirectory(third_party/vikit_common)

# ─── Algorithm sources ────────────────────────────────────────────────────────
set(FAST_LIVO_SRCS
    src/fast_livo/LIVMapper.cpp
    src/fast_livo/IMU_Processing.cpp
    src/fast_livo/preprocess.cpp
    src/fast_livo/voxel_map.cpp
    src/fast_livo/vio.cpp
    src/fast_livo/frame.cpp
    src/fast_livo/visual_point.cpp
)

# ─── Main target ──────────────────────────────────────────────────────────────
add_executable(hesai_livo2
    src/main.cpp
    src/bridge.cpp
    src/lidar_driver.cpp
    src/camera_driver.cpp
    src/foxglove_streamer.cpp
    src/camera_loader.cpp
    ${FAST_LIVO_SRCS}
)

target_include_directories(hesai_livo2 PRIVATE
    include/
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    third_party/vikit_common/include
    third_party/HesaiLidar_SDK_2.0/include
    third_party/foxglove-sdk/cpp/foxglove/include
    third_party/foxglove-sdk/c/include
)

target_link_libraries(hesai_livo2 PRIVATE
    hesai_sdk_lib           # from HesaiLidar_SDK_2.0
    foxglove_cpp_shared     # from foxglove-sdk (confirmed target name)
    vikit_common
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    Eigen3::Eigen
    pthread
    stdc++fs                # std::filesystem — required on GCC 7 (Ubuntu 18.04)
)
```

**Build command (from `standalones/livo2/build/`):**
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j12
```

---

## Thread Diagram

```
┌───────────────────────────────────────────────────────────────────────────────────┐
│  main process: hesai_livo2                                                        │
│                                                                                   │
│  ┌─────────────────────┐         ┌──────────────────────────────────────────────┐ │
│  │  lidar_driver_thread│         │              Bridge                          │ │
│  │  (Hesai SDK internal│──cloud→─│  imu_queue   cloud_queue   img_queue         │ │
│  │   threads + our cb) │──imu──→─│              pose_queue    viz_cloud_queue   │ │
│  └─────────────────────┘         │              viz_img_queue                   │ │
│                                  └────────────────────────────────────────────┬─┘ │
│  ┌─────────────────────┐                                                      │   │
│  │  camera_driver_thread                                                      │   │
│  │  (GStreamer appsink ├─────────────────────── img → ─────────────────────┐  │   │
│  │   + our callback)   │                                                    │  │   │
│  └─────────────────────┘                                                    ▼  │   │
│                                                                             Bridge   │
│  ┌──────────────────────┐                                                   │       │
│  │  slam_thread         │◄── reads imu, cloud, img queues                  │       │
│  │  (LIVMapper::run()   │                                                   │       │
│  │   every 5ms)         ├─── writes pose, viz_cloud, viz_img queues ───────┘       │
│  └──────────────────────┘                                                           │
│                                                                                     │
│  ┌─────────────────────────────────────────────────────────────────┐               │
│  │  foxglove_streamer_thread                                        │               │
│  │  reads: pose_queue, viz_cloud_queue, viz_img_queue              │               │
│  │  writes: WebSocketServer (port 8765) + McapWriter (MCAP file)   │               │
│  │  channels: /livo2/odom, /livo2/map_cloud, /livo2/image_tracked  │               │
│  │           /livo2/path, /tf, /livo2/camera_info                  │               │
│  └─────────────────────────────────────────────────────────────────┘               │
└───────────────────────────────────────────────────────────────────────────────────┘
```

---

## ROS Dependency Removal Checklist

| ROS Dependency | Location | Replacement |
|---|---|---|
| `sensor_msgs::Imu` | `common_lib.h`, `IMU_Processing.h` | `livo::ImuData` (Stage 3.1) |
| `sensor_msgs::PointCloud2` | `preprocess.h`, `LIVMapper.cpp` | `livo::CloudData` (Stage 3.3) |
| `sensor_msgs::Image` | `LIVMapper.cpp` | `livo::ImageData` (Stage 3.5) |
| `nav_msgs::Odometry` | `LIVMapper.h/cpp` | `livo::PoseData` → bridge (Stage 3.4) |
| `nav_msgs::Path` | `LIVMapper.h/cpp` | in-memory path buffer (Stage 3.4) |
| `geometry_msgs::*` | `LIVMapper.h/cpp` | Eigen types (Stage 3.4) |
| `image_transport` | `LIVMapper.h/cpp` | bridge.viz_img_queue (Stage 3.5) |
| `cv_bridge` | `LIVMapper.cpp` | direct `cv::Mat` (Stage 3.5) |
| `tf/transform_broadcaster.h` | `common_lib.h`, `LIVMapper.cpp` | FrameTransformChannel (Stage 3.2 + 6) |
| `visualization_msgs` | `LIVMapper.cpp` | bridge.viz_cloud_queue (Stage 3.6) |
| `ros::NodeHandle` params | `LIVMapper.cpp::readParameters` | `params::SlamParams` struct (Stage 3.7) |
| `vk::camera_loader::loadFromNh` | `LIVMapper.cpp::initializeComponents` | `livo::loadCameraFromYaml` (Stage 3.8) |
| `ros::Timer` | `LIVMapper.h/cpp` | `std::thread` + `sleep_for` (Stage 3.9) |
| `ros::init`, `ros::spin` | `main.cpp` | plain `main()` (Stage 3.10) |
| `pcl_conversions` | `preprocess.h` | removed (Stage 3.3) |
| `HesaiLidar_ROS_2.0` | entire package | `HesaiLidarSdk<>` direct (Stage 1) |
| `camera_stream_node` ROS | `camera_stream_node.cpp` | bridge.img_queue (Stage 2) |

---

## Validation Milestones

| After Stage | Validation |
|---|---|
| 0 | Directory exists, files copied. No build attempt. |
| 1 | `lidar_driver.cpp` compiles standalone (no SLAM). Test: bridge imu/cloud queues fill when LiDAR is live. |
| 2 | `camera_driver.cpp` compiles. Test: bridge img_queue fills when FPV camera is live. |
| 3.1 | `grep -r "sensor_msgs::Imu" include/ src/` returns empty. SLAM compiles. |
| 3.3 | `grep -r "pcl_conversions" include/ src/` returns empty. |
| 3.4–3.6 | `grep -r "nav_msgs\|geometry_msgs\|visualization_msgs" include/ src/` returns empty. |
| 3.7–3.10 | `grep -r "#include <ros/" include/ src/` returns empty. Binary builds. |
| 4 | `config/livo2.yaml` and `config/camera.yaml` load without error. All params accessible. |
| 5 | MCAP file written to `/media/internal_logs/livo2/` with odom channel visible in Foxglove desktop. |
| 6 | Connect Foxglove Studio to `ws://jetson:8765`. All 6 channels appear. Odom, cloud, image stream live. |
| 7 | Full system test: LiDAR + camera + SLAM + Foxglove + MCAP all running simultaneously. |

---

## Key Design Decisions and Rationale

### Why Foxglove SDK Not Rosbridge

The official `foxglove-sdk` C++ library (`github.com/foxglove/foxglove-sdk`) provides `foxglove::WebSocketServer` — a pure C++ implementation of the Foxglove WebSocket protocol. No ROS, no rosbridge process, no network overhead from a separate process. The same typed channels (`OdometryChannel`, `PointCloudChannel`, `RawImageChannel`, etc.) write to both the WebSocket and MCAP simultaneously. This is exactly the Foxglove-intended workflow for non-ROS systems.

### Why Not Modify ros1/livo2 In-Place

The ros1 directory is a working ROS1 system. Making breaking changes there would disrupt flight-critical software. The standalone is a clean port — source files copied, not modified in place.

### Why Keep vikit_common (Not vikit_ros)

`vikit_common` is the camera model math library with zero ROS dependencies. `vikit_ros` is a thin wrapper that adds `ros::NodeHandle`-based loading on top of `vikit_common`. We need only `vikit_common` + our own `camera_loader.cpp` that reads from YAML. This is simpler and has fewer dependencies than any ROS-based approach.

### Why `cv::FileStorage` for Config (Not yaml-cpp)

`params.h` in lio_lc uses `cv::FileStorage`. OpenCV is already a required dependency (FAST-LIVO2 uses it heavily). Adding yaml-cpp as a new dependency just for config loading would be unnecessary. `cv::FileStorage` supports YAML natively and is already the established pattern in this codebase.

### No CUDA in FAST_LIVO2 (confirmed)

FAST_LIVO2 is pure C++. There are zero `.cu` / `.cuh` files in `src/FAST_LIVO2/src/` or `include/`. No `USE_CUDA`, no `__global__`, no CUDA keywords anywhere. The CUDA references seen in the repo are from `HesaiLidar_ROS_2.0`, not from the SLAM algorithm. The standalone CMakeLists.txt uses `LANGUAGES CXX` only.

### foxglove-sdk Build (Rust/Corrosion dependency)

The foxglove-sdk has a Rust core. Building from source requires Rust + Corrosion on the build machine. Jetson Ubuntu 18.04 does not ship with Rust. The recommended path is to cross-compile on the dev machine (which can have Rust installed) and supply the prebuilt `libfoxglove.so` via `-DFOXGLOVE_PREBUILT_LIB_DIR`. Alternatively, install Rust on the Jetson via `rustup`. Both paths use the same `foxglove_cpp_shared` CMake target.

### Queue Depth Design

| Queue | Max | Reasoning |
|---|---|---|
| `imu_queue` | 2000 | IMU at 400Hz × 5s buffer = 2000. Allows SLAM core to fall behind briefly. |
| `cloud_queue` | 10 | LiDAR at 10Hz. >1 frame backup means SLAM is running too slow. |
| `img_queue` | 10 | Camera at 30fps. >10 frames means GStreamer is running head of SLAM. |
| `pose_queue` | — | Unbounded OK: poses are tiny and SLAM is the bottleneck, not the streamer. |
| `viz_cloud_queue` | 10 | Rate-limited in streamer; old frames discarded. |
| `viz_img_queue` | 10 | Rate-limited in streamer; old frames discarded. |

---

## Confirmed Findings (all 6 questions resolved)

### Q1 — FAST_LIVO2 CUDA: NONE

FAST_LIVO2 has **zero `.cu` / `.cuh` files**. No `USE_CUDA`, no `__global__`, no CUDA keywords in any header or source. The CUDA references in the repo belong to `HesaiLidar_ROS_2.0` (the driver), not the SLAM algorithm.

**Correction applied:** Stage 8 CMakeLists.txt uses `LANGUAGES CXX` only. All CUDA flags and `.cu` sources removed.

### Q2 — vikit_common PinholeCamera Constructor

Confirmed signature (from `include/vikit/pinhole_camera.h`):
```cpp
PinholeCamera(double width, double height,
              double fx, double fy, double cx, double cy,
              double d0=0.0, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);
```
Five distortion params (`d0`–`d4`), all defaulting to 0. The existing `vikit/camera_loader.h` includes `<ros/ros.h>` — it cannot be used in the standalone. Our `camera_loader.cpp` constructs `vk::PinholeCamera` directly after reading YAML.

### Q3 — Hesai IMU Struct Fields

Confirmed from `libhesai/lidar_types.h`:
```cpp
struct LidarImuData {
    double timestamp;
    double imu_accel_x, imu_accel_y, imu_accel_z;
    double imu_ang_vel_x, imu_ang_vel_y, imu_ang_vel_z;
    bool flag;
};
```
**Correction applied:** lidar_driver pseudocode updated; `imu.acc[0]` → `imu.imu_accel_x`, `imu.gyro[0]` → `imu.imu_ang_vel_x`, etc.

### Q4 — GStreamer Socket Path

Confirmed: `/tmp/fpv_cam` is the Unix domain socket on the Jetson (created by `stream_server` process). `gstd` runs as a separate process on port 5000. GStreamer 1.14.5, `shmsrc` plugin available.

**Action:** Default `cam_socket: "/tmp/fpv_cam"` in `livo2.yaml`. Keep configurable.

### Q5 — foxglove-sdk CMake Target

Confirmed target name: **`foxglove_cpp_shared`** — exactly what the plan had. Also provides `foxglove_cpp_static`. MCAP writing is bundled inside `foxglove_cpp_shared` (no separate MCAP target needed). Include paths: `foxglove/include/` and `../c/include/` within the SDK `cpp/` directory.

**Important:** SDK requires Rust + Corrosion to build. See "foxglove-sdk Build" in Key Design Decisions for deployment strategy.

### Q6 — Jetson Environment (confirmed)

| Property | Value |
|---|---|
| Model | NVIDIA Jetson Xavier NX Developer Kit |
| OS | Ubuntu 18.04.6 LTS (Bionic Beaver) |
| Kernel | 4.9.253-tegra (aarch64) |
| CUDA | 10.2.300 at `/usr/local/cuda/`, nvcc at `/usr/local/cuda/bin/nvcc` |
| GStreamer | 1.14.5, `shmsrc` plugin available |
| OpenCV | 4.1.1 (`libopencv-dev`) |
| Eigen3 | 3.3.4 (`libeigen3-dev`) |
| PCL | 1.8.1 (`libpcl-dev`) |
| Boost | 1.65.1 |
| GTSAM | 4.2.0 at `/usr/local/lib/libgtsam.so`, cmake config at `/usr/local/lib/cmake/GTSAM/` (not needed for livo2) |
| cmake | 3.26.4 (satisfies foxglove-sdk's 3.22 requirement) |
| ROS | Melodic at `/opt/ros/melodic/` — can extract code/libraries, **do not link** |
| Log path | `/media/internal_logs/` ✅ exists — create `/media/internal_logs/livo2/` |
| FPV socket | `/tmp/fpv_cam` |
| GCC | 7.5 (Ubuntu 18.04 default) — C++17 ok, but `std::filesystem` needs `-lstdc++fs` |

### Q7 — Path Channel Design (bonus)

Use a sliding window of **last 1000 poses** for WebSocket (prevents browser memory growth). Write **full trajectory** to MCAP (unbounded). Implement with `std::deque<livo::PoseData> path_window` in `foxglove_streamer.cpp`, trimmed to 1000 entries on each new pose.
