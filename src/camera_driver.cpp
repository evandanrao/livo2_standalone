// standalones/livo2/src/camera_driver.cpp
// Stage 2 — GStreamer shmsrc camera thread (stub)
// TODO: implement in Stage 2
// standalones/livo2/src/camera_driver.cpp
// Stage 2 — GStreamer shmsrc camera driver (replaces camera_stream_node ROS
// node). Reads BGRx frames from the GStreamer shmsink socket written by
// stream_server, converts to cv::Mat (BGR), and pushes livo::ImageData into
// bridge.img_queue.
//
// Pipeline (identical to camera_stream_node.cpp, ROS stripped):
//   shmsrc → videoconvert (BGRx→BGR) → appsink (max-buffers=1, drop=true)
//
// Timestamp: derived from the buffer PTS in pipeline-clock space, converted to
// absolute UNIX seconds so it can be correlated with the IMU/lidar timestamps.
// Falls back to std::chrono::system_clock if PTS is invalid.

#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <opencv2/core.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <glob.h>
#include <string>
#include <thread>

// ── Socket discovery ─────────────────────────────────────────────────────────
static std::string find_socket(const std::string &pattern) {
  glob_t g{};
  std::string result;
  if (::glob(pattern.c_str(), 0, nullptr, &g) == 0) {
    for (std::size_t i = 0; i < g.gl_pathc; ++i) {
      std::string p = g.gl_pathv[i];
      if (p > result)
        result = p;
    }
  }
  globfree(&g);
  return result;
}

static std::string wait_for_socket(const std::string &glob_pattern,
                                   int timeout_sec,
                                   const std::atomic<bool> &running) {
  const int iters = timeout_sec * 2;
  for (int i = 0; i < iters && running.load(); ++i) {
    std::string path = find_socket(glob_pattern);
    if (!path.empty())
      return path;
    if (i % 10 == 0)
      fprintf(stderr, "[camera_driver] waiting for socket '%s'...\n",
              glob_pattern.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return {};
}

static double now_unix_seconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

void camera_driver_thread(livo::Bridge &bridge, const params::Params &p) {
  if (!p.slam.img_en) {
    // Camera disabled in config — exit cleanly without touching bridge.running
    return;
  }
  // TODO Stage 4: read socket path and timeout from p
  const std::string camera_name = "fpv";
  const std::string socket_glob = "/tmp/" + camera_name + "_cam*";
  constexpr int TIMEOUT_SEC = 30;
  constexpr int WIDTH = 1920;
  constexpr int HEIGHT = 1080;
  constexpr int CHANNELS = 3; // BGR
  constexpr std::size_t FRAME_BYTES = (std::size_t)WIDTH * HEIGHT * CHANNELS;

  std::string socket_path =
      wait_for_socket(socket_glob, TIMEOUT_SEC, bridge.running);
  if (socket_path.empty()) {
    fprintf(stderr,
            "[camera_driver] socket '%s' not found after %d s — "
            "is stream_server running?\n",
            socket_glob.c_str(), TIMEOUT_SEC);
    bridge.running.store(false);
    return;
  }
  fprintf(stderr, "[camera_driver] using socket %s\n", socket_path.c_str());

  // ── Build GStreamer pipeline ──────────────────────────────────────────────
  gst_init(nullptr, nullptr);

  const std::string pipe_desc =
      "shmsrc socket-path=" + socket_path +
      " is-live=true "
      "! video/x-raw,format=BGRx,width=1920,height=1080,framerate=30/1 "
      "! videoconvert "
      "! video/x-raw,format=BGR "
      "! appsink name=sink max-buffers=1 drop=true sync=false "
      "emit-signals=false";

  GError *gerr = nullptr;
  GstElement *pipeline = gst_parse_launch(pipe_desc.c_str(), &gerr);
  if (!pipeline || gerr) {
    fprintf(stderr, "[camera_driver] failed to build pipeline: %s\n",
            gerr ? gerr->message : "unknown error");
    if (gerr)
      g_error_free(gerr);
    bridge.running.store(false);
    return;
  }

  GstElement *sink_elem = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  if (!sink_elem) {
    fprintf(stderr, "[camera_driver] appsink element not found\n");
    gst_object_unref(pipeline);
    bridge.running.store(false);
    return;
  }
  GstAppSink *appsink = GST_APP_SINK(sink_elem);

  if (gst_element_set_state(pipeline, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    fprintf(stderr, "[camera_driver] failed to set pipeline to PLAYING\n");
    gst_object_unref(GST_OBJECT(appsink));
    gst_object_unref(pipeline);
    bridge.running.store(false);
    return;
  }
  fprintf(stderr,
          "[camera_driver] pipeline playing — pulling BGR 1920x1080 @ 30fps\n");

  const GstClockTime FRAME_TIMEOUT = GST_SECOND / 30; // ~33 ms

  while (bridge.running.load()) {
    GstSample *sample = gst_app_sink_try_pull_sample(appsink, FRAME_TIMEOUT);
    if (!sample)
      continue; // timeout — loop back

    GstBuffer *buf = gst_sample_get_buffer(sample);
    GstMapInfo map{};

    if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
      if (map.size >= FRAME_BYTES) {
        // ── Timestamp ────────────────────────────────────────────────
        double ts = now_unix_seconds(); // fallback
        GstClockTime buf_pts = GST_BUFFER_PTS(buf);
        if (GST_CLOCK_TIME_IS_VALID(buf_pts)) {
          GstClock *clk = gst_element_get_clock(pipeline);
          GstClockTime clk_now = gst_clock_get_time(clk);
          GstClockTime run_now = gst_element_get_current_running_time(pipeline);
          gst_object_unref(clk);
          if (run_now >= buf_pts && clk_now >= (run_now - buf_pts)) {
            GstClockTime abs_ns = clk_now - (run_now - buf_pts);
            ts = static_cast<double>(abs_ns) * 1e-9;
          }
        }

        // ── Build ImageData ──────────────────────────────────────────
        livo::ImageData data;
        data.timestamp = ts;
        // Zero-copy into cv::Mat, then clone so the buffer can be released
        cv::Mat view(HEIGHT, WIDTH, CV_8UC3, map.data,
                     (size_t)(WIDTH * CHANNELS));
        data.image = view.clone();

        std::lock_guard<std::mutex> lk(bridge.img_mtx);
        if (bridge.img_queue.size() < livo::kImageQueueMax)
          bridge.img_queue.push(std::move(data));
      } else {
        fprintf(stderr,
                "[camera_driver] unexpected buffer size %zu (expected %zu)\n",
                map.size, FRAME_BYTES);
      }
      gst_buffer_unmap(buf, &map);
    }
    gst_sample_unref(sample);
  }

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(appsink));
  gst_object_unref(pipeline);
  fprintf(stderr, "[camera_driver] stopped\n");
}
