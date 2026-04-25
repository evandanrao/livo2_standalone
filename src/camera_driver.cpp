// standalones/livo2/src/camera_driver.cpp
//
// Payload camera driver — replaces payload_cam_stream_node (ROS stripped).
// Streams the Allied Vision GigE payload camera (BayerRG8, 1280×720, 30 fps)
// via the VimbaX C SDK and pushes livo::ImageData (BGR cv::Mat) into
// bridge.img_queue.
//
// Init sequence mirrors payload_cam_stream_node.cpp:
//   1. VmbStartup
//   2. VmbTransportLayersList  — forces GigE rescan
//   3. VmbCamerasList          — skip simulators
//   4. VmbCameraOpen           — FULL → READ fallback
//   5. UserSetLoad("Default")  — factory reset
//   6. Configure ROI, BayerRG8, 30 fps, ExposureAuto=Off, GainAuto=Continuous,
//      BalanceWhiteAuto=Continuous, GevSCPSPacketSize=1500, GevSCPD=2000
//   7. Announce 5 frame buffers, CaptureStart, queue all
//   8. AcquisitionStart → warm-up stop/revoke/re-announce/restart → skip 10 frames
//   9. Capture loop: VmbCaptureFrameWait(50 ms) → validate → demosaic → push
//  10. On >100 consecutive errors or error -19: clean close → restart detection
//
// VimbaX SDK expected at /opt/VimbaX_2025-3 (override via cmake -DVIMBAX_ROOT=).

#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"

#include <VmbC/VmbC.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

// ── Timestamp helper ─────────────────────────────────────────────────────────
static double now_unix_seconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

// ── Bayer RG8 → RGB nearest-neighbour demosaic ───────────────────────────────
// Pattern:  R G       Each 2×2 block samples R, G, B and fills all four
//           G B       output pixels with the same colour (nearest-neighbour).
// Output:   packed RGB bytes (3 bytes/pixel, row-major).
static void bayer_rg8_to_rgb(const uint8_t *bayer, uint8_t *rgb,
                              int width, int height)
{
    for (int by = 0; by < height / 2; ++by) {
        const int y = by * 2;
        for (int bx = 0; bx < width / 2; ++bx) {
            const int x   = bx * 2;
            const int idx = y * width + x;
            const uint8_t r = bayer[idx];
            const uint8_t g = bayer[idx + 1];
            const uint8_t b = bayer[idx + width + 1];
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    uint8_t *p = rgb + ((y + dy) * width + (x + dx)) * 3;
                    p[0] = r;
                    p[1] = g;
                    p[2] = b;
                }
            }
        }
    }
}

// ── Camera validation ────────────────────────────────────────────────────────
static bool is_simulator(const VmbCameraInfo_t &cam)
{
    const std::string name(cam.modelName      ? cam.modelName      : "");
    const std::string id  (cam.cameraIdString ? cam.cameraIdString : "");
    return name.find("Simulator") != std::string::npos ||
           id  .find("Simulator") != std::string::npos ||
           id  .rfind("DEV_Cam", 0) == 0;
}

// ── VimbaX feature helpers ───────────────────────────────────────────────────
static VmbError_t set_enum (VmbHandle_t h, const char *f, const char *v)  { return VmbFeatureEnumSet (h, f, v); }
static VmbError_t set_int  (VmbHandle_t h, const char *f, VmbInt64_t v)   { return VmbFeatureIntSet  (h, f, v); }
static VmbError_t set_float(VmbHandle_t h, const char *f, double v)       { return VmbFeatureFloatSet(h, f, v); }
static VmbError_t set_bool (VmbHandle_t h, const char *f, VmbBool_t v)    { return VmbFeatureBoolSet (h, f, v); }
static VmbInt64_t get_int  (VmbHandle_t h, const char *f, VmbInt64_t def = 0)
{
    VmbInt64_t v = def;
    VmbFeatureIntGet(h, f, &v);
    return v;
}

// ── Clean close ──────────────────────────────────────────────────────────────
static void revoke_and_close(VmbHandle_t handle, std::vector<VmbFrame_t> &frames)
{
    VmbCaptureEnd(handle);
    for (auto &f : frames)
        VmbFrameRevoke(handle, &f);
    VmbCameraClose(handle);
}

// ── camera_driver_thread ─────────────────────────────────────────────────────
void camera_driver_thread(livo::Bridge &bridge, const params::Params &p)
{
    if (!p.slam.img_en) {
        // Camera disabled in config — exit cleanly
        return;
    }

    constexpr int    TIMEOUT_SEC    = 60;
    constexpr int    DESIRED_WIDTH  = 1280;
    constexpr int    DESIRED_HEIGHT = 720;

    fprintf(stderr,
            "[camera_driver] payload cam starting (VimbaX)  roi=%dx%d\n",
            DESIRED_WIDTH, DESIRED_HEIGHT);

    // ── Step 1: VmbStartup ────────────────────────────────────────────────────
    if (VmbStartup(nullptr) != VmbErrorSuccess) {
        fprintf(stderr, "[camera_driver] VmbStartup failed — camera thread exiting\n");
        return;
    }
    fprintf(stderr, "[camera_driver] VimbaX SDK started\n");

    // ── Pre-allocate RGB scratch buffer (reused every frame) ─────────────────
    const std::size_t FRAME_RGB_BYTES =
        static_cast<std::size_t>(DESIRED_WIDTH) * DESIRED_HEIGHT * 3;
    std::vector<uint8_t> rgb_buf(FRAME_RGB_BYTES, 0);

    // ── Detection + streaming loop ────────────────────────────────────────────
    int  detection_attempts = 0;
    bool sdk_ok             = true;
    auto detection_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(TIMEOUT_SEC);

    while (bridge.running.load() && sdk_ok) {

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // ── Step 2: Transport layer rescan ────────────────────────────────────
        {
            VmbHandle_t tl_handles[10]{};
            VmbUint32_t tl_count = 0;
            VmbTransportLayersList(tl_handles, 10, &tl_count, sizeof(VmbHandle_t));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // ── Step 3: Enumerate cameras ─────────────────────────────────────────
        VmbCameraInfo_t cameras[10]{};
        VmbUint32_t     num_found = 0;
        VmbCamerasList(cameras, 10, &num_found, sizeof(VmbCameraInfo_t));

        if (num_found == 0) {
            ++detection_attempts;
            if (detection_attempts % 10 == 1)
                fprintf(stderr,
                        "[camera_driver] no cameras found (attempt %d) — waiting...\n",
                        detection_attempts);

            if (std::chrono::steady_clock::now() >= detection_deadline) {
                fprintf(stderr,
                        "[camera_driver] %ds timeout — restarting VimbaX SDK\n",
                        TIMEOUT_SEC);
                VmbShutdown();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (VmbStartup(nullptr) != VmbErrorSuccess) {
                    fprintf(stderr, "[camera_driver] VmbStartup re-init failed — exiting\n");
                    sdk_ok = false;
                    break;
                }
                detection_attempts = 0;
                detection_deadline = std::chrono::steady_clock::now() +
                                     std::chrono::seconds(TIMEOUT_SEC);
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        // Skip simulators, take first real camera
        int real_idx = -1;
        for (int i = 0; i < static_cast<int>(num_found); ++i) {
            if (!is_simulator(cameras[i])) { real_idx = i; break; }
        }
        if (real_idx < 0) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            continue;
        }
        detection_attempts = 0;
        detection_deadline = std::chrono::steady_clock::now() +
                             std::chrono::seconds(TIMEOUT_SEC);

        const VmbCameraInfo_t &cam = cameras[real_idx];
        fprintf(stderr, "[camera_driver] found camera '%s'  id='%s'\n",
                cam.modelName, cam.cameraIdString);

        // ── Step 4: Open camera (FULL → READ fallback) ────────────────────────
        VmbHandle_t     handle      = nullptr;
        VmbAccessMode_t access_used = VmbAccessModeFull;

        VmbError_t open_err = VmbCameraOpen(cam.cameraIdString,
                                            VmbAccessModeFull, &handle);
        if (open_err != VmbErrorSuccess) {
            fprintf(stderr,
                    "[camera_driver] FULL open failed (%d), trying READ\n", open_err);
            open_err    = VmbCameraOpen(cam.cameraIdString, VmbAccessModeRead, &handle);
            access_used = VmbAccessModeRead;
        }
        if (open_err != VmbErrorSuccess) {
            fprintf(stderr,
                    "[camera_driver] camera open failed (%d) — retrying in 2 s\n",
                    open_err);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }
        fprintf(stderr, "[camera_driver] camera opened (access=%s)\n",
                access_used == VmbAccessModeFull ? "FULL" : "READ");

        // ── Steps 5 & 6: Configure (write access only) ───────────────────────
        if (access_used == VmbAccessModeFull) {
            // Factory reset
            set_enum(handle, "UserSetSelector", "Default");
            VmbFeatureCommandRun(handle, "UserSetLoad");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Clear offsets before changing size
            set_int(handle, "OffsetX", 0);
            set_int(handle, "OffsetY", 0);

            // ROI
            set_int(handle, "Width",  DESIRED_WIDTH);
            set_int(handle, "Height", DESIRED_HEIGHT);

            // Centre ROI on sensor
            VmbInt64_t sw = get_int(handle, "SensorWidth");
            VmbInt64_t sh = get_int(handle, "SensorHeight");
            if (sw > 0 && sh > 0) {
                VmbInt64_t ox = ((sw - DESIRED_WIDTH)  / 2) & ~1;
                VmbInt64_t oy = ((sh - DESIRED_HEIGHT) / 2) & ~1;
                ox = std::max<VmbInt64_t>(ox, 0);
                oy = std::max<VmbInt64_t>(oy, 0);
                set_int(handle, "OffsetX", ox);
                set_int(handle, "OffsetY", oy);
                fprintf(stderr,
                        "[camera_driver] sensor=%ldx%ld  roi=%dx%d  offset=(%ld,%ld)\n",
                        (long)sw, (long)sh, DESIRED_WIDTH, DESIRED_HEIGHT,
                        (long)ox, (long)oy);
            }

            set_enum (handle, "PixelFormat",                "BayerRG8");
            set_bool (handle, "AcquisitionFrameRateEnable", VmbBoolTrue);
            set_float(handle, "AcquisitionFrameRate",       30.0);
            set_enum (handle, "ExposureAuto",               "Off");
            set_float(handle, "ExposureTime",               15000.0); // µs
            set_enum (handle, "GainAuto",                   "Continuous");
            set_enum (handle, "BalanceWhiteAuto",           "Continuous");
            set_int  (handle, "GevSCPSPacketSize",          1500);
            set_int  (handle, "GevSCPD",                    2000);
        }

        // ── Read back actual dimensions ───────────────────────────────────────
        const int act_w = static_cast<int>(get_int(handle, "Width",  DESIRED_WIDTH));
        const int act_h = static_cast<int>(get_int(handle, "Height", DESIRED_HEIGHT));
        fprintf(stderr,
                "[camera_driver] configured %dx%d @ 30 fps BayerRG8\n",
                act_w, act_h);

        // Resize scratch buffer if actual dims differ from desired
        const std::size_t act_rgb_bytes =
            static_cast<std::size_t>(act_w) * act_h * 3;
        if (rgb_buf.size() < act_rgb_bytes)
            rgb_buf.resize(act_rgb_bytes, 0);

        // ── Step 7: Announce 5 frame buffers ─────────────────────────────────
        VmbUint32_t payload_size = 0;
        VmbPayloadSizeGet(handle, &payload_size);

        constexpr int NUM_FRAMES = 5;
        std::vector<std::vector<uint8_t>> buffers(NUM_FRAMES,
            std::vector<uint8_t>(payload_size, 0));
        std::vector<VmbFrame_t> frames(NUM_FRAMES);

        for (int i = 0; i < NUM_FRAMES; ++i) {
            std::memset(&frames[i], 0, sizeof(VmbFrame_t));
            frames[i].buffer     = buffers[i].data();
            frames[i].bufferSize = payload_size;
            VmbFrameAnnounce(handle, &frames[i], sizeof(VmbFrame_t));
        }

        VmbCaptureStart(handle);
        for (int i = 0; i < NUM_FRAMES; ++i)
            VmbCaptureFrameQueue(handle, &frames[i], nullptr);

        // ── Step 8: AcquisitionStart + warm-up cycle ─────────────────────────
        VmbFeatureCommandRun(handle, "AcquisitionStart");

        fprintf(stderr, "[camera_driver] sensor warm-up — flushing buffers...\n");
        VmbFeatureCommandRun(handle, "AcquisitionStop");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        VmbCaptureEnd(handle);
        for (int i = 0; i < NUM_FRAMES; ++i)
            VmbFrameRevoke(handle, &frames[i]);
        for (auto &buf : buffers) buf.assign(buf.size(), 0);

        for (int i = 0; i < NUM_FRAMES; ++i)
            VmbFrameAnnounce(handle, &frames[i], sizeof(VmbFrame_t));
        VmbCaptureStart(handle);
        for (int i = 0; i < NUM_FRAMES; ++i)
            VmbCaptureFrameQueue(handle, &frames[i], nullptr);

        VmbFeatureCommandRun(handle, "AcquisitionStart");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        fprintf(stderr, "[camera_driver] warm-up done — streaming\n");

        // ── Step 9: Capture loop ──────────────────────────────────────────────
        int  frames_to_skip     = 10;
        int  consecutive_errors = 0;
        int  frame_index        = 0;
        auto last_frame_time    = std::chrono::steady_clock::now();

        while (bridge.running.load()) {
            VmbError_t result = VmbCaptureFrameWait(
                handle, &frames[frame_index], 50 /*ms*/);

            if (result != VmbErrorSuccess) {
                ++consecutive_errors;

                // ── Step 10: disconnect / sustained-error handling ────────────
                if (result == -19 /*disconnect*/) {
                    fprintf(stderr,
                            "[camera_driver] camera disconnected (err -19) "
                            "— restarting detection\n");
                    revoke_and_close(handle, frames);
                    break;
                }
                if (consecutive_errors > 100) {
                    fprintf(stderr,
                            "[camera_driver] %d consecutive errors "
                            "— restarting detection\n", consecutive_errors);
                    revoke_and_close(handle, frames);
                    break;
                }
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(
                        now - last_frame_time).count() > 5) {
                    fprintf(stderr,
                            "[camera_driver] no frames for 5 s "
                            "— restarting detection\n");
                    revoke_and_close(handle, frames);
                    break;
                }
                continue;
            }

            VmbFrame_t &fr = frames[frame_index];

            if (fr.receiveStatus != VmbFrameStatusComplete) {
                VmbCaptureFrameQueue(handle, &fr, nullptr);
                frame_index = (frame_index + 1) % NUM_FRAMES;
                continue;
            }

            consecutive_errors = 0;
            last_frame_time    = std::chrono::steady_clock::now();

            // Skip warm-up frames
            if (frames_to_skip > 0) {
                --frames_to_skip;
                VmbCaptureFrameQueue(handle, &fr, nullptr);
                frame_index = (frame_index + 1) % NUM_FRAMES;
                continue;
            }

            // Validate: skip if first 32 bytes are zero or >50 % of first 1024 are zero
            const uint8_t *raw  = static_cast<const uint8_t *>(fr.buffer);
            const std::size_t check =
                std::min<std::size_t>(1024,
                    static_cast<std::size_t>(act_w) * act_h);

            bool first32_zero = true;
            for (int k = 0; k < 32 && k < static_cast<int>(check); ++k)
                if (raw[k] != 0) { first32_zero = false; break; }

            std::size_t zeros = 0;
            for (std::size_t k = 0; k < check; ++k) if (raw[k] == 0) ++zeros;
            const bool mostly_zero = (static_cast<float>(zeros) / check) > 0.50f;

            if (first32_zero || mostly_zero) {
                VmbCaptureFrameQueue(handle, &fr, nullptr);
                frame_index = (frame_index + 1) % NUM_FRAMES;
                continue;
            }

            // Demosaic BayerRG8 → RGB, then convert to BGR for FAST-LIVO2
            bayer_rg8_to_rgb(raw, rgb_buf.data(), act_w, act_h);

            livo::ImageData data;
            data.timestamp = now_unix_seconds();
            {
                // Wrap scratch RGB buffer in a Mat, then convert to BGR clone
                cv::Mat rgb_view(act_h, act_w, CV_8UC3, rgb_buf.data(),
                                 static_cast<std::size_t>(act_w) * 3);
                cv::cvtColor(rgb_view, data.image, cv::COLOR_RGB2BGR);
            }

            {
                std::lock_guard<std::mutex> lk(bridge.img_mtx);
                if (bridge.img_queue.size() < livo::kImageQueueMax)
                    bridge.img_queue.push(std::move(data));
            }

            VmbCaptureFrameQueue(handle, &fr, nullptr);
            frame_index = (frame_index + 1) % NUM_FRAMES;
        } // inner capture loop

        // If bridge.running went false, also exit outer detection loop
        if (!bridge.running.load()) {
            revoke_and_close(handle, frames);
            break;
        }

    } // outer detection loop

    VmbShutdown();
    fprintf(stderr, "[camera_driver] shutdown complete\n");
}
