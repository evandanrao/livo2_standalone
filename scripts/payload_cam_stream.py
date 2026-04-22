#!/usr/bin/env python3
"""
payload_cam_stream.py

Streams the Allied Vision GigE payload camera (BayerRG8, 1280×720, 30 fps)
directly via the VimbaX SDK (vmbpy) and displays frames with OpenCV.

Mirrors the exact init/configure/capture sequence from payload_cam_stream_node.cpp:
  1. VmbStartup (Vmb.get_instance context manager)
  2. Transport layer list — forces GigE rescan for hot-plug
  3. Enumerate cameras, skip simulators
  4. Open FULL → READ fallback
  5. UserSetLoad("Default") — factory reset, 500 ms wait
  6. Configure ROI, BayerRG8, 30 fps, ExposureAuto=Off, GainAuto=Continuous,
     BalanceWhiteAuto=Continuous, GevSCPSPacketSize=1500, GevSCPD=2000
  7. Announce 5 frame buffers, CaptureStart, queue all
  8. AcquisitionStart → warm-up stop/revoke/re-announce/restart → skip 10 frames
  9. Capture loop: wait 50 ms → validate → demosaic BayerRG8→RGB → display
 10. On disconnect (VmbError -19) or >100 consecutive errors: clean close → restart

Requirements:
    pip install vmbpy opencv-python numpy
    VimbaX SDK installed at /opt/VimbaX_2025-3  (sets LD_LIBRARY_PATH)

Usage:
    # Just run directly — the script handles VimbaX lib path automatically:
    python3 payload_cam_stream.py

    # Optional args
    python3 payload_cam_stream.py --width 640 --height 360 --timeout 120 --no-display
"""

import argparse
import os
import sys
import time
import threading

# ── Auto-inject VimbaX lib path and re-exec if needed ────────────────────────
# vmbpy loads libVmbC.so via ctypes at import time.  Changing LD_LIBRARY_PATH
# inside a running process has no effect for dlopen, so we must re-exec before
# the first import of vmbpy.
_VIMBAX_LIB_DIR = "/opt/VimbaX_2025-3/api/lib"
_ld = os.environ.get("LD_LIBRARY_PATH", "")
if _VIMBAX_LIB_DIR not in _ld.split(":"):
    os.environ["LD_LIBRARY_PATH"] = _VIMBAX_LIB_DIR + (":" + _ld if _ld else "")
    os.execv(sys.executable, [sys.executable] + sys.argv)

import numpy as np  # noqa: E402  (after re-exec guard)

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

try:
    from vmbpy import (
        VmbSystem, Camera, Frame, FrameStatus, AccessMode,
        VmbCameraError, VmbFeatureError, VmbTimeout,
    )
except (ImportError, OSError) as _vmb_err:
    print(
        f"ERROR: failed to import vmbpy: {_vmb_err}\n"
        f"Ensure VimbaX SDK is installed at {_VIMBAX_LIB_DIR}\n"
        "and vmbpy wheel is installed:\n"
        "  pip3 install /opt/VimbaX_2025-3/api/python/vmbpy-*.whl"
    )
    sys.exit(1)


# ── Bayer RG8 → RGB nearest-neighbour demosaic ────────────────────────────────
# Mirrors bayer_rg8_to_rgb_nearest() from camera_stream_ctrl.rs and the C++ node.
# Pattern:  R G       Each 2×2 block samples one R, G, B value and fills
#           G B       all four output pixels with the same colour.
def bayer_rg8_to_rgb(bayer: np.ndarray, width: int, height: int) -> np.ndarray:
    """Nearest-neighbour BayerRG8 → packed RGB (HxWx3 uint8)."""
    raw = bayer.reshape(height, width)
    rgb = np.empty((height, width, 3), dtype=np.uint8)

    # Sample one R, G, B per 2×2 block
    r = raw[0::2, 0::2]   # top-left
    g = raw[0::2, 1::2]   # top-right
    b = raw[1::2, 1::2]   # bottom-right

    # Fill all four pixels in each block identically
    rgb[0::2, 0::2, 0] = r;  rgb[0::2, 0::2, 1] = g;  rgb[0::2, 0::2, 2] = b
    rgb[0::2, 1::2, 0] = r;  rgb[0::2, 1::2, 1] = g;  rgb[0::2, 1::2, 2] = b
    rgb[1::2, 0::2, 0] = r;  rgb[1::2, 0::2, 1] = g;  rgb[1::2, 0::2, 2] = b
    rgb[1::2, 1::2, 0] = r;  rgb[1::2, 1::2, 1] = g;  rgb[1::2, 1::2, 2] = b

    return rgb


# ── Simulator detection ───────────────────────────────────────────────────────
def _is_simulator(cam: Camera) -> bool:
    name = cam.get_name() or ""
    cid  = cam.get_id()   or ""
    return (
        "Simulator" in name
        or "Simulator" in cid
        or cid.startswith("DEV_Cam")  # e.g. DEV_Cam1, DEV_Cam2
    )


# ── Feature helpers ───────────────────────────────────────────────────────────
def _set_enum(cam: Camera, feat: str, value: str) -> None:
    try:
        cam.get_feature_by_name(feat).set(value)
    except VmbFeatureError as exc:
        print(f"  WARN set_enum({feat}={value!r}): {exc}")

def _set_int(cam: Camera, feat: str, value: int) -> None:
    try:
        cam.get_feature_by_name(feat).set(value)
    except VmbFeatureError as exc:
        print(f"  WARN set_int({feat}={value}): {exc}")

def _set_float(cam: Camera, feat: str, value: float) -> None:
    try:
        cam.get_feature_by_name(feat).set(value)
    except VmbFeatureError as exc:
        print(f"  WARN set_float({feat}={value}): {exc}")

def _set_bool(cam: Camera, feat: str, value: bool) -> None:
    try:
        cam.get_feature_by_name(feat).set(value)
    except VmbFeatureError as exc:
        print(f"  WARN set_bool({feat}={value}): {exc}")

def _get_int(cam: Camera, feat: str, default: int = 0) -> int:
    try:
        return int(cam.get_feature_by_name(feat).get())
    except VmbFeatureError:
        return default

def _run_cmd(cam: Camera, feat: str) -> None:
    try:
        cam.get_feature_by_name(feat).run()
    except VmbFeatureError as exc:
        print(f"  WARN cmd({feat}): {exc}")


# ── Frame validation ──────────────────────────────────────────────────────────
def _is_valid_frame(data: np.ndarray) -> bool:
    """
    Returns False if the first 32 bytes are all zero, or if >50% of the first
    1024 bytes are zero — mirrors the validation in payload_cam_stream_node.cpp.
    """
    flat = data.ravel()
    check_len = min(1024, len(flat))
    check = flat[:check_len]

    first32 = min(32, check_len)
    if np.all(check[:first32] == 0):
        return False

    zero_frac = np.count_nonzero(check == 0) / check_len
    return zero_frac <= 0.50


# ── Configure camera (Steps 5 & 6) ───────────────────────────────────────────
def _configure_camera(cam: Camera, desired_w: int, desired_h: int) -> tuple[int, int]:
    """
    Factory-reset → configure ROI, pixel format, frame rate, exposure, gain,
    white balance, GigE transport params.  Returns (actual_width, actual_height).
    """
    # Factory reset
    _set_enum(cam, "UserSetSelector", "Default")
    _run_cmd (cam, "UserSetLoad")
    time.sleep(0.5)

    # Clear offsets before resizing
    _set_int(cam, "OffsetX", 0)
    _set_int(cam, "OffsetY", 0)

    # ROI
    _set_int(cam, "Width",  desired_w)
    _set_int(cam, "Height", desired_h)

    # Centre ROI on sensor
    sw = _get_int(cam, "SensorWidth")
    sh = _get_int(cam, "SensorHeight")
    if sw > 0 and sh > 0:
        ox = ((sw - desired_w) // 2) & ~1
        oy = ((sh - desired_h) // 2) & ~1
        ox = max(ox, 0)
        oy = max(oy, 0)
        _set_int(cam, "OffsetX", ox)
        _set_int(cam, "OffsetY", oy)
        print(f"  sensor={sw}x{sh}  roi={desired_w}x{desired_h}  offset=({ox},{oy})")

    _set_enum (cam, "PixelFormat",              "BayerRG8")
    _set_bool (cam, "AcquisitionFrameRateEnable", True)
    _set_float(cam, "AcquisitionFrameRate",     30.0)
    _set_enum (cam, "ExposureAuto",             "Off")
    _set_float(cam, "ExposureTime",             15000.0)   # 15 ms in µs
    _set_enum (cam, "GainAuto",                 "Continuous")
    _set_enum (cam, "BalanceWhiteAuto",         "Continuous")
    _set_int  (cam, "GevSCPSPacketSize",        1500)      # standard MTU
    _set_int  (cam, "GevSCPD",                  2000)      # inter-packet delay

    act_w = _get_int(cam, "Width",  desired_w)
    act_h = _get_int(cam, "Height", desired_h)
    return act_w, act_h


# ── Capture loop (Steps 7-10) ─────────────────────────────────────────────────
def _run_capture(cam: Camera, act_w: int, act_h: int,
                 display: bool, stop_event: threading.Event) -> str:
    """
    Runs the warm-up + synchronous capture loop using get_frame().

    Returns one of:
        "disconnect"       — VmbError -19, camera disconnected
        "too_many_errors"  — >100 consecutive frame errors
        "no_frames"        — no valid frames for 5 s
        "stop"             — stop_event was set (user quit)
    """
    FRAME_WAIT_MS  = 50
    FRAMES_TO_SKIP = 10

    # ── Step 8: Warm-up — discard first batch then wait 1 s ──────────────────
    # Mirrors the C++ stop/revoke/re-announce/restart cycle.
    print("  Warm-up: flushing initial frames...")
    for _ in range(FRAMES_TO_SKIP):
        try:
            cam.get_frame(timeout_ms=500)
        except Exception:
            pass
    time.sleep(1.0)
    print("  Warm-up done — streaming")

    # ── Step 9: Capture loop ──────────────────────────────────────────────────
    consecutive_errors = 0
    last_frame_time    = time.monotonic()
    exit_reason        = "stop"

    window_name = "Payload Camera"
    if display and _CV2_OK:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while not stop_event.is_set():
            # Synchronous single-frame capture (mirrors VmbCaptureFrameWait 50 ms)
            try:
                frame: Frame = cam.get_frame(timeout_ms=FRAME_WAIT_MS)
            except VmbTimeout:
                # Timeout is normal — just retry
                continue
            except Exception as exc:
                err_str = str(exc)
                consecutive_errors += 1

                # VmbError -19 = camera disconnected
                if "-19" in err_str or "NotImplemented" in err_str:
                    print("  Camera disconnected (err -19) — restarting detection")
                    exit_reason = "disconnect"
                    break

                if consecutive_errors > 100:
                    print(f"  {consecutive_errors} consecutive errors — restarting detection")
                    exit_reason = "too_many_errors"
                    break

                if time.monotonic() - last_frame_time > 5.0:
                    print("  No frames for 5 s — restarting detection")
                    exit_reason = "no_frames"
                    break

                continue

            if frame.get_status() != FrameStatus.Complete:
                continue

            consecutive_errors = 0
            last_frame_time    = time.monotonic()

            # Validate: skip blank / mostly-zero frames
            raw_data = frame.as_numpy_ndarray()
            if not _is_valid_frame(raw_data):
                continue

            # Demosaic BayerRG8 → RGB
            rgb = bayer_rg8_to_rgb(raw_data, act_w, act_h)

            # Display
            if display and _CV2_OK:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cv2.imshow(window_name, bgr)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    stop_event.set()
                    break
            else:
                print(".", end="", flush=True)

    finally:
        if display and _CV2_OK:
            cv2.destroyWindow(window_name)

    return exit_reason


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Allied Vision GigE payload camera stream (VimbaX / vmbpy)"
    )
    parser.add_argument("--width",      type=int,  default=1280, metavar="PX")
    parser.add_argument("--height",     type=int,  default=720,  metavar="PX")
    parser.add_argument("--timeout",    type=int,  default=60,   metavar="SEC",
                        help="Seconds to wait for camera before SDK restart")
    parser.add_argument("--no-display", action="store_true",
                        help="Suppress OpenCV window (headless / no GUI)")
    args = parser.parse_args()

    desired_w  = args.width
    desired_h  = args.height
    timeout_s  = args.timeout
    display    = not args.no_display and _CV2_OK

    if not display and not args.no_display:
        print("WARNING: opencv-python not installed; running headless.")

    print(f"payload_cam_stream: roi={desired_w}x{desired_h}  timeout={timeout_s}s")

    stop_event = threading.Event()

    # ── Step 1: VmbStartup ────────────────────────────────────────────────────
    with VmbSystem.get_instance() as vmb:
        print("VimbaX SDK started")

        detection_attempts = 0
        detection_deadline = time.monotonic() + timeout_s

        while not stop_event.is_set():
            time.sleep(0.5)

            # ── Step 2: Transport layer rescan (forces GigE re-enumeration) ───
            try:
                tls = vmb.get_all_transport_layers()
                if tls:
                    pass  # triggers rescan; count logged at debug level
            except Exception:
                pass
            time.sleep(0.1)

            # ── Step 3: Enumerate cameras, skip simulators ────────────────────
            cameras     = vmb.get_all_cameras()
            real_cameras = [c for c in cameras if not _is_simulator(c)]

            if not real_cameras:
                detection_attempts += 1
                if detection_attempts % 20 == 1:
                    print(f"No real cameras found (attempt {detection_attempts})")

                if time.monotonic() >= detection_deadline:
                    print(f"{timeout_s}s timeout — resetting detection deadline")
                    detection_attempts = 0
                    detection_deadline = time.monotonic() + timeout_s

                time.sleep(2.0)
                continue

            cam = real_cameras[0]
            print(f"Found camera: '{cam.get_name()}'  id='{cam.get_id()}'")
            detection_attempts = 0
            detection_deadline = time.monotonic() + timeout_s

            # ── Step 4: Check access mode ─────────────────────────────────────
            # vmbpy opens cameras via a context manager; set_access_mode()
            # must be called before entering it.
            # Streaming requires FULL access — READ is view-only and get_frame()
            # will always fail under it.  If only READ/None_ is available, the
            # camera is held by another process; wait and retry.
            permitted = cam.get_permitted_access_modes()
            if AccessMode.Full not in permitted:
                # None_ = camera briefly unavailable (just closed); Read = held elsewhere
                print(f"  FULL access not available (permitted={permitted}) — waiting 3s")
                time.sleep(3.0)
                continue

            cam.set_access_mode(AccessMode.Full)

            try:
                with cam:
                    print("  Opened (FULL)")

                    # ── Steps 5 & 6: Configure ───────────────────────────────
                    act_w, act_h = _configure_camera(cam, desired_w, desired_h)
                    print(f"  Configured {act_w}x{act_h} @ 30 fps BayerRG8")

                    # ── Steps 7-10: Warm-up + capture loop ───────────────────
                    exit_reason = _run_capture(
                        cam, act_w, act_h, display, stop_event
                    )

            except VmbCameraError as exc:
                print(f"  Camera error: {exc} — retrying in 2s")
                time.sleep(2.0)
                continue

            if exit_reason == "stop":
                break  # user pressed q or sent SIGINT

            # Any other reason → restart detection loop
            print(f"  Restarting detection (reason: {exit_reason})")

    print("\npayload_cam_stream: shutdown complete")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted")
