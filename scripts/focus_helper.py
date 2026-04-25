#!/usr/bin/env python3
"""
focus_helper.py

Live camera view with real-time Laplacian-variance sharpness overlay to help
mechanically focus the Allied Vision GigE payload camera lens.

Overlay:
  - Sharpness score (Laplacian variance) + RED/YELLOW/GREEN label
  - Sparkline history graph (top-right)

Controls:
    q / ESC / Ctrl-C  — quit

Usage:
    python3 focus_helper.py
    python3 focus_helper.py --width 1280 --height 720
"""

import argparse
import collections
import os
import sys
import time
import threading

# ── Auto-inject VimbaX lib path (same guard as payload_cam_stream.py) ─────────
_VIMBAX_LIB_DIR = "/opt/VimbaX_2025-3/api/lib"
_ld = os.environ.get("LD_LIBRARY_PATH", "")
if _VIMBAX_LIB_DIR not in _ld.split(":"):
    os.environ["LD_LIBRARY_PATH"] = _VIMBAX_LIB_DIR + (":" + _ld if _ld else "")
    os.execv(sys.executable, [sys.executable] + sys.argv)

import numpy as np  # noqa: E402

try:
    import cv2
except (ImportError, RuntimeError) as _cv2_err:
    if "ABI" in str(_cv2_err) or "numpy" in str(_cv2_err).lower():
        print(f"ERROR: cv2 numpy ABI mismatch: {_cv2_err}")
        print("  A system cv2 compiled against numpy 1.x is shadowing your install.")
        print("  Fix:  pip install --force-reinstall opencv-python")
    else:
        print(f"ERROR: failed to import cv2: {_cv2_err}")
        print("  pip install opencv-python")
    sys.exit(1)

# ── Import shared helpers from payload_cam_stream (same directory) ─────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from payload_cam_stream import (  # noqa: E402
    bayer_rg8_to_rgb, _is_simulator, _configure_camera, _is_valid_frame,
)

try:
    from vmbpy import (
        VmbSystem, Frame, FrameStatus, AccessMode,
        VmbCameraError, VmbTimeout,
    )
except (ImportError, OSError) as _e:
    print(f"ERROR: vmbpy not available: {_e}")
    sys.exit(1)


# ── Sharpness thresholds ──────────────────────────────────────────────────────
# Laplacian variance: higher = sharper.  Tune these for your lens/scene.
_SHARP_LOW  = 100.0   # below → RED (blurry)
_SHARP_HIGH = 500.0   # above → GREEN (sharp)


def _laplacian_variance(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


# ── Overlay rendering ─────────────────────────────────────────────────────────
def _draw_overlay(bgr: np.ndarray, score: float,
                  history: collections.deque) -> None:
    h, w = bgr.shape[:2]

    if score < _SHARP_LOW:
        colour = (0, 0, 220)
        label  = "BLURRY"
    elif score < _SHARP_HIGH:
        colour = (0, 200, 220)
        label  = "OK"
    else:
        colour = (0, 200, 60)
        label  = "SHARP"

    # Score text (black shadow + colour)
    txt = f"Sharpness: {score:,.0f}  [{label}]"
    cv2.putText(bgr, txt, (12, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(bgr, txt, (12, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2, cv2.LINE_AA)

    # Quit hint (bottom-left)
    cv2.putText(bgr, "q / ESC : quit", (12, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

    # Sparkline graph (top-right corner)
    if len(history) >= 2:
        pts = list(history)
        mn  = min(pts)
        mx  = max(pts) + 1e-6
        gw, gh = 200, 60
        gx = w - gw - 10
        gy = 10
        # Dark background
        cv2.rectangle(bgr,
                      (gx - 2, gy - 2), (gx + gw + 2, gy + gh + 2),
                      (40, 40, 40), -1)
        n = len(pts)
        for i in range(n - 1):
            x0 = gx + int(i       * gw / (n - 1))
            x1 = gx + int((i + 1) * gw / (n - 1))
            y0 = gy + gh - int((pts[i]     - mn) / (mx - mn) * gh)
            y1 = gy + gh - int((pts[i + 1] - mn) / (mx - mn) * gh)
            cv2.line(bgr, (x0, y0), (x1, y1), colour, 1)


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Camera focus helper — sharpness overlay (vmbpy)"
    )
    parser.add_argument("--width",   type=int, default=1280, metavar="PX")
    parser.add_argument("--height",  type=int, default=720,  metavar="PX")
    parser.add_argument("--timeout", type=int, default=60,   metavar="SEC",
                        help="Seconds to wait for camera before giving up")
    args = parser.parse_args()

    stop_event = threading.Event()
    history    = collections.deque(maxlen=80)

    with VmbSystem.get_instance() as vmb:
        print("VimbaX SDK started — focus_helper")
        deadline = time.monotonic() + args.timeout

        while not stop_event.is_set():
            time.sleep(0.3)
            try:
                vmb.get_all_transport_layers()
            except Exception:
                pass

            cameras   = vmb.get_all_cameras()
            real_cams = [c for c in cameras if not _is_simulator(c)]

            if not real_cams:
                if time.monotonic() >= deadline:
                    print("Timeout waiting for camera.")
                    break
                continue

            cam = real_cams[0]
            print(f"Camera: {cam.get_name()}  ({cam.get_id()})")

            permitted = cam.get_permitted_access_modes()
            if AccessMode.Full not in permitted:
                print("  FULL access not available — waiting 3 s")
                time.sleep(3.0)
                continue

            cam.set_access_mode(AccessMode.Full)

            win = "Focus Helper  —  q / ESC = quit"
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)

            try:
                with cam:
                    act_w, act_h = _configure_camera(cam, args.width, args.height)
                    print(f"  Streaming {act_w}x{act_h}")
                    print("  Adjust lens until sharpness turns GREEN, then lock focus.")

                    # Warm-up
                    for _ in range(10):
                        try:
                            cam.get_frame(timeout_ms=500)
                        except Exception:
                            pass
                    time.sleep(1.0)

                    while not stop_event.is_set():
                        try:
                            frame: Frame = cam.get_frame(timeout_ms=50)
                        except VmbTimeout:
                            continue
                        except Exception as exc:
                            if "-19" in str(exc):
                                print("  Camera disconnected — exiting")
                                stop_event.set()
                                break
                            continue

                        if frame.get_status() != FrameStatus.Complete:
                            continue

                        raw = frame.as_numpy_ndarray()
                        if not _is_valid_frame(raw):
                            continue

                        rgb  = bayer_rg8_to_rgb(raw, act_w, act_h)
                        bgr  = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

                        score = _laplacian_variance(gray)
                        history.append(score)

                        _draw_overlay(bgr, score, history)
                        cv2.imshow(win, bgr)

                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), 27):   # q or ESC
                            stop_event.set()

            except VmbCameraError as exc:
                print(f"  Camera error: {exc}")

            cv2.destroyAllWindows()
            break

    print("focus_helper: done")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted")
