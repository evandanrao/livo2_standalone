#!/usr/bin/env bash
# deploy.sh — rsync livo2 source to Jetson NX, build, deploy binary + config.
#
# Usage:
#   ./scripts/deploy.sh [--full] [jetson_host]
#
#   --full       First-time setup: rsync everything (incl. submodules),
#                ensure Rust is installed on Jetson, run cmake configure,
#                then build.  Run once per fresh Jetson checkout.
#   jetson_host  SSH host alias (default: rad-obc)
#
# Incremental (default): rsyncs changed source/include/CMakeLists.txt only,
#   then runs cmake --build -j4.  Fast for day-to-day iterations.
#
# Prerequisites on Jetson (one-time, already satisfied on rad-obc):
#   - GStreamer dev install at /home/skygauge/gst/  (already present)
#   - PCL, OpenCV, Eigen, Boost packages from apt
#   - Rust toolchain (installed automatically by --full, or manually:
#       curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y)
#
# GStreamer note: the Jetson has a local GStreamer build under ~/gst/ (no apt
# dev packages needed).  PKG_CONFIG_PATH is set automatically during cmake.

set -euo pipefail

FULL=0
JETSON_HOST="rad-obc"

for arg in "$@"; do
    case "$arg" in
        --full) FULL=1 ;;
        *)      JETSON_HOST="$arg" ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

REMOTE_DIR="/media/internal_logs/livo2"
DEPLOY_BIN="/home/skygauge/hesai_livo2"
DEPLOY_CFG="/home/skygauge/config"

# ─────────────────────────────────────────────────────────────────────────────
# FULL: first-time setup — sync everything, configure cmake, build
# ─────────────────────────────────────────────────────────────────────────────
# The prebuilt aarch64 libfoxglove.so lives in prebuilt/jetson_nx/lib/ (cross-compiled
# on dev machine). This means the Jetson needs NO Rust and NO internet access during build.
PREBUILT_LIB_DIR="${REMOTE_DIR}/prebuilt/jetson_nx/lib"

if [[ "$FULL" -eq 1 ]]; then
    echo "=== [FULL 1/3] Creating remote directory on ${JETSON_HOST} ==="
    ssh "${JETSON_HOST}" "mkdir -p ${REMOTE_DIR}/build"

    echo "=== [FULL 2/3] Rsyncing full source tree to ${JETSON_HOST}:${REMOTE_DIR} ==="
    rsync -av --progress \
        --exclude='build/' \
        --exclude='.git/' \
        --exclude='third_party/foxglove-sdk/target/' \
        --exclude='*.o' \
        "${REPO_DIR}/" \
        "${JETSON_HOST}:${REMOTE_DIR}/"

    echo "=== [FULL 3/3] CMake configure + build on ${JETSON_HOST} ==="
    ssh "${JETSON_HOST}" bash -s <<ENDSSH
        set -e
        export PKG_CONFIG_PATH="/home/skygauge/gst/lib/aarch64-linux-gnu/pkgconfig:\$PKG_CONFIG_PATH"
        cd ${REMOTE_DIR}
        cmake -B build -DCMAKE_BUILD_TYPE=Release -Wno-dev \
            -DFOXGLOVE_PREBUILT_LIB_DIR=${PREBUILT_LIB_DIR}
        cmake --build build -j4 2>&1 | tail -30
        echo "Build complete: \$(ls -lh build/hesai_livo2)"
ENDSSH

# ─────────────────────────────────────────────────────────────────────────────
# INCREMENTAL (default): sync changed files, rebuild
# ─────────────────────────────────────────────────────────────────────────────
else
    echo "=== [1/2] Rsyncing changed source files to ${JETSON_HOST}:${REMOTE_DIR} ==="

    rsync -av \
        "${REPO_DIR}/src/" \
        "${JETSON_HOST}:${REMOTE_DIR}/src/"

    rsync -av \
        "${REPO_DIR}/include/" \
        "${JETSON_HOST}:${REMOTE_DIR}/include/"

    rsync -av \
        "${REPO_DIR}/prebuilt/" \
        "${JETSON_HOST}:${REMOTE_DIR}/prebuilt/"

    rsync -av \
        "${REPO_DIR}/CMakeLists.txt" \
        "${JETSON_HOST}:${REMOTE_DIR}/CMakeLists.txt"

    echo "=== [2/2] Building on ${JETSON_HOST} ==="
    ssh "${JETSON_HOST}" bash -s <<ENDSSH
        set -e
        export PKG_CONFIG_PATH="/home/skygauge/gst/lib/aarch64-linux-gnu/pkgconfig:\$PKG_CONFIG_PATH"
        cmake --build ${REMOTE_DIR}/build -j4 2>&1 | tail -20
        echo "Build complete: \$(ls -lh ${REMOTE_DIR}/build/hesai_livo2)"
ENDSSH

fi

# ─────────────────────────────────────────────────────────────────────────────
# Deploy binary + config
# ─────────────────────────────────────────────────────────────────────────────
# libfoxglove.so lives on /media/internal_logs which is mounted noexec — the
# dynamic linker cannot load it from there at runtime.  Copy it to /home/skygauge/
# (root filesystem, exec-capable) and set LD_LIBRARY_PATH via a launcher script.
echo "=== Deploying binary to ${JETSON_HOST}:${DEPLOY_BIN} ==="
ssh "${JETSON_HOST}" "cp ${REMOTE_DIR}/build/hesai_livo2 ${DEPLOY_BIN}.new && chmod +x ${DEPLOY_BIN}.new && mv -f ${DEPLOY_BIN}.new ${DEPLOY_BIN}"

echo "=== Deploying libfoxglove.so to ${JETSON_HOST}:/home/skygauge/ ==="
rsync -av \
    "${REPO_DIR}/prebuilt/jetson_nx/lib/libfoxglove.so" \
    "${JETSON_HOST}:/home/skygauge/libfoxglove.so"

DEPLOY_LAUNCHER="/home/skygauge/run_hesai_livo2.sh"
echo "=== Creating launcher script ${JETSON_HOST}:${DEPLOY_LAUNCHER} ==="
ssh "${JETSON_HOST}" cat > /dev/null << ENDSSH
cat > "${DEPLOY_LAUNCHER}" << 'SCRIPT'
#!/bin/bash
# Auto-generated by deploy.sh
# Needed because libfoxglove.so is on a noexec filesystem at runtime.
export LD_LIBRARY_PATH=/home/skygauge:\${LD_LIBRARY_PATH}
export PKG_CONFIG_PATH=/home/skygauge/gst/lib/aarch64-linux-gnu/pkgconfig:\${PKG_CONFIG_PATH}
exec /home/skygauge/hesai_livo2 "\$@"
SCRIPT
chmod +x "${DEPLOY_LAUNCHER}"
ENDSSH
ssh "${JETSON_HOST}" bash -c "cat > '${DEPLOY_LAUNCHER}' << 'SCRIPT'
#!/bin/bash
# Auto-generated by deploy.sh
# Needed: libfoxglove.so is on a noexec filesystem; load it from /home/skygauge/.
# LD_LIBRARY_PATH must also include the custom GStreamer lib dir so that
# GStreamer plugins (videoconvert etc.) resolve against the correct libgstvideo.
export LD_LIBRARY_PATH=/home/skygauge:/home/skygauge/gst/lib/aarch64-linux-gnu:\${LD_LIBRARY_PATH:-}
exec /home/skygauge/hesai_livo2 \"\$@\"
SCRIPT
chmod +x '${DEPLOY_LAUNCHER}'"

echo "=== Deploying config to ${JETSON_HOST}:${DEPLOY_CFG}/ ==="
ssh "${JETSON_HOST}" "mkdir -p ${DEPLOY_CFG}"
rsync -av \
    "${REPO_DIR}/config/livo2.yaml" \
    "${JETSON_HOST}:${DEPLOY_CFG}/livo2.yaml"
rsync -av \
    "${REPO_DIR}/config/camera.yaml" \
    "${JETSON_HOST}:${DEPLOY_CFG}/camera.yaml"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Done."
echo "  Binary:       ${DEPLOY_BIN}"
echo "  Launcher:     ${DEPLOY_LAUNCHER}  ← use this to run"
echo "  Config:       ${DEPLOY_CFG}/"
echo ""
echo "On Jetson run:"
echo "  ${DEPLOY_LAUNCHER} --config ${DEPLOY_CFG}/livo2.yaml --camera ${DEPLOY_CFG}/camera.yaml"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
