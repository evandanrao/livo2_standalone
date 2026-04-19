# Prebuilt — Jetson Xavier NX (Ubuntu 20.04 / JetPack 5.x, aarch64)

Place aarch64 build artefacts here after cross-compiling or building natively on Jetson.

## Contents (once populated)

| Path | Description |
|---|---|
| `bin/hesai_livo2` | Main binary (aarch64, dynamically linked) |
| `lib/libfoxglove.so` | Foxglove C core (Rust, aarch64) |
| `lib/libfoxglove_cpp_shared.so` | Foxglove C++ wrapper (aarch64) |

Standard dependencies (install via apt on Jetson):
```bash
sudo apt-get install -y \
    libpcl-dev libopencv-dev libeigen3-dev \
    libboost-all-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

## Building natively on Jetson (recommended for first build)

```bash
# 1. Clone repo + submodules
git clone --recurse-submodules git@github.com:evandanrao/livo2_standalone.git /media/internal_logs/livo2

# 2. Install Rust (needed for foxglove-sdk)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source ~/.cargo/env

# 3. Build
cd /media/internal_logs/livo2
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

## Using prebuilt foxglove libs (skip Rust on Jetson)

If you don't want Rust on the Jetson, copy `libfoxglove.so` and
`libfoxglove_cpp_shared.so` built for aarch64 into `prebuilt/jetson_nx/lib/`,
then configure cmake with:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release \
    -DFOXGLOVE_PREBUILT_LIB_DIR=/media/internal_logs/livo2/prebuilt/jetson_nx/lib
cmake --build build -j4
```

## Run

```bash
/home/skygauge/hesai_livo2 \
    --config /home/skygauge/livo2_config/livo2.yaml \
    --camera /home/skygauge/livo2_config/camera.yaml
```
