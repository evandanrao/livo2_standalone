# Prebuilt Libraries — Jetson NX (aarch64 / Ubuntu 18.04)

This directory holds the prebuilt Foxglove SDK libraries for the Jetson NX (`rad-obc`).

**Why prebuilt?**

- The Jetson has **no internet access** → `FetchContent` / Corrosion fail at configure time.
- The Jetson has **no Rust toolchain** → cannot build `libfoxglove.so` from source.
- `libfoxglove.so` built on a modern host (Ubuntu 22.04) requires **glibc 2.34**, but the Jetson runs Ubuntu 18.04 with **glibc 2.27** → runtime `GLIBC_2.34 not found` errors.
- The cross-compiled `libfoxglove_cpp.a` hits a **libstdc++ ABI mismatch**: the cross-compiler emits `C1Ev` references for `std::ostringstream` that GCC 7's `libstdc++.so.6` does not export as a dynamic symbol → native compilation on the Jetson is required.

---

## Artifacts

| File | Arch | Built by | How |
|---|---|---|---|
| `jetson_nx/lib/libfoxglove.so` | aarch64 | Ubuntu 18.04 Docker (dev machine) | Rust `cargo build`, cross-compiled via `Dockerfile.aarch64-bionic` |
| `jetson_nx/lib/libfoxglove_cpp.a` | aarch64 | GCC 7 **on the Jetson** | Native `g++` — cross-compilation ABI-incompatible |

**Runtime note:** `/media/internal_logs/` is mounted **noexec** on the Jetson. `libfoxglove.so` must be copied to `/home/skygauge/` (root fs, exec-capable) before the binary can run. `deploy.sh` does this automatically.

---

## Rebuild procedure

Two separate steps are needed. Read both before starting.

---

### Step 1 — Rebuild `libfoxglove.so` (dev machine, Docker)

The `Dockerfile.aarch64-bionic` in this directory builds `libfoxglove.so` inside an
Ubuntu 18.04 container so glibc symbols are pinned to ≤ 2.27.

**Prerequisites (one-time on dev machine):**
```bash
# Docker Engine 20+
docker --version

# The Cargo registry cache is volume-mounted to speed up subsequent builds.
# No other host setup needed — Rust and the cross-compiler live inside the image.
```

**Build:**
```bash
cd standalones/livo2

# Build the Docker image (only needed once, or after Dockerfile changes)
docker build \
  -f prebuilt/Dockerfile.aarch64-bionic \
  -t foxglove-bionic-build \
  prebuilt/

# Run the container — mounts cargo cache from host for faster rebuilds
docker run --rm \
  -v "$(pwd)/third_party/foxglove-sdk:/src" \
  -v "$(pwd)/prebuilt/jetson_nx/lib:/out" \
  -v "$HOME/.cargo/registry:/root/.cargo/registry" \
  -v "$HOME/.cargo/git:/root/.cargo/git" \
  foxglove-bionic-build
```

The container outputs `libfoxglove.so` directly into `prebuilt/jetson_nx/lib/`.
Takes ~2 min on first run (compiles `aws-lc-sys`), ~10 s after that (cached).

**Verify:**
```bash
# Must be aarch64, max glibc 2.27
file prebuilt/jetson_nx/lib/libfoxglove.so
# ELF 64-bit LSB shared object, ARM aarch64 ...

objdump -T prebuilt/jetson_nx/lib/libfoxglove.so \
  | grep -oP 'GLIBC_[0-9.]+' | sort -V | uniq | tail -3
# GLIBC_2.17  GLIBC_2.18  GLIBC_2.25   ← must be ≤ 2.27
```

---

### Step 2 — Rebuild `libfoxglove_cpp.a` (on the Jetson, natively)

**Why not cross-compile?**
The Ubuntu 18.04 GCC 7 cross-compiler emits an `undefined reference` to
`std::__cxx11::basic_ostringstream::basic_ostringstream()` (the no-arg `C1Ev` constructor).
The Jetson's `libstdc++.so.6` (GCC 7.5, GLIBCXX_3.4.25) does not export that symbol as a
dynamic symbol — only the with-mode-arg constructor exists in the DSO. The native compiler
generates the correct call site that resolves at link time without that symbol. **Always
compile `libfoxglove_cpp.a` natively on the Jetson.**

**Prerequisites (one-time, check these exist on Jetson):**
```bash
ssh rad-obc "g++ --version"
# g++ (Ubuntu/Linaro 7.5.0-3ubuntu1~18.04) 7.5.0  ← required

ssh rad-obc "pkg-config --exists eigen3 && echo OK"
# OK
```

**Step 2a — Prepare header-only dependencies (on dev machine):**

The foxglove C++ wrapper needs two header-only libraries that are not vendored in the repo.
Download them and rsync to the Jetson:
```bash
mkdir -p /tmp/foxglove_hdrs/nlohmann

# nlohmann/json v3.12.0 (matches what foxglove-sdk CMakeLists fetches)
wget -q https://github.com/nlohmann/json/releases/download/v3.12.0/json.hpp \
     -O /tmp/foxglove_hdrs/nlohmann/json.hpp

# base64 header (exact commit that foxglove-sdk uses)
wget -q https://raw.githubusercontent.com/tobiaslocker/base64/8d96a2a737ac1396304b1de289beb3a5ea0cb752/include/base64.hpp \
     -O /tmp/foxglove_hdrs/base64.hpp

rsync -av /tmp/foxglove_hdrs/ rad-obc:/tmp/foxglove_hdrs/
```

**Step 2b — Rsync the foxglove C++ sources to the Jetson:**
```bash
SDK=standalones/livo2/third_party/foxglove-sdk

rsync -av ${SDK}/cpp/foxglove/ rad-obc:/tmp/foxglove_cpp/
rsync -av ${SDK}/c/include/    rad-obc:/tmp/foxglove_c_include/
```

**Step 2c — Compile + archive on the Jetson:**
```bash
ssh rad-obc bash -s << 'EOF'
set -e
mkdir -p /tmp/foxglove_cpp_build

for f in /tmp/foxglove_cpp/src/*.cpp; do
  OBJ="/tmp/foxglove_cpp_build/$(basename "${f%.cpp}").o"
  g++ -std=c++17 -O2 -fPIC \
      -include cstddef \
      -I/tmp/foxglove_cpp/include \
      -I/tmp/foxglove_c_include \
      -I/tmp/foxglove_hdrs \
      -c "$f" -o "$OBJ"
done

ar rcs /tmp/libfoxglove_cpp_native.a /tmp/foxglove_cpp_build/*.o
echo "Built: $(ls -lh /tmp/libfoxglove_cpp_native.a)"
EOF
```

**Step 2d — Retrieve and verify:**
```bash
rsync -av rad-obc:/tmp/libfoxglove_cpp_native.a \
  standalones/livo2/prebuilt/jetson_nx/lib/libfoxglove_cpp.a

# Check: no undefined basic_ostringstream C1Ev symbols
nm standalones/livo2/prebuilt/jetson_nx/lib/libfoxglove_cpp.a \
  | grep 'basic_ostringstream..C[12]Ev'
# (no output = good)
```

---

### Step 3 — Commit and push

```bash
cd standalones/livo2

git add -f prebuilt/jetson_nx/lib/libfoxglove.so \
           prebuilt/jetson_nx/lib/libfoxglove_cpp.a

git commit -m "prebuilt: rebuild foxglove libs (foxglove-sdk $(cd third_party/foxglove-sdk && git describe --tags 2>/dev/null || git rev-parse --short HEAD))"

git pull --rebase origin main && git push origin main
```

The `.so` and `.a` are force-added because `*.so` / `*.a` patterns in `.gitignore` would
otherwise exclude them. This is intentional — they are large prebuilt binaries.

---

## Runtime on the Jetson

`/media/internal_logs/` is mounted **noexec** (`mount | grep internal_logs` to verify).
The dynamic linker cannot `mmap` executable pages from there, so `libfoxglove.so` must
live on the root filesystem. `deploy.sh` handles this by copying it to `/home/skygauge/`:

```
/home/skygauge/libfoxglove.so      ← loaded at runtime via LD_LIBRARY_PATH
/home/skygauge/hesai_livo2          ← binary
/home/skygauge/run_hesai_livo2.sh   ← launcher: sets LD_LIBRARY_PATH, then exec
```

Always run via the launcher:
```bash
/home/skygauge/run_hesai_livo2.sh \
  --config /home/skygauge/livo2_config/livo2.yaml \
  --camera /home/skygauge/livo2_config/camera.yaml
```

---

## How CMakeLists.txt uses the prebuilt libs

When `-DFOXGLOVE_PREBUILT_LIB_DIR=<path>` is given, the foxglove-sdk subdirectory is
**not added** — no Rust, no FetchContent, no internet needed on the Jetson:

```cmake
if(DEFINED FOXGLOVE_PREBUILT_LIB_DIR)
  add_library(foxglove-shared SHARED IMPORTED GLOBAL)
  set_target_properties(foxglove-shared PROPERTIES
      IMPORTED_LOCATION "${FOXGLOVE_PREBUILT_LIB_DIR}/libfoxglove.so")

  add_library(foxglove_cpp_shared STATIC IMPORTED GLOBAL)
  set_target_properties(foxglove_cpp_shared PROPERTIES
      IMPORTED_LOCATION "${FOXGLOVE_PREBUILT_LIB_DIR}/libfoxglove_cpp.a"
      INTERFACE_INCLUDE_DIRECTORIES "...cpp/foxglove/include;...c/include"
      INTERFACE_LINK_LIBRARIES "foxglove-shared;pthread;dl;m")
endif()
```

`deploy.sh --full` passes this automatically via:
```
-DFOXGLOVE_PREBUILT_LIB_DIR=/media/internal_logs/livo2/prebuilt/jetson_nx/lib
```

---

## Known gotchas

| Symptom | Root cause | Fix |
|---|---|---|
| `GLIBC_2.34 not found` at runtime | `.so` built on Ubuntu 22.04 host | Use `Dockerfile.aarch64-bionic` (Step 1) |
| `undefined reference to basic_ostringstream()` at link | Cross-compiled `.a` emits `C1Ev` ABI | Compile `.a` natively on Jetson (Step 2) |
| `failed to map segment from shared object` | `libfoxglove.so` on noexec `/media/` | Copy to `/home/skygauge/`, use launcher script |
| `cannot find -lgstreamer-1.0` | GStreamer at non-standard path | `PKG_CONFIG_PATH=/home/skygauge/gst/lib/aarch64-linux-gnu/pkgconfig` (set automatically by `deploy.sh`) |
| `libvikit_common.so: failed to map segment` | vikit_common built as SHARED on noexec fs | `vikit_common/CMakeLists.txt` uses `STATIC` — do not change it back |
