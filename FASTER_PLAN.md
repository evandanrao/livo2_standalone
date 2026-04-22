# FASTER Integration Plan
## Branch: `faster`  |  Base: `bb29b8e` (Jetson NX: CUDA GPU parser, global map, PCD export, UDP publisher)

---

## 1. Overview

Integrate the [FASTER](https://github.com/mit-acl/faster) trajectory planner natively into livo2 without ROS, replacing the EGO-Planner-v2 integration that lives on `main`. The solver replacement (Gurobi → HiGHS) is the core non-trivial engineering problem. Everything else is wiring.

### What FASTER does
FASTER (Fast and Safe Trajectory planner for fREe-space Replanning) plans minimum-jerk polynomial trajectories through free space. At each replan cycle it:
1. Runs JPS (Jump Point Search) to get a collision-free geometric path through the occupied/unknown voxel map.
2. Decomposes free space along that path into a sequence of convex polytopes (SFC — Safe Flight Corridor) using DecompUtil.
3. Solves a **Mixed-Integer Quadratic Program (MIQP)**: find cubic Bézier polynomial segments minimizing ∫jerk² subject to:
   - Dynamic constraints (continuity of pos/vel/accel between segments)
   - Velocity, acceleration, jerk bounds
   - Each segment's 4 Bézier control points must lie inside at least one polytope in the SFC (via binary indicator variables)
4. Tracks the "whole" trajectory (planned into unknown space) plus a "safe" fallback (only through known-free space). The safe path is always a rescue option if the whole path fails.

### Why HiGHS instead of Gurobi
Gurobi requires a commercial or academic license. HiGHS (Huangfu, Hall — U. Edinburgh) is a fully open-source LP/MIQP/MIP solver available as `libhighs-dev` on Ubuntu 22.04 and with an aarch64 build for Jetson. It directly supports mixed-integer quadratic programs using the same mathematical structure that FASTER's Gurobi formulation uses.

HiGHS is slower than Gurobi (roughly 3–5× in benchmarks on small MIQPs), but FASTER's problem sizes are small (N=10 segments, ~5 polytopes) so the absolute solve time stays well under the 50ms replan budget.

---

## 2. Architecture After Integration

```
┌─────────────────────────────────────────────────────────────┐
│                        livo2 process                        │
│                                                             │
│  ┌──────────────┐    pose_queue     ┌──────────────────┐   │
│  │  LIVMapper   │──────────────────▶│ foxglove_streamer│   │
│  │  (SLAM core) │    cloud_queue    │  (WebSocket srv) │   │
│  └──────┬───────┘──────────────────▶└──────────────────┘   │
│         │                                    ▲              │
│         │ faster_pose_queue                  │ /planner/goal│
│         │ faster_cloud_queue                 │ (JSON)       │
│         ▼                                    │              │
│  ┌──────────────┐                   ┌────────┴──────────┐  │
│  │   FASTER     │◀──────────────────│  Foxglove client  │  │
│  │   planner    │   goal (state)    │  (user sends goal)│  │
│  │              │                   └───────────────────┘  │
│  │  JPS3D       │                                          │
│  │  DecompUtil  │──── faster_cmd_queue ──────────────────▶ │
│  │  HiGHS MIQP  │    (position commands)   foxglove_str   │
│  └──────────────┘                          /faster/cmd    │
│                                            /faster/traj   │
│                                            /faster/jps    │
└─────────────────────────────────────────────────────────────┘
         │ UDP
         ▼
  lidar_publisher  (CMD72 odom, CMD73 scan, CMD74 path, CMD75 faster_cmd)
```

### Data flows
| Queue | Producer | Consumer | Type |
|---|---|---|---|
| `faster_pose_queue` | `LIVMapper::publish_odometry()` | `FasterThread` | `PoseData` |
| `faster_cloud_queue` | `LIVMapper` (map update) | `FasterThread` | `CloudData` (accumulated) |
| `faster_goal_queue` | `foxglove_streamer` (`/planner/goal` ClientPublish) | `FasterThread` | `GoalData` (`b_types.hpp`) |
| `faster_cmd_queue` | `FasterThread` (after replan) | `foxglove_streamer` + `lidar_publisher` | `faster::state` next_goal |
| `faster_viz_queue` | `FasterThread` (after replan) | `foxglove_streamer` | `VizData` (traj + JPS) |

---

## 3. Dependencies

### System packages (must be present before building)

| Package | Ubuntu 22.04 apt | Jetson NX (aarch64) | Notes |
|---|---|---|---|
| HiGHS | `libhighs-dev` | Build from source (see §4.1) | MIQP solver |
| PCL 1.12 | `libpcl-dev` | ✅ Already installed | KD-tree, point cloud types |
| Eigen 3 | `libeigen3-dev` | ✅ Already present | Linear algebra |
| glog | `libgoogle-glog-dev` | ✅ Already present | Logging |

### Third-party submodule/subtree clones (`third_party/`)

| Library | Repo | Notes |
|---|---|---|
| `faster/` | `git@github.com:evandanrao/faster.git` | Fork of mit-acl/faster. Only `src/` and `include/` used. |
| `jps3d/` | `git@github.com:KumarRobotics/jps3d.git` | CMake-installable JPS library |
| `decomp_util/` | `git@github.com:sikang/DecompUtil.git` | Convex decomposition along path |

### Files from `third_party/faster/faster/` that ARE used

```
src/faster.cpp          — core planner logic (no ROS)
src/utils.cpp           — math helpers (no ROS)
src/jps_manager.cpp     — JPS wrapper (no ROS)
include/faster.hpp
include/faster_types.hpp
include/jps_manager.hpp
include/utils.hpp
include/solverGurobi_utils.hpp  — math utils only (no GRB calls), keep
include/timer.hpp
include/termcolor.hpp
include/read_map.hpp
```

### Files from `third_party/faster/faster/` that are DELETED / replaced

```
src/faster_ros.cpp      — entire ROS interface layer → DELETE
src/solverGurobi.cpp    — Gurobi solver → REPLACE with src/solverHiGHS.cpp
src/main.cpp            — ROS node entry → DELETE
include/faster_ros.hpp  — ROS header → DELETE
include/solverGurobi.hpp → REPLACE with include/solverHiGHS.hpp
```

### Symbols that must be re-declared (previously from ROS msgs)

| Original ROS symbol | Replacement |
|---|---|
| `snapstack_msgs::State` | `faster::state` (already in `faster_types.hpp`, has pos/vel/accel/jerk/yaw) |
| `snapstack_msgs::Goal` / `snapstack_msgs::QuadGoal` | `faster::state` (same struct, goal is a state with target pos and zero vel/accel) |
| `faster_msgs::Mode` | Local enum `FasterMode { GO=0, STOP=1 }` in `include/fast_livo/faster_planner.hpp` |
| `sensor_msgs::PointCloud2` | `pcl::PointCloud<pcl::PointXYZ>::Ptr` (already used internally) |
| `ros::NodeHandle` / `ros::Timer` / `ros::Publisher` / `ros::Subscriber` | std::thread + bridge queues |
| `ros::package::getPath("faster")` | Removed (was only used for debug `.lp` file writes, strip those lines) |
| `geometry_msgs::PoseStamped` (terminal goal topic) | Bridge `faster_goal_queue` fed by Foxglove `/planner/goal` |

---

## 4. Staged Implementation Plan

### Stage 0 — Prerequisites and repo structure
**Effort:** ~1 hour. No code yet.

**Steps:**
1. On branch `faster`, clone the three dependency repos into `third_party/`:
   ```bash
   cd third_party
   git clone git@github.com:evandanrao/faster.git faster
   git clone git@github.com:KumarRobotics/jps3d.git jps3d
   git clone git@github.com:sikang/DecompUtil.git decomp_util
   ```
2. Build and install jps3d + decomp_util as CMake packages (or add as subdirectories — see Stage 1).
3. Install HiGHS on x86:
   ```bash
   sudo apt install libhighs-dev
   ```
4. Install HiGHS on Jetson NX (aarch64):
   ```bash
   git clone https://github.com/ERGO-Code/HiGHS.git
   cd HiGHS && mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
   make -j12 && sudo make install
   ```
5. Cherry-pick from `main` the following standalone infrastructure pieces that were added after `bb29b8e`:
   - `Makefile` additions (`make nx-full`)
   - `scripts/deploy.sh` updates
   - `--desktop` flag in `main.cpp`
   - `src/params.cpp` YAML bool specialization
   - `config/livo2.yaml` bool→int conversion and pcd_save fix

   These are clean isolated patches with no EGO-planner logic mixed in. The cherry-pick for each should be conflict-free.

**Verification:** `make -j12 x86` still builds cleanly (nothing has changed yet in planner code).

---

### Stage 1 — Build FASTER core as a static library, no linkage to main binary yet
**Effort:** ~2 hours. One new CMake target, one new source file.

**Goal:** Confirm that `faster.cpp`, `utils.cpp`, `jps_manager.cpp` compile with HiGHS and without ROS. No integration with livo2 yet.

**Steps:**
1. Write `src/solverHiGHS.cpp` + `include/fast_livo/solverHiGHS.hpp` implementing the same public interface as `SolverGurobi`:
   - Same method signatures: `setN`, `setX0`, `setXf`, `setBounds`, `setPolytopes`, `genNewTraj`, `fillX`, `StopExecution`, `ResetToNormalState`
   - HiGHS MIQP formulation (see §5 for full mathematical spec)

2. In `faster_types.hpp`, add a forward declaration of `SolverHiGHS` and change `faster.hpp` to include `solverHiGHS.hpp` instead of `solverGurobi.hpp`.

3. Add CMake target in `CMakeLists.txt`:
   ```cmake
   add_library(faster_core STATIC
     third_party/faster/faster/src/faster.cpp
     third_party/faster/faster/src/utils.cpp
     third_party/faster/faster/src/jps_manager.cpp
     src/solverHiGHS.cpp
   )
   target_include_directories(faster_core PUBLIC
     third_party/faster/faster/include
     include/fast_livo
     ${JPS3D_INCLUDE_DIRS}
     ${DECOMP_UTIL_INCLUDE_DIRS}
     ${HIGHS_INCLUDE_DIRS}
     ${PCL_INCLUDE_DIRS}
     ${EIGEN3_INCLUDE_DIR}
   )
   target_link_libraries(faster_core
     ${JPS3D_LIBRARIES}
     ${DECOMP_UTIL_LIBRARIES}
     highs
     ${PCL_LIBRARIES}
   )
   ```

4. Write a minimal `src/faster_test.cpp` with `main()` that constructs a `Faster` object, calls `updateState` with a fake pose, and exits. Link against `faster_core`.

**Verification:** `cmake --build build --target faster_test` exits zero. Running it prints no errors.

---

### Stage 2 — Bridge: add FASTER-specific queues and types
**Effort:** ~1 hour. Header-only changes plus LIVMapper push.

**Goal:** Plumb the data paths from SLAM → FASTER queues. No FASTER thread yet.

**Steps:**
1. Add to `include/fast_livo/bridge.hpp`:
   ```cpp
   // FASTER input queues
   std::mutex faster_pose_mtx;
   std::queue<PoseData> faster_pose_queue;    // dedicate: not shared with foxglove

   std::mutex faster_cloud_mtx;
   std::queue<CloudData> faster_cloud_queue;  // dedicate: accumulated global cloud

   // FASTER output queues
   std::mutex faster_cmd_mtx;
   struct FasterCmd {
     Eigen::Vector3d pos;
     Eigen::Vector3d vel;
     Eigen::Vector3d accel;
     double yaw;
   };
   std::queue<FasterCmd> faster_cmd_queue;

   struct FasterViz {
     std::vector<Eigen::Vector3d> whole_traj;
     std::vector<Eigen::Vector3d> safe_traj;
     std::vector<Eigen::Vector3d> jps_path;
   };
   std::mutex faster_viz_mtx;
   std::queue<FasterViz> faster_viz_queue;
   ```

2. In `LIVMapper::publish_odometry()` — add a push to `faster_pose_queue` (same pattern as the existing `pose_queue` push, cap at 100).

3. In `LIVMapper` map-update path (wherever global map is published) — push to `faster_cloud_queue` (same pattern as existing cloud push, cap at 5 — clouds are large).

4. Re-use existing `b_types.hpp` `GoalData` struct and `bridge.planner_goal_queue` for the goal input (Foxglove's `/planner/goal` already populates this queue from `foxglove_streamer.cpp`).

**Note:** The `foxglove_streamer.cpp` ClientPublish code (receiving `/planner/goal` from Foxglove) was added in commit `1cb05b9` (on `main`). Cherry-pick that diff in Stage 0 step 5 or apply it manually here. It is ~130 lines touching only `foxglove_streamer.cpp`.

**Verification:** `make -j12 x86` still builds. The queues exist but nothing drains them yet.

---

### Stage 3 — FasterThread: planner loop
**Effort:** ~4 hours. New source files.

**Goal:** A thread that feeds SLAM data into FASTER and runs the replan loop.

**New files:**
- `include/fast_livo/faster_planner.hpp`
- `src/faster_planner.cpp`

**`faster_planner.hpp`:**
```cpp
#pragma once
#include "bridge.hpp"
#include "params.h"
#include "faster.hpp"         // from third_party/faster
#include "faster_types.hpp"

class FasterPlanner {
public:
  FasterPlanner(Bridge* bridge, const params::Params& p);
  void run();   // blocking thread entry point
  void stop();

private:
  Bridge* bridge_;
  std::atomic<bool> running_{true};

  // FASTER core (no ROS)
  std::unique_ptr<Faster> faster_;
  parameters par_;  // faster_types.hpp parameters struct

  void load_faster_params(const params::Params& p);
  void drain_pose();
  void drain_cloud();
  void drain_goal();
  void do_replan();
  void publish_cmd(const faster::state& cmd);
  void publish_viz(const std::vector<faster::state>& whole,
                   const std::vector<faster::state>& safe,
                   const vec_Vecf<3>& jps_path);
};
```

**`faster_planner.cpp` loop structure:**
```
initialize Faster(par_)
while running_:
    drain_pose()      // call faster_->updateState() for each new PoseData
    drain_cloud()     // call faster_->updateMap(occ_pcl, empty_unk_pcl)
    drain_goal()      // call faster_->setTerminalGoal() on new goal
    do_replan()       // call faster_->replan() if initialized + has goal
                      //   extract X_whole, X_safe, JPS paths
                      //   push FasterCmd + FasterViz to bridge queues
    sleep_for(50ms)   // ~20Hz replan rate
```

**Unknown space handling (initial conservative approach):**
Pass `pclptr_unk` as an empty cloud (`pcl::PointCloud<pcl::PointXYZ>::Ptr(new ...)`). This tells FASTER there is no unknown space — it will plan only through observed free space. This is safe but conservative (won't explore). Stage 6 adds proper unknown space tracking.

**Parameters source:**
FASTER's `parameters` struct (`faster_types.hpp`) is filled from `config/livo2.yaml` under a new `faster:` section. Add `readOr` calls in `params.cpp`.

**`main.cpp` changes:**
```cpp
#include "faster_planner.hpp"
// In main(), after bridge and LIVMapper construction:
FasterPlanner faster_planner(&bridge, p);
std::thread faster_thread(&FasterPlanner::run, &faster_planner);
// In shutdown:
faster_planner.stop();
faster_thread.join();
```

**Verification:** Run with `--desktop`. Send a goal from Foxglove `/planner/goal`. Confirm log output:
```
[FASTER] State updated: pos=(x,y,z) vel=(vx,vy,vz)
[FASTER] Map updated: 1234 occupied voxels
[FASTER] Goal received: (1.0, 1.0, 1.0)
[FASTER] Replanning...
[FASTER] Replan result: solved=1, dt=0.23s, trials=1
[FASTER] Next cmd: pos=(0.12, 0.08, 0.04) vel=(0.3, 0.2, 0.1)
```

---

### Stage 4 — Foxglove output: position commands and trajectory visualization
**Effort:** ~2 hours. Additions to `foxglove_streamer.cpp`.

**Goal:** Stream FASTER outputs to Foxglove as inspectable channels.

**New Foxglove channels (JSON-encoded):**

| Channel | Content | Update rate |
|---|---|---|
| `/faster/cmd` | `{pos:{x,y,z}, vel:{x,y,z}, accel:{x,y,z}, yaw:f}` | ~20Hz (each replan) |
| `/faster/traj/whole` | `{points:[{x,y,z},...]}` | ~20Hz |
| `/faster/traj/safe` | `{points:[{x,y,z},...]}` | ~20Hz |
| `/faster/jps` | `{points:[{x,y,z},...]}` | ~20Hz |

**`foxglove_streamer.cpp` changes:**
- Register the 4 channels at startup (same `channel_ids_` map pattern).
- In the main streamer loop, drain `bridge_->faster_cmd_queue` and `bridge_->faster_viz_queue` and publish.

**UDP changes (`lidar_publisher.cpp`):**
- Add CMD75 for FASTER position command (same format as CMD72 odom but from FASTER output). The flight controller / downstream consumer can use this for tracking.

**Verification:** In Foxglove, subscribe to `/faster/cmd`, `/faster/traj/whole`, `/faster/traj/safe`. With a goal active, confirm they update at the replan rate and the trajectory visually avoids obstacles.

---

### Stage 5 — `config/livo2.yaml` FASTER parameters section
**Effort:** ~30 minutes.

**Add to `livo2.yaml`:**
```yaml
faster:
  # Dynamics limits
  v_max: 3.0       # m/s
  a_max: 5.0       # m/s²
  j_max: 8.0       # m/s³
  w_max: 3.14      # rad/s (yaw rate)

  # Map window
  wdx: 10.0        # world window x [m]
  wdy: 10.0        # world window y [m]
  wdz: 4.0         # world window z [m]
  res: 0.2         # voxel resolution [m]

  # Trajectory
  N_whole: 10      # segments in whole trajectory
  N_safe: 6        # segments in safe trajectory
  dc: 0.01         # time discretization for fillX [s]
  Ra: 3.5          # radius of sphere A (replanning trigger) [m]
  goal_radius: 0.3 # distance to consider goal reached [m]
  drone_radius: 0.3 # collision radius [m]
  use_faster: 1    # 1=use FASTER (whole+safe), 0=safe only
  is_ground_robot: 0

  # JPS
  inflation_jps: 0.5   # inflation of obstacles for JPS [m]
  factor_jps: 2.0      # factor to expand JPS path

  # Safe flight corridor
  max_poly_whole: 5    # max polytopes for whole traj
  max_poly_safe: 3     # max polytopes for safe traj
  dist_max_vertexes: 1.5

  # Solver
  gurobi_threads: 1    # ignored (HiGHS), kept for config compat
  gurobi_verbose: 0    # ignored (HiGHS), kept for config compat

  # SFC factor sweep
  gamma_whole: 2.0
  gammap_whole: 2.0
  increment_whole: 0.5
  gamma_safe: 2.0
  gammap_safe: 2.0
  increment_safe: 0.5
  delta_a: 1.0
  delta_H: 1.0

  # Goals
  goal_height: 1.0
  force_goal_height: 0

  z_ground: 0.2
  z_max: 3.5
  alpha_filter_dyaw: 0.1
  spinup_time: 1.0

  visual: 1
  use_ff: 0
```

**`params.h`:** Add `struct FasterParams { ... }` mirroring the yaml section.
**`params.cpp`:** Add `readOr` calls to populate it.

---

### Stage 6 — Unknown space tracking (raycasting voxel grid)
**Effort:** ~3 hours.

**Goal:** Populate `pclptr_unk` with unobserved voxels so FASTER correctly treats unmapped regions as unknown (won't plan through them unless `use_faster=1` with the whole-trajectory mode).

**Design:**
- Maintain a 3D occupancy bitmask (or use PCL VoxelGrid) over a sliding window centered on the drone.
- When a new LiDAR scan arrives, ray-cast from sensor origin through each return point. Voxels along each ray = observed (free or occupied at endpoint). Voxels inside bounding box but never ray-cast = unknown.
- Output two PCL clouds per map update:
  - `pclptr_occ`: occupied voxels (endpoint of a ray within distance threshold)
  - `pclptr_unk`: voxels in bounding box that have zero ray-cast hits
- Pass both to `faster_->updateMap(pclptr_occ, pclptr_unk)`.

**Implementation options:**
- **Option A (simple):** Use `std::unordered_set<Eigen::Vector3i>` (hash on voxel index) for observed set. Fast enough for 200×200×40 voxels at 0.2m resolution.
- **Option B (full):** Port the voxel_map already in livo2 (`src/fast_livo/voxel_map.cpp`) — it already tracks occupancy; add a "visited" bit per voxel.

Option A is recommended for initial implementation (Stage 6a). Option B is an upgrade (Stage 6b).

**Verification:** Drive drone to edge of explored area. Confirm FASTER safe path stops at the unknown boundary rather than penetrating it. Confirm `/faster/traj/whole` enters unknown space while `/faster/traj/safe` does not.

---

### Stage 7 — Deploy to Jetson NX
**Effort:** ~2 hours (mostly HiGHS build on aarch64).

**Steps:**
1. Build HiGHS from source on Jetson NX aarch64 (see Stage 0 §4 — same commands, run on NX or cross-compile).
2. Build jps3d + decomp_util on NX.
3. Update `scripts/deploy.sh` to rsync HiGHS headers + lib if not using system install.
4. `make nx` — verify full binary builds on NX.
5. Run with real LiDAR. Confirm replan at ~20Hz, no solver timeouts.

**NX-specific tuning:** HiGHS on the NX Cortex-A57 is slower than x86. If replan budget is tight, reduce `N_whole` from 10 to 7 and `max_poly_whole` from 5 to 3. These are YAML params, no recompile needed.

---

## 5. HiGHS MIQP Formulation (Solver Spec)

This section fully specifies what `solverHiGHS.cpp` must implement. It is a direct translation of the Gurobi formulation in `solverGurobi.cpp` into HiGHS API calls.

### Problem structure

**Decision variables:**
For each of N segments `t = 0..N-1` and each axis `i = {x,y,z}`, cubic polynomial coefficients:
```
a_t_i, b_t_i, c_t_i, d_t_i    (a=jerk/6, b=accel_0/2, c=vel_0, d=pos_0)
```
Total continuous vars: `12*N` per solve call.

For M polytopes in the SFC and N+1 time knots:
```
s_t_k  ∈ {0,1}    t=0..N, k=0..M-1    (binary: segment t ∈ polytope k)
```
Total binary vars: `(N+1)*M`.

Grand total variables: `12*N + (N+1)*M`.

**Objective:** Minimize total squared jerk (integral):
```
min  Σ_{t=0}^{N-1}  Σ_{i=x,y,z}  (6*a_t_i)² * dt_t
```
This is purely quadratic in the continuous variables; no cross terms between binary and continuous.

**Constraints (all linear after expanding the cubic polynomials):**

1. **Initial conditions** (equality, 9 constraints):
   ```
   pos(t=0, τ=0, i) = x0[i]      i=x,y,z
   vel(t=0, τ=0, i) = x0[3+i]
   accel(t=0, τ=0, i) = x0[6+i]
   ```

2. **Final conditions** (equality, 6–9 constraints, optional):
   ```
   vel(t=N-1, τ=dt, i) = xf[3+i]
   accel(t=N-1, τ=dt, i) = xf[6+i]
   [optionally] pos(t=N-1, τ=dt, i) = xf[i]  (only if forceFinalConstraint_=true)
   ```

3. **Continuity** (equality, `3*(N-1)*3` = `9*(N-1)` constraints):
   ```
   pos(t, dt_t, i)   = pos(t+1, 0, i)     t=0..N-2
   vel(t, dt_t, i)   = vel(t+1, 0, i)
   accel(t, dt_t, i) = accel(t+1, 0, i)
   ```

4. **Velocity bounds** (inequality, `6*N` constraints):
   ```
   -v_max ≤ vel(t, 0, i) ≤ v_max    t=0..N-1, i=x,y,z
   ```

5. **Acceleration bounds** (inequality, `6*N` constraints):
   ```
   -a_max ≤ accel(t, 0, i) ≤ a_max
   ```

6. **Jerk bounds** (inequality, `6*N` constraints):
   ```
   -j_max ≤ jerk(t, 0, i) ≤ j_max
   ```

7. **At-least-one-polytope** (equality, N constraints):
   ```
   Σ_{k=0}^{M-1} s_t_k = 1    t=0..N-1
   ```

8. **Polytope indicator** (conditional inequality, `4*N*M*F` constraints, F=faces per polytope):
   ```
   if s_t_k = 1:  A_k * cp_j(t) ≤ b_k    j=0,1,2,3 (Bézier control points)
   ```
   In HiGHS this is linearized with big-M:
   ```
   A_k * cp_j(t) ≤ b_k + BigM*(1 - s_t_k)    ∀ k, j, face f
   ```
   BigM is chosen as the diagonal of the map bounding box (e.g., `norm(wdx, wdy, wdz)`).

### Key implementation note on big-M

Gurobi uses `addGenConstrIndicator` which is a native indicator constraint. HiGHS does not have native indicator constraints — use big-M linearization. For well-conditioned problems (trajectory inside a ~10m box) BigM = 30 is sufficient and does not cause numerical issues.

### HiGHS API sketch (C++ `Highs.h`)

```cpp
#include <Highs.h>

// Setup
Highs highs;
highs.silent();                    // suppress output if gurobi_verbose=0

// Add continuous variables (box: [-inf, +inf])
for (int v = 0; v < num_cont_vars; v++)
  highs.addVar(-kHighsInf, kHighsInf);

// Add binary variables
for (int v = 0; v < num_bin_vars; v++) {
  highs.addVar(0.0, 1.0);
  highs.changeColIntegrality(num_cont_vars + v, kHighsVarTypeInteger);
}

// Set quadratic objective (jerk minimization)
// HiGHS expects Hessian in sparse format (only upper triangle)
highs.passHessian(...);  // or passLpHessian for LP relaxation

// Add linear constraints
highs.addRows(...);      // all constraints as Ax = b or lb ≤ Ax ≤ ub

// Solve
highs.run();

// Extract solution
if (highs.getInfoValue("primal_solution_status", ...) == kSolutionStatusFeasible)
  highs.getSolution(); // returns HighsSolution with col_value
```

### `SolverHiGHS` public API (mirrors `SolverGurobi` exactly)

```cpp
class SolverHiGHS {
public:
  SolverHiGHS();
  void setN(int N);
  void setX0(state& data);
  void setXf(state& data);
  void resetX();
  void setBounds(double max_values[3]);   // [v_max, a_max, j_max]
  bool genNewTraj();                       // sets up and solves MIQP, returns solved
  void fillX();                            // extract state trajectory from solution
  void setPolytopes(std::vector<LinearConstraint3D> polytopes);
  void setDC(double dc);
  void setMode(int mode);
  void setThreads(int threads);
  void setVerbose(int verbose);
  void StopExecution();
  void ResetToNormalState();
  void setFactorInitialAndFinalAndIncrement(double f0, double ff, double fi);
  void setForceFinalConstraint(bool force);
  void setWMax(double w_max);
  bool isWmaxSatisfied();
  void setDistances(vec_Vecf<3>& samples, std::vector<double> dist_near_obs);
  void setDistanceConstraints();

  std::vector<state> X_temp_;
  double dt_;
  int trials_ = 0;
  double runtime_ms_ = 0;
  double factor_that_worked_ = 0;
  int N_ = 10;
};
```

---

## 6. Files Changed Summary

### New files
| File | Description |
|---|---|
| `include/fast_livo/solverHiGHS.hpp` | HiGHS-based MIQP solver (replaces solverGurobi.hpp) |
| `src/solverHiGHS.cpp` | Implementation (~500 lines) |
| `include/fast_livo/faster_planner.hpp` | FasterPlanner class header |
| `src/faster_planner.cpp` | Thread + loop + bridge wiring (~350 lines) |
| `src/faster_test.cpp` | Minimal build verification binary |

### Modified files
| File | Change |
|---|---|
| `CMakeLists.txt` | Add `faster_core` static lib, jps3d/decomp_util/HiGHS find_package, link to main binary |
| `include/fast_livo/bridge.hpp` | Add `faster_pose_queue`, `faster_cloud_queue`, `faster_cmd_queue`, `faster_viz_queue`, `FasterCmd`, `FasterViz` structs |
| `include/fast_livo/params.h` | Add `FasterParams` struct |
| `src/params.cpp` | Add YAML loading for `faster:` section |
| `config/livo2.yaml` | Add `faster:` section |
| `src/fast_livo/LIVMapper.cpp` | Push to `faster_pose_queue` and `faster_cloud_queue` |
| `src/foxglove_streamer.cpp` | Add `/faster/cmd`, `/faster/traj/whole`, `/faster/traj/safe`, `/faster/jps` channels; drain viz/cmd queues |
| `src/lidar_publisher.cpp` | Add CMD75 FASTER position command UDP encoding |
| `src/main.cpp` | Instantiate and thread `FasterPlanner`; add faster params to log |
| `third_party/faster/faster/include/faster.hpp` | Change `#include "solverGurobi.hpp"` → `#include "solverHiGHS.hpp"` |
| `third_party/faster/faster/include/solverGurobi.hpp` | Delete (or keep and guard with `#ifdef USE_GUROBI`) |

### Deleted / not compiled files (from faster source)
- `third_party/faster/faster/src/faster_ros.cpp`
- `third_party/faster/faster/src/main.cpp`
- `third_party/faster/faster/src/solverGurobi.cpp`
- `third_party/faster/faster/include/faster_ros.hpp`

---

## 7. Known Risks and Mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| HiGHS big-M numerics produce infeasible solutions | Medium | Tune BigM value; fallback: increase `factor_initial_` sweep range |
| HiGHS MIQP too slow on Jetson NX A57 cores | Medium | Reduce N_whole to 7, max_poly_whole to 3; profile with `runtime_ms_` field |
| JPS3D / DecompUtil CMake find_package broken on NX | Low | Add as CMake `add_subdirectory` instead of installed packages |
| FASTER replan stuttering due to cloud queue pressure | Low | Cap `faster_cloud_queue` at 3, drop oldest on overflow |
| `decomp_util` requires `decomp_ros_utils` indirectly | Low | `decomp_ros_utils` is visualization only; include only `decomp_util` core |
| Unknown space = empty cloud causes FASTER to enter walls | High (Stage 3–5) | Explicitly noted: Stage 6 fixes this. Until Stage 6 set `use_faster=0` and rely only on safe path |

---

## 8. Commit Sequence (expected)

```
[S0] faster: cherry-pick infra from main (Makefile, deploy.sh, --desktop, YAML bool fix)
[S0] faster: add jps3d, decomp_util, faster as third_party submodules
[S1] faster: solverHiGHS — full MIQP formulation with HiGHS C++ API
[S1] faster: CMakeLists — faster_core static lib, HiGHS/jps3d/decomp find_package
[S1] faster: build verification smoke test (faster_test binary)
[S2] faster: bridge — faster_pose_queue, cloud_queue, cmd_queue, viz_queue
[S2] faster: LIVMapper — dual push to faster queues
[S3] faster: FasterPlanner — thread, loop, drain, replan
[S3] faster: main — FasterPlanner instantiation and thread launch
[S3] faster: params — FasterParams struct and YAML loading
[S3] faster: livo2.yaml — faster: section with default params
[S4] faster: foxglove_streamer — /faster/cmd, /faster/traj/*, /faster/jps channels
[S4] faster: lidar_publisher — CMD75 FASTER position command
[S5] faster: livo2.yaml — tune params after first x86 run
[S6] faster: voxel_raycast — unknown space cloud for faster->updateMap
[S7] faster: deploy.sh — HiGHS aarch64 build step; NX-tuned default params
```

---

## 9. What Is NOT Changed

- `src/fast_livo/` (SLAM core) — untouched except the two queue pushes in LIVMapper
- `src/foxglove_streamer.cpp` existing channels — `/livo2/pose`, `/livo2/cloud`, `/livo2/global_map` untouched
- `src/lidar_publisher.cpp` existing CMDs — CMD72/73/74 untouched
- `src/camera_driver.cpp`, `camera_loader.cpp` — untouched
- `third_party/HesaiLidar_SDK_2.0`, `sophus`, `vikit_common` — untouched
- The EGO-Planner integration on `main` — completely separate branch, no merge

---

## 10. Reference Links

- FASTER paper: https://arxiv.org/abs/2001.04420
- FASTER upstream: https://github.com/mit-acl/faster
- FASTER fork (this project): https://github.com/evandanrao/faster
- HiGHS solver: https://highs.dev / https://github.com/ERGO-Code/HiGHS
- HiGHS C++ API: https://ergo-code.github.io/HiGHS/dev/
- JPS3D: https://github.com/KumarRobotics/jps3d
- DecompUtil: https://github.com/sikang/DecompUtil
