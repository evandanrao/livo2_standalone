#pragma once
#include <chrono>

/// Returns wall-clock time in seconds from an arbitrary epoch (steady_clock).
/// Every EGO-Planner component uses this — no ROS dependency.
inline double ego_now_s() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}
