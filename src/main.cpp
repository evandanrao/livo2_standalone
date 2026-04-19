// standalones/livo2/src/main.cpp
// Stage 7 — Entry point: parse CLI args, load params, spin all threads.
#include "fast_livo/LIVMapper.h"
#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"

#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

// Forward declarations (defined in their respective translation units)
void lidar_driver_thread(livo::Bridge &bridge, const params::Params &p);
void camera_driver_thread(livo::Bridge &bridge, const params::Params &p);
void foxglove_streamer_thread(livo::Bridge &bridge, const params::Params &p);
void lidar_publisher_thread(livo::Bridge &bridge, const params::Params &p);

// Global bridge pointer for the SIGINT handler
static livo::Bridge *g_bridge = nullptr;

static void sigint_handler(int) {
  if (g_bridge) {
    g_bridge->running.store(false);
  }
}

static void print_usage(const char *prog) {
  fprintf(stderr,
          "Usage: %s --config <livo2.yaml> --camera <camera.yaml>\n"
          "  --config   path to livo2 parameter YAML\n"
          "  --camera   path to camera intrinsics YAML\n"
          "  --help     print this message\n",
          prog);
}

int main(int argc, char **argv) {
  std::string livo2_yaml;
  std::string camera_yaml;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
      livo2_yaml = argv[++i];
    } else if ((arg == "--camera" || arg == "-k") && i + 1 < argc) {
      camera_yaml = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    } else {
      fprintf(stderr, "[main] Unknown argument: %s\n", arg.c_str());
      print_usage(argv[0]);
      return 1;
    }
  }

  if (livo2_yaml.empty() || camera_yaml.empty()) {
    fprintf(stderr, "[main] --config and --camera are required.\n");
    print_usage(argv[0]);
    return 1;
  }

  // Load parameters
  params::Params p;
  try {
    p = params::load(livo2_yaml, camera_yaml);
  } catch (const std::exception &e) {
    fprintf(stderr, "[main] Failed to load parameters: %s\n", e.what());
    return 1;
  }

  // Create inter-thread bridge
  livo::Bridge bridge;
  g_bridge = &bridge;

  // Set up graceful shutdown on Ctrl-C
  std::signal(SIGINT, sigint_handler);
  std::signal(SIGTERM, sigint_handler);

  // Build and initialize SLAM core
  LIVMapper slam(bridge, p);
  slam.initializeComponents(p);

  // Start worker threads
  std::thread t_lidar(lidar_driver_thread, std::ref(bridge), std::cref(p));
  std::thread t_camera(camera_driver_thread, std::ref(bridge), std::cref(p));
  std::thread t_fox(foxglove_streamer_thread, std::ref(bridge), std::cref(p));
  std::thread t_udp(lidar_publisher_thread, std::ref(bridge), std::cref(p));

  // SLAM runs on this thread (blocking until bridge.running == false)
  slam.run();

  // Signal all threads to stop (in case SLAM returned naturally)
  bridge.running.store(false);

  t_lidar.join();
  t_camera.join();
  t_fox.join();
  t_udp.join();

  fprintf(stdout, "[main] Shutdown complete.\n");
  return 0;
}
