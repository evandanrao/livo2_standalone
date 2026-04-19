/**
 * lidar_publisher.hpp — UDP publisher for livo2 viz data
 *
 * Reads odom + cloud from the Bridge queues, serializes using the same
 * CommandAPI binary wire protocol as lio_lc_standalone, and broadcasts
 * over UDP port 8892.
 *
 * Runs as a standalone thread — no Qt dependency.
 * Wire format is identical to lio_lc so stream_client works unchanged.
 */
#pragma once

#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"
#include <atomic>

void lidar_publisher_thread(livo::Bridge &bridge, const params::Params &p);
