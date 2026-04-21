/**
 * goal_listener.hpp — UDP listener for inbound planner goal commands.
 *
 * Receives CMD_PLANNER_GOAL (cmd=78) packets on UDP port 8891.
 * Wire format: [0x99 0x99 PKT_ID(2) CMD(1) VER_MAJ(1) VER_MIN(1) LEN(2)
 *               x_f(4) y_f(4) z_f(4) CRC(2)]  — 23 bytes total
 *
 * Validated packets are pushed into bridge.planner_goal_queue for
 * Planner::run() to consume.
 */
#pragma once

#include "fast_livo/bridge.hpp"

void goal_listener_thread(livo::Bridge &bridge);
