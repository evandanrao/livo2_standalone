/**
 * goal_listener.cpp — UDP listener for inbound planner goal commands.
 *
 * Receives packets on UDP port 8891 encoded in the CommandCodec wire format
 * (matches command_codec.h from common/lib/command_apis).
 *
 *   packet[0-1] : SOP = 0x99 0x99
 *   packet[2-3] : PACKET_ID (uint16, big-endian)
 *   packet[4]   : CMD = 78 (CMD_PLANNER_GOAL)
 *   packet[5]   : VER_MAJOR = 1
 *   packet[6]   : VER_MINOR = 0
 *   packet[7-8] : DATA_LEN = 12 (big-endian)
 *   packet[9-20]: x (float32 LE), y (float32 LE), z (float32 LE)
 *   packet[21-22]: CRC-16 CCITT (big-endian)
 *
 * Total packet size: 23 bytes.
 */

#include "goal_listener.hpp"
#include "fast_livo/bridge.hpp"

#include <arpa/inet.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

// Wire protocol constants — mirror command_codec.h
static constexpr uint8_t SOP = 0x99;
static constexpr uint8_t CMD_PLANNER_GOAL = 78;
static constexpr uint8_t GOAL_VER_MAJOR = 1;
static constexpr uint8_t GOAL_VER_MINOR = 0;
static constexpr uint16_t GOAL_DATA_LEN = 12; // 3 × float32
static constexpr int GOAL_PACKET_LEN =
    11 + GOAL_DATA_LEN + 2; // header + data + CRC

static constexpr uint16_t LISTEN_PORT = 8891;

// CRC-16 CCITT table (same polynomial as command_codec.cpp)
static const uint16_t s_crc16_tab[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

static uint16_t crc16(const uint8_t *buf, int len) {
  uint16_t cksum = 0;
  for (int i = 0; i < len; i++) {
    cksum = s_crc16_tab[((cksum >> 8) ^ buf[i]) & 0xFF] ^ (cksum << 8);
  }
  return cksum;
}

// Parse a validated 23-byte goal packet.  Returns true on success.
static bool parse_goal_packet(const uint8_t *buf, int len, float &x, float &y,
                              float &z) {
  if (len < GOAL_PACKET_LEN) {
    return false;
  }
  if (buf[0] != SOP || buf[1] != SOP) {
    return false;
  }
  if (buf[4] != CMD_PLANNER_GOAL) {
    return false;
  }
  // Decode data length (big-endian)
  uint16_t data_len = (static_cast<uint16_t>(buf[7]) << 8) | buf[8];
  if (data_len != GOAL_DATA_LEN) {
    return false;
  }
  // Verify CRC over header + data
  int crc_offset = 9 + data_len;
  uint16_t expected_crc = crc16(buf, crc_offset);
  uint16_t got_crc =
      (static_cast<uint16_t>(buf[crc_offset]) << 8) | buf[crc_offset + 1];
  if (expected_crc != got_crc) {
    fprintf(stderr,
            "[goal_listener] CRC mismatch (expected 0x%04x, got 0x%04x)\n",
            expected_crc, got_crc);
    return false;
  }
  // Decode float32 payload (little-endian native float)
  memcpy(&x, buf + 9 + 0, sizeof(float));
  memcpy(&y, buf + 9 + 4, sizeof(float));
  memcpy(&z, buf + 9 + 8, sizeof(float));
  return true;
}

void goal_listener_thread(livo::Bridge &bridge) {
  // Open a UDP socket bound to LISTEN_PORT on all interfaces
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("[goal_listener] socket");
    return;
  }

  // Allow reuse so restart is quick
  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(LISTEN_PORT);

  if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) <
      0) {
    perror("[goal_listener] bind");
    close(sock);
    return;
  }

  fprintf(stdout,
          "[goal_listener] Listening for planner goals on UDP port %d "
          "(CMD=78)\n",
          LISTEN_PORT);

  uint8_t buf[256];
  while (bridge.running.load()) {
    // Use select with a 200 ms timeout so we check bridge.running periodically
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    struct timeval tv {
      0, 200'000
    }; // 200 ms
    int sel = select(sock + 1, &rfds, nullptr, nullptr, &tv);
    if (sel <= 0) {
      continue; // timeout or error — loop and check running flag
    }

    struct sockaddr_in sender {};
    socklen_t sender_len = sizeof(sender);
    ssize_t n =
        recvfrom(sock, buf, sizeof(buf), 0,
                 reinterpret_cast<struct sockaddr *>(&sender), &sender_len);
    if (n < 0) {
      continue;
    }

    float x = 0, y = 0, z = 0;
    if (!parse_goal_packet(buf, static_cast<int>(n), x, y, z)) {
      continue;
    }

    fprintf(stdout, "[goal_listener] Goal received: (%.2f, %.2f, %.2f)\n", x, y,
            z);

    livo::Goal goal{x, y, z};
    {
      std::lock_guard<std::mutex> lk(bridge.planner_goal_mtx);
      // Discard oldest if queue is saturated
      while (bridge.planner_goal_queue.size() >= livo::kPlannerGoalQueueMax) {
        bridge.planner_goal_queue.pop();
      }
      bridge.planner_goal_queue.push(goal);
    }
  }

  close(sock);
  fprintf(stdout, "[goal_listener] Thread exiting.\n");
}
