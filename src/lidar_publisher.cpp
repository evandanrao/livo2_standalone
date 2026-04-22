/**
 * lidar_publisher.cpp — UDP publisher for livo2 viz data
 *
 * Reads latest odom + cloud from Bridge queues, serializes using the
 * CommandAPI binary protocol (SOP+cmd+CRC16), and broadcasts over UDP.
 *
 * Wire format is byte-for-byte identical to lio_lc_standalone so the
 * stream_client app (lidar_message_parser) works without modification.
 *
 * Commands published:
 *   CMD_LIDAR_ODOM   (72) — 10 Hz  — current pose
 *   CMD_LIDAR_SCAN   (73) —  5 Hz  — per-frame world cloud (chunked)
 *   CMD_LIDAR_PATH   (74) —  1 Hz  — accumulated path trail (chunked)
 *   CMD_OCC_VOXELS   (75) —  2 Hz  — inflated occupancy voxels (chunked)
 */

#include "lidar_publisher.hpp"
#include "fast_livo/b_types.hpp"
#include "fast_livo/bridge.hpp"
#include "fast_livo/params.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

// ── Wire protocol constants (mirrors command_codec.h in stream_client) ──────
static constexpr uint8_t SOP = 0x99;
static constexpr int FRAME_OVERHEAD =
    11; // 2 SOP + 2 pktid + 1 cmd + 2 ver + 2 len + 2 CRC
static constexpr uint16_t MAX_DATA_LEN = 1024;

// Command IDs (must match commands.h / lidar_message_parser.h in stream_client)
static constexpr uint8_t CMD_LIDAR_ODOM = 72;
static constexpr uint8_t CMD_LIDAR_SCAN = 73;
static constexpr uint8_t CMD_LIDAR_PATH = 74;
static constexpr uint8_t CMD_OCC_VOXELS = 75;

static constexpr uint8_t VER_MAJOR = 1;
static constexpr uint8_t VER_MINOR = 0;

// Broadcast target — overridden at runtime from params (livo2.yaml
// publisher.broadcast_addr) Default matches lio_lc drone subnet; set to
// 255.255.255.255 for desktop use.
static constexpr const char *BROADCAST_ADDR_DEFAULT = "192.168.168.255";
static constexpr uint16_t BROADCAST_PORT_DEFAULT = 8892;

// Publisher rate control
static constexpr int ODOM_INTERVAL_MS = 100;  // 10 Hz
static constexpr int SCAN_INTERVAL_MS = 200;  //  5 Hz
static constexpr int PATH_INTERVAL_MS = 1000; //  1 Hz
static constexpr int OCC_INTERVAL_MS  = 500;   //  2 Hz
// Downsample the per-frame cloud before sending (keeps UDP budget reasonable)
static constexpr float DOWNSAMPLE_LEAF = 0.12f; // metres

// ── CRC-16 CCITT ─────────────────────────────────────────────────────────────
static const uint16_t crc16_tab[256] = {
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
  for (int i = 0; i < len; i++)
    cksum = crc16_tab[((cksum >> 8) ^ buf[i]) & 0xFF] ^ (cksum << 8);
  return cksum;
}

// ── Packet builder
// ────────────────────────────────────────────────────────────
static int build_packet(uint8_t *buf, uint16_t &pkt_id, uint8_t cmd,
                        const uint8_t *payload, int payload_len) {
  int i = 0;
  pkt_id++;
  buf[i++] = SOP;
  buf[i++] = SOP;
  buf[i++] = (pkt_id >> 8) & 0xFF;
  buf[i++] = pkt_id & 0xFF;
  buf[i++] = cmd;
  buf[i++] = VER_MAJOR;
  buf[i++] = VER_MINOR;
  buf[i++] = (payload_len >> 8) & 0xFF;
  buf[i++] = payload_len & 0xFF;
  memcpy(buf + i, payload, payload_len);
  i += payload_len;
  uint16_t c = crc16(buf, i);
  buf[i++] = (c >> 8) & 0xFF;
  buf[i++] = c & 0xFF;
  return i;
}

// ── Wire structs (must stay in sync with stream_client lidar_message_parser) ─
#pragma pack(push, 1)

// CMD_LIDAR_ODOM payload (matches LidarOdomState in lidar_odom_api.h)
struct OdomPayload {
  double timestamp_s;
  float x, y, z;
  float qx, qy, qz, qw;
  uint16_t keyframe_count;
  uint8_t status; // 1 = mapping
  uint8_t reserved;
};

// CMD_LIDAR_SCAN — chunk header (matches LidarScanHeader in lidar_scan_api.h)
struct ScanHeader {
  uint16_t scan_seq;
  uint8_t chunk_id;
  uint8_t total_chunks;
  double timestamp_s;
  uint16_t num_points;
  uint16_t reserved;
};

// CMD_LIDAR_SCAN — single point (matches LidarScanPoint in lidar_scan_api.h)
struct ScanPoint {
  int16_t x_mm;
  int16_t y_mm;
  int16_t z_mm;
  uint8_t intensity;
  uint8_t reserved;
};

// CMD_LIDAR_PATH — chunk header (matches LidarPathHeader in
// lidar_message_parser.cpp)
struct PathHeader {
  uint16_t path_seq;
  uint8_t chunk_id;
  uint8_t total_chunks;
  uint32_t total_poses;
  double timestamp_s;
}; // 16 bytes

// CMD_LIDAR_PATH — single pose
struct PathPose {
  float x, y, z;
  float qx, qy, qz, qw;
}; // 28 bytes → (1024-16)/28 = 36 poses per chunk

// CMD_OCC_VOXELS — chunk header
struct OccHeader {
    uint16_t occ_seq;
    uint8_t  chunk_id;
    uint8_t  total_chunks;
    uint32_t total_voxels;
    double   timestamp_s;
}; // 16 bytes

// CMD_OCC_VOXELS — single voxel (int16 cm to save bandwidth)
struct OccVoxel {
    int16_t x_cm;
    int16_t y_cm;
    int16_t z_cm;
    uint16_t reserved;
}; // 8 bytes → (1024-16)/8 = 126 voxels per chunk

#pragma pack(pop)

static constexpr int MAX_POINTS_PER_CHUNK = 126; // (1024-16) / 8
static constexpr int MAX_POSES_PER_CHUNK  =  36; // (1024-16) / 28
static constexpr int MAX_VOXELS_PER_CHUNK = 126; // (1024-16) / 8

// ──────────────────────────────────────────────────────────────
struct StreamState {
  using Clock = std::chrono::steady_clock;
  int interval_ms{};
  Clock::time_point last{Clock::now()};

  bool due(const Clock::time_point &now) const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - last)
               .count() >= interval_ms;
  }
  void mark(const Clock::time_point &now) { last = now; }
};

// ── Publisher thread
// ──────────────────────────────────────────────────────────
void lidar_publisher_thread(livo::Bridge &bridge, const params::Params &p) {
  const std::string &bcast_addr = p.publisher.broadcast_addr.empty()
                                      ? std::string(BROADCAST_ADDR_DEFAULT)
                                      : p.publisher.broadcast_addr;
  const uint16_t bcast_port =
      p.publisher.broadcast_port > 0
          ? static_cast<uint16_t>(p.publisher.broadcast_port)
          : BROADCAST_PORT_DEFAULT;

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    fprintf(stderr, "[lidar_publisher] Failed to create socket\n");
    return;
  }

  int bcast = 1;
  setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

  struct sockaddr_in dest {};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(bcast_port);
  inet_pton(AF_INET, bcast_addr.c_str(), &dest.sin_addr);

  fprintf(stderr, "[lidar_publisher] Broadcasting to %s:%d\n",
          bcast_addr.c_str(), bcast_port);

  uint16_t pkt_id = 0;
  uint16_t scan_seq = 0;
  uint16_t path_seq = 0;
  uint16_t occ_seq = 0;
  uint8_t packet_buf[MAX_DATA_LEN + FRAME_OVERHEAD];

  // VoxelGrid for per-frame scan downsampling
  pcl::VoxelGrid<pcl::PointXYZINormal> voxel;
  voxel.setLeafSize(DOWNSAMPLE_LEAF, DOWNSAMPLE_LEAF, DOWNSAMPLE_LEAF);

  // Path accumulator — grows every time a new pose arrives
  struct PoseEntry {
    float x, y, z, qx, qy, qz, qw;
  };
  std::vector<PoseEntry> path_accum;
  double path_last_ts = 0.0;

  StreamState odom_st{ODOM_INTERVAL_MS};
  StreamState scan_st{SCAN_INTERVAL_MS};
  StreamState path_st{PATH_INTERVAL_MS};
  StreamState occ_st{OCC_INTERVAL_MS};

  // Helper: send path_accum as chunked CMD_LIDAR_PATH packets
  auto send_path = [&]() {
    if (path_accum.empty())
      return;
    path_seq++;
    int total = (int)path_accum.size();
    int nchunks = (total + MAX_POSES_PER_CHUNK - 1) / MAX_POSES_PER_CHUNK;
    if (nchunks > 255)
      nchunks = 255;

    int idx = 0;
    for (int c = 0; c < nchunks && idx < total; c++) {
      int n = std::min(MAX_POSES_PER_CHUNK, total - idx);
      uint8_t
          chunk[sizeof(PathHeader) + MAX_POSES_PER_CHUNK * sizeof(PathPose)];
      auto *hdr = reinterpret_cast<PathHeader *>(chunk);
      hdr->path_seq = path_seq;
      hdr->chunk_id = (uint8_t)c;
      hdr->total_chunks = (uint8_t)nchunks;
      hdr->total_poses = (uint32_t)total;
      hdr->timestamp_s = path_last_ts;

      auto *pp = reinterpret_cast<PathPose *>(chunk + sizeof(PathHeader));
      for (int p = 0; p < n; p++, idx++) {
        const auto &e = path_accum[idx];
        pp[p] = {e.x, e.y, e.z, e.qx, e.qy, e.qz, e.qw};
      }
      int payload_len = (int)(sizeof(PathHeader) + n * sizeof(PathPose));
      int plen =
          build_packet(packet_buf, pkt_id, CMD_LIDAR_PATH, chunk, payload_len);
      sendto(sock, packet_buf, plen, 0, (struct sockaddr *)&dest, sizeof(dest));
    }
  };

  while (bridge.running.load()) {
    auto now = StreamState::Clock::now();

    // ── Drain pose_queue — feed odom + path accumulator ────────────────
    {
      std::queue<livo::PoseData> local;
      {
        std::lock_guard<std::mutex> lk(bridge.pose_mtx);
        std::swap(local, bridge.pose_queue);
      }
      while (!local.empty()) {
        const auto &pd = local.front();

        // Accumulate into path
        Eigen::Quaterniond q(pd.rotation);
        PoseEntry e{(float)pd.position.x(),
                    (float)pd.position.y(),
                    (float)pd.position.z(),
                    (float)q.x(),
                    (float)q.y(),
                    (float)q.z(),
                    (float)q.w()};
        path_accum.push_back(e);
        path_last_ts = pd.timestamp;

        // Odom — send at rate-limited intervals
        if (odom_st.due(now)) {
          OdomPayload pl{};
          pl.timestamp_s = pd.timestamp;
          pl.x = e.x;
          pl.y = e.y;
          pl.z = e.z;
          pl.qx = e.qx;
          pl.qy = e.qy;
          pl.qz = e.qz;
          pl.qw = e.qw;
          pl.keyframe_count = 0;
          pl.status = 1; // mapping
          int plen = build_packet(packet_buf, pkt_id, CMD_LIDAR_ODOM,
                                  (const uint8_t *)&pl, sizeof(pl));
          sendto(sock, packet_buf, plen, 0, (struct sockaddr *)&dest,
                 sizeof(dest));
          odom_st.mark(now);
        }

        local.pop();
      }
    }

    // ── Drain viz_cloud_queue — send per-frame scan ─────────────────────
    if (scan_st.due(now)) {
      livo::CloudData cd;
      bool has_cloud = false;
      {
        std::lock_guard<std::mutex> lk(bridge.viz_cloud_mtx);
        if (!bridge.viz_cloud_queue.empty()) {
          cd = std::move(bridge.viz_cloud_queue.back());
          // discard stale frames
          while (!bridge.viz_cloud_queue.empty())
            bridge.viz_cloud_queue.pop();
          has_cloud = true;
        }
      }
      if (has_cloud && cd.cloud && !cd.cloud->empty()) {
        PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
        voxel.setInputCloud(cd.cloud);
        voxel.filter(*filtered);

        int total_pts = (int)filtered->size();
        int total_chunks =
            (total_pts + MAX_POINTS_PER_CHUNK - 1) / MAX_POINTS_PER_CHUNK;
        if (total_chunks == 0)
          total_chunks = 1;
        if (total_chunks > 255)
          total_chunks = 255;

        scan_seq++;
        int pt_idx = 0;
        for (int c = 0; c < total_chunks && pt_idx < total_pts; c++) {
          int n = std::min(MAX_POINTS_PER_CHUNK, total_pts - pt_idx);
          uint8_t chunk[sizeof(ScanHeader) +
                        MAX_POINTS_PER_CHUNK * sizeof(ScanPoint)];
          auto *hdr = reinterpret_cast<ScanHeader *>(chunk);
          hdr->scan_seq = scan_seq;
          hdr->chunk_id = (uint8_t)c;
          hdr->total_chunks = (uint8_t)total_chunks;
          hdr->timestamp_s = cd.timestamp;
          hdr->num_points = (uint16_t)n;
          hdr->reserved = 0;

          auto *pts = reinterpret_cast<ScanPoint *>(chunk + sizeof(ScanHeader));
          for (int p = 0; p < n; p++, pt_idx++) {
            const auto &pt = filtered->points[pt_idx];
            auto clamp = [](float v) -> int16_t {
              float mm = v * 1000.0f;
              if (mm > 32767.0f)
                return 32767;
              if (mm < -32768.0f)
                return -32768;
              return (int16_t)mm;
            };
            pts[p].x_mm = clamp(pt.x);
            pts[p].y_mm = clamp(pt.y);
            pts[p].z_mm = clamp(pt.z);
            pts[p].intensity =
                (uint8_t)std::min(255.0f, std::max(0.0f, pt.intensity));
            pts[p].reserved = 0;
          }

          int payload_len = (int)(sizeof(ScanHeader) + n * sizeof(ScanPoint));
          int plen = build_packet(packet_buf, pkt_id, CMD_LIDAR_SCAN, chunk,
                                  payload_len);
          sendto(sock, packet_buf, plen, 0, (struct sockaddr *)&dest,
                 sizeof(dest));
        }
      }
      scan_st.mark(now);
    }

    // ── Send accumulated path snapshot ──────────────────────────────────
    if (path_st.due(now)) {
      send_path();
      path_st.mark(now);
    }

    // ── Drain planner_occ_queue — send occupancy voxel snapshot ─────────
    if (occ_st.due(now)) {
      livo::OccupancyViz ov;
      bool has_occ = false;
      {
        std::lock_guard<std::mutex> lk(bridge.planner_occ_mtx);
        if (!bridge.planner_occ_queue.empty()) {
          ov = std::move(bridge.planner_occ_queue.back());
          while (!bridge.planner_occ_queue.empty())
            bridge.planner_occ_queue.pop();
          has_occ = true;
        }
      }
      if (has_occ && !ov.occupied_voxels.empty()) {
        const int total_vox = (int)ov.occupied_voxels.size();
        const int max_v = MAX_VOXELS_PER_CHUNK;
        int total_chunks = (total_vox + max_v - 1) / max_v;
        if (total_chunks > 255) total_chunks = 255;
        occ_seq++;
        int vi = 0;
        for (int c = 0; c < total_chunks && vi < total_vox; c++) {
          int n = std::min(max_v, total_vox - vi);
          uint8_t chunk[sizeof(OccHeader) + MAX_VOXELS_PER_CHUNK * sizeof(OccVoxel)];
          auto *hdr = reinterpret_cast<OccHeader *>(chunk);
          hdr->occ_seq      = occ_seq;
          hdr->chunk_id     = (uint8_t)c;
          hdr->total_chunks = (uint8_t)total_chunks;
          hdr->total_voxels = (uint32_t)total_vox;
          hdr->timestamp_s  = ov.timestamp;
          auto *vx = reinterpret_cast<OccVoxel *>(chunk + sizeof(OccHeader));
          for (int i = 0; i < n; i++, vi++) {
            const auto &p = ov.occupied_voxels[vi];
            // Convert metres → cm, clamp to int16 range (±327m)
            auto to_cm = [](float m) -> int16_t {
              float cm = m * 100.0f;
              if (cm >  32767.f) return  32767;
              if (cm < -32768.f) return -32768;
              return (int16_t)cm;
            };
            vx[i].x_cm     = to_cm(p.x());
            vx[i].y_cm     = to_cm(p.y());
            vx[i].z_cm     = to_cm(p.z());
            vx[i].reserved = 0;
          }
          int payload_len = (int)(sizeof(OccHeader) + n * sizeof(OccVoxel));
          int plen = build_packet(packet_buf, pkt_id, CMD_OCC_VOXELS, chunk, payload_len);
          sendto(sock, packet_buf, plen, 0, (struct sockaddr *)&dest, sizeof(dest));
        }
      }
      occ_st.mark(now);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  close(sock);
  fprintf(stderr, "[lidar_publisher] Stopped.\n");
}
