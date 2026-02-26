/// Read motor 21 (RS03) firmware default parameters.
/// Uses ONLY a raw CAN socket — no driver receive thread to compete with.

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "steveros_hardware/robstride_protocol.hpp"

using namespace robstride;

static constexpr int kMotorId = 21;

static int open_can(const char * iface)
{
  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) { perror("socket"); return -1; }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(fd); return -1; }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("bind"); close(fd); return -1;
  }
  return fd;
}

static bool send_frame(int fd, const can_frame & frame)
{
  return write(fd, &frame, sizeof(frame)) == sizeof(frame);
}

static bool read_frame(int fd, can_frame & out, int timeout_ms)
{
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct timeval tv{0, 10000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (read(fd, &out, sizeof(out)) == sizeof(can_frame)) return true;
  }
  return false;
}

static void dump_frame(const char * label, const can_frame & f)
{
  uint32_t id = f.can_id & CAN_EFF_MASK;
  uint32_t comm_type = (id >> 24) & 0x1F;
  printf("  %s: id=0x%08X type=%u data=", label, id, comm_type);
  for (int i = 0; i < f.can_dlc; i++) printf("%02X ", f.data[i]);
  printf("\n");
}

static bool read_param(int fd, uint16_t index, float & out)
{
  // Send param read
  can_frame req = encode_param_read(kMotorId, index);
  if (!send_frame(fd, req)) {
    printf("  SEND FAILED for 0x%04X\n", index);
    return false;
  }

  // Read all responses for 500ms, look for matching param response
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    can_frame resp;
    if (read_frame(fd, resp, 50)) {
      if (!(resp.can_id & CAN_EFF_FLAG)) continue;

      uint32_t arb_id = resp.can_id & CAN_EFF_MASK;
      uint32_t comm_type = (arb_id >> 24) & 0x1F;

      // Param responses come as Type 0 (some firmware) or Type 17
      if (comm_type == 0 || comm_type == 17) {
        uint16_t resp_index =
          static_cast<uint16_t>(resp.data[0]) |
          (static_cast<uint16_t>(resp.data[1]) << 8);

        dump_frame("PARAM RESP", resp);

        if (resp_index == index) {
          uint32_t bits =
            static_cast<uint32_t>(resp.data[4]) |
            (static_cast<uint32_t>(resp.data[5]) << 8) |
            (static_cast<uint32_t>(resp.data[6]) << 16) |
            (static_cast<uint32_t>(resp.data[7]) << 24);
          std::memcpy(&out, &bits, sizeof(out));
          return true;
        }
      }
      // Skip Type 2 feedback frames silently
      else if (comm_type != 2) {
        dump_frame("OTHER", resp);
      }
    }
  }
  return false;
}

int main()
{
  printf("=== Motor %d (RS03) — Read Firmware Defaults (raw socket only) ===\n\n", kMotorId);

  int fd = open_can("can0");
  if (fd < 0) return 1;

  // Clear faults first to wake up the motor
  printf("Sending clear_fault...\n");
  send_frame(fd, encode_stop_clear_fault(kMotorId));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Drain any queued frames
  {
    can_frame drain;
    struct timeval tv{0, 1000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    while (read(fd, &drain, sizeof(drain)) > 0) {}
  }

  struct { uint16_t index; const char * name; float expected; } params[] = {
    {0x700B, "limit_torque", 60.0f},
    {0x7017, "limit_spd",    20.0f},
    {0x7018, "limit_cur",    27.0f},
    {0x7005, "run_mode",      0.0f},
  };

  printf("\nReading firmware parameters:\n");
  for (const auto & p : params) {
    float value = -999.0f;
    bool ok = read_param(fd, p.index, value);
    if (ok) {
      printf("  >> 0x%04X %-15s = %.4f", p.index, p.name, value);
      if (p.expected > 0.0f) printf("  (expected max: %.1f)", p.expected);
      printf("\n\n");
    } else {
      printf("  >> 0x%04X %-15s = NO RESPONSE\n\n", p.index, p.name);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  close(fd);
  printf("Done.\n");
  return 0;
}
