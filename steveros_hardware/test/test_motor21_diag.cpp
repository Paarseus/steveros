/// Motor 21 (RS03) diagnostic test.
///
/// Phase 1: Read firmware parameters (limit_torque, limit_cur, limit_spd)
/// Phase 2: Test encoding ranges — send known Kp, observe torque response
/// Phase 3: Write limit_torque=60, limit_cur=27, retest
///
/// This test determines:
///   (a) Are firmware limits capping our torque output?
///   (b) Are our Kp/Kd encoding ranges correct for the RS03 firmware?

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "steveros_hardware/robstride_can_driver.hpp"
#include "steveros_hardware/robstride_protocol.hpp"

using namespace robstride;

static constexpr int kMotorId = 21;
static constexpr MotorType kMotorType = MotorType::RS03;
static constexpr auto kParams = get_motor_params(kMotorType);

static RobstrideCanDriver * g_driver = nullptr;

static void signal_handler(int sig)
{
  printf("\nCaught signal %d, disabling motor...\n", sig);
  if (g_driver) {
    MitCommand zero{};
    zero.motor_id = kMotorId;
    g_driver->send_frame(encode_mit_command(zero, kParams));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    g_driver->disable_motor(kMotorId);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    g_driver->close();
  }
  _exit(1);
}

// ---------------------------------------------------------------------------
// Raw CAN read helper — reads any frame from the bus (not just Type 2)
// ---------------------------------------------------------------------------

/// Read a raw CAN frame with timeout. Returns true if a frame was received.
static bool read_raw_frame(int socket_fd, can_frame & out_frame, int timeout_ms)
{
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10ms poll
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ssize_t nbytes = read(socket_fd, &out_frame, sizeof(out_frame));
    if (nbytes == sizeof(can_frame)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

/// Try to read a parameter response. Motor replies with comm_type=0 or 17.
/// Response format: arb_id has motor_id in bits 7:0,
/// data[0:1] = index LE, data[4:7] = value LE float.
static bool read_param_response(int socket_fd, uint16_t expected_index,
  float & out_value, int timeout_ms = 500)
{
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    can_frame frame;
    if (read_raw_frame(socket_fd, frame, 50)) {
      // Check if this is a param response (Type 0 or Type 17)
      if (frame.can_id & CAN_EFF_FLAG) {
        uint32_t arb_id = frame.can_id & CAN_EFF_MASK;
        uint32_t comm_type = (arb_id >> 24) & 0x1F;
        uint32_t motor_id_resp = arb_id & 0xFF;

        // Type 0 (broadcast/response) or Type 17 (param read response)
        if ((comm_type == 0 || comm_type == 17) &&
            motor_id_resp == static_cast<uint32_t>(kMotorId))
        {
          uint16_t resp_index =
            static_cast<uint16_t>(frame.data[0]) |
            (static_cast<uint16_t>(frame.data[1]) << 8);

          if (resp_index == expected_index) {
            uint32_t val_bits =
              static_cast<uint32_t>(frame.data[4]) |
              (static_cast<uint32_t>(frame.data[5]) << 8) |
              (static_cast<uint32_t>(frame.data[6]) << 16) |
              (static_cast<uint32_t>(frame.data[7]) << 24);
            std::memcpy(&out_value, &val_bits, sizeof(out_value));
            return true;
          }
        }
      }
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Phase 1: Read firmware parameters
// ---------------------------------------------------------------------------

struct ParamInfo {
  uint16_t index;
  const char * name;
};

static const ParamInfo kParamsToRead[] = {
  {0x700B, "limit_torque"},
  {0x7017, "limit_spd"},
  {0x7018, "limit_cur"},
  {0x7014, "cur_filt_gain"},
  {0x701A, "iqf (filtered current)"},
  {0x701C, "VBUS (bus voltage)"},
};

static void phase1_read_params(RobstrideCanDriver & driver, int socket_fd)
{
  printf("=== Phase 1: Read Firmware Parameters ===\n");
  printf("  Sending Type 17 (param read) requests...\n\n");

  for (const auto & p : kParamsToRead) {
    // Send param read request
    driver.send_frame(encode_param_read(kMotorId, p.index));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    float value = 0.0f;
    if (read_param_response(socket_fd, p.index, value)) {
      printf("  0x%04X %-25s = %.4f\n", p.index, p.name, value);
    } else {
      printf("  0x%04X %-25s = NO RESPONSE\n", p.index, p.name);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  printf("\n");
}

// ---------------------------------------------------------------------------
// Phase 2: Test encoding ranges — Kp stiffness test
// ---------------------------------------------------------------------------

static void send_mit(RobstrideCanDriver & driver,
  float pos, float vel, float kp, float kd, float t_ff = 0.0f)
{
  MitCommand cmd{};
  cmd.motor_id = kMotorId;
  cmd.p_des = pos;
  cmd.v_des = vel;
  cmd.kp = kp;
  cmd.kd = kd;
  cmd.t_ff = t_ff;
  cmd.max_torque = 30.0f;
  driver.send_frame(encode_mit_command(cmd, kParams));
}

static void phase2_test_encoding(RobstrideCanDriver & driver)
{
  printf("=== Phase 2: Encoding Range Test ===\n");
  printf("  Motor is enabled at current position.\n");
  printf("  We send MIT commands with known Kp and p_des offset,\n");
  printf("  then measure the resulting torque to verify Kp is correct.\n\n");

  // Read current position
  auto fb = driver.get_feedback(kMotorId);
  float home = fb.position;
  printf("  Home position: %+.2f° (%+.4f rad)\n\n", home * 180.0f / kPi, home);

  // Test: hold at home with various Kp values, then offset p_des by 0.05 rad (2.86°)
  // and measure torque. Expected torque = Kp * 0.05 rad
  float offset = 0.05f;  // ~2.86°, small enough to not cause big movement

  // First hold at home
  printf("  Holding at home with Kp=0, Kd=4 for 0.5s...\n");
  for (int i = 0; i < 50; i++) {
    send_mit(driver, home, 0.0f, 0.0f, 4.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  struct KpTest {
    float kp;
    const char * label;
  };

  KpTest tests[] = {
    {10.0f,  "Kp=10  → expect τ=0.5 Nm (if 10x: τ=0.05 Nm)"},
    {50.0f,  "Kp=50  → expect τ=2.5 Nm (if 10x: τ=0.25 Nm)"},
    {100.0f, "Kp=100 → expect τ=5.0 Nm (if 10x: τ=0.50 Nm)"},
    {200.0f, "Kp=200 → expect τ=10  Nm (if 10x: τ=1.0  Nm)"},
    {500.0f, "Kp=500 → expect τ=25  Nm (if 10x: τ=2.5  Nm)"},
  };

  printf("  Offset: %+.3f rad (%+.1f°)\n\n", offset, offset * 180.0f / kPi);
  printf("  %-50s  actual_torque  actual_pos°  conclusion\n", "test");
  printf("  %-50s  ------------  -----------  ----------\n", "----");

  for (const auto & t : tests) {
    // Ramp to this Kp at home position
    for (int i = 0; i < 30; i++) {
      float alpha = static_cast<float>(i) / 30;
      send_mit(driver, home, 0.0f, alpha * t.kp, 4.0f);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Hold at home briefly
    for (int i = 0; i < 20; i++) {
      send_mit(driver, home, 0.0f, t.kp, 4.0f);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Now offset p_des by +offset (motor will try to move there)
    // Apply for 0.5s and measure steady-state torque
    float torque_sum = 0.0f;
    float pos_sum = 0.0f;
    int samples = 0;
    for (int i = 0; i < 50; i++) {
      send_mit(driver, home + offset, 0.0f, t.kp, 4.0f);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (i >= 30) {  // Last 200ms for measurement
        fb = driver.get_feedback(kMotorId);
        torque_sum += fb.torque;
        pos_sum += fb.position;
        samples++;
      }
    }

    float avg_torque = torque_sum / samples;
    float avg_pos = pos_sum / samples;
    float pos_err = (home + offset) - avg_pos;
    float expected_torque = t.kp * pos_err;

    // Determine if the torque matches our encoding or CyberGear encoding
    const char * conclusion;
    if (std::fabs(avg_torque) < 0.1f) {
      conclusion = "NO TORQUE (motor stuck or Kp way too low)";
    } else if (std::fabs(avg_torque - t.kp * offset) < std::fabs(avg_torque) * 0.5f) {
      conclusion = "MATCHES our ranges";
    } else if (std::fabs(avg_torque - (t.kp / 10.0f) * offset) < std::fabs(avg_torque) * 0.5f) {
      conclusion = "MATCHES CyberGear ranges (10x lower)";
    } else {
      conclusion = "INCONCLUSIVE";
    }

    printf("  %-50s  %+6.2f Nm     %+7.2f°     %s\n",
      t.label, avg_torque, avg_pos * 180.0f / kPi, conclusion);

    // Return to home
    for (int i = 0; i < 30; i++) {
      send_mit(driver, home, 0.0f, t.kp, 4.0f);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Ramp down
  for (int i = 30; i >= 0; i--) {
    send_mit(driver, home, 0.0f, static_cast<float>(i) / 30 * 10.0f, 4.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  printf("\n");
}

// ---------------------------------------------------------------------------
// Phase 3: Write limits and retest
// ---------------------------------------------------------------------------

static void phase3_write_limits_and_test(RobstrideCanDriver & driver, int socket_fd)
{
  printf("=== Phase 3: Write Firmware Limits and Retest ===\n");

  // Write limit_torque = 60 Nm (RS03 peak)
  printf("  Writing limit_torque (0x700B) = 60.0 Nm...\n");
  driver.send_frame(encode_param_write(kMotorId, kParamLimitTorque, 60.0f));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Write limit_cur = 27 A (RS03 needs ~25A for 60 Nm)
  printf("  Writing limit_cur (0x7018) = 27.0 A...\n");
  driver.send_frame(encode_param_write(kMotorId, kParamLimitCur, 27.0f));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Write limit_spd = 20 rad/s (RS03 max)
  printf("  Writing limit_spd (0x7017) = 20.0 rad/s...\n");
  driver.send_frame(encode_param_write(kMotorId, kParamLimitSpd, 20.0f));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Read back to confirm
  printf("\n  Reading back parameters:\n");
  for (const auto & p : kParamsToRead) {
    driver.send_frame(encode_param_read(kMotorId, p.index));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    float value = 0.0f;
    if (read_param_response(socket_fd, p.index, value)) {
      printf("  0x%04X %-25s = %.4f\n", p.index, p.name, value);
    } else {
      printf("  0x%04X %-25s = NO RESPONSE\n", p.index, p.name);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  // Now do a small movement test (±10°) to see if behavior improved
  printf("\n  Small movement test: ±10° with Kp=200, Kd=5\n");

  auto fb = driver.get_feedback(kMotorId);
  float home = fb.position;
  float move = 0.1745f;  // 10°

  // Enable and hold at home
  driver.set_run_mode(kMotorId, kRunModeMotionControl);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  driver.enable_motor(kMotorId);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // Ramp Kp
  for (int i = 0; i < 100; i++) {
    float alpha = static_cast<float>(i) / 100;
    send_mit(driver, home, 0.0f, alpha * 200.0f, 5.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Move +10° over 2 seconds with cosine trajectory + v_des
  printf("  step  cmd_pos°   v_des   actual°    vel    err°   torque\n");
  int move_steps = 200;
  float duration = 2.0f;
  float max_err = 0.0f;

  for (int i = 0; i <= move_steps; i++) {
    float t = static_cast<float>(i) / move_steps;
    float s = 0.5f * (1.0f - std::cos(kPi * t));
    float cmd_pos = home + s * move;
    float v_des = 0.5f * kPi * std::sin(kPi * t) * move / duration;

    send_mit(driver, cmd_pos, v_des, 200.0f, 5.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (i % 20 == 0 || i == move_steps) {
      fb = driver.get_feedback(kMotorId);
      float err = fb.position - cmd_pos;
      max_err = std::fmax(max_err, std::fabs(err));
      printf("  %4d %+8.1f %+6.3f  %+8.1f %+6.3f %+6.1f %+6.2f\n",
        i, cmd_pos * 180.0f / kPi, v_des,
        fb.position * 180.0f / kPi, fb.velocity,
        err * 180.0f / kPi, fb.torque);
    }
  }

  // Hold
  for (int i = 0; i < 50; i++) {
    send_mit(driver, home + move, 0.0f, 200.0f, 5.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  fb = driver.get_feedback(kMotorId);
  printf("  [hold] pos=%+.1f° err=%+.1f° torque=%+.2f\n",
    fb.position * 180.0f / kPi,
    (fb.position - (home + move)) * 180.0f / kPi,
    fb.torque);

  // Return home
  printf("\n  Returning home...\n");
  for (int i = 0; i <= move_steps; i++) {
    float t = static_cast<float>(i) / move_steps;
    float s = 0.5f * (1.0f - std::cos(kPi * t));
    float cmd_pos = (home + move) + s * (-move);
    float v_des = 0.5f * kPi * std::sin(kPi * t) * (-move) / duration;
    send_mit(driver, cmd_pos, v_des, 200.0f, 5.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Hold at home
  for (int i = 0; i < 50; i++) {
    send_mit(driver, home, 0.0f, 200.0f, 5.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  fb = driver.get_feedback(kMotorId);
  printf("  [home] pos=%+.1f° err=%+.2f°\n",
    fb.position * 180.0f / kPi,
    (fb.position - home) * 180.0f / kPi);

  // Ramp down and disable
  for (int i = 100; i >= 0; i--) {
    send_mit(driver, home, 0.0f, static_cast<float>(i) / 100 * 200.0f, 5.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  send_mit(driver, 0.0f, 0.0f, 0.0f, 0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  driver.disable_motor(kMotorId);

  printf("\n  Max tracking error: %.2f° (%.4f rad)\n",
    max_err * 180.0f / kPi, max_err);
  printf("  %s\n\n", max_err < 0.087f ? "PASS (<5°)" : "FAIL (>=5°)");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main()
{
  printf("=== Motor %d (RS03) Diagnostic Test ===\n\n", kMotorId);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  RobstrideCanDriver driver;
  g_driver = &driver;

  if (!driver.open("can0")) {
    fprintf(stderr, "ERROR: Failed to open can0\n");
    return 1;
  }
  driver.register_motor(kMotorId, kMotorType);

  // We also need a raw socket for reading param responses
  // (the driver's receive_loop filters for Type 2 only)
  // HACK: open a second socket for raw reads
  int raw_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (raw_fd < 0) {
    fprintf(stderr, "WARNING: Could not open raw CAN socket for param reads.\n");
    fprintf(stderr, "         Phase 1 and 3 param reads will not work.\n");
  } else {
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, "can0", sizeof(ifr.ifr_name));
    ioctl(raw_fd, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(raw_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
  }

  // Clear faults and get initial feedback
  driver.clear_fault(kMotorId);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if (!driver.has_valid_feedback(kMotorId)) {
    // Wait longer
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  if (!driver.has_valid_feedback(kMotorId)) {
    fprintf(stderr, "ERROR: No feedback from motor %d\n", kMotorId);
    driver.close();
    return 1;
  }
  auto fb = driver.get_feedback(kMotorId);
  printf("Motor %d online: pos=%+.2f° temp=%.1f°C fault=0x%02X mode=%u\n\n",
    kMotorId, fb.position * 180.0f / kPi,
    fb.temperature, fb.fault_bits, fb.mode);

  // --- Phase 1: Read firmware parameters ---
  if (raw_fd >= 0) {
    phase1_read_params(driver, raw_fd);
  } else {
    printf("=== Phase 1: SKIPPED (no raw socket) ===\n\n");
  }

  // --- Phase 2: Encoding range test ---
  printf("  Enabling motor for Phase 2...\n");
  driver.set_run_mode(kMotorId, kRunModeMotionControl);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  driver.enable_motor(kMotorId);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  send_mit(driver, 0.0f, 0.0f, 0.0f, 4.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  phase2_test_encoding(driver);

  // Disable between phases
  send_mit(driver, 0.0f, 0.0f, 0.0f, 0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  driver.disable_motor(kMotorId);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // --- Phase 3: Write limits and retest ---
  if (raw_fd >= 0) {
    phase3_write_limits_and_test(driver, raw_fd);
  } else {
    printf("=== Phase 3: SKIPPED (no raw socket) ===\n\n");
  }

  // Final cleanup
  printf("=== Diagnostic Complete ===\n");
  if (raw_fd >= 0) ::close(raw_fd);
  g_driver = nullptr;
  driver.close();
  return 0;
}
