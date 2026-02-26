#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "steveros_hardware/robstride_can_driver.hpp"
#include "steveros_hardware/robstride_protocol.hpp"

using namespace robstride;

// ===========================================================================
// Helpers
// ===========================================================================

static const char * kVcanInterface = "vcan0";

static bool vcan_available()
{
  int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) return false;
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, kVcanInterface, IFNAMSIZ - 1);
  bool ok = ::ioctl(fd, SIOCGIFINDEX, &ifr) >= 0;
  ::close(fd);
  return ok;
}

/// Open a "spy" socket that receives ALL frame types (no filter).
static int open_spy_socket()
{
  int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) return -1;

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, kVcanInterface, IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd);
    return -1;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }

  // Disable loopback so spy doesn't see its own frames
  int loopback = 0;
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  // Short read timeout
  struct timeval tv {};
  tv.tv_usec = 50000;  // 50ms
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  return fd;
}

/// Read all available frames from spy socket (non-blocking drain).
static std::vector<can_frame> drain_spy(int spy_fd, int timeout_ms = 100)
{
  std::vector<can_frame> frames;
  auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    struct pollfd pfd {};
    pfd.fd = spy_fd;
    pfd.events = POLLIN;

    int ret = ::poll(&pfd, 1, 10);
    if (ret <= 0) break;

    can_frame frame {};
    ssize_t nbytes = ::read(spy_fd, &frame, sizeof(frame));
    if (nbytes == sizeof(frame)) {
      frames.push_back(frame);
    }
  }
  return frames;
}

/// Decode a CAN frame type from its arbitration ID.
static const char * frame_type_name(uint32_t can_id)
{
  uint32_t arb = can_id & CAN_EFF_MASK;
  uint32_t type = (arb >> 24) & 0x1F;
  switch (type) {
    case 1:  return "MIT_CMD";
    case 2:  return "FEEDBACK";
    case 3:  return "ENABLE";
    case 4:  return "STOP";
    case 18: return "PARAM_WRITE";
    default: return "UNKNOWN";
  }
}

/// Print a CAN frame in human-readable format.
static void print_frame(const can_frame & frame, const char * label = "")
{
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint32_t type = (arb >> 24) & 0x1F;
  int motor_id = arb & 0xFF;

  printf("  %-20s | Type=%2u %-12s | motor=%3d | arb=0x%08X | data=",
    label, type, frame_type_name(frame.can_id), motor_id, arb);
  for (int i = 0; i < frame.can_dlc; ++i) {
    printf("%02X ", frame.data[i]);
  }
  printf("\n");
}

/// Decode a MIT command frame and print the control values.
static void decode_and_print_mit(const can_frame & frame, const MotorParams & params)
{
  uint32_t arb = frame.can_id & CAN_EFF_MASK;

  uint16_t p_uint = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  uint16_t v_uint = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  uint16_t kp_uint = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
  uint16_t kd_uint = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
  uint16_t t_uint = (arb >> 8) & 0xFFFF;

  float pos = uint_to_float(p_uint, params.pos_min, params.pos_max, 16);
  float vel = uint_to_float(v_uint, params.vel_min, params.vel_max, 16);
  float kp = uint_to_float(kp_uint, params.kp_min, params.kp_max, 16);
  float kd = uint_to_float(kd_uint, params.kd_min, params.kd_max, 16);
  float torque = uint_to_float(t_uint, params.torque_min, params.torque_max, 16);

  printf("    -> Decoded: pos=%+7.3f rad  vel=%+7.3f rad/s  Kp=%7.1f  Kd=%6.2f  t_ff=%+6.2f Nm\n",
    pos, vel, kp, kd, torque);
}

// ===========================================================================
// Test fixture
// ===========================================================================

class CanBusInspection : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!vcan_available()) {
      GTEST_SKIP() << "vcan0 not available";
    }
  }

  void SetUp() override
  {
    if (!vcan_available()) {
      GTEST_SKIP() << "vcan0 not available";
    }
    driver_ = std::make_unique<RobstrideCanDriver>();
    ASSERT_TRUE(driver_->open(kVcanInterface));
    spy_fd_ = open_spy_socket();
    ASSERT_GE(spy_fd_, 0);
    // Small delay to let the receive thread start
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // Drain any stale frames
    drain_spy(spy_fd_, 20);
  }

  void TearDown() override
  {
    if (spy_fd_ >= 0) {
      ::close(spy_fd_);
      spy_fd_ = -1;
    }
    driver_.reset();
  }

  std::unique_ptr<RobstrideCanDriver> driver_;
  int spy_fd_ = -1;
};

// ===========================================================================
// Tests: Observe how different motor values appear on the CAN bus
// ===========================================================================

TEST_F(CanBusInspection, ActivationSequenceOnBus)
{
  printf("\n=== Activation sequence for motor 21 (RS03) ===\n");
  auto params = get_motor_params(MotorType::RS03);

  // Phase 0: Clear faults
  driver_->clear_fault(21);
  auto frames = drain_spy(spy_fd_);
  ASSERT_EQ(frames.size(), 1u);
  print_frame(frames[0], "clear_fault(21)");
  EXPECT_EQ(frames[0].data[0], 1);  // fault clear flag

  // Phase 1a: Set run mode
  driver_->set_run_mode(21, kRunModeMotionControl);
  frames = drain_spy(spy_fd_);
  ASSERT_EQ(frames.size(), 1u);
  print_frame(frames[0], "set_run_mode(21,0)");

  // Phase 1b: Enable
  driver_->enable_motor(21);
  frames = drain_spy(spy_fd_);
  ASSERT_EQ(frames.size(), 1u);
  print_frame(frames[0], "enable(21)");

  // Phase 1c: Soft enable (Kp=0, Kd=4.0)
  MitCommand soft;
  soft.motor_id = 21;
  soft.kd = 4.0f;
  soft.max_torque = 60.0f;
  driver_->send_frame(encode_mit_command(soft, params));
  frames = drain_spy(spy_fd_);
  ASSERT_EQ(frames.size(), 1u);
  print_frame(frames[0], "soft_enable");
  decode_and_print_mit(frames[0], params);
}

TEST_F(CanBusInspection, PositionChangesOnBus)
{
  printf("\n=== Position commands at different angles (RS03, motor 21) ===\n");
  auto params = get_motor_params(MotorType::RS03);

  float positions[] = {-3.14f, -1.0f, 0.0f, 0.5f, 1.57f, 3.14f};

  for (float pos : positions) {
    MitCommand cmd;
    cmd.motor_id = 21;
    cmd.p_des = pos;
    cmd.kp = 200.0f;
    cmd.kd = 10.0f;
    cmd.max_torque = 60.0f;

    driver_->send_frame(encode_mit_command(cmd, params));
    auto frames = drain_spy(spy_fd_);
    ASSERT_EQ(frames.size(), 1u);

    char label[64];
    snprintf(label, sizeof(label), "pos=%+.2f rad", pos);
    print_frame(frames[0], label);
    decode_and_print_mit(frames[0], params);
  }
}

TEST_F(CanBusInspection, KpRampOnBus)
{
  printf("\n=== Kp ramp: 0 -> 200 in steps (RS03, motor 21) ===\n");
  auto params = get_motor_params(MotorType::RS03);

  float kp_values[] = {0.0f, 20.0f, 50.0f, 100.0f, 150.0f, 200.0f};

  for (float kp : kp_values) {
    MitCommand cmd;
    cmd.motor_id = 21;
    cmd.p_des = 1.0f;
    cmd.kp = kp;
    cmd.kd = 10.0f;
    cmd.max_torque = 60.0f;

    driver_->send_frame(encode_mit_command(cmd, params));
    auto frames = drain_spy(spy_fd_);
    ASSERT_EQ(frames.size(), 1u);

    char label[64];
    snprintf(label, sizeof(label), "Kp=%.0f", kp);
    print_frame(frames[0], label);
    decode_and_print_mit(frames[0], params);
  }
}

TEST_F(CanBusInspection, SameKpDifferentMotorTypes)
{
  printf("\n=== Same Kp=100 on RS02 vs RS03 vs RS04 — byte-level difference ===\n");

  struct MotorSetup {
    const char * name;
    MotorType type;
    int motor_id;
  };

  MotorSetup setups[] = {
    {"RS02", MotorType::RS02, 23},
    {"RS03", MotorType::RS03, 21},
    {"RS04", MotorType::RS04, 41},
  };

  for (auto & s : setups) {
    auto params = get_motor_params(s.type);
    MitCommand cmd;
    cmd.motor_id = s.motor_id;
    cmd.p_des = 1.0f;
    cmd.kp = 100.0f;
    cmd.kd = 5.0f;
    cmd.max_torque = std::min(17.0f, params.torque_max);

    driver_->send_frame(encode_mit_command(cmd, params));
    auto frames = drain_spy(spy_fd_);
    ASSERT_EQ(frames.size(), 1u);

    uint16_t kp_uint = (static_cast<uint16_t>(frames[0].data[4]) << 8) | frames[0].data[5];

    char label[64];
    snprintf(label, sizeof(label), "%s (motor %d)", s.name, s.motor_id);
    print_frame(frames[0], label);
    decode_and_print_mit(frames[0], params);
    printf("    -> Kp raw uint16 = %u (0x%04X)\n\n", kp_uint, kp_uint);
  }
}

TEST_F(CanBusInspection, TorqueFeedforwardInCanId)
{
  printf("\n=== Torque feedforward encoded in CAN ID bits (RS03) ===\n");
  auto params = get_motor_params(MotorType::RS03);

  float torques[] = {-10.0f, -5.0f, 0.0f, 5.0f, 10.0f};

  for (float t : torques) {
    MitCommand cmd;
    cmd.motor_id = 21;
    cmd.p_des = 0.0f;
    cmd.kp = 100.0f;
    cmd.kd = 10.0f;
    cmd.t_ff = t;
    cmd.max_torque = 60.0f;

    driver_->send_frame(encode_mit_command(cmd, params));
    auto frames = drain_spy(spy_fd_);
    ASSERT_EQ(frames.size(), 1u);

    uint32_t arb = frames[0].can_id & CAN_EFF_MASK;
    uint16_t t_in_id = (arb >> 8) & 0xFFFF;

    char label[64];
    snprintf(label, sizeof(label), "t_ff=%+.1f Nm", t);
    print_frame(frames[0], label);
    printf("    -> Torque in CAN ID: raw=%u (0x%04X), decoded=%+.2f Nm\n\n",
      t_in_id, t_in_id,
      uint_to_float(t_in_id, params.torque_min, params.torque_max, 16));
  }
}

TEST_F(CanBusInspection, StopVsClearFaultDifference)
{
  printf("\n=== Stop vs Clear Fault — only data[0] differs ===\n");

  driver_->disable_motor(21);
  auto frames = drain_spy(spy_fd_);
  ASSERT_EQ(frames.size(), 1u);
  print_frame(frames[0], "stop(21)");

  driver_->clear_fault(21);
  frames = drain_spy(spy_fd_);
  ASSERT_EQ(frames.size(), 1u);
  print_frame(frames[0], "clear_fault(21)");
}
