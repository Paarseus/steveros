#include <gtest/gtest.h>

#include <chrono>
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

// ===========================================================================
// Helpers
// ===========================================================================

static const char * kVcanInterface = "vcan0";

/// Check if vcan0 exists and is usable.
static bool vcan_available()
{
  int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return false;
  }
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, kVcanInterface, IFNAMSIZ - 1);
  bool ok = ::ioctl(fd, SIOCGIFINDEX, &ifr) >= 0;
  ::close(fd);
  return ok;
}

/// Open a second raw CAN socket on vcan0 for injecting frames.
/// Returns fd >= 0 on success, -1 on failure.
static int open_inject_socket()
{
  int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return -1;
  }

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

  return fd;
}

/// Construct a Type 2 feedback CAN frame with known values.
static can_frame make_feedback_frame(
  int motor_id,
  float position,
  float velocity,
  float torque,
  float temperature_c = 25.0f,
  uint8_t fault_bits = 0,
  uint8_t mode = 0)
{
  uint16_t pos_raw = float_to_uint(position, kFbPosMin, kFbPosMax, 16);
  uint16_t vel_raw = float_to_uint(velocity, kFbVelMin, kFbVelMax, 16);
  uint16_t torque_raw = float_to_uint(torque, kFbTorqueMin, kFbTorqueMax, 16);

  can_frame frame {};
  frame.can_id =
    (kTypeFeedback << 24) |
    (static_cast<uint32_t>(mode & 0x03) << 22) |
    (static_cast<uint32_t>(fault_bits & 0x3F) << 16) |
    (static_cast<uint32_t>(motor_id) << 8) |
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>(pos_raw >> 8);
  frame.data[1] = static_cast<uint8_t>(pos_raw & 0xFF);
  frame.data[2] = static_cast<uint8_t>(vel_raw >> 8);
  frame.data[3] = static_cast<uint8_t>(vel_raw & 0xFF);
  frame.data[4] = static_cast<uint8_t>(torque_raw >> 8);
  frame.data[5] = static_cast<uint8_t>(torque_raw & 0xFF);
  uint16_t temp_raw = static_cast<uint16_t>(temperature_c * 10.0f);
  frame.data[6] = static_cast<uint8_t>(temp_raw >> 8);
  frame.data[7] = static_cast<uint8_t>(temp_raw & 0xFF);
  return frame;
}

/// Inject a CAN frame on the given socket. Returns true on success.
static bool inject_frame(int fd, const can_frame & frame)
{
  ssize_t nbytes = ::write(fd, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

/// Poll driver until feedback appears for motor_id, or timeout.
/// Returns true if feedback arrived, false on timeout.
static bool wait_for_feedback(
  const RobstrideCanDriver & driver,
  int motor_id,
  int timeout_ms = 500)
{
  auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (driver.has_valid_feedback(motor_id)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

// ===========================================================================
// Test fixture
// ===========================================================================

class CanDriverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!vcan_available()) {
      GTEST_SKIP() << "vcan0 not available. Set up with:\n"
                    << "  sudo modprobe vcan\n"
                    << "  sudo ip link add dev vcan0 type vcan\n"
                    << "  sudo ip link set up vcan0";
    }
  }

  void SetUp() override
  {
    if (!vcan_available()) {
      GTEST_SKIP() << "vcan0 not available";
    }
    driver_ = std::make_unique<RobstrideCanDriver>();
  }

  void TearDown() override
  {
    if (inject_fd_ >= 0) {
      ::close(inject_fd_);
      inject_fd_ = -1;
    }
    driver_.reset();
  }

  /// Open the driver on vcan0 and prepare inject socket.
  /// Many tests need both, so bundle them.
  void open_driver_and_inject_socket()
  {
    ASSERT_TRUE(driver_->open(kVcanInterface));
    inject_fd_ = open_inject_socket();
    ASSERT_GE(inject_fd_, 0) << "Failed to open inject socket on vcan0";
  }

  std::unique_ptr<RobstrideCanDriver> driver_;
  int inject_fd_ = -1;
};

// ===========================================================================
// Lifecycle tests
// ===========================================================================

TEST_F(CanDriverTest, OpenOnVcanSucceeds)
{
  EXPECT_TRUE(driver_->open(kVcanInterface));
}

TEST_F(CanDriverTest, OpenOnInvalidInterfaceFails)
{
  EXPECT_FALSE(driver_->open("nonexistent_iface_99"));
}

TEST_F(CanDriverTest, CloseIsIdempotent)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  driver_->close();
  driver_->close();  // second close should not crash
}

TEST_F(CanDriverTest, CloseWithoutOpenIsHarmless)
{
  driver_->close();  // never opened — should not crash
}

TEST_F(CanDriverTest, DestructorClosesCleanly)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  driver_.reset();  // destructor calls close()
  // If we get here without hanging or crashing, the thread joined properly
}

TEST_F(CanDriverTest, ReopenAfterClose)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  driver_->close();
  EXPECT_TRUE(driver_->open(kVcanInterface));
}

TEST_F(CanDriverTest, OpenWhenAlreadyOpen)
{
  // open() when already open should close first and re-open
  ASSERT_TRUE(driver_->open(kVcanInterface));
  EXPECT_TRUE(driver_->open(kVcanInterface));
}

// ===========================================================================
// Send tests
// ===========================================================================

TEST_F(CanDriverTest, SendClearFaultSucceeds)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  EXPECT_TRUE(driver_->clear_fault(21));
}

TEST_F(CanDriverTest, SendEnableMotorSucceeds)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  EXPECT_TRUE(driver_->enable_motor(21));
}

TEST_F(CanDriverTest, SendDisableMotorSucceeds)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  EXPECT_TRUE(driver_->disable_motor(21));
}

TEST_F(CanDriverTest, SendSetRunModeSucceeds)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  EXPECT_TRUE(driver_->set_run_mode(21, kRunModeMotionControl));
}

TEST_F(CanDriverTest, SendMitCommandSucceeds)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  MitCommand cmd;
  cmd.motor_id = 21;
  cmd.p_des = 1.0f;
  cmd.kp = 100.0f;
  cmd.kd = 5.0f;
  auto params = get_motor_params(MotorType::RS03);
  auto frame = encode_mit_command(cmd, params);
  EXPECT_TRUE(driver_->send_frame(frame));
}

TEST_F(CanDriverTest, SendMultipleFramesInSequence)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  // Simulate the activation sequence for one motor
  EXPECT_TRUE(driver_->clear_fault(21));
  EXPECT_TRUE(driver_->set_run_mode(21, kRunModeMotionControl));
  EXPECT_TRUE(driver_->enable_motor(21));

  MitCommand cmd;
  cmd.motor_id = 21;
  cmd.kd = 4.0f;
  cmd.max_torque = 60.0f;
  auto params = get_motor_params(MotorType::RS03);
  EXPECT_TRUE(driver_->send_frame(encode_mit_command(cmd, params)));
}

// ===========================================================================
// Receive / feedback cache tests
// ===========================================================================

TEST_F(CanDriverTest, NoFeedbackInitially)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  EXPECT_FALSE(driver_->has_valid_feedback(21));
}

TEST_F(CanDriverTest, GetFeedbackReturnsDefaultWhenEmpty)
{
  ASSERT_TRUE(driver_->open(kVcanInterface));
  auto fb = driver_->get_feedback(21);
  EXPECT_EQ(fb.motor_id, 0);
  EXPECT_FLOAT_EQ(fb.position, 0.0f);
  EXPECT_FLOAT_EQ(fb.velocity, 0.0f);
  EXPECT_FLOAT_EQ(fb.torque, 0.0f);
  EXPECT_FLOAT_EQ(fb.temperature, 0.0f);
  EXPECT_EQ(fb.fault_bits, 0);
  EXPECT_EQ(fb.mode, 0);
}

TEST_F(CanDriverTest, ReceivesFeedbackFromVcan)
{
  open_driver_and_inject_socket();

  // Inject a Type 2 feedback frame for motor 21
  auto frame = make_feedback_frame(21, 1.5f, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame));

  // Wait for background thread to process it
  ASSERT_TRUE(wait_for_feedback(*driver_, 21))
    << "Timed out waiting for feedback from motor 21";
  EXPECT_TRUE(driver_->has_valid_feedback(21));
}

TEST_F(CanDriverTest, DecodesPositionCorrectly)
{
  open_driver_and_inject_socket();

  float target_pos = 3.5f;
  auto frame = make_feedback_frame(21, target_pos, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));

  auto fb = driver_->get_feedback(21);
  EXPECT_EQ(fb.motor_id, 21);
  // 16-bit quantization over 25.0 range -> step ~0.00038
  EXPECT_NEAR(fb.position, target_pos, 0.001f);
}

TEST_F(CanDriverTest, DecodesVelocityCorrectly)
{
  open_driver_and_inject_socket();

  float target_vel = -10.5f;
  auto frame = make_feedback_frame(21, 0.0f, target_vel, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));

  auto fb = driver_->get_feedback(21);
  // 16-bit quantization over 90.0 range -> step ~0.00137
  EXPECT_NEAR(fb.velocity, target_vel, 0.01f);
}

TEST_F(CanDriverTest, DecodesTorqueCorrectly)
{
  open_driver_and_inject_socket();

  float target_torque = 5.25f;
  auto frame = make_feedback_frame(21, 0.0f, 0.0f, target_torque);
  ASSERT_TRUE(inject_frame(inject_fd_, frame));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));

  auto fb = driver_->get_feedback(21);
  // 16-bit quantization over 24.0 range -> step ~0.00037
  EXPECT_NEAR(fb.torque, target_torque, 0.001f);
}

TEST_F(CanDriverTest, DecodesAllFieldsTogether)
{
  open_driver_and_inject_socket();

  float pos = -2.1f;
  float vel = 8.0f;
  float torque = -3.3f;
  float temp = 42.0f;

  auto frame = make_feedback_frame(21, pos, vel, torque, temp);
  ASSERT_TRUE(inject_frame(inject_fd_, frame));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));

  auto fb = driver_->get_feedback(21);
  EXPECT_EQ(fb.motor_id, 21);
  EXPECT_NEAR(fb.position, pos, 0.001f);
  EXPECT_NEAR(fb.velocity, vel, 0.01f);
  EXPECT_NEAR(fb.torque, torque, 0.001f);
  EXPECT_NEAR(fb.temperature, temp, 0.1f);
}

TEST_F(CanDriverTest, MultipleMotosCachedIndependently)
{
  open_driver_and_inject_socket();

  // Inject feedback for motor 21 (position = 1.0)
  auto frame21 = make_feedback_frame(21, 1.0f, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame21));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));

  // Inject feedback for motor 42 (position = -2.0)
  auto frame42 = make_feedback_frame(42, -2.0f, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame42));
  ASSERT_TRUE(wait_for_feedback(*driver_, 42));

  // Both should be independently cached
  EXPECT_TRUE(driver_->has_valid_feedback(21));
  EXPECT_TRUE(driver_->has_valid_feedback(42));

  auto fb21 = driver_->get_feedback(21);
  auto fb42 = driver_->get_feedback(42);
  EXPECT_NEAR(fb21.position, 1.0f, 0.001f);
  EXPECT_NEAR(fb42.position, -2.0f, 0.001f);
  EXPECT_EQ(fb21.motor_id, 21);
  EXPECT_EQ(fb42.motor_id, 42);
}

TEST_F(CanDriverTest, FeedbackUpdatesWithNewerFrame)
{
  open_driver_and_inject_socket();

  // First feedback: position = 1.0
  auto frame1 = make_feedback_frame(21, 1.0f, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame1));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));

  auto fb1 = driver_->get_feedback(21);
  EXPECT_NEAR(fb1.position, 1.0f, 0.001f);

  // Second feedback: position = 5.0 (should overwrite)
  auto frame2 = make_feedback_frame(21, 5.0f, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame2));

  // Poll until position changes (background thread processes new frame)
  auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  bool updated = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto fb = driver_->get_feedback(21);
    if (fb.position > 3.0f) {
      updated = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(updated) << "Feedback cache did not update with second frame";

  auto fb2 = driver_->get_feedback(21);
  EXPECT_NEAR(fb2.position, 5.0f, 0.001f);
}

TEST_F(CanDriverTest, NonFeedbackFrameNotCached)
{
  open_driver_and_inject_socket();

  // Inject a Type 1 (MIT command) frame — should be filtered by the
  // driver's kernel-level CAN_RAW_FILTER (only Type 2 passes)
  can_frame type1_frame {};
  type1_frame.can_id =
    (kTypeMitCommand << 24) |
    (static_cast<uint32_t>(21) << 8) |
    CAN_EFF_FLAG;
  type1_frame.can_dlc = 8;
  std::memset(type1_frame.data, 0, 8);
  ASSERT_TRUE(inject_frame(inject_fd_, type1_frame));

  // Also inject a Type 3 (enable) frame
  can_frame type3_frame {};
  type3_frame.can_id =
    (kTypeEnable << 24) |
    (static_cast<uint32_t>(21) << 8) |
    CAN_EFF_FLAG;
  type3_frame.can_dlc = 8;
  std::memset(type3_frame.data, 0, 8);
  ASSERT_TRUE(inject_frame(inject_fd_, type3_frame));

  // Give background thread time to process (if it were going to)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Motor 21 should NOT have any feedback cached
  EXPECT_FALSE(driver_->has_valid_feedback(21));
}

TEST_F(CanDriverTest, UnrelatedMotorNotAffected)
{
  open_driver_and_inject_socket();

  // Inject feedback for motor 21 only
  auto frame = make_feedback_frame(21, 1.0f, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));

  // Motor 42 should still have no feedback
  EXPECT_FALSE(driver_->has_valid_feedback(42));
}

TEST_F(CanDriverTest, FeedbackCacheClearedOnClose)
{
  open_driver_and_inject_socket();

  auto frame = make_feedback_frame(21, 1.0f, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));
  EXPECT_TRUE(driver_->has_valid_feedback(21));

  driver_->close();

  // After close, cache should be cleared
  EXPECT_FALSE(driver_->has_valid_feedback(21));
}

// ===========================================================================
// Edge case: feedback at extreme values
// ===========================================================================

TEST_F(CanDriverTest, FeedbackAtPositionExtremes)
{
  open_driver_and_inject_socket();

  // Position at maximum (12.5 rad)
  auto frame_max = make_feedback_frame(21, kFbPosMax, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame_max));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));
  auto fb = driver_->get_feedback(21);
  EXPECT_NEAR(fb.position, kFbPosMax, 0.001f);

  // Position at minimum (-12.5 rad) — inject for motor 22 to avoid
  // needing to wait for cache update on the same motor
  auto frame_min = make_feedback_frame(22, kFbPosMin, 0.0f, 0.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame_min));
  ASSERT_TRUE(wait_for_feedback(*driver_, 22));
  fb = driver_->get_feedback(22);
  EXPECT_NEAR(fb.position, kFbPosMin, 0.001f);
}

TEST_F(CanDriverTest, FeedbackWithHighTemperature)
{
  open_driver_and_inject_socket();

  auto frame = make_feedback_frame(21, 0.0f, 0.0f, 0.0f, 80.0f);
  ASSERT_TRUE(inject_frame(inject_fd_, frame));
  ASSERT_TRUE(wait_for_feedback(*driver_, 21));

  auto fb = driver_->get_feedback(21);
  EXPECT_NEAR(fb.temperature, 80.0f, 0.1f);
}

// ===========================================================================
// Burst: multiple motors in quick succession
// ===========================================================================

TEST_F(CanDriverTest, BurstFeedbackForManyMotors)
{
  open_driver_and_inject_socket();

  // Simulate 20-motor feedback burst (like real robot)
  const int motor_ids[] = {
    11, 12, 13, 14, 15,   // left arm
    21, 22, 23, 24, 25,   // right arm
    31, 32, 33, 34, 35,   // left leg
    41, 42, 43, 44, 45    // right leg
  };

  for (int id : motor_ids) {
    float pos = static_cast<float>(id) * 0.1f;  // unique position per motor
    auto frame = make_feedback_frame(id, pos, 0.0f, 0.0f);
    ASSERT_TRUE(inject_frame(inject_fd_, frame));
  }

  // Wait for the last motor to appear
  ASSERT_TRUE(wait_for_feedback(*driver_, 45))
    << "Timed out waiting for all 20 motors";

  // Verify all 20 motors cached with correct positions
  for (int id : motor_ids) {
    ASSERT_TRUE(driver_->has_valid_feedback(id))
      << "Missing feedback for motor " << id;
    auto fb = driver_->get_feedback(id);
    float expected_pos = static_cast<float>(id) * 0.1f;
    EXPECT_NEAR(fb.position, expected_pos, 0.001f)
      << "Wrong position for motor " << id;
  }
}
