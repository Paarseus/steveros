#ifndef STEVEROS_HARDWARE__ROBSTRIDE_PROTOCOL_HPP_
#define STEVEROS_HARDWARE__ROBSTRIDE_PROTOCOL_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>

#include <linux/can.h>

namespace robstride
{

// ---------------------------------------------------------------------------
// Protocol constants (Robstride MIT Mode)
// ---------------------------------------------------------------------------

static constexpr int kHostId = 0xFD;

// Feedback field ranges
static constexpr float kPosMin = -12.5f;
static constexpr float kPosMax = 12.5f;
static constexpr float kVelMin = -45.0f;
static constexpr float kVelMax = 45.0f;
static constexpr float kKpMin = 0.0f;
static constexpr float kKpMax = 500.0f;
static constexpr float kKdMin = 0.0f;
static constexpr float kKdMax = 5.0f;
static constexpr float kTorqueMin = -12.0f;
static constexpr float kTorqueMax = 12.0f;

// CAN frame type IDs
static constexpr uint32_t kTypeMitCommand = 1;
static constexpr uint32_t kTypeFeedback = 2;
static constexpr uint32_t kTypeEnable = 3;
static constexpr uint32_t kTypeStop = 4;

// ---------------------------------------------------------------------------
// Conversion helpers
// ---------------------------------------------------------------------------

inline uint16_t float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
  x = std::clamp(x, x_min, x_max);
  float span = x_max - x_min;
  return static_cast<uint16_t>((x - x_min) * ((1 << bits) - 1) / span);
}

inline float uint_to_float(uint16_t x_int, float x_min, float x_max, unsigned int bits)
{
  float span = x_max - x_min;
  return static_cast<float>(x_int) * span / ((1 << bits) - 1) + x_min;
}

// ---------------------------------------------------------------------------
// Command structs
// ---------------------------------------------------------------------------

struct MitCommand
{
  int motor_id = 0;
  float p_des = 0.0f;
  float v_des = 0.0f;
  float kp = 0.0f;
  float kd = 0.0f;
  float t_ff = 0.0f;
  float max_torque = 12.0f;  // per-joint torque clamp
};

struct MotorFeedback
{
  int motor_id = 0;
  float position = 0.0f;    // radians
  float velocity = 0.0f;    // rad/s
  float torque = 0.0f;      // Nm
  uint8_t temperature = 0;  // degrees C
  uint8_t status = 0;       // fault/status byte
};

// ---------------------------------------------------------------------------
// Frame encoding
// ---------------------------------------------------------------------------

/// Encode MIT mode command (Type 1 frame). Clamps t_ff to [-max_torque, max_torque].
inline can_frame encode_mit_command(const MitCommand & cmd)
{
  float t_ff = std::clamp(cmd.t_ff, -cmd.max_torque, cmd.max_torque);

  uint16_t p_uint = float_to_uint(cmd.p_des, kPosMin, kPosMax, 16);
  uint16_t v_uint = float_to_uint(cmd.v_des, kVelMin, kVelMax, 16);
  uint16_t kp_uint = float_to_uint(cmd.kp, kKpMin, kKpMax, 16);
  uint16_t kd_uint = float_to_uint(cmd.kd, kKdMin, kKdMax, 16);
  uint16_t t_uint = float_to_uint(t_ff, kTorqueMin, kTorqueMax, 16);

  can_frame frame{};
  frame.can_id =
    (kTypeMitCommand << 24) |
    (static_cast<uint32_t>(t_uint) << 8) |
    static_cast<uint32_t>(cmd.motor_id) |
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>(p_uint >> 8);
  frame.data[1] = static_cast<uint8_t>(p_uint & 0xFF);
  frame.data[2] = static_cast<uint8_t>(v_uint >> 8);
  frame.data[3] = static_cast<uint8_t>(v_uint & 0xFF);
  frame.data[4] = static_cast<uint8_t>(kp_uint >> 8);
  frame.data[5] = static_cast<uint8_t>(kp_uint & 0xFF);
  frame.data[6] = static_cast<uint8_t>(kd_uint >> 8);
  frame.data[7] = static_cast<uint8_t>(kd_uint & 0xFF);
  return frame;
}

/// Encode motor enable command (Type 3 frame).
inline can_frame encode_enable(int motor_id)
{
  can_frame frame{};
  frame.can_id =
    (kTypeEnable << 24) |
    (static_cast<uint32_t>(kHostId) << 8) |
    static_cast<uint32_t>(motor_id) |
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  return frame;
}

/// Encode motor stop command (Type 4 frame).
inline can_frame encode_stop(int motor_id)
{
  can_frame frame{};
  frame.can_id =
    (kTypeStop << 24) |
    (static_cast<uint32_t>(kHostId) << 8) |
    static_cast<uint32_t>(motor_id) |
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  return frame;
}

// ---------------------------------------------------------------------------
// Frame decoding
// ---------------------------------------------------------------------------

/// Decode a CAN frame as motor feedback. Returns nullopt if not a Type 2 frame.
inline std::optional<MotorFeedback> decode_feedback(const can_frame & frame)
{
  uint32_t arb_id = frame.can_id & CAN_EFF_MASK;
  uint32_t comm_type = (arb_id >> 24) & 0x1F;

  if (comm_type != kTypeFeedback) {
    return std::nullopt;
  }

  MotorFeedback fb;
  fb.motor_id = (arb_id >> 8) & 0xFFFF;

  uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  uint16_t vel_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];

  fb.position = uint_to_float(pos_raw, kPosMin, kPosMax, 16);
  fb.velocity = uint_to_float(vel_raw, kVelMin, kVelMax, 16);
  fb.torque = uint_to_float(torque_raw, kTorqueMin, kTorqueMax, 16);
  fb.temperature = frame.data[6];
  fb.status = frame.data[7];

  return fb;
}

}  // namespace robstride

#endif  // STEVEROS_HARDWARE__ROBSTRIDE_PROTOCOL_HPP_
