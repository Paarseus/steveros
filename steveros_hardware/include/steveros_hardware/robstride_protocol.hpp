#ifndef STEVEROS_HARDWARE__ROBSTRIDE_PROTOCOL_HPP_
#define STEVEROS_HARDWARE__ROBSTRIDE_PROTOCOL_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

#include <linux/can.h>

namespace robstride
{

// ---------------------------------------------------------------------------
// Protocol constants (Robstride MIT Mode)
// ---------------------------------------------------------------------------

static constexpr float kPi = 3.14159265358979323846f;

static constexpr int kHostId = 0xFD;

// Feedback decode range for position (same for all motor types)
static constexpr float kFbPosMin = -4 * kPi;
static constexpr float kFbPosMax = 4 * kPi;

// CAN frame type IDs
static constexpr uint32_t kTypeMitCommand = 1;
static constexpr uint32_t kTypeFeedback = 2;
static constexpr uint32_t kTypeEnable = 3;
static constexpr uint32_t kTypeStop = 4;
static constexpr uint32_t kTypeParamRead = 17;
static constexpr uint32_t kTypeParamWrite = 18;

// Parameter indices
static constexpr uint16_t kParamRunMode = 0x7005;
static constexpr uint16_t kParamLimitTorque = 0x700B;
static constexpr uint16_t kParamCurFilterGain = 0x7014;
static constexpr uint16_t kParamLimitSpd = 0x7017;
static constexpr uint16_t kParamLimitCur = 0x7018;

// Run modes
static constexpr uint8_t kRunModeMotionControl = 0;

// ---------------------------------------------------------------------------
// Per-model motor parameters
// ---------------------------------------------------------------------------

enum class MotorType { RS01, RS02, RS03, RS04 };

struct MotorParams
{
  float pos_min, pos_max;
  float vel_min, vel_max;
  float torque_min, torque_max;
  float kp_min, kp_max;
  float kd_min, kd_max;
};

constexpr MotorParams get_motor_params(MotorType type)
{
  switch (type) {
    case MotorType::RS01:
      return {-4*kPi, 4*kPi, -44.0f, 44.0f, -17.0f, 17.0f, 0.0f, 500.0f, 0.0f, 5.0f};
    case MotorType::RS02:
      return {-4*kPi, 4*kPi, -44.0f, 44.0f, -17.0f, 17.0f, 0.0f, 500.0f, 0.0f, 5.0f};
    case MotorType::RS03:
      return {-4*kPi, 4*kPi, -20.0f, 20.0f, -60.0f, 60.0f, 0.0f, 5000.0f, 0.0f, 100.0f};
    case MotorType::RS04:
      return {-4*kPi, 4*kPi, -15.0f, 15.0f, -120.0f, 120.0f, 0.0f, 5000.0f, 0.0f, 100.0f};
  }
  // Unreachable, but silences compiler warnings
  return {-4*kPi, 4*kPi, -44.0f, 44.0f, -17.0f, 17.0f, 0.0f, 500.0f, 0.0f, 5.0f};
}

inline MotorType parse_motor_type(const std::string & s)
{
  if (s == "RS01") return MotorType::RS01;
  if (s == "RS02") return MotorType::RS02;
  if (s == "RS03") return MotorType::RS03;
  if (s == "RS04") return MotorType::RS04;
  throw std::invalid_argument("Unknown motor_type: " + s);
}

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
  float max_torque = 12.0f;
};

struct MotorFeedback
{
  int motor_id = 0;
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  float temperature = 0.0f;
  uint8_t fault_bits = 0;  // arb ID bits 21:16
  uint8_t mode = 0;        // arb ID bits 23:22 (0=Reset 1=Cali 2=Run)
};

// ---------------------------------------------------------------------------
// Frame encoding
// ---------------------------------------------------------------------------

/// Encode MIT command with per-model encoding ranges.
inline can_frame encode_mit_command(const MitCommand & cmd, const MotorParams & params)
{
  float t_ff = std::clamp(cmd.t_ff, -cmd.max_torque, cmd.max_torque);

  uint16_t p_uint = float_to_uint(cmd.p_des, params.pos_min, params.pos_max, 16);
  uint16_t v_uint = float_to_uint(cmd.v_des, params.vel_min, params.vel_max, 16);
  uint16_t kp_uint = float_to_uint(cmd.kp, params.kp_min, params.kp_max, 16);
  uint16_t kd_uint = float_to_uint(cmd.kd, params.kd_min, params.kd_max, 16);
  uint16_t t_uint = float_to_uint(t_ff, params.torque_min, params.torque_max, 16);

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

/// Legacy overload: encodes using hardcoded RS01/RS02 ranges for backward compatibility.
inline can_frame encode_mit_command(const MitCommand & cmd)
{
  static constexpr MotorParams kDefaultParams = get_motor_params(MotorType::RS02);
  return encode_mit_command(cmd, kDefaultParams);
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

/// Encode motor stop command (Type 4 frame, data[0]=0).
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

/// Encode stop + clear fault (Type 4 frame, data[0]=1).
inline can_frame encode_stop_clear_fault(int motor_id)
{
  can_frame frame{};
  frame.can_id =
    (kTypeStop << 24) |
    (static_cast<uint32_t>(kHostId) << 8) |
    static_cast<uint32_t>(motor_id) |
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  frame.data[0] = 1;
  return frame;
}

/// Encode parameter read request (Type 17 frame).
/// Data layout: index[0:1] LE, rest zeroed. Motor replies with Type 0 frame.
inline can_frame encode_param_read(int motor_id, uint16_t index)
{
  can_frame frame{};
  frame.can_id =
    (kTypeParamRead << 24) |
    (static_cast<uint32_t>(kHostId) << 8) |
    static_cast<uint32_t>(motor_id) |
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);

  // Index: little-endian uint16
  frame.data[0] = static_cast<uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  return frame;
}

/// Encode parameter write (Type 18 frame).
/// Data layout: index[0:1] LE, pad[2:3], value[4:7] LE float.
inline can_frame encode_param_write(int motor_id, uint16_t index, float value)
{
  can_frame frame{};
  frame.can_id =
    (kTypeParamWrite << 24) |
    (static_cast<uint32_t>(kHostId) << 8) |
    static_cast<uint32_t>(motor_id) |
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);

  // Index: little-endian uint16
  frame.data[0] = static_cast<uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);

  // Value: little-endian float
  uint32_t val_bits;
  std::memcpy(&val_bits, &value, sizeof(val_bits));
  frame.data[4] = static_cast<uint8_t>(val_bits & 0xFF);
  frame.data[5] = static_cast<uint8_t>((val_bits >> 8) & 0xFF);
  frame.data[6] = static_cast<uint8_t>((val_bits >> 16) & 0xFF);
  frame.data[7] = static_cast<uint8_t>((val_bits >> 24) & 0xFF);
  return frame;
}

// ---------------------------------------------------------------------------
// Frame decoding
// ---------------------------------------------------------------------------

/// Decode a CAN frame as motor feedback using per-model velocity/torque ranges.
/// Returns nullopt if not a Type 2 frame.
inline std::optional<MotorFeedback> decode_feedback(
  const can_frame & frame, const MotorParams & params)
{
  uint32_t arb_id = frame.can_id & CAN_EFF_MASK;
  uint32_t comm_type = (arb_id >> 24) & 0x1F;

  if (comm_type != kTypeFeedback) {
    return std::nullopt;
  }

  MotorFeedback fb;
  fb.motor_id = (arb_id >> 8) & 0xFF;
  fb.fault_bits = (arb_id >> 16) & 0x3F;
  fb.mode = (arb_id >> 22) & 0x03;

  uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  uint16_t vel_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];

  fb.position = uint_to_float(pos_raw, kFbPosMin, kFbPosMax, 16);
  fb.velocity = uint_to_float(vel_raw, params.vel_min, params.vel_max, 16);
  fb.torque = uint_to_float(torque_raw, params.torque_min, params.torque_max, 16);

  uint16_t temp_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
  fb.temperature = static_cast<float>(temp_raw) / 10.0f;

  return fb;
}

}  // namespace robstride

#endif  // STEVEROS_HARDWARE__ROBSTRIDE_PROTOCOL_HPP_
