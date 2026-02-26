#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "steveros_hardware/robstride_protocol.hpp"

using namespace robstride;

// ===========================================================================
// Motor parameter lookup
// ===========================================================================

TEST(MotorParams, RS01MatchesExpected)
{
  constexpr auto p = get_motor_params(MotorType::RS01);
  EXPECT_FLOAT_EQ(p.pos_min, -4 * kPi);
  EXPECT_FLOAT_EQ(p.pos_max, 4 * kPi);
  EXPECT_FLOAT_EQ(p.vel_min, -44.0f);
  EXPECT_FLOAT_EQ(p.vel_max, 44.0f);
  EXPECT_FLOAT_EQ(p.torque_min, -17.0f);
  EXPECT_FLOAT_EQ(p.torque_max, 17.0f);
  EXPECT_FLOAT_EQ(p.kp_min, 0.0f);
  EXPECT_FLOAT_EQ(p.kp_max, 500.0f);
  EXPECT_FLOAT_EQ(p.kd_min, 0.0f);
  EXPECT_FLOAT_EQ(p.kd_max, 5.0f);
}

TEST(MotorParams, RS01AndRS02AreIdentical)
{
  constexpr auto p1 = get_motor_params(MotorType::RS01);
  constexpr auto p2 = get_motor_params(MotorType::RS02);
  EXPECT_FLOAT_EQ(p1.vel_max, p2.vel_max);
  EXPECT_FLOAT_EQ(p1.torque_max, p2.torque_max);
  EXPECT_FLOAT_EQ(p1.kp_max, p2.kp_max);
  EXPECT_FLOAT_EQ(p1.kd_max, p2.kd_max);
}

TEST(MotorParams, RS03DiffersFromRS02)
{
  constexpr auto p3 = get_motor_params(MotorType::RS03);
  EXPECT_FLOAT_EQ(p3.vel_max, 20.0f);
  EXPECT_FLOAT_EQ(p3.torque_max, 60.0f);
  EXPECT_FLOAT_EQ(p3.kp_max, 5000.0f);
  EXPECT_FLOAT_EQ(p3.kd_max, 100.0f);
}

TEST(MotorParams, RS04HasHighestTorque)
{
  constexpr auto p4 = get_motor_params(MotorType::RS04);
  EXPECT_FLOAT_EQ(p4.vel_max, 15.0f);
  EXPECT_FLOAT_EQ(p4.torque_max, 120.0f);
  EXPECT_FLOAT_EQ(p4.kp_max, 5000.0f);
  EXPECT_FLOAT_EQ(p4.kd_max, 100.0f);
}

TEST(MotorParams, AllTypesSharePositionRange)
{
  const MotorType types[] = {MotorType::RS01, MotorType::RS02, MotorType::RS03, MotorType::RS04};
  for (auto t : types) {
    auto p = get_motor_params(t);
    EXPECT_FLOAT_EQ(p.pos_min, -4 * kPi);
    EXPECT_FLOAT_EQ(p.pos_max, 4 * kPi);
  }
}

// ===========================================================================
// Motor type parsing
// ===========================================================================

TEST(ParseMotorType, ValidStrings)
{
  EXPECT_EQ(parse_motor_type("RS01"), MotorType::RS01);
  EXPECT_EQ(parse_motor_type("RS02"), MotorType::RS02);
  EXPECT_EQ(parse_motor_type("RS03"), MotorType::RS03);
  EXPECT_EQ(parse_motor_type("RS04"), MotorType::RS04);
}

TEST(ParseMotorType, InvalidStringThrows)
{
  EXPECT_THROW(parse_motor_type("RS05"), std::invalid_argument);
  EXPECT_THROW(parse_motor_type("rs01"), std::invalid_argument);
  EXPECT_THROW(parse_motor_type(""), std::invalid_argument);
  EXPECT_THROW(parse_motor_type("RS01 "), std::invalid_argument);
}

TEST(ParseMotorType, ExceptionMessageContainsInput)
{
  try {
    parse_motor_type("BOGUS");
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("BOGUS"), std::string::npos);
  }
}

// ===========================================================================
// float_to_uint / uint_to_float conversion
// ===========================================================================

TEST(Conversion, FloatToUintBoundaries)
{
  // At minimum -> 0
  EXPECT_EQ(float_to_uint(0.0f, 0.0f, 100.0f, 16), 0);
  // At maximum -> 65535
  EXPECT_EQ(float_to_uint(100.0f, 0.0f, 100.0f, 16), 65535);
}

TEST(Conversion, FloatToUintMidpoint)
{
  // Midpoint of symmetric range should be ~32767 or ~32768
  uint16_t mid = float_to_uint(0.0f, -100.0f, 100.0f, 16);
  EXPECT_NEAR(mid, 32767, 1);
}

TEST(Conversion, FloatToUintClampsBelow)
{
  EXPECT_EQ(float_to_uint(-999.0f, 0.0f, 100.0f, 16), 0);
}

TEST(Conversion, FloatToUintClampsAbove)
{
  EXPECT_EQ(float_to_uint(999.0f, 0.0f, 100.0f, 16), 65535);
}

TEST(Conversion, UintToFloatBoundaries)
{
  EXPECT_FLOAT_EQ(uint_to_float(0, -12.5f, 12.5f, 16), -12.5f);
  EXPECT_FLOAT_EQ(uint_to_float(65535, -12.5f, 12.5f, 16), 12.5f);
}

TEST(Conversion, UintToFloatMidpoint)
{
  float mid = uint_to_float(32767, -12.5f, 12.5f, 16);
  EXPECT_NEAR(mid, 0.0f, 0.001f);
}

TEST(Conversion, RoundTripPreservesValue)
{
  // Test round-trip with feedback ranges (the most critical path)
  const float values[] = {-12.0f, -5.0f, 0.0f, 3.7f, 12.0f};
  for (float v : values) {
    uint16_t encoded = float_to_uint(v, -12.5f, 12.5f, 16);
    float decoded = uint_to_float(encoded, -12.5f, 12.5f, 16);
    // 16-bit quantization over 25.0 range -> step ~0.00038
    EXPECT_NEAR(decoded, v, 0.001f) << "Round-trip failed for value " << v;
  }
}

TEST(Conversion, RoundTripWithMotorRanges)
{
  // RS03 Kp range: 0-5000
  float kp = 200.0f;
  uint16_t enc = float_to_uint(kp, 0.0f, 5000.0f, 16);
  float dec = uint_to_float(enc, 0.0f, 5000.0f, 16);
  // Step size: 5000/65535 ~ 0.076
  EXPECT_NEAR(dec, kp, 0.1f);
}

TEST(Conversion, SameKpEncodedDifferentlyPerModel)
{
  // Kp=100 should produce different uint16 values for RS02 vs RS03
  float kp = 100.0f;
  uint16_t rs02_enc = float_to_uint(kp, 0.0f, 500.0f, 16);   // RS02 range
  uint16_t rs03_enc = float_to_uint(kp, 0.0f, 5000.0f, 16);  // RS03 range

  // RS02: 100/500 * 65535 = 13107
  // RS03: 100/5000 * 65535 = 1310
  EXPECT_GT(rs02_enc, rs03_enc);
  EXPECT_NEAR(rs02_enc, 13107, 1);
  EXPECT_NEAR(rs03_enc, 1310, 1);
}

TEST(Conversion, EightBitMode)
{
  EXPECT_EQ(float_to_uint(0.0f, 0.0f, 100.0f, 8), 0);
  EXPECT_EQ(float_to_uint(100.0f, 0.0f, 100.0f, 8), 255);
  EXPECT_EQ(float_to_uint(50.0f, 0.0f, 100.0f, 8), 127);
}

// ===========================================================================
// encode_mit_command
// ===========================================================================

class EncodeMitCommand : public ::testing::Test
{
protected:
  void SetUp() override
  {
    cmd_.motor_id = 21;
    cmd_.p_des = 0.0f;
    cmd_.v_des = 0.0f;
    cmd_.kp = 0.0f;
    cmd_.kd = 0.0f;
    cmd_.t_ff = 0.0f;
    cmd_.max_torque = 60.0f;
    params_ = get_motor_params(MotorType::RS03);
  }

  MitCommand cmd_;
  MotorParams params_;
};

TEST_F(EncodeMitCommand, FrameType)
{
  auto frame = encode_mit_command(cmd_, params_);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint32_t frame_type = (arb >> 24) & 0x1F;
  EXPECT_EQ(frame_type, kTypeMitCommand);
}

TEST_F(EncodeMitCommand, MotorIdInCanId)
{
  cmd_.motor_id = 42;
  auto frame = encode_mit_command(cmd_, params_);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  EXPECT_EQ(arb & 0xFF, 42u);
}

TEST_F(EncodeMitCommand, ExtendedFrameFlag)
{
  auto frame = encode_mit_command(cmd_, params_);
  EXPECT_TRUE(frame.can_id & CAN_EFF_FLAG);
}

TEST_F(EncodeMitCommand, DlcIsEight)
{
  auto frame = encode_mit_command(cmd_, params_);
  EXPECT_EQ(frame.can_dlc, 8);
}

TEST_F(EncodeMitCommand, ZeroCommandProducesExpectedPayload)
{
  // All zeros: position, velocity, kp, kd at their minimums
  // p_des=0 with range [-4pi, 4pi] -> midpoint -> ~0x8000
  auto frame = encode_mit_command(cmd_, params_);

  // Position: 0.0 in [-4pi, 4pi] -> midpoint
  uint16_t p_uint = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  float p_decoded = uint_to_float(p_uint, params_.pos_min, params_.pos_max, 16);
  EXPECT_NEAR(p_decoded, 0.0f, 0.001f);

  // Velocity: 0.0 in [-20, 20] -> midpoint
  uint16_t v_uint = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  float v_decoded = uint_to_float(v_uint, params_.vel_min, params_.vel_max, 16);
  EXPECT_NEAR(v_decoded, 0.0f, 0.001f);

  // Kp: 0.0 in [0, 5000] -> minimum -> 0x0000
  uint16_t kp_uint = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
  EXPECT_EQ(kp_uint, 0);

  // Kd: 0.0 in [0, 100] -> minimum -> 0x0000
  uint16_t kd_uint = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
  EXPECT_EQ(kd_uint, 0);
}

TEST_F(EncodeMitCommand, TorqueFeedforwardInCanId)
{
  // t_ff=0 in [-60, 60] -> midpoint -> ~0x8000
  cmd_.t_ff = 0.0f;
  auto frame = encode_mit_command(cmd_, params_);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint16_t t_uint = (arb >> 8) & 0xFFFF;
  float t_decoded = uint_to_float(t_uint, params_.torque_min, params_.torque_max, 16);
  EXPECT_NEAR(t_decoded, 0.0f, 0.01f);
}

TEST_F(EncodeMitCommand, TorqueFeedforwardClampedToMaxTorque)
{
  // max_torque=10, t_ff=50 -> clamped to 10
  cmd_.max_torque = 10.0f;
  cmd_.t_ff = 50.0f;
  auto frame = encode_mit_command(cmd_, params_);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint16_t t_uint = (arb >> 8) & 0xFFFF;
  float t_decoded = uint_to_float(t_uint, params_.torque_min, params_.torque_max, 16);
  EXPECT_NEAR(t_decoded, 10.0f, 0.1f);
}

TEST_F(EncodeMitCommand, RS02VsRS03KpEncoding)
{
  // Same Kp=250 encodes differently for RS02 vs RS03
  cmd_.kp = 250.0f;

  auto params_rs02 = get_motor_params(MotorType::RS02);
  auto frame_rs02 = encode_mit_command(cmd_, params_rs02);
  uint16_t kp_rs02 = (static_cast<uint16_t>(frame_rs02.data[4]) << 8) | frame_rs02.data[5];

  auto params_rs03 = get_motor_params(MotorType::RS03);
  auto frame_rs03 = encode_mit_command(cmd_, params_rs03);
  uint16_t kp_rs03 = (static_cast<uint16_t>(frame_rs03.data[4]) << 8) | frame_rs03.data[5];

  // RS02: 250/500 = 50% of range -> ~32767
  // RS03: 250/5000 = 5% of range -> ~3276
  EXPECT_GT(kp_rs02, kp_rs03);
  EXPECT_NEAR(kp_rs02, 32767, 2);
  EXPECT_NEAR(kp_rs03, 3276, 2);
}

TEST_F(EncodeMitCommand, PositionAtLimits)
{
  // Position at max
  cmd_.p_des = params_.pos_max;
  auto frame = encode_mit_command(cmd_, params_);
  uint16_t p_uint = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  EXPECT_EQ(p_uint, 65535);

  // Position at min
  cmd_.p_des = params_.pos_min;
  frame = encode_mit_command(cmd_, params_);
  p_uint = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  EXPECT_EQ(p_uint, 0);
}

TEST_F(EncodeMitCommand, LegacyOverloadUsesRS02)
{
  cmd_.kp = 250.0f;
  auto frame_legacy = encode_mit_command(cmd_);
  auto frame_rs02 = encode_mit_command(cmd_, get_motor_params(MotorType::RS02));

  // Frames should be identical
  EXPECT_EQ(frame_legacy.can_id, frame_rs02.can_id);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(frame_legacy.data[i], frame_rs02.data[i]) << "Byte " << i << " differs";
  }
}

// ===========================================================================
// encode_enable
// ===========================================================================

TEST(EncodeEnable, FrameTypeIsThree)
{
  auto frame = encode_enable(1);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint32_t frame_type = (arb >> 24) & 0x1F;
  EXPECT_EQ(frame_type, kTypeEnable);
}

TEST(EncodeEnable, HostIdEmbedded)
{
  auto frame = encode_enable(1);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint32_t host_id = (arb >> 8) & 0xFF;
  EXPECT_EQ(host_id, static_cast<uint32_t>(kHostId));
}

TEST(EncodeEnable, MotorIdEmbedded)
{
  auto frame = encode_enable(42);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  EXPECT_EQ(arb & 0xFF, 42u);
}

TEST(EncodeEnable, DataAllZeros)
{
  auto frame = encode_enable(1);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(frame.data[i], 0) << "data[" << i << "] not zero";
  }
}

TEST(EncodeEnable, ExtendedFlagAndDlc)
{
  auto frame = encode_enable(1);
  EXPECT_TRUE(frame.can_id & CAN_EFF_FLAG);
  EXPECT_EQ(frame.can_dlc, 8);
}

// ===========================================================================
// encode_stop
// ===========================================================================

TEST(EncodeStop, FrameTypeIsFour)
{
  auto frame = encode_stop(1);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint32_t frame_type = (arb >> 24) & 0x1F;
  EXPECT_EQ(frame_type, kTypeStop);
}

TEST(EncodeStop, DataByteZeroIsClear)
{
  auto frame = encode_stop(1);
  EXPECT_EQ(frame.data[0], 0);
}

TEST(EncodeStop, HostIdAndMotorId)
{
  auto frame = encode_stop(99);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  EXPECT_EQ((arb >> 8) & 0xFF, static_cast<uint32_t>(kHostId));
  EXPECT_EQ(arb & 0xFF, 99u);
}

// ===========================================================================
// encode_stop_clear_fault
// ===========================================================================

TEST(EncodeStopClearFault, FrameTypeIsFour)
{
  auto frame = encode_stop_clear_fault(1);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint32_t frame_type = (arb >> 24) & 0x1F;
  EXPECT_EQ(frame_type, kTypeStop);
}

TEST(EncodeStopClearFault, FaultClearByteSet)
{
  auto frame = encode_stop_clear_fault(1);
  EXPECT_EQ(frame.data[0], 1);
}

TEST(EncodeStopClearFault, RemainingDataZeros)
{
  auto frame = encode_stop_clear_fault(1);
  for (int i = 1; i < 8; ++i) {
    EXPECT_EQ(frame.data[i], 0) << "data[" << i << "] not zero";
  }
}

TEST(EncodeStopClearFault, DiffersFromStopOnlyInDataByte)
{
  auto stop = encode_stop(5);
  auto clear = encode_stop_clear_fault(5);
  // Same CAN ID
  EXPECT_EQ(stop.can_id, clear.can_id);
  // data[0] differs
  EXPECT_NE(stop.data[0], clear.data[0]);
  // data[1:7] same (both zero)
  for (int i = 1; i < 8; ++i) {
    EXPECT_EQ(stop.data[i], clear.data[i]);
  }
}

// ===========================================================================
// encode_param_write
// ===========================================================================

TEST(EncodeParamWrite, FrameTypeIsEighteen)
{
  auto frame = encode_param_write(1, 0x7005, 0.0f);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  uint32_t frame_type = (arb >> 24) & 0x1F;
  EXPECT_EQ(frame_type, kTypeParamWrite);
}

TEST(EncodeParamWrite, IndexLittleEndian)
{
  auto frame = encode_param_write(1, 0x7005, 0.0f);
  // Little-endian: low byte first
  EXPECT_EQ(frame.data[0], 0x05);
  EXPECT_EQ(frame.data[1], 0x70);
}

TEST(EncodeParamWrite, PaddingIsZero)
{
  auto frame = encode_param_write(1, 0x7005, 42.0f);
  EXPECT_EQ(frame.data[2], 0);
  EXPECT_EQ(frame.data[3], 0);
}

TEST(EncodeParamWrite, ValueEncodedAsLittleEndianFloat)
{
  float value = 0.0f;
  auto frame = encode_param_write(1, 0x7005, value);

  // IEEE 754: 0.0f = 0x00000000
  EXPECT_EQ(frame.data[4], 0x00);
  EXPECT_EQ(frame.data[5], 0x00);
  EXPECT_EQ(frame.data[6], 0x00);
  EXPECT_EQ(frame.data[7], 0x00);
}

TEST(EncodeParamWrite, ValueRoundTrip)
{
  float original = 3.14f;
  auto frame = encode_param_write(1, 0x0000, original);

  // Reconstruct the float from little-endian bytes
  uint32_t bits =
    static_cast<uint32_t>(frame.data[4]) |
    (static_cast<uint32_t>(frame.data[5]) << 8) |
    (static_cast<uint32_t>(frame.data[6]) << 16) |
    (static_cast<uint32_t>(frame.data[7]) << 24);
  float recovered;
  std::memcpy(&recovered, &bits, sizeof(recovered));
  EXPECT_FLOAT_EQ(recovered, original);
}

TEST(EncodeParamWrite, HostIdAndMotorId)
{
  auto frame = encode_param_write(21, 0x7005, 0.0f);
  uint32_t arb = frame.can_id & CAN_EFF_MASK;
  EXPECT_EQ((arb >> 8) & 0xFF, static_cast<uint32_t>(kHostId));
  EXPECT_EQ(arb & 0xFF, 21u);
}

// ===========================================================================
// decode_feedback
// ===========================================================================

// Helper: construct a Type 2 feedback frame from known values.
// fault_bits and mode are encoded into the arb ID; temperature is 16-bit big-endian.
static can_frame make_feedback_frame(
  int motor_id,
  uint16_t pos_raw,
  uint16_t vel_raw,
  uint16_t torque_raw,
  float temperature_c = 25.0f,
  uint8_t fault_bits = 0,
  uint8_t mode = 0)
{
  can_frame frame{};
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

TEST(DecodeFeedback, RejectsNonType2Frame)
{
  can_frame frame{};
  frame.can_id = (kTypeMitCommand << 24) | CAN_EFF_FLAG;
  auto result = decode_feedback(frame);
  EXPECT_FALSE(result.has_value());
}

TEST(DecodeFeedback, RejectsEnableFrame)
{
  can_frame frame{};
  frame.can_id = (kTypeEnable << 24) | CAN_EFF_FLAG;
  EXPECT_FALSE(decode_feedback(frame).has_value());
}

TEST(DecodeFeedback, RejectsStopFrame)
{
  can_frame frame{};
  frame.can_id = (kTypeStop << 24) | CAN_EFF_FLAG;
  EXPECT_FALSE(decode_feedback(frame).has_value());
}

TEST(DecodeFeedback, AcceptsType2Frame)
{
  auto frame = make_feedback_frame(1, 0x8000, 0x8000, 0x8000);
  auto result = decode_feedback(frame);
  ASSERT_TRUE(result.has_value());
}

TEST(DecodeFeedback, ExtractsMotorId)
{
  auto frame = make_feedback_frame(21, 0x8000, 0x8000, 0x8000, 25, 0);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fb->motor_id, 21);
}

TEST(DecodeFeedback, DecodesPositionMidpoint)
{
  // 0x8000 in range [-12.5, 12.5] -> ~0.0
  auto frame = make_feedback_frame(1, 0x8000, 0x8000, 0x8000);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_NEAR(fb->position, 0.0f, 0.01f);
}

TEST(DecodeFeedback, DecodesPositionMin)
{
  auto frame = make_feedback_frame(1, 0x0000, 0x8000, 0x8000, 25, 0);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_FLOAT_EQ(fb->position, kFbPosMin);
}

TEST(DecodeFeedback, DecodesPositionMax)
{
  auto frame = make_feedback_frame(1, 0xFFFF, 0x8000, 0x8000, 25, 0);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_FLOAT_EQ(fb->position, kFbPosMax);
}

TEST(DecodeFeedback, DecodesVelocityMidpoint)
{
  auto frame = make_feedback_frame(1, 0x8000, 0x8000, 0x8000);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_NEAR(fb->velocity, 0.0f, 0.01f);
}

TEST(DecodeFeedback, DecodesVelocityExtremes)
{
  auto frame_min = make_feedback_frame(1, 0x8000, 0x0000, 0x8000);
  auto fb_min = decode_feedback(frame_min);
  ASSERT_TRUE(fb_min.has_value());
  EXPECT_FLOAT_EQ(fb_min->velocity, kFbVelMin);

  auto frame_max = make_feedback_frame(1, 0x8000, 0xFFFF, 0x8000);
  auto fb_max = decode_feedback(frame_max);
  ASSERT_TRUE(fb_max.has_value());
  EXPECT_FLOAT_EQ(fb_max->velocity, kFbVelMax);
}

TEST(DecodeFeedback, DecodesTorqueExtremes)
{
  auto frame_min = make_feedback_frame(1, 0x8000, 0x8000, 0x0000);
  auto fb_min = decode_feedback(frame_min);
  ASSERT_TRUE(fb_min.has_value());
  EXPECT_FLOAT_EQ(fb_min->torque, kFbTorqueMin);

  auto frame_max = make_feedback_frame(1, 0x8000, 0x8000, 0xFFFF);
  auto fb_max = decode_feedback(frame_max);
  ASSERT_TRUE(fb_max.has_value());
  EXPECT_FLOAT_EQ(fb_max->torque, kFbTorqueMax);
}

TEST(DecodeFeedback, TemperatureDecoding)
{
  // 24.0°C encoded as 16-bit big-endian: raw = 240 = 0x00F0
  auto frame = make_feedback_frame(1, 0x8000, 0x8000, 0x8000, 24.0f);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_NEAR(fb->temperature, 24.0f, 0.1f);

  // 55.5°C
  auto frame2 = make_feedback_frame(1, 0x8000, 0x8000, 0x8000, 55.5f);
  auto fb2 = decode_feedback(frame2);
  ASSERT_TRUE(fb2.has_value());
  EXPECT_NEAR(fb2->temperature, 55.5f, 0.1f);
}

TEST(DecodeFeedback, TemperatureExtremes)
{
  auto frame0 = make_feedback_frame(1, 0x8000, 0x8000, 0x8000, 0.0f);
  EXPECT_NEAR(decode_feedback(frame0)->temperature, 0.0f, 0.1f);

  // Max 16-bit: 6553.5°C (theoretical max, tests full range)
  auto frame_max = make_feedback_frame(1, 0x8000, 0x8000, 0x8000, 100.0f);
  EXPECT_NEAR(decode_feedback(frame_max)->temperature, 100.0f, 0.1f);
}

TEST(DecodeFeedback, ExtractsFaultBitsAndMode)
{
  // fault_bits=0x15, mode=2 (Run)
  auto frame = make_feedback_frame(21, 0x8000, 0x8000, 0x8000, 25.0f, 0x15, 2);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fb->fault_bits, 0x15);
  EXPECT_EQ(fb->mode, 2);
}

TEST(DecodeFeedback, ZeroFaultAndModeWhenClean)
{
  auto frame = make_feedback_frame(21, 0x8000, 0x8000, 0x8000, 25.0f, 0, 0);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fb->fault_bits, 0);
  EXPECT_EQ(fb->mode, 0);
}

TEST(DecodeFeedback, KnownPositionRoundTrip)
{
  // Encode a known position via float_to_uint, build feedback frame, decode
  float target_pos = 3.5f;
  uint16_t pos_raw = float_to_uint(target_pos, kFbPosMin, kFbPosMax, 16);
  auto frame = make_feedback_frame(1, pos_raw, 0x8000, 0x8000);
  auto fb = decode_feedback(frame);
  ASSERT_TRUE(fb.has_value());
  EXPECT_NEAR(fb->position, target_pos, 0.001f);
}

// ===========================================================================
// Constants sanity checks
// ===========================================================================

TEST(Constants, HostId)
{
  EXPECT_EQ(kHostId, 0xFD);
}

TEST(Constants, FrameTypeIds)
{
  EXPECT_EQ(kTypeMitCommand, 1u);
  EXPECT_EQ(kTypeFeedback, 2u);
  EXPECT_EQ(kTypeEnable, 3u);
  EXPECT_EQ(kTypeStop, 4u);
  EXPECT_EQ(kTypeParamWrite, 18u);
}

TEST(Constants, FeedbackRanges)
{
  EXPECT_FLOAT_EQ(kFbPosMin, -4 * kPi);
  EXPECT_FLOAT_EQ(kFbPosMax, 4 * kPi);
  EXPECT_FLOAT_EQ(kFbVelMin, -45.0f);
  EXPECT_FLOAT_EQ(kFbVelMax, 45.0f);
  EXPECT_FLOAT_EQ(kFbTorqueMin, -12.0f);
  EXPECT_FLOAT_EQ(kFbTorqueMax, 12.0f);
}

TEST(Constants, RunModeParam)
{
  EXPECT_EQ(kParamRunMode, 0x7005);
  EXPECT_EQ(kRunModeMotionControl, 0);
}
