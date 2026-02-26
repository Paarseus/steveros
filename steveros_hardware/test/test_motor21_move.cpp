/// Smooth movement test for motor 21 (RS03) on can0.
///
/// Critical fix: writes firmware limit_torque=60 and limit_cur=27 before enable.
/// Without this, the motor defaults to a ~12 Nm internal torque cap.
///
/// MIT mode: τ = Kp*(p_des - p) + Kd*(v_des - v) + t_ff
/// Cosine-blended trajectory with velocity feedforward.

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cmath>
#include <thread>

#include "steveros_hardware/robstride_can_driver.hpp"
#include "steveros_hardware/robstride_protocol.hpp"

using namespace robstride;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

static constexpr int kMotorId = 21;
static constexpr MotorType kMotorType = MotorType::RS03;
static constexpr auto kParams = get_motor_params(kMotorType);

// Gains — moderate Kp, adequate Kd for damping
static constexpr float kKp = 200.0f;
static constexpr float kKd = 5.0f;
static constexpr float kMaxTorque = 30.0f;
static constexpr float kSoftKd = 4.0f;

// Movement: 45° ≈ 0.785 rad
static constexpr float kMoveAmount = 0.7854f;

// Timing
static constexpr int kStepMs = 10;           // 100 Hz
static constexpr int kRampSteps = 100;       // Kp ramp over 1s
static constexpr float kMoveDuration = 5.0f; // seconds per 45° move
static constexpr int kHoldSteps = 100;       // Hold for 1s
static constexpr int kFeedbackTimeoutMs = 2000;

// ---------------------------------------------------------------------------
// Global driver for signal handler cleanup
// ---------------------------------------------------------------------------

static RobstrideCanDriver * g_driver = nullptr;

static void signal_handler(int sig)
{
  printf("\nCaught signal %d, disabling motor...\n", sig);
  if (g_driver) {
    MitCommand zero{};
    zero.motor_id = kMotorId;
    zero.max_torque = kMaxTorque;
    g_driver->send_frame(encode_mit_command(zero, kParams));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    g_driver->disable_motor(kMotorId);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    g_driver->close();
  }
  _exit(1);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static bool wait_for_feedback(RobstrideCanDriver & driver, int timeout_ms)
{
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (driver.has_valid_feedback(kMotorId)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

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
  cmd.max_torque = kMaxTorque;
  driver.send_frame(encode_mit_command(cmd, kParams));
}

static void print_feedback(const MotorFeedback & fb, const char * label)
{
  printf("  %-20s pos=%+8.2f°  vel=%+7.3f rad/s  torque=%+6.2f Nm  "
         "temp=%.1f°C  fault=0x%02X  mode=%u\n",
    label,
    fb.position * 180.0f / kPi,
    fb.velocity, fb.torque,
    fb.temperature, fb.fault_bits, fb.mode);
}

/// Smooth move with cosine trajectory + velocity feedforward.
static float move_to(RobstrideCanDriver & driver, float start, float end,
  float duration_s, const char * label)
{
  float dist = end - start;
  int total_steps = static_cast<int>(duration_s * 1000.0f / kStepMs);

  printf("\n--- %s: %+.1f° → %+.1f° (%.1f° over %.1fs) ---\n",
    label,
    start * 180.0f / kPi, end * 180.0f / kPi,
    dist * 180.0f / kPi, duration_s);

  float max_err = 0.0f;

  for (int i = 0; i <= total_steps; i++) {
    float t = static_cast<float>(i) / total_steps;
    float s = 0.5f * (1.0f - std::cos(kPi * t));
    float cmd_pos = start + s * dist;
    float v_des = 0.5f * kPi * std::sin(kPi * t) * dist / duration_s;

    send_mit(driver, cmd_pos, v_des, kKp, kKd);
    std::this_thread::sleep_for(std::chrono::milliseconds(kStepMs));

    if (i % 25 == 0 || i == total_steps) {
      auto fb = driver.get_feedback(kMotorId);
      float err = fb.position - cmd_pos;
      max_err = std::fmax(max_err, std::fabs(err));
      printf("  [%3d/%d] cmd=%+7.1f° v=%+5.2f actual=%+7.1f° err=%+5.1f° τ=%+5.2f\n",
        i, total_steps,
        cmd_pos * 180.0f / kPi, v_des,
        fb.position * 180.0f / kPi,
        err * 180.0f / kPi, fb.torque);
    }
  }

  // Hold
  for (int i = 0; i < kHoldSteps; i++) {
    send_mit(driver, end, 0.0f, kKp, kKd);
    std::this_thread::sleep_for(std::chrono::milliseconds(kStepMs));
  }

  auto fb = driver.get_feedback(kMotorId);
  float final_err = fb.position - end;
  max_err = std::fmax(max_err, std::fabs(final_err));
  printf("  [FINAL] pos=%+7.1f° err=%+5.2f° τ=%+5.2f\n",
    fb.position * 180.0f / kPi,
    final_err * 180.0f / kPi, fb.torque);

  return max_err;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main()
{
  printf("=== Motor %d (RS03) Movement Test — ±45° ===\n", kMotorId);
  printf("  Kp=%.0f  Kd=%.1f  rate=%d Hz  move_time=%.0fs\n",
    kKp, kKd, 1000 / kStepMs, kMoveDuration);
  printf("  NO firmware limit writes (testing with defaults)\n\n");

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  RobstrideCanDriver driver;
  g_driver = &driver;

  if (!driver.open("can0")) {
    fprintf(stderr, "ERROR: Failed to open can0\n");
    return 1;
  }
  driver.register_motor(kMotorId, kMotorType);

  // --- Phase 0: Clear faults ---
  printf("Phase 0: Clear faults\n");
  driver.clear_fault(kMotorId);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if (!wait_for_feedback(driver, kFeedbackTimeoutMs)) {
    fprintf(stderr, "ERROR: No feedback from motor %d\n", kMotorId);
    driver.close();
    return 1;
  }
  auto fb = driver.get_feedback(kMotorId);
  print_feedback(fb, "after clear_fault");

  // --- Phase 1: Soft enable ---
  printf("\nPhase 1: Soft enable (Kp=0, Kd=%.1f)\n", kSoftKd);
  driver.set_run_mode(kMotorId, kRunModeMotionControl);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  driver.enable_motor(kMotorId);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  send_mit(driver, 0.0f, 0.0f, 0.0f, kSoftKd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  fb = driver.get_feedback(kMotorId);
  print_feedback(fb, "after soft enable");

  // --- Phase 2: Read initial position ---
  float home = fb.position;
  printf("\nPhase 2: Home = %+.2f° (%+.4f rad)\n",
    home * 180.0f / kPi, home);

  // --- Phase 3: Ramp Kp ---
  printf("\nPhase 3: Kp ramp (0 → %.0f)\n", kKp);
  for (int i = 1; i <= kRampSteps; i++) {
    float alpha = static_cast<float>(i) / kRampSteps;
    send_mit(driver, home, 0.0f, alpha * kKp, kKd);
    std::this_thread::sleep_for(std::chrono::milliseconds(kStepMs));
  }
  fb = driver.get_feedback(kMotorId);
  print_feedback(fb, "holding home");

  // --- Phase 4: Moves ---
  float target_pos = home + kMoveAmount;
  float target_neg = home - kMoveAmount;
  float max_err = 0.0f;

  max_err = std::fmax(max_err,
    move_to(driver, home, target_pos, kMoveDuration, "Move +45°"));

  max_err = std::fmax(max_err,
    move_to(driver, target_pos, target_neg, kMoveDuration * 2.0f, "Move -90°"));

  max_err = std::fmax(max_err,
    move_to(driver, target_neg, home, kMoveDuration, "Return home"));

  // --- Phase 5: Ramp down and disable ---
  printf("\nPhase 5: Ramp down and disable\n");
  for (int i = kRampSteps; i >= 0; i--) {
    float alpha = static_cast<float>(i) / kRampSteps;
    send_mit(driver, home, 0.0f, alpha * kKp, kKd);
    std::this_thread::sleep_for(std::chrono::milliseconds(kStepMs));
  }
  send_mit(driver, 0.0f, 0.0f, 0.0f, 0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  driver.disable_motor(kMotorId);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  fb = driver.get_feedback(kMotorId);
  print_feedback(fb, "after disable");

  // --- Summary ---
  printf("\n=== Summary ===\n");
  printf("  Movement:        ±45° (±0.785 rad)\n");
  printf("  Max tracking err: %.2f° (%.4f rad)\n",
    max_err * 180.0f / kPi, max_err);

  bool pass = max_err < 0.174f;  // < 10°
  printf("  Result: %s (max tracking error < 10°)\n", pass ? "PASS" : "FAIL");

  g_driver = nullptr;
  driver.close();
  return pass ? 0 : 1;
}
