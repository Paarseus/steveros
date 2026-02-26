#include "steveros_hardware/steveros_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace steveros_hardware
{

static constexpr double DEG_TO_RAD = M_PI / 180.0;

// Default Kd for soft-enable phase (per EDULITE_A3: Kd=4.0 for RS01/RS02).
// RS03/RS04 have range 0-100; 4.0 is still conservative. Tune after testing.
static constexpr float kSoftEnableKd = 4.0f;

// ---------------------------------------------------------------------------
// Lifecycle: on_init
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_interface_ = info_.hardware_parameters.count("can_interface")
    ? info_.hardware_parameters.at("can_interface")
    : "can0";

  if (info_.hardware_parameters.count("feedback_timeout")) {
    feedback_timeout_s_ = std::stod(info_.hardware_parameters.at("feedback_timeout"));
  }

  if (info_.hardware_parameters.count("kp_ramp_duration")) {
    kp_ramp_duration_s_ = std::stod(info_.hardware_parameters.at("kp_ramp_duration"));
  }

  const auto num_joints = info_.joints.size();
  RCLCPP_INFO(
    get_logger(),
    "Initializing with %zu joints on CAN interface '%s'",
    num_joints, can_interface_.c_str());

  joint_configs_.resize(num_joints);
  joint_states_.resize(num_joints);

  for (size_t i = 0; i < num_joints; i++) {
    const auto & joint = info_.joints[i];

    auto get_param = [&](const std::string & key, double default_val) -> double {
      auto it = joint.parameters.find(key);
      if (it != joint.parameters.end()) {
        return std::stod(it->second);
      }
      return default_val;
    };

    auto get_string_param = [&](const std::string & key,
      const std::string & default_val) -> std::string {
      auto it = joint.parameters.find(key);
      if (it != joint.parameters.end()) {
        return it->second;
      }
      return default_val;
    };

    joint_configs_[i].motor_id = static_cast<int>(get_param("motor_id", 0));
    joint_configs_[i].sign = static_cast<int>(get_param("sign", 1));
    joint_configs_[i].zero_offset_rad = get_param("zero_offset_deg", 0.0) * DEG_TO_RAD;
    joint_configs_[i].kp = get_param("kp", 20.0);
    joint_configs_[i].kd = get_param("kd", 2.0);

    // Parse motor_type from URDF param
    std::string motor_type_str = get_string_param("motor_type", "RS02");
    try {
      joint_configs_[i].motor_type = robstride::parse_motor_type(motor_type_str);
    } catch (const std::invalid_argument & e) {
      RCLCPP_FATAL(get_logger(), "Joint '%s': %s", joint.name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Per-joint torque limit from URDF <limit effort="...">
    auto motor_params = robstride::get_motor_params(joint_configs_[i].motor_type);
    double max_protocol_torque = static_cast<double>(motor_params.torque_max);
    auto limits_it = info_.limits.find(joint.name);
    if (limits_it != info_.limits.end() && limits_it->second.has_effort_limits) {
      joint_configs_[i].max_torque =
        std::min(limits_it->second.max_effort, max_protocol_torque);
    } else {
      joint_configs_[i].max_torque = max_protocol_torque;
    }

    RCLCPP_INFO(
      get_logger(),
      "  Joint '%s': motor_id=%d, type=%s, sign=%d, zero_offset=%.2f deg, "
      "kp=%.1f, kd=%.1f, max_torque=%.1f Nm",
      joint.name.c_str(),
      joint_configs_[i].motor_id,
      motor_type_str.c_str(),
      joint_configs_[i].sign,
      get_param("zero_offset_deg", 0.0),
      joint_configs_[i].kp,
      joint_configs_[i].kd,
      joint_configs_[i].max_torque);

    // Validate interfaces
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' must have exactly one position command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' must have exactly 3 state interfaces (position, velocity, effort).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Build motor_id → joint index map
  motor_id_to_index_.clear();
  for (size_t i = 0; i < num_joints; i++) {
    int mid = joint_configs_[i].motor_id;
    if (mid == 0) {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has motor_id=0 (unconfigured).",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    auto [it, inserted] = motor_id_to_index_.emplace(mid, i);
    if (!inserted) {
      RCLCPP_FATAL(
        get_logger(),
        "Duplicate motor_id=%d on joint '%s' (already used by joint '%s').",
        mid, info_.joints[i].name.c_str(),
        info_.joints[it->second].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  last_feedback_time_.resize(num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_configure
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring: opening CAN driver on '%s'...", can_interface_.c_str());

  driver_ = std::make_unique<robstride::RobstrideCanDriver>();
  if (!driver_->open(can_interface_)) {
    RCLCPP_FATAL(get_logger(), "Failed to open CAN driver on '%s'.", can_interface_.c_str());
    driver_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Configured: CAN driver open on '%s'.", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_activate — 4-phase safe activation sequence (ADR-2)
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating: %zu motors...", joint_configs_.size());

  // Phase 0: Clear faults
  RCLCPP_INFO(get_logger(), "Phase 0: Clearing faults...");
  for (const auto & cfg : joint_configs_) {
    driver_->clear_fault(cfg.motor_id);
    ::usleep(50'000);
  }
  ::usleep(200'000);

  // Phase 1: Soft enable (Kp=0)
  RCLCPP_INFO(get_logger(), "Phase 1: Soft enable (Kp=0)...");
  for (const auto & cfg : joint_configs_) {
    auto params = robstride::get_motor_params(cfg.motor_type);

    driver_->set_run_mode(cfg.motor_id, robstride::kRunModeMotionControl);
    ::usleep(30'000);

    driver_->enable_motor(cfg.motor_id);

    // MIT command with Kp=0 (motor is enabled but limp)
    robstride::MitCommand cmd;
    cmd.motor_id = cfg.motor_id;
    cmd.kd = kSoftEnableKd;
    cmd.max_torque = static_cast<float>(cfg.max_torque);
    driver_->send_frame(robstride::encode_mit_command(cmd, params));
    ::usleep(50'000);
  }

  // Phase 2: Read initial positions from feedback cache
  RCLCPP_INFO(get_logger(), "Phase 2: Reading initial positions...");
  for (size_t i = 0; i < joint_configs_.size(); i++) {
    const auto & cfg = joint_configs_[i];

    if (!driver_->has_valid_feedback(cfg.motor_id)) {
      RCLCPP_ERROR(
        get_logger(),
        "No feedback from motor %d ('%s') after activation.",
        cfg.motor_id, info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto fb = driver_->get_feedback(cfg.motor_id);

    joint_states_[i].position = motor_to_joint(cfg, static_cast<double>(fb.position));
    joint_states_[i].velocity = cfg.sign * static_cast<double>(fb.velocity);
    joint_states_[i].effort = cfg.sign * static_cast<double>(fb.torque);
    joint_states_[i].command_position = joint_states_[i].position;
  }

  // Phase 3: Position hold — ramp begins in first write() cycle
  RCLCPP_INFO(get_logger(), "Phase 3: Starting position hold with Kp ramp...");
  for (size_t i = 0; i < joint_configs_.size(); i++) {
    const auto & cfg = joint_configs_[i];
    auto params = robstride::get_motor_params(cfg.motor_type);
    float motor_pos = static_cast<float>(joint_to_motor(cfg, joint_states_[i].command_position));

    robstride::MitCommand cmd;
    cmd.motor_id = cfg.motor_id;
    cmd.p_des = motor_pos;
    cmd.kp = static_cast<float>(cfg.kp);
    cmd.kd = static_cast<float>(cfg.kd);
    cmd.max_torque = static_cast<float>(cfg.max_torque);
    driver_->send_frame(robstride::encode_mit_command(cmd, params));
    ::usleep(30'000);
  }

  rclcpp::Time now = get_clock()->now();
  for (size_t i = 0; i < last_feedback_time_.size(); i++) {
    last_feedback_time_[i] = now;
  }
  activation_time_ = now;
  ramping_ = true;

  RCLCPP_INFO(
    get_logger(), "Activated: all motors enabled (Kp ramp over %.1fs).",
    kp_ramp_duration_s_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_deactivate
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating: sending zero-torque commands...");

  for (const auto & cfg : joint_configs_) {
    auto params = robstride::get_motor_params(cfg.motor_type);
    robstride::MitCommand cmd;
    cmd.motor_id = cfg.motor_id;
    cmd.max_torque = static_cast<float>(cfg.max_torque);
    driver_->send_frame(robstride::encode_mit_command(cmd, params));
  }

  ::usleep(5000);

  for (const auto & cfg : joint_configs_) {
    driver_->disable_motor(cfg.motor_id);
  }

  ramping_ = false;

  RCLCPP_INFO(get_logger(), "Deactivated: all motors stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_cleanup
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");

  if (driver_) {
    driver_->close();
    driver_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface export
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
SteveROSHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_states_[i].position);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_states_[i].velocity);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_states_[i].effort);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SteveROSHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &joint_states_[i].command_position);
  }
  return command_interfaces;
}

// ---------------------------------------------------------------------------
// read() — copy from background thread feedback cache
// ---------------------------------------------------------------------------

hardware_interface::return_type SteveROSHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < joint_configs_.size(); i++) {
    const auto & cfg = joint_configs_[i];

    if (!driver_->has_valid_feedback(cfg.motor_id)) {
      continue;
    }

    auto fb = driver_->get_feedback(cfg.motor_id);

    joint_states_[i].position = motor_to_joint(cfg, static_cast<double>(fb.position));
    joint_states_[i].velocity = cfg.sign * static_cast<double>(fb.velocity);
    joint_states_[i].effort = cfg.sign * static_cast<double>(fb.torque);

    if (fb.fault_bits != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Motor %d ('%s') fault: fault_bits=0x%02X, mode=%u, temp=%.1f°C",
        fb.motor_id, info_.joints[i].name.c_str(),
        fb.fault_bits, fb.mode, fb.temperature);
    }

    last_feedback_time_[i] = time;
  }

  // Check for feedback timeout
  for (size_t i = 0; i < joint_configs_.size(); i++) {
    double elapsed = (time - last_feedback_time_[i]).seconds();
    if (elapsed > feedback_timeout_s_) {
      RCLCPP_ERROR(
        get_logger(),
        "Feedback timeout on motor %d ('%s'): %.3fs since last feedback (limit=%.3fs)",
        joint_configs_[i].motor_id, info_.joints[i].name.c_str(),
        elapsed, feedback_timeout_s_);
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write() — send MIT commands with per-model params and Kp ramp
// ---------------------------------------------------------------------------

hardware_interface::return_type SteveROSHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  double kp_scale = 1.0;
  if (ramping_) {
    double elapsed = (time - activation_time_).seconds();
    kp_scale = std::clamp(elapsed / kp_ramp_duration_s_, 0.0, 1.0);
    if (kp_scale >= 1.0) {
      ramping_ = false;
    }
  }

  for (size_t i = 0; i < joint_configs_.size(); i++) {
    const auto & cfg = joint_configs_[i];
    auto params = robstride::get_motor_params(cfg.motor_type);
    float motor_pos = static_cast<float>(joint_to_motor(cfg, joint_states_[i].command_position));

    robstride::MitCommand cmd;
    cmd.motor_id = cfg.motor_id;
    cmd.p_des = motor_pos;
    cmd.kp = static_cast<float>(kp_scale * cfg.kp);
    cmd.kd = static_cast<float>(cfg.kd);
    cmd.max_torque = static_cast<float>(cfg.max_torque);

    driver_->send_frame(robstride::encode_mit_command(cmd, params));
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Coordinate transforms
// ---------------------------------------------------------------------------

double SteveROSHardware::motor_to_joint(const JointConfig & cfg, double motor_rad) const
{
  return cfg.sign * (motor_rad - cfg.zero_offset_rad);
}

double SteveROSHardware::joint_to_motor(const JointConfig & cfg, double joint_rad) const
{
  return joint_rad / cfg.sign + cfg.zero_offset_rad;
}

}  // namespace steveros_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  steveros_hardware::SteveROSHardware,
  hardware_interface::SystemInterface)
