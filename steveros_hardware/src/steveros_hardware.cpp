#include "steveros_hardware/steveros_hardware.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "steveros_hardware/robstride_protocol.hpp"

namespace steveros_hardware
{

// Degrees to radians
static constexpr double DEG_TO_RAD = M_PI / 180.0;

// ---------------------------------------------------------------------------
// CAN send helper
// ---------------------------------------------------------------------------

bool SteveROSHardware::send_can_frame(const can_frame & frame)
{
  ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
  if (nbytes != sizeof(frame)) {
    RCLCPP_ERROR(
      get_logger(), "CAN write failed: %s (arb_id=0x%08X)",
      std::strerror(errno), frame.can_id);
    return false;
  }
  return true;
}

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

  // Parse hardware-level params
  can_interface_ = info_.hardware_parameters.count("can_interface")
    ? info_.hardware_parameters.at("can_interface")
    : "can0";

  // Feedback timeout (hardware parameter, default 100ms)
  if (info_.hardware_parameters.count("feedback_timeout")) {
    feedback_timeout_s_ = std::stod(info_.hardware_parameters.at("feedback_timeout"));
  }

  // Kp ramp duration (hardware parameter, default 1s)
  if (info_.hardware_parameters.count("kp_ramp_duration")) {
    kp_ramp_duration_s_ = std::stod(info_.hardware_parameters.at("kp_ramp_duration"));
  }

  const auto num_joints = info_.joints.size();
  RCLCPP_INFO(
    get_logger(),
    "Initializing with %zu joints on CAN interface '%s'",
    num_joints, can_interface_.c_str());

  // Parse per-joint params from URDF ros2_control tags
  joint_configs_.resize(num_joints);
  for (size_t i = 0; i < num_joints; i++) {
    const auto & joint = info_.joints[i];

    auto get_param = [&](const std::string & key, double default_val) -> double {
      auto it = joint.parameters.find(key);
      if (it != joint.parameters.end()) {
        return std::stod(it->second);
      }
      return default_val;
    };

    joint_configs_[i].motor_id = static_cast<int>(get_param("motor_id", 0));
    joint_configs_[i].sign = static_cast<int>(get_param("sign", 1));
    joint_configs_[i].zero_offset_rad = get_param("zero_offset_deg", 0.0) * DEG_TO_RAD;
    joint_configs_[i].kp = get_param("kp", 20.0);
    joint_configs_[i].kd = get_param("kd", 2.0);

    // Per-joint torque limit from URDF <limit effort="..."> with fallback to protocol max
    auto limits_it = info_.limits.find(joint.name);
    if (limits_it != info_.limits.end() && limits_it->second.has_effort_limits) {
      joint_configs_[i].max_torque =
        std::min(limits_it->second.max_effort, 12.0);  // cap at protocol max
    } else {
      joint_configs_[i].max_torque = 12.0;
    }

    RCLCPP_INFO(
      get_logger(),
      "  Joint '%s': motor_id=%d, sign=%d, zero_offset=%.2f deg, "
      "kp=%.1f, kd=%.1f, max_torque=%.1f Nm",
      joint.name.c_str(),
      joint_configs_[i].motor_id,
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

  // Build motor_id → joint index map and validate
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

  // Allocate state/command vectors
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);
  hw_motor_temp_.resize(num_joints, 0);
  hw_motor_status_.resize(num_joints, 0);
  last_feedback_time_.resize(num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_configure — open SocketCAN
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring: opening CAN socket on '%s'...", can_interface_.c_str());

  // Create raw CAN socket
  can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to create CAN socket: %s", std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Resolve interface name to index
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_FATAL(
      get_logger(), "Failed to find CAN interface '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    ::close(can_socket_);
    can_socket_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Bind socket to interface
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_FATAL(
      get_logger(), "Failed to bind CAN socket: %s", std::strerror(errno));
    ::close(can_socket_);
    can_socket_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set receive filter: accept only Type 2 (feedback) frames
  // Type 2 has bits [28:24] = 0x02, so mask those bits
  struct can_filter rfilter{};
  rfilter.can_id = (2u << 24) | CAN_EFF_FLAG;
  rfilter.can_mask = (0x1Fu << 24) | CAN_EFF_FLAG;
  if (::setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER,
    &rfilter, sizeof(rfilter)) < 0)
  {
    RCLCPP_WARN(
      get_logger(), "Failed to set CAN receive filter (non-fatal): %s",
      std::strerror(errno));
  }

  // Initialize state vectors to zero
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
    hw_motor_temp_[i] = 0;
    hw_motor_status_[i] = 0;
  }

  RCLCPP_INFO(get_logger(), "Configured: CAN socket open on '%s'.", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_activate — enable motors and read initial positions
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating: enabling %zu motors...", joint_configs_.size());

  // Send motor enable (Type 3) for all joints
  for (const auto & cfg : joint_configs_) {
    send_can_frame(robstride::encode_enable(cfg.motor_id));
  }

  // Wait for feedback frames to arrive (~2.6ms needed at 1Mbps for 20 motors)
  ::usleep(10000);

  // Drain feedback to read initial motor positions
  struct can_frame frame{};
  while (::recv(can_socket_, &frame, sizeof(frame), MSG_DONTWAIT) == sizeof(frame)) {
    auto fb = robstride::decode_feedback(frame);
    if (!fb) {
      continue;
    }

    auto it = motor_id_to_index_.find(fb->motor_id);
    if (it == motor_id_to_index_.end()) {
      continue;
    }

    size_t idx = it->second;
    hw_positions_[idx] = motor_to_joint(
      joint_configs_[idx], static_cast<double>(fb->position));
  }

  // Set commands to current position to prevent snap-to-zero
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = hw_positions_[i];
  }

  // Initialize feedback timestamps and start Kp ramp
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
// Lifecycle: on_deactivate — zero-torque then stop
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating: sending zero-torque commands...");

  // Step 1: send zero-torque MIT commands (kp=0, kd=0, t_ff=0) — go limp safely
  for (const auto & cfg : joint_configs_) {
    robstride::MitCommand cmd;
    cmd.motor_id = cfg.motor_id;
    send_can_frame(robstride::encode_mit_command(cmd));
  }

  // Brief pause for commands to process
  ::usleep(5000);

  // Step 2: send motor stop (Type 4)
  for (const auto & cfg : joint_configs_) {
    send_can_frame(robstride::encode_stop(cfg.motor_id));
  }

  ramping_ = false;

  RCLCPP_INFO(get_logger(), "Deactivated: all motors stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_cleanup — close CAN socket
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn SteveROSHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");

  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
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
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SteveROSHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

// ---------------------------------------------------------------------------
// read() — non-blocking CAN drain loop with timeout and fault parsing
// ---------------------------------------------------------------------------

hardware_interface::return_type SteveROSHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  struct can_frame frame{};
  while (::recv(can_socket_, &frame, sizeof(frame), MSG_DONTWAIT) == sizeof(frame)) {
    auto fb = robstride::decode_feedback(frame);
    if (!fb) {
      continue;
    }

    auto it = motor_id_to_index_.find(fb->motor_id);
    if (it == motor_id_to_index_.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received feedback from unknown motor_id=%d", fb->motor_id);
      continue;
    }

    size_t idx = it->second;
    const auto & cfg = joint_configs_[idx];

    hw_positions_[idx] = motor_to_joint(cfg, static_cast<double>(fb->position));
    hw_velocities_[idx] = cfg.sign * static_cast<double>(fb->velocity);
    hw_efforts_[idx] = cfg.sign * static_cast<double>(fb->torque);

    // Parse temperature and status/fault byte
    hw_motor_temp_[idx] = fb->temperature;
    hw_motor_status_[idx] = fb->status;

    if (fb->status != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Motor %d ('%s') fault: status=0x%02X, temp=%u°C",
        fb->motor_id, info_.joints[idx].name.c_str(),
        fb->status, fb->temperature);
    }

    last_feedback_time_[idx] = time;
  }

  // Check for feedback timeout on each joint
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
// write() — send MIT mode commands with Kp ramp and per-joint torque clamp
// ---------------------------------------------------------------------------

hardware_interface::return_type SteveROSHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Compute Kp ramp factor (0→1 over kp_ramp_duration_s_ after activation)
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
    float motor_pos = static_cast<float>(joint_to_motor(cfg, hw_commands_[i]));

    robstride::MitCommand cmd;
    cmd.motor_id = cfg.motor_id;
    cmd.p_des = motor_pos;
    cmd.v_des = 0.0f;
    cmd.kp = static_cast<float>(kp_scale * cfg.kp);
    cmd.kd = static_cast<float>(cfg.kd);
    cmd.t_ff = 0.0f;
    cmd.max_torque = static_cast<float>(cfg.max_torque);

    send_can_frame(robstride::encode_mit_command(cmd));
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
