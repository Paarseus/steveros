#include "steveros_hardware/steveros_hardware.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace steveros_hardware
{

// Degrees to radians
static constexpr double DEG_TO_RAD = M_PI / 180.0;

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

  const auto num_joints = info_.joints.size();
  RCLCPP_INFO(
    rclcpp::get_logger("SteveROSHardware"),
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

    RCLCPP_INFO(
      rclcpp::get_logger("SteveROSHardware"),
      "  Joint '%s': motor_id=%d, sign=%d, zero_offset=%.2f deg",
      joint.name.c_str(),
      joint_configs_[i].motor_id,
      joint_configs_[i].sign,
      get_param("zero_offset_deg", 0.0));

    // Validate interfaces
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SteveROSHardware"),
        "Joint '%s' must have exactly one position command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("SteveROSHardware"),
        "Joint '%s' must have exactly 3 state interfaces (position, velocity, effort).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Allocate state/command vectors
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteveROSHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SteveROSHardware"), "Configuring...");

  // TODO: Open CAN bus socket
  // int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  // struct ifreq ifr; strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ);
  // ioctl(sock, SIOCGIFINDEX, &ifr);
  // struct sockaddr_can addr; addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
  // bind(sock, (struct sockaddr *)&addr, sizeof(addr));

  // TODO: Read initial motor positions via CAN
  // For now, initialize to zero (placeholder)
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SteveROSHardware"), "Configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteveROSHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SteveROSHardware"), "Activating...");

  // TODO: Enable motors via CAN (Communication Type 3: motor enable)
  // For each motor: send CAN frame with msg_type=3, motor_id, host_id=0xFD

  // Set commands to current position (prevent jump on activation)
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("SteveROSHardware"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteveROSHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SteveROSHardware"), "Deactivating...");

  // TODO: Disable motors via CAN (Communication Type 4: motor stop)

  RCLCPP_INFO(rclcpp::get_logger("SteveROSHardware"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteveROSHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SteveROSHardware"), "Cleaning up...");

  // TODO: Close CAN socket
  // close(sock);

  RCLCPP_INFO(rclcpp::get_logger("SteveROSHardware"), "Cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

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

hardware_interface::return_type SteveROSHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Read motor feedback via CAN (MIT mode response)
  //
  // For each joint:
  //   raw = read_CAN_feedback(joint_configs_[i].motor_id);
  //   hw_positions_[i] = motor_to_joint(joint_configs_[i], raw.position);
  //   hw_velocities_[i] = joint_configs_[i].sign * raw.velocity;
  //   hw_efforts_[i] = joint_configs_[i].sign * raw.torque;
  //
  // For now: simulate position tracking (placeholder).
  // Position follows command with simple first-order response.
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = hw_commands_[i];
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SteveROSHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Send MIT mode commands via CAN
  //
  // For each joint:
  //   motor_rad = joint_to_motor(joint_configs_[i], hw_commands_[i]);
  //   send_MIT_frame(joint_configs_[i].motor_id, motor_rad, kp=20.0, kd=2.0, t_ff=0.0);
  //
  // MIT frame format (Communication Type 1, from driver.py lines 189-224):
  //   Arbitration ID: motor_id | (host_id << 5) | (1 << 24)
  //   Data[0-1]: p_des  (16-bit, range [-12.5, 12.5] rad)
  //   Data[2-3]: v_des  (16-bit, range [-45, 45] rad/s)
  //   Data[4-5]: kp     (16-bit, range [0, 500])
  //   Data[6-7]: kd     (16-bit, range [0, 5])  -- but lower 4 bits of [6] = torque MSB
  //   Actually: Data[4]: kp (8-bit float mapped), etc. See Robstride protocol docs.
  //
  // Placeholder: commands are applied instantly in read() above.

  return hardware_interface::return_type::OK;
}

double SteveROSHardware::motor_to_joint(const JointConfig & cfg, double motor_rad) const
{
  // Matches main.py: joint_deg = degrees(sign * (motor_rad - offset))
  // But we stay in radians for ros2_control:
  return cfg.sign * (motor_rad - cfg.zero_offset_rad);
}

double SteveROSHardware::joint_to_motor(const JointConfig & cfg, double joint_rad) const
{
  // Matches main.py: motor_rad = radians(joint_deg) / sign + offset
  // Already in radians:
  return joint_rad / cfg.sign + cfg.zero_offset_rad;
}

}  // namespace steveros_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  steveros_hardware::SteveROSHardware,
  hardware_interface::SystemInterface)
