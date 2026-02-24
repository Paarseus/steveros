#ifndef STEVEROS_HARDWARE__STEVEROS_HARDWARE_HPP_
#define STEVEROS_HARDWARE__STEVEROS_HARDWARE_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "steveros_hardware/visibility_control.h"

namespace steveros_hardware
{

/// Per-joint configuration parsed from URDF ros2_control tags.
struct JointConfig
{
  int motor_id = 0;
  int sign = 1;
  double zero_offset_rad = 0.0;  // zero_offset_deg converted to radians
  double kp = 20.0;
  double kd = 2.0;
  double max_torque = 12.0;  // per-joint torque clamp from URDF effort limit (Nm)
};

/// Hardware interface for KBot 20-DOF Robstride humanoid over CAN bus.
///
/// Implements the ros2_control SystemInterface lifecycle.
/// Uses SocketCAN with the Robstride MIT Mode protocol to control motors.
class SteveROSHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SteveROSHardware)

  STEVEROS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  STEVEROS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  STEVEROS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  STEVEROS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  STEVEROS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  STEVEROS_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  STEVEROS_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  STEVEROS_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  STEVEROS_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Convert motor radians to joint radians: sign * (motor_rad - offset)
  double motor_to_joint(const JointConfig & cfg, double motor_rad) const;

  /// Convert joint radians to motor radians: joint_rad / sign + offset
  double joint_to_motor(const JointConfig & cfg, double joint_rad) const;

  /// Send a raw CAN extended frame
  bool send_can_frame(const can_frame & frame);

  // CAN interface name (e.g. "can0")
  std::string can_interface_;

  // SocketCAN file descriptor
  int can_socket_ = -1;

  // Per-joint config (parallel to info_.joints)
  std::vector<JointConfig> joint_configs_;

  // Motor ID → joint index for O(1) feedback routing
  std::unordered_map<int, size_t> motor_id_to_index_;

  // State interfaces: position (rad), velocity (rad/s), effort (Nm)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Command interfaces: position (rad)
  std::vector<double> hw_commands_;

  // Motor status from feedback frame byte [6] and [7]
  std::vector<uint8_t> hw_motor_temp_;
  std::vector<uint8_t> hw_motor_status_;

  // Feedback timeout: last time we received feedback for each joint
  std::vector<rclcpp::Time> last_feedback_time_;
  double feedback_timeout_s_ = 0.1;  // 100ms default (10 missed cycles at 100Hz)

  // Kp ramp on activation
  rclcpp::Time activation_time_{0, 0, RCL_ROS_TIME};
  bool ramping_ = false;
  double kp_ramp_duration_s_ = 1.0;
};

}  // namespace steveros_hardware

#endif  // STEVEROS_HARDWARE__STEVEROS_HARDWARE_HPP_
