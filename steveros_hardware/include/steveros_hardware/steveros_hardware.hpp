#ifndef STEVEROS_HARDWARE__STEVEROS_HARDWARE_HPP_
#define STEVEROS_HARDWARE__STEVEROS_HARDWARE_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "steveros_hardware/robstride_can_driver.hpp"
#include "steveros_hardware/robstride_protocol.hpp"
#include "steveros_hardware/visibility_control.h"

namespace steveros_hardware
{

struct JointConfig
{
  int motor_id = 0;
  int sign = 1;
  double zero_offset_rad = 0.0;
  double kp = 20.0;
  double kd = 2.0;
  double max_torque = 12.0;
  robstride::MotorType motor_type = robstride::MotorType::RS02;
};

struct JointState
{
  double position = std::numeric_limits<double>::quiet_NaN();
  double velocity = std::numeric_limits<double>::quiet_NaN();
  double effort   = std::numeric_limits<double>::quiet_NaN();
  double command_position = 0.0;
  double command_velocity = 0.0;
  bool online = false;
};

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
  double motor_to_joint(const JointConfig & cfg, double motor_rad) const;
  double joint_to_motor(const JointConfig & cfg, double joint_rad) const;

  std::string can_interface_;
  std::unique_ptr<robstride::RobstrideCanDriver> driver_;

  std::vector<JointConfig> joint_configs_;
  std::vector<JointState> joint_states_;
  std::vector<double> hold_position_;
  std::unordered_map<int, size_t> motor_id_to_index_;

  std::vector<rclcpp::Time> last_feedback_time_;
  double feedback_timeout_s_ = 0.1;

  rclcpp::Time activation_time_{0, 0, RCL_ROS_TIME};
  bool ramping_ = false;
  double kp_ramp_duration_s_ = 1.0;
};

}  // namespace steveros_hardware

#endif  // STEVEROS_HARDWARE__STEVEROS_HARDWARE_HPP_
