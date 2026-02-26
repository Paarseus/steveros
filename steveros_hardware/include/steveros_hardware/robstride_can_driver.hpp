#ifndef STEVEROS_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_
#define STEVEROS_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <linux/can.h>

#include "steveros_hardware/robstride_protocol.hpp"

namespace robstride
{

class RobstrideCanDriver
{
public:
  RobstrideCanDriver() = default;
  ~RobstrideCanDriver();

  // Non-copyable, non-movable (owns thread + socket)
  RobstrideCanDriver(const RobstrideCanDriver &) = delete;
  RobstrideCanDriver & operator=(const RobstrideCanDriver &) = delete;

  /// Open SocketCAN interface and start background receive thread.
  bool open(const std::string & interface);

  /// Stop receive thread and close socket.
  void close();

  /// Send a raw CAN frame with retries (10 retries, 500us linear backoff).
  /// Thread-safe via send_mutex_.
  bool send_frame(const can_frame & frame);

  /// Get the latest feedback for a motor. Thread-safe copy from cache.
  MotorFeedback get_feedback(int motor_id) const;

  /// Check if any feedback has been received for this motor.
  bool has_valid_feedback(int motor_id) const;

  /// Register a motor's type so feedback can be decoded with correct ranges.
  void register_motor(int motor_id, MotorType type);

  // Motor control sequences (each sends one or more frames via send_frame)
  bool clear_fault(int motor_id);
  bool set_run_mode(int motor_id, uint8_t mode);
  bool enable_motor(int motor_id);
  bool disable_motor(int motor_id);

private:
  void receive_loop();

  int socket_fd_ = -1;
  std::thread receive_thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex send_mutex_;
  mutable std::mutex feedback_mutex_;
  std::unordered_map<int, MotorFeedback> feedback_cache_;
  std::unordered_map<int, MotorType> motor_types_;
};

}  // namespace robstride

#endif  // STEVEROS_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_
