#include "steveros_hardware/robstride_can_driver.hpp"

#include <cerrno>
#include <cstring>

#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace robstride
{

static constexpr int kMaxSendRetries = 10;
static constexpr int kSendBackoffBaseUs = 500;
static constexpr int kPollTimeoutMs = 10;
static constexpr int kRecvTimeoutMs = 100;

static auto logger() { return rclcpp::get_logger("RobstrideCanDriver"); }

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

RobstrideCanDriver::~RobstrideCanDriver()
{
  close();
}

bool RobstrideCanDriver::open(const std::string & interface)
{
  if (socket_fd_ >= 0) {
    RCLCPP_WARN(logger(), "Socket already open, closing first.");
    close();
  }

  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(logger(), "Failed to create CAN socket: %s", std::strerror(errno));
    return false;
  }

  // Resolve interface name to index
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      logger(), "Failed to find CAN interface '%s': %s",
      interface.c_str(), std::strerror(errno));
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Bind socket to interface
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger(), "Failed to bind CAN socket: %s", std::strerror(errno));
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Set receive filter: accept only Type 2 (feedback) frames
  struct can_filter rfilter{};
  rfilter.can_id = (kTypeFeedback << 24) | CAN_EFF_FLAG;
  rfilter.can_mask = (0x1Fu << 24) | CAN_EFF_FLAG;
  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER,
    &rfilter, sizeof(rfilter)) < 0)
  {
    RCLCPP_WARN(
      logger(), "Failed to set CAN receive filter (non-fatal): %s",
      std::strerror(errno));
  }

  // Set SO_RCVTIMEO as safety backstop
  struct timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = kRecvTimeoutMs * 1000;
  ::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Start background receive thread
  running_ = true;
  receive_thread_ = std::thread(&RobstrideCanDriver::receive_loop, this);

  RCLCPP_INFO(logger(), "Opened CAN interface '%s', receive thread started.", interface.c_str());
  return true;
}

void RobstrideCanDriver::close()
{
  if (running_) {
    running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
  }

  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_cache_.clear();
    motor_types_.clear();
  }

  RCLCPP_INFO(logger(), "CAN driver closed.");
}

// ---------------------------------------------------------------------------
// Background receive thread
// ---------------------------------------------------------------------------

void RobstrideCanDriver::receive_loop()
{
  struct pollfd pfd{};
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  while (running_) {
    int ret = ::poll(&pfd, 1, kPollTimeoutMs);
    if (ret <= 0) {
      continue;
    }

    struct can_frame frame{};
    ssize_t nbytes = ::read(socket_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
      continue;
    }

    // Extract motor_id from arb_id to look up motor type
    uint32_t arb_id = frame.can_id & CAN_EFF_MASK;
    int motor_id = (arb_id >> 8) & 0xFF;

    MotorParams params;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      auto it = motor_types_.find(motor_id);
      if (it == motor_types_.end()) {
        continue;  // Skip frame if motor_id not registered
      }
      params = get_motor_params(it->second);
    }

    auto fb = decode_feedback(frame, params);
    if (!fb) {
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback_cache_[fb->motor_id] = *fb;
    }
  }
}

// ---------------------------------------------------------------------------
// Send with retries
// ---------------------------------------------------------------------------

bool RobstrideCanDriver::send_frame(const can_frame & frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);

  for (int attempt = 0; attempt <= kMaxSendRetries; ++attempt) {
    ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(frame));
    if (nbytes == sizeof(frame)) {
      return true;
    }

    if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK) {
      ::usleep(kSendBackoffBaseUs * (attempt + 1));
      continue;
    }

    RCLCPP_ERROR(
      logger(), "CAN write failed (non-retryable): %s (arb_id=0x%08X)",
      std::strerror(errno), frame.can_id);
    return false;
  }

  RCLCPP_WARN(
    logger(), "CAN write failed after %d retries (ENOBUFS): arb_id=0x%08X",
    kMaxSendRetries, frame.can_id);
  return false;
}

// ---------------------------------------------------------------------------
// Feedback cache access
// ---------------------------------------------------------------------------

MotorFeedback RobstrideCanDriver::get_feedback(int motor_id) const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  auto it = feedback_cache_.find(motor_id);
  if (it != feedback_cache_.end()) {
    return it->second;
  }
  return MotorFeedback{};
}

bool RobstrideCanDriver::has_valid_feedback(int motor_id) const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  return feedback_cache_.count(motor_id) > 0;
}

void RobstrideCanDriver::register_motor(int motor_id, MotorType type)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  motor_types_[motor_id] = type;
}

// ---------------------------------------------------------------------------
// Motor control sequences
// ---------------------------------------------------------------------------

bool RobstrideCanDriver::clear_fault(int motor_id)
{
  return send_frame(encode_stop_clear_fault(motor_id));
}

bool RobstrideCanDriver::set_run_mode(int motor_id, uint8_t mode)
{
  return send_frame(encode_param_write(motor_id, kParamRunMode, static_cast<float>(mode)));
}

bool RobstrideCanDriver::enable_motor(int motor_id)
{
  return send_frame(encode_enable(motor_id));
}

bool RobstrideCanDriver::disable_motor(int motor_id)
{
  return send_frame(encode_stop(motor_id));
}

}  // namespace robstride
