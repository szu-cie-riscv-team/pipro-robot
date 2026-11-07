#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <k1muse_interfaces/action/arm_command.hpp>
#include <k1muse_interfaces/action/lift_command.hpp>
#include <k1muse_interfaces/msg/arm_status.hpp>
#include <k1muse_interfaces/msg/chassis_status.hpp>
#include <k1muse_interfaces/msg/lift_status.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <poll.h>
#include <string>
#include <thread>
#include <utility>

#include <termios.h>
#include <unistd.h>

#include "k1muse_communication/protocol.hpp"
#include "k1muse_communication/stm32_communication_node.hpp"

namespace k1muse_communication
{
namespace
{

using protocol::Frame;
using LiftCommand = k1muse_interfaces::action::LiftCommand;
using ArmCommand = k1muse_interfaces::action::ArmCommand;
using LiftGoalHandle = rclcpp_action::ServerGoalHandle<LiftCommand>;
using ArmGoalHandle = rclcpp_action::ServerGoalHandle<ArmCommand>;

struct PendingRequest
{
  bool active{};
  uint8_t sequence{};
  uint8_t command{};
};

struct LiftPending
{
  PendingRequest request;
  std::shared_ptr<LiftGoalHandle> goal_handle;
};

struct ArmPending
{
  PendingRequest request;
  std::shared_ptr<ArmGoalHandle> goal_handle;
  bool demo_valid{false};
  protocol::ArmDemo demo;
};

bool is_terminal_result(uint8_t result)
{
  return result == protocol::kResultRejected || result == protocol::kResultCompleted ||
         result == protocol::kResultAborted || result == protocol::kResultFailed ||
         result == protocol::kResultSuperseded;
}

class Stm32CommunicationNode : public rclcpp::Node
{
public:
  explicit Stm32CommunicationNode(const rclcpp::NodeOptions & options)
  : Node("stm32_communication", options),
    serial_device_(declare_parameter<std::string>("serial_device", "")),
    command_timeout_(declare_parameter<int>("cmd_vel_timeout_ms", 200)),
    reconnect_period_(declare_parameter<int>("reconnect_period_ms", 1000))
  {
    status_publisher_ = create_publisher<k1muse_interfaces::msg::ChassisStatus>(
      "/k1muse/chassis/status", 10);
    lift_status_publisher_ = create_publisher<k1muse_interfaces::msg::LiftStatus>(
      "/k1muse/lift/status", rclcpp::QoS(1).reliable());
    arm_status_publisher_ = create_publisher<k1muse_interfaces::msg::ArmStatus>(
      "/k1muse/arm/status", rclcpp::QoS(1).reliable());
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(1).reliable(),
      std::bind(&Stm32CommunicationNode::handle_command, this, std::placeholders::_1));
    reset_odometry_ = create_service<std_srvs::srv::Trigger>("/k1muse/chassis/reset_odometry",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr reply) {
        std::scoped_lock lock(command_mutex_, pending_mutex_);
        const auto & s = last_chassis_;
        if (command_active_ || lift_pending_.request.active || arm_pending_.request.active ||
          std::chrono::steady_clock::now() - chassis_received_ > std::chrono::milliseconds(500) ||
          !std::isfinite(s.wcs_vx) || !std::isfinite(s.wcs_vy) || !std::isfinite(s.omega) ||
          std::hypot(s.wcs_vx, s.wcs_vy) >= .02 || std::abs(s.omega) >= .05) {
          reply->message = "requires fresh stopped chassis and no active actuator command";return;
        }
        // MCU ODOM payload: direction,x,y, each IEEE float32 zero. No motion/owner change.
        reply->success = write_frame(Frame{protocol::kSourceHost, protocol::kTargetChassis,
          protocol::kChassisOdom, next_sequence(), std::vector<uint8_t>(12, 0)});
        reply->message = reply->success ? "reset sent; verify a subsequent zero ODOM status" : "serial write failed";
      });
    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&Stm32CommunicationNode::check_command_timeout, this));

    lift_action_server_ = rclcpp_action::create_server<LiftCommand>(
      this, "/k1muse/lift/command",
      std::bind(
        &Stm32CommunicationNode::handle_lift_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&Stm32CommunicationNode::reject_lift_cancel, this, std::placeholders::_1),
      std::bind(&Stm32CommunicationNode::handle_lift_accepted, this, std::placeholders::_1));
    arm_action_server_ = rclcpp_action::create_server<ArmCommand>(
      this, "/k1muse/arm/command",
      std::bind(
        &Stm32CommunicationNode::handle_arm_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&Stm32CommunicationNode::reject_arm_cancel, this, std::placeholders::_1),
      std::bind(&Stm32CommunicationNode::handle_arm_accepted, this, std::placeholders::_1));

    if (serial_device_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'serial_device' is required");
    }
    reader_thread_ = std::thread(&Stm32CommunicationNode::reader_loop, this);
  }

  ~Stm32CommunicationNode() override
  {
    send_stop();
    running_.store(false);
    close_serial();
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

private:
  bool open_serial()
  {
    if (serial_device_.empty()) {
      return false;
    }

    const int fd = ::open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(), "Cannot open %s: %s", serial_device_.c_str(), std::strerror(errno));
      return false;
    }

    termios settings{};
    if (tcgetattr(fd, &settings) != 0) {
      RCLCPP_ERROR(get_logger(), "Cannot read serial settings: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }
    cfmakeraw(&settings);
    settings.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD | CS8);
    settings.c_cflag &= static_cast<tcflag_t>(~PARENB);
    settings.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
#ifdef CRTSCTS
    settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, B115200) != 0 || cfsetospeed(&settings, B115200) != 0 ||
      tcsetattr(fd, TCSANOW, &settings) != 0)
    {
      RCLCPP_ERROR(get_logger(), "Cannot configure USART2: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }
    tcflush(fd, TCIFLUSH);

    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!running_.load()) {
      ::close(fd);
      return false;
    }
    ++connection_generation_;
    serial_fd_.store(fd);
    RCLCPP_INFO(get_logger(), "Connected to %s", serial_device_.c_str());
    return true;
  }

  void close_serial()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    const int fd = serial_fd_.exchange(-1);
    if (fd >= 0) {
      ::close(fd);
    }
  }

  void close_serial_if_current(int fd)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_.load() == fd) {
      ::close(fd);
      serial_fd_.store(-1);
    }
  }

  bool write_frame(const Frame & frame, uint64_t generation = 0)
  {
    const auto bytes = protocol::encode_frame(frame);
    if (bytes.empty()) {
      return false;
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);
    const int fd = serial_fd_.load();
    if (fd < 0 || (generation && generation != connection_generation_.load())) {
      return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    size_t offset = 0;
    while (offset < bytes.size()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        errno = ETIMEDOUT;
        break;
      }
      const ssize_t written = ::write(fd, bytes.data() + offset, bytes.size() - offset);
      if (written > 0) {
        offset += static_cast<size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) {
          errno = ETIMEDOUT;
          break;
        }
        pollfd poll_fd{fd, POLLOUT, 0};
        const int ready = ::poll(&poll_fd, 1, static_cast<int>(remaining));
        if (ready > 0 && (poll_fd.revents & POLLOUT) &&
          !(poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)))
        {
          continue;
        }
        if (ready < 0 && errno == EINTR) {
          continue;
        }
        if (ready == 0) {
          errno = ETIMEDOUT;
        } else if (ready > 0) {
          errno = EIO;
        }
      }
      break;
    }
    if (offset == bytes.size()) {
      return true;
    }
    RCLCPP_ERROR(get_logger(), "Serial write failed: %s", std::strerror(errno));
    ::close(fd);
    serial_fd_.store(-1);
    return false;
  }

  uint8_t next_sequence()
  {
    return next_sequence_++;
  }

  bool send_stop()
  {
    return write_frame(protocol::make_chassis_stop(next_sequence()));
  }

  void invalidate_command()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_active_ = false;
  }

  void handle_command(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (message->linear.z != 0.0 || message->angular.x != 0.0 || message->angular.y != 0.0) {
      RCLCPP_WARN(get_logger(), "Ignoring non-planar Twist components");
    }

    if (!std::isfinite(message->linear.x) || !std::isfinite(message->linear.y) ||
      !std::isfinite(message->angular.z) ||
      !std::isfinite(static_cast<float>(std::hypot(message->linear.x, message->linear.y))) ||
      !std::isfinite(static_cast<float>(message->angular.z)))
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite or unrepresentable /cmd_vel; sending STOP");
      invalidate_command();
      send_stop();
      return;
    }

    const bool stop = message->linear.x == 0.0 && message->linear.y == 0.0 &&
      message->angular.z == 0.0;
    const bool sent = stop ? send_stop() :
      write_frame(
      protocol::make_chassis_mov(
        next_sequence(), message->linear.x, message->linear.y, message->angular.z));

    std::lock_guard<std::mutex> lock(command_mutex_);
    command_active_ = sent && !stop;
    if (command_active_) {
      last_command_time_ = std::chrono::steady_clock::now();
    }
  }

  void check_command_timeout()
  {
    bool should_stop = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      should_stop = command_active_ &&
        std::chrono::steady_clock::now() - last_command_time_ >= command_timeout_;
      if (should_stop) {
        command_active_ = false;
      }
    }
    if (should_stop) {
      RCLCPP_WARN(get_logger(), "/cmd_vel timed out; sending STOP");
      send_stop();
    }
  }

  rclcpp_action::GoalResponse handle_lift_goal(
    const rclcpp_action::GoalUUID & id, std::shared_ptr<const LiftCommand::Goal> goal)
  {
    if (serial_fd_.load() < 0 || !is_valid_lift_goal(*goal)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (lift_pending_.request.active) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Only accepted callbacks own an actuator slot; a lost admission ACK must not reserve it.
    lift_prepared_ = std::make_pair(id, connection_generation_.load());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse reject_lift_cancel(const std::shared_ptr<LiftGoalHandle>)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_lift_accepted(const std::shared_ptr<LiftGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const uint8_t sequence = next_sequence();
    const Frame frame = goal->command == LiftCommand::Goal::HOME ?
      protocol::make_lift_home(sequence) :
      protocol::make_lift_move_z(sequence, goal->target_z_m);

    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      if (!lift_prepared_ || lift_prepared_->first != goal_handle->get_goal_id() ||
        lift_prepared_->second != connection_generation_.load() ||
        serial_fd_.load() < 0 || lift_pending_.request.active)
      {
        goal_handle->abort(std::make_shared<LiftCommand::Result>());
        return;
      }
      lift_pending_.request.active = true;
      lift_pending_.request.sequence = sequence;
      lift_pending_.request.command = goal->command;
      lift_pending_.goal_handle = goal_handle;
    }
    const auto generation = lift_prepared_->second;
    lift_prepared_.reset();
    if (!write_frame(frame, generation)) {
      abort_lift_transport();
    }
  }

  rclcpp_action::GoalResponse handle_arm_goal(
    const rclcpp_action::GoalUUID & id, std::shared_ptr<const ArmCommand::Goal> goal)
  {
    if (serial_fd_.load() < 0 || !is_valid_arm_goal(*goal)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (arm_pending_.request.active) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    arm_prepared_ = std::make_pair(id, connection_generation_.load());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse reject_arm_cancel(const std::shared_ptr<ArmGoalHandle>)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_arm_accepted(const std::shared_ptr<ArmGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const uint8_t sequence = next_sequence();
    Frame frame;
    switch (goal->command) {
      case ArmCommand::Goal::HOME:
        frame = protocol::make_arm_home(sequence);
        break;
      case ArmCommand::Goal::MOVE_POSE:
        frame = protocol::make_arm_move_pose(
          sequence, goal->x, goal->y, goal->z, goal->roll, goal->pitch);
        break;
      case ArmCommand::Goal::GRIPPER:
        frame = protocol::make_arm_gripper(sequence, goal->target_rad);
        break;
      case ArmCommand::Goal::GRIP:
        frame = protocol::make_arm_grip(sequence);
        break;
      case ArmCommand::Goal::TOP_PICK_HOLD:
      case ArmCommand::Goal::TOP_PICK_PLACE_LEFT:
      case ArmCommand::Goal::TOP_PICK_PLACE_RIGHT:
      case ArmCommand::Goal::PLACE_LEFT:
      case ArmCommand::Goal::PLACE_RIGHT:
        frame = protocol::make_arm_demo(
          sequence, goal->command, goal->x, goal->y, goal->z, goal->expected_lift_z_m);
        break;
      default:
        return;
    }

    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      if (!arm_prepared_ || arm_prepared_->first != goal_handle->get_goal_id() ||
        arm_prepared_->second != connection_generation_.load() ||
        serial_fd_.load() < 0 || arm_pending_.request.active)
      {
        goal_handle->abort(std::make_shared<ArmCommand::Result>());
        return;
      }
      arm_pending_.request.active = true;
      arm_pending_.request.sequence = sequence;
      arm_pending_.request.command = goal->command;
      arm_pending_.goal_handle = goal_handle;
    }
    const auto generation = arm_prepared_->second;
    arm_prepared_.reset();
    if (!write_frame(frame, generation)) {
      abort_arm_transport();
    }
  }

  bool is_valid_lift_goal(const LiftCommand::Goal & goal) const
  {
    return (goal.command == LiftCommand::Goal::HOME || goal.command == LiftCommand::Goal::MOVE_Z) &&
           std::isfinite(goal.target_z_m);
  }

  bool is_valid_arm_goal(const ArmCommand::Goal & goal) const
  {
    const bool supported = goal.command == ArmCommand::Goal::HOME ||
      goal.command == ArmCommand::Goal::MOVE_POSE || goal.command == ArmCommand::Goal::GRIPPER ||
      goal.command == ArmCommand::Goal::GRIP ||
      goal.command == ArmCommand::Goal::TOP_PICK_HOLD ||
      goal.command == ArmCommand::Goal::TOP_PICK_PLACE_LEFT ||
      goal.command == ArmCommand::Goal::TOP_PICK_PLACE_RIGHT ||
      goal.command == ArmCommand::Goal::PLACE_LEFT || goal.command == ArmCommand::Goal::PLACE_RIGHT;
    return supported && std::isfinite(goal.x) && std::isfinite(goal.y) &&
           std::isfinite(goal.z) && std::isfinite(goal.roll) && std::isfinite(goal.pitch) &&
           std::isfinite(goal.target_rad) && std::isfinite(goal.expected_lift_z_m);
  }

  void publish_status(const protocol::ChassisStatus & status)
  {
    k1muse_interfaces::msg::ChassisStatus message;
    message.header.stamp = now();
    message.tick_ms = status.tick_ms;
    message.state = status.state;
    message.move_cs = status.move_cs;
    message.motor_block_flags = status.motor_block_flags;
    message.wcs_vx = status.wcs_vx;
    message.wcs_vy = status.wcs_vy;
    message.omega = status.omega;
    message.wcs_x = status.wcs_x;
    message.wcs_y = status.wcs_y;
    message.wcs_direction = status.wcs_direction;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      last_chassis_ = status;chassis_received_ = std::chrono::steady_clock::now();
    }
    status_publisher_->publish(message);
  }

  void publish_lift_status(const protocol::LiftStatus & status)
  {
    k1muse_interfaces::msg::LiftStatus message;
    message.header.stamp = now();
    message.is_homed = status.is_homed;
    message.is_busy = status.is_busy;
    message.has_fault = status.has_fault;
    message.fault_reason = status.fault_reason;
    message.current_z = status.current_z;
    message.target_z = status.target_z;
    lift_status_publisher_->publish(message);

    std::shared_ptr<LiftGoalHandle> goal_handle;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      goal_handle = lift_pending_.goal_handle;
    }
    if (goal_handle) {
      auto feedback = std::make_shared<LiftCommand::Feedback>();
      feedback->status = message;
      goal_handle->publish_feedback(feedback);
    }
  }

  void publish_arm_status(const protocol::ArmStatus & status)
  {
    k1muse_interfaces::msg::ArmStatus message;
    message.header.stamp = now();
    message.tick_ms = status.tick_ms;
    message.state = status.state;
    message.status = status.status;
    message.is_busy = status.is_busy;
    message.has_fault = status.has_fault;
    message.active_cmd = status.active_cmd;
    message.active_source = status.active_source;
    message.active_seq = status.active_seq;
    message.diag_code = status.diag_code;
    message.current_x = status.current_x;
    message.current_y = status.current_y;
    message.current_z = status.current_z;
    message.target_x = status.target_x;
    message.target_y = status.target_y;
    message.target_z = status.target_z;
    message.current_gripper_rad = status.current_gripper_rad;
    message.target_gripper_rad = status.target_gripper_rad;
    message.fault_flags = status.fault_flags;
    arm_status_publisher_->publish(message);

    std::shared_ptr<ArmGoalHandle> goal_handle;
    auto feedback = std::make_shared<ArmCommand::Feedback>();
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      goal_handle = arm_pending_.goal_handle;
      feedback->demo_valid = arm_pending_.demo_valid;
      feedback->holding = arm_pending_.demo.holding;
      feedback->demo_step = arm_pending_.demo.step;
      feedback->demo_step_count = arm_pending_.demo.step_count;
      feedback->demo_progress = arm_pending_.demo.progress;
    }
    if (goal_handle) {
      feedback->status = message;
      goal_handle->publish_feedback(feedback);
    }
  }

  void handle_lift_result(const protocol::LiftResult & result)
  {
    if (!is_terminal_result(result.result)) {
      return;
    }

    std::shared_ptr<LiftGoalHandle> goal_handle;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      if (!lift_pending_.request.active ||
        result.request_source != protocol::kSourceHost ||
        result.request_cmd != lift_pending_.request.command ||
        result.request_seq != lift_pending_.request.sequence)
      {
        return;
      }
      goal_handle = lift_pending_.goal_handle;
      lift_pending_ = LiftPending{};
    }
    if (!goal_handle) {
      return;
    }

    auto action_result = std::make_shared<LiftCommand::Result>();
    action_result->result = result.result;
    action_result->reject_reason = result.reject_reason;
    action_result->fault_reason = result.fault_reason;
    if (result.result == protocol::kResultCompleted) {
      goal_handle->succeed(action_result);
    } else {
      goal_handle->abort(action_result);
    }
  }

  void handle_arm_result(const protocol::ArmResult & result)
  {
    if (!is_terminal_result(result.result)) {
      return;
    }

    std::shared_ptr<ArmGoalHandle> goal_handle;
    auto action_result = std::make_shared<ArmCommand::Result>();
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      if (!arm_pending_.request.active ||
        result.request_source != protocol::kSourceHost ||
        result.request_cmd != arm_pending_.request.command ||
        result.request_seq != arm_pending_.request.sequence)
      {
        return;
      }
      goal_handle = arm_pending_.goal_handle;
      action_result->demo_valid = arm_pending_.demo_valid;
      action_result->holding = arm_pending_.demo.holding;
      action_result->demo_step = arm_pending_.demo.step;
      arm_pending_ = ArmPending{};
    }
    if (!goal_handle) {
      return;
    }

    action_result->result = result.result;
    action_result->reason = result.reason;
    action_result->fault_source = result.fault_source;
    action_result->diag_code = result.diag_code;
    if (result.result == protocol::kResultCompleted) {
      goal_handle->succeed(action_result);
    } else {
      goal_handle->abort(action_result);
    }
  }

  void abort_lift_transport()
  {
    std::shared_ptr<LiftGoalHandle> goal_handle;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      goal_handle = lift_pending_.goal_handle;
      lift_pending_ = LiftPending{};
    }
    if (goal_handle) {
      goal_handle->abort(std::make_shared<LiftCommand::Result>());
    }
  }

  void abort_arm_transport()
  {
    std::shared_ptr<ArmGoalHandle> goal_handle;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      goal_handle = arm_pending_.goal_handle;
      arm_pending_ = ArmPending{};
    }
    if (goal_handle) {
      goal_handle->abort(std::make_shared<ArmCommand::Result>());
    }
  }

  void handle_transport_failure()
  {
    invalidate_command();
    abort_lift_transport();
    abort_arm_transport();
  }

  void reader_loop()
  {
    protocol::StreamParser parser;
    std::array<uint8_t, 256> bytes{};
    while (running_.load()) {
      int fd = serial_fd_.load();
      if (fd < 0) {
        handle_transport_failure();
        parser = protocol::StreamParser{};
        if (!open_serial()) {
          std::this_thread::sleep_for(reconnect_period_);
        }
        continue;
      }

      pollfd poll_fd{fd, POLLIN, 0};
      const int poll_result = ::poll(&poll_fd, 1, 100);
      if (poll_result < 0 && errno == EINTR) {
        continue;
      }
      if (poll_result <= 0) {
        continue;
      }
      if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        RCLCPP_ERROR(get_logger(), "Serial connection lost");
        close_serial_if_current(fd);
        handle_transport_failure();
        continue;
      }

      const ssize_t count = ::read(fd, bytes.data(), bytes.size());
      if (count > 0) {
        auto result = parser.consume(bytes.data(), static_cast<size_t>(count));
        if (result.rejected_frames > 0) {
          RCLCPP_WARN(
            get_logger(), "Rejected %zu malformed serial frame(s)",
            result.rejected_frames);
        }
        for (const auto & frame : result.frames) {
          dispatch_frame(frame);
        }
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_ERROR(get_logger(), "Serial read failed: %s", std::strerror(errno));
        close_serial_if_current(fd);
        handle_transport_failure();
      }
    }
  }

  void dispatch_frame(const Frame & frame)
  {
    if (frame.source != protocol::kSourceMcu) {
      return;
    }
    if (frame.target == protocol::kTargetChassis && frame.command == protocol::kChassisStatus) {
      protocol::ChassisStatus status;
      if (protocol::decode_chassis_status(frame, &status)) {
        publish_status(status);
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring CHASSIS STATUS with invalid payload length");
      }
      return;
    }
    if (frame.target == protocol::kTargetLift && frame.command == protocol::kLiftStatus) {
      protocol::LiftStatus status;
      if (protocol::decode_lift_status(frame, &status)) {
        publish_lift_status(status);
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring LIFT STATUS with invalid payload length");
      }
      return;
    }
    if (frame.target == protocol::kTargetLift && frame.command == protocol::kLiftResult) {
      protocol::LiftResult result;
      if (protocol::decode_lift_result(frame, &result)) {
        handle_lift_result(result);
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring LIFT RESULT with invalid payload length");
      }
      return;
    }
    if (frame.target == protocol::kTargetArm && frame.command == protocol::kArmStatus) {
      protocol::ArmStatus status;
      if (protocol::decode_arm_status(frame, &status)) {
        publish_arm_status(status);
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring ARM STATUS with invalid payload length");
      }
      return;
    }
    if (frame.target == protocol::kTargetArm && frame.command == protocol::kArmResult) {
      protocol::ArmResult result;
      if (protocol::decode_arm_result(frame, &result)) {
        handle_arm_result(result);
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring ARM RESULT with invalid payload length");
      }
    }
    if (frame.target == protocol::kTargetArm && frame.command == protocol::kArmDemo) {
      protocol::ArmDemo demo;
      if (!protocol::decode_arm_demo(frame, &demo)) {return;}
      std::lock_guard<std::mutex> lock(pending_mutex_);
      const auto & request = arm_pending_.request;
      if (request.active && demo.request_source == protocol::kSourceHost &&
        demo.request_cmd == request.command && demo.request_seq == request.sequence)
      {
        arm_pending_.demo = demo;
        arm_pending_.demo_valid = true;
      }
    }
  }

  std::string serial_device_;
  std::chrono::milliseconds command_timeout_;
  std::chrono::milliseconds reconnect_period_;
  std::atomic<bool> running_{true};
  std::atomic<int> serial_fd_{-1};
  std::atomic<uint64_t> connection_generation_{0};
  std::optional<std::pair<rclcpp_action::GoalUUID, uint64_t>> lift_prepared_, arm_prepared_;
  std::mutex serial_mutex_;
  std::mutex command_mutex_;
  std::mutex pending_mutex_;
  std::thread reader_thread_;
  uint8_t next_sequence_{};
  bool command_active_{};
  protocol::ChassisStatus last_chassis_;
  std::chrono::steady_clock::time_point chassis_received_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odometry_;
  std::chrono::steady_clock::time_point last_command_time_{};
  LiftPending lift_pending_;
  ArmPending arm_pending_;
  rclcpp::Publisher<k1muse_interfaces::msg::ChassisStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<k1muse_interfaces::msg::LiftStatus>::SharedPtr lift_status_publisher_;
  rclcpp::Publisher<k1muse_interfaces::msg::ArmStatus>::SharedPtr arm_status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp_action::Server<LiftCommand>::SharedPtr lift_action_server_;
  rclcpp_action::Server<ArmCommand>::SharedPtr arm_action_server_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_stm32_communication_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<Stm32CommunicationNode>(options);
}

}  // namespace k1muse_communication
