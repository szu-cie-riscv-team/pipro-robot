#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <k1muse_interfaces/srv/web_request.hpp>
#include <nlohmann/json_fwd.hpp>
#include "k1muse_perception/ocr_engine.hpp"

namespace k1muse_perception
{
// Internal vision child: no camera subscription and no manager permit of its own.
class OcrNode : public rclcpp::Node
{
public:
  explicit OcrNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OcrNode() override;
  std::string authorize(const std::string & operation, const std::string & work, uint64_t generation);
  void try_submit(sensor_msgs::msg::Image::ConstSharedPtr image);
  // For synchronous lifecycle/destructor paths only. Does not depend on executor progress.
  void stop_and_wait();
  void shutdown();

private:
  using Control = k1muse_interfaces::srv::WebRequest;
  using Header = std::shared_ptr<rmw_request_id_t>;
  struct Command
  {
    Header header;
    std::shared_ptr<Control::Request> request;
    std::string work, authorization;
    uint64_t generation{0};
  };
  struct Frame
  {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    std::string work;
    uint64_t generation{0};
  };
  void control(Header header, std::shared_ptr<Control::Request> request);
  void respond(Header header, const Control::Request & request, bool ok, const std::string & error);
  void worker_loop();
  void process_frame(OcrEngine & engine, const Frame & frame);
  void record_perf(const sensor_msgs::msg::Image & image, const nlohmann::json & result);
  void report_fault(const Frame & frame, const std::string & error);

  OcrConfig config_;
  bool enable_perf_stats_{false};
  // Only the existing OCR worker accesses these fields; no control-thread flush.
  std::chrono::steady_clock::time_point perf_started_{};
  uint64_t perf_frames_{0}, perf_failed_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<Control>::SharedPtr service_;
  rclcpp::Client<Control>::SharedPtr parent_;
  std::mutex mutex_;
  std::condition_variable wake_, idle_;
  std::optional<Command> command_;
  std::optional<Frame> pending_;
  bool busy_{false}, enabled_{false}, ready_{false}, quitting_{false};
  std::atomic_bool stop_{true};
  uint64_t authorization_{0}, generation_{0};
  std::string expected_operation_, expected_work_, work_;
  uint64_t expected_generation_{0};
  std::thread worker_;
};
}  // namespace k1muse_perception
