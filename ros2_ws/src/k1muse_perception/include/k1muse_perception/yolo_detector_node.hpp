#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <k1muse_interfaces/srv/web_request.hpp>

#include "k1muse_perception/byte_tracker.hpp"
#include "k1muse_perception/yolo_engine.hpp"

namespace k1muse_perception
{

class OcrNode;

class YoloDetectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit YoloDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::shared_ptr<OcrNode> ocr = {});
  ~YoloDetectorNode() override;

private:
  friend class YoloPerfTestAccess;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Control = k1muse_interfaces::srv::WebRequest;
  using Header = std::shared_ptr<rmw_request_id_t>;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message);
  void set_enabled_callback(
    Header header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request);
  void reply_enabled(Header header, bool success, const std::string & message);
  void prepare(const Control::Request & request, Control::Response & response);
  void release_inference(Header header = {}, const std::string & failure = {});
  void finish_prepare(const std::string & work, bool success, const std::string & error);
  void call_ocr(const std::string & operation, const std::string & work,
    std::function<void(bool, const std::string &)> completion);
  void open_input(Header header);
  void stop_group(Header header = {}, const std::string & failure = {});
  void stop_local();
  bool read_engine_config(YoloEngineConfig & config, std::string & error) const;
  void publish_detections(
    const sensor_msgs::msg::Image::ConstSharedPtr & message,
    std::vector<vision_msgs::msg::Detection2D> detections,
    YoloFrameTiming * timing = nullptr,
    std::chrono::steady_clock::time_point callback_started = {});
  bool record_perf(const YoloFrameTiming & timing, bool success);
  void report_perf(std::chrono::steady_clock::time_point finished_at, bool partial,
    const vision_msgs::msg::Detection2DArray * current = nullptr, const std::string & error = {});
  void disable_inference();
  void release_ros_resources();

  struct PerfWindow
  {
    std::chrono::steady_clock::time_point started{};
    uint64_t frames{0};
    uint64_t failed{0};
    int64_t pre_ns{0};
    int64_t infer_ns{0};
    int64_t post_ns{0};
    int64_t track_ns{0}, callback_ns{0}, total_max_ns{0}, callback_max_ns{0};
    uint64_t age_samples{0};
    int64_t input_age_ns{0}, output_age_ns{0}, input_age_max_ns{0}, output_age_max_ns{0};
  };

  PerfWindow perf_window_;
  bool enable_perf_stats_{false};
  std::string image_topic_;
  std::unique_ptr<YoloEngine> engine_;
  ByteTracker tracker_;
  bool use_tracker_{true};
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr
    detections_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr
    perception_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;
  bool inference_enabled_{false};
  std::shared_ptr<OcrNode> ocr_;
  rclcpp::Client<Control>::SharedPtr ocr_control_;
  uint64_t vision_generation_{0};
  bool stop_requested_{true}, preparation_cancelled_{false};
  rclcpp::Client<Control>::SharedPtr inference_control_;
  rclcpp::Service<Control>::SharedPtr prepare_service_;
  rclcpp::TimerBase::SharedPtr prepare_timer_, control_deadline_, ocr_deadline_;
  std::string instance_, prepare_work_, inference_work_;
  uint64_t sequence_{0};
  bool preparing_{false}, ready_{false}, switching_{false};
};

}  // namespace k1muse_perception
