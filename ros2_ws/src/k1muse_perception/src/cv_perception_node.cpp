#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <iostream>
#include <sched.h>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "k1muse_perception/cv_perception.hpp"

namespace k1muse_perception
{
using Json = nlohmann::json;
class CvPerceptionNode : public rclcpp::Node
{
public:
  CvPerceptionNode() : Node("cv_perception")
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    const int core = declare_parameter<int>("cpu_core", 4, startup);
    if (core != 4 && core != 5) {throw std::invalid_argument("CV cpu_core must be 4 or 5 (zero-based)");}
    cpu_set_t cpus; CPU_ZERO(&cpus); CPU_SET(core, &cpus);
    // This thread runs the single executor and every decode; no algorithm worker.
    if (sched_setaffinity(0, sizeof(cpus), &cpus) != 0) {
      throw std::runtime_error(std::string("Cannot bind CV executor: ") + std::strerror(errno));
    }
    enable_perf_stats_ = declare_parameter<bool>("enable_perf_stats", false, startup);
    image_topic_ = declare_parameter<std::string>("image_topic", "/k1muse/front_camera/color/image_raw", startup);
    const auto output = declare_parameter<std::string>("observations_topic", "/k1muse/perception/cv/observations", startup);
    const auto max_age = declare_parameter<int64_t>("max_frame_age_ms", 750, startup);
    const double hz = declare_parameter<double>("max_scan_hz", 5.0, startup);
    if (image_topic_.empty() || output.empty() || max_age <= 0 || max_age > 60000 || !std::isfinite(hz) || hz <= 0 || hz > 60) {
      throw std::invalid_argument("Invalid CV topics, frame age or scan rate");
    }
    max_age_ns_ = max_age * 1000000;
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / hz));
    publisher_ = create_publisher<std_msgs::msg::String>(output, rclcpp::QoS(1).reliable());
    control_ = create_service<std_srvs::srv::SetBool>("/k1muse/perception/cv/set_enabled",
      [this](std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
        response->success = true;
        if (request->data == enabled_) {response->message = enabled_ ? "already_enabled" : "already_disabled"; return;}
        if (!request->data) {
          // Single executor: an in-flight decode has finished before this ACK.
          enabled_ = false; image_sub_.reset(); response->message = "disabled"; return;
        }
        enabled_at_ns_ = now().nanoseconds(); last_stamp_ns_ = -1;
        next_scan_ = std::chrono::steady_clock::time_point{};
        perf_started_ = std::chrono::steady_clock::now(); perf_frames_ = 0;
        try {
          auto qos = rclcpp::SensorDataQoS(); qos.keep_last(1);
          image_sub_ = create_subscription<sensor_msgs::msg::Image>(image_topic_, qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr image) {receive_image(*image);});
          enabled_ = true; response->message = "enabled";
        } catch (const std::exception &) {
          image_sub_.reset(); response->success = false; response->message = "subscription_failed";
        }
      });
    RCLCPP_INFO(get_logger(), "ArUco DICT_4X4_50 ready, disabled; CPU%d single-thread, scan <=%.1f Hz; RGB only", core, hz);
  }
private:
  bool fresh(int64_t stamp) const
  {
    const auto current = now().nanoseconds();
    return stamp >= enabled_at_ns_ && stamp <= current && current - stamp <= max_age_ns_;
  }
  void receive_image(const sensor_msgs::msg::Image & image)
  {
    const auto tick = std::chrono::steady_clock::now();
    if (!enabled_ || tick < next_scan_ || image.header.stamp.sec < 0 || image.header.stamp.nanosec >= 1000000000U) {return;}
    const int64_t stamp = int64_t(image.header.stamp.sec) * 1000000000 + image.header.stamp.nanosec;
    if (stamp <= last_stamp_ns_ || !fresh(stamp)) {return;}
    // Skip before conversion/decode; no timer, queued frames or repeated old results.
    last_stamp_ns_ = stamp; next_scan_ = tick + period_;
    CvFrameObservation observation;
    if (image.encoding != "rgb8") {observation.error = "image_encoding_unsupported";}
    else if (!image.width || !image.height || image.header.frame_id.empty() ||
      image.width > uint32_t(std::numeric_limits<int>::max()) || image.height > uint32_t(std::numeric_limits<int>::max()) ||
      image.step < uint64_t(image.width) * 3 || image.data.size() < uint64_t(image.step) * image.height)
    {observation.error = "image_layout_invalid";}
    else {
      const cv::Mat rgb(int(image.height), int(image.width), CV_8UC3, const_cast<uint8_t *>(image.data.data()), image.step);
      observation = algorithm_.process_rgb8(rgb);
    }
    if (!rclcpp::ok() || !fresh(stamp)) {return;}
    Json markers = Json::array();
    for (const auto & marker : observation.markers) {
      Json corners = Json::array();
      for (const auto & p : marker.corners) {corners.push_back({p.x, p.y});}
      markers.push_back({{"id", marker.id}, {"corners", std::move(corners)}});
    }
    std_msgs::msg::String result;
    result.data = Json{{"header", {{"stamp", {{"sec", image.header.stamp.sec}, {"nanosec", image.header.stamp.nanosec}}},
      {"frame_id", image.header.frame_id}}}, {"image_width", image.width}, {"image_height", image.height},
      {"error", observation.error}, {"markers", std::move(markers)}}.dump();
    publisher_->publish(result);
    if (enable_perf_stats_) {
      ++perf_frames_;
      const auto finished = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(finished - perf_started_).count();
      if (elapsed >= 5) {
        RCLCPP_INFO(get_logger(), "[CV perf] frames=%llu fps=%.2f age_ms=%.1f decode_ms=%.1f result=%s",
          static_cast<unsigned long long>(perf_frames_), perf_frames_ / elapsed,
          (now().nanoseconds()-stamp)*1e-6, std::chrono::duration<double,std::milli>(finished-tick).count(), result.data.c_str());
        perf_frames_ = 0; perf_started_ = finished;
      }
    }
  }
  CvPerception algorithm_;
  bool enabled_{false}, enable_perf_stats_{false};
  int64_t max_age_ns_{0}, enabled_at_ns_{0}, last_stamp_ns_{-1};
  std::string image_topic_;
  uint64_t perf_frames_{0};
  std::chrono::steady_clock::duration period_{};
  std::chrono::steady_clock::time_point next_scan_{}, perf_started_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_;
};
}  // namespace k1muse_perception

int main(int argc, char * argv[])
{
  try {
    cv::setNumThreads(1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<k1muse_perception::CvPerceptionNode>());
  } catch (const std::exception & error) {
    std::cerr << "CV failed: " << error.what() << '\n';
    if (rclcpp::ok()) {rclcpp::shutdown();} return 1;
  }
  rclcpp::shutdown(); return 0;
}
