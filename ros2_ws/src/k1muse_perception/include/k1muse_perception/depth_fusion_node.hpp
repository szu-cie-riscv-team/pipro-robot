#pragma once

#include <memory>
#include <deque>
#include <string>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <k1muse_interfaces/srv/web_request.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "k1muse_perception/depth_estimation.hpp"

namespace k1muse_perception
{

class DepthFusionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit DepthFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  using Detections = vision_msgs::msg::Detection2DArray;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Policy = message_filters::sync_policies::ApproximateTime<Detections, Image>;
  using Synchronizer = message_filters::Synchronizer<Policy>;

  void reset_matching();
  void stop_inputs();
  void start_inputs();
  std::string input_mode_{"both"};
  bool inputs_active_{false}, mode_pending_{false};
  std::shared_ptr<rclcpp::AsyncParametersClient> camera_parameters_;
  rclcpp::Service<k1muse_interfaces::srv::WebRequest>::SharedPtr control_;
  void detections_callback(Detections::ConstSharedPtr message);
  void depth_callback(Image::ConstSharedPtr message);
  void info_callback(CameraInfo::ConstSharedPtr message);
  void matched_callback(Detections::ConstSharedPtr detections, Image::ConstSharedPtr depth);
  void log_pairing();
  void qr_callback(Detections::ConstSharedPtr message);
  void try_qr();
  void fuse(Detections::ConstSharedPtr detections, Image::ConstSharedPtr depth,
    const rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr & output,
    bool qr = false);
  std::deque<Image::ConstSharedPtr> recent_depths_;
  Detections::ConstSharedPtr qr_request_;
  int64_t qr_stamp_{0};
  std::string qr_topic_, qr_output_topic_;
  rclcpp::Subscription<Detections>::SharedPtr qr_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr qr_publisher_;

  DepthConfig config_;
  DepthAlgorithm algorithm_{nullptr};
  DepthScratch scratch_;
  int queue_size_{32};
  double tolerance_{0.03};
  std::string detections_topic_, depth_topic_, info_topic_;
  std::unique_ptr<Synchronizer> synchronizer_;
  CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Subscription<Detections>::SharedPtr detections_subscription_;
  rclcpp::Subscription<Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
  bool have_detection_{false}, have_depth_{false};
  int64_t detection_stamp_{0}, depth_stamp_{0};
  std::string detection_frame_, depth_frame_, depth_encoding_;
  uint32_t depth_width_{0}, depth_height_{0};
  uint64_t detection_count_{0}, depth_count_{0}, pair_count_{0};
};

}  // namespace k1muse_perception
