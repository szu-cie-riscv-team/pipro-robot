#include "k1muse_perception/target_transform_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace k1muse_perception
{
namespace
{

bool valid_frame(const std::string & frame)
{
  return !frame.empty() && frame.front() != '/' &&
         std::none_of(frame.begin(), frame.end(), [](unsigned char c) {return std::isspace(c);});
}

bool valid_header(const std_msgs::msg::Header & header)
{
  return valid_frame(header.frame_id) && header.stamp.sec >= 0 &&
         header.stamp.nanosec < 1000000000 &&
         (header.stamp.sec != 0 || header.stamp.nanosec != 0);
}

bool finite_point(const geometry_msgs::msg::Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

geometry_msgs::msg::Point apply_point(
  const tf2::Transform & transform, const geometry_msgs::msg::Point & point)
{
  const auto result = transform * tf2::Vector3(point.x, point.y, point.z);
  geometry_msgs::msg::Point output;
  output.x = result.x();
  output.y = result.y();
  output.z = result.z();
  return output;
}

geometry_msgs::msg::TransformStamped mount_transform(
  const std::string & parent, const std::string & child,
  const std::vector<double> & xyz, const std::vector<double> & rpy,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.header.stamp = stamp;
  transform.child_frame_id = child;
  transform.transform.translation.x = xyz[0];
  transform.transform.translation.y = xyz[1];
  transform.transform.translation.z = xyz[2];
  tf2::Quaternion rotation;
  rotation.setRPY(rpy[0], rpy[1], rpy[2]);
  transform.transform.rotation = tf2::toMsg(rotation);
  return transform;
}

}  // namespace

bool transform_targets(
  const vision_msgs::msg::Detection3DArray & input,
  const geometry_msgs::msg::TransformStamped & transform,
  vision_msgs::msg::Detection3DArray & output)
{
  output = vision_msgs::msg::Detection3DArray();
  const auto & rotation = transform.transform.rotation;
  const auto & translation = transform.transform.translation;
  const double norm = rotation.x * rotation.x + rotation.y * rotation.y +
    rotation.z * rotation.z + rotation.w * rotation.w;
  if (!valid_header(input.header) || transform.child_frame_id != input.header.frame_id ||
    !valid_frame(transform.header.frame_id) || !std::isfinite(norm) ||
    std::abs(norm - 1.0) > 1e-3 || !std::isfinite(translation.x) ||
    !std::isfinite(translation.y) || !std::isfinite(translation.z))
  {
    return false;
  }
  tf2::Transform rigid;
  tf2::fromMsg(transform.transform, rigid);
  output.header = input.header;
  output.header.frame_id = transform.header.frame_id;
  output.detections.reserve(input.detections.size());
  for (const auto & detection : input.detections) {
    if (detection.header != input.header || detection.results.empty() ||
      !finite_point(detection.bbox.center.position) ||
      !std::all_of(
        detection.results.begin(), detection.results.end(),
        [](const auto & h) {return finite_point(h.pose.pose.position);}))
    {
      continue;
    }
    vision_msgs::msg::Detection3D result;
    result.header = output.header;
    result.id = detection.id;
    result.results = detection.results;
    result.bbox.center.position = apply_point(rigid, detection.bbox.center.position);
    result.bbox.center.orientation.w = 1.0;
    for (auto & hypothesis : result.results) {
      const auto position = apply_point(rigid, hypothesis.pose.pose.position);
      hypothesis.pose = geometry_msgs::msg::PoseWithCovariance();
      hypothesis.pose.pose.position = position;
      hypothesis.pose.pose.orientation.w = 1.0;
    }
    if (finite_point(result.bbox.center.position) &&
      std::all_of(
        result.results.begin(), result.results.end(),
        [](const auto & h) {return finite_point(h.pose.pose.position);}))
    {
      output.detections.push_back(std::move(result));
    }
  }
  return true;
}

TargetTransformNode::TargetTransformNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("target_transform", options)
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  const auto input = declare_parameter<std::string>(
    "input_topic", "/k1muse/perception/detections_3d", startup);
  const auto output = declare_parameter<std::string>(
    "output_topic", "/k1muse/perception/arm_targets", startup);
  target_frame_ = declare_parameter<std::string>("target_frame", "arm_base", startup);
  const auto stage = declare_parameter<std::string>("stage_frame", "lift_stage_link", startup);
  const bool publish = declare_parameter<bool>("publish_mount_tf", true, startup);
  const auto camera = declare_parameter<std::string>(
    "camera_link_frame", "front_camera_link", startup);
  const auto vector = [this, &startup](const std::string & name, std::vector<double> value) {
      auto result = declare_parameter<std::vector<double>>(name, value, startup);
      if (result.size() != 3 ||
        !std::all_of(result.begin(), result.end(), [](double x) {return std::isfinite(x);}))
      {
        throw std::invalid_argument(name + " must contain three finite values");
      }
      return result;
    };
  const auto camera_xyz = vector("camera_mount_xyz", {0.140406, 0.0, 0.0});
  const auto camera_rpy = vector("camera_mount_rpy", {0.0, 0.0, 0.0});
  const auto arm_xyz = vector("arm_mount_xyz", {0.030406, 0.0, 0.0});
  const auto arm_rpy = vector("arm_mount_rpy", {0.0, 0.0, 0.0});
  const std::string frames[] = {stage, camera, target_frame_};
  for (size_t i = 0; i < 3; ++i) {
    if (!valid_frame(frames[i])) {throw std::invalid_argument("Invalid C11 mounting frame");}
    for (size_t j = i + 1; j < 3; ++j) {
      if (frames[i] == frames[j]) {
        throw std::invalid_argument("C11 mounting frames must be distinct");
      }
    }
  }
  const auto topics = get_node_topics_interface();
  if (input.empty() || output.empty() ||
    topics->resolve_topic_name(input) == topics->resolve_topic_name(output))
  {
    throw std::invalid_argument("C11 input/output topics must be nonempty and distinct");
  }
  buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, false);
  if (publish) {
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    const builtin_interfaces::msg::Time stamp = now();
    broadcaster_->sendTransform(
      std::vector<geometry_msgs::msg::TransformStamped>{
        mount_transform(stage, camera, camera_xyz, camera_rpy, stamp),
        mount_transform(stage, target_frame_, arm_xyz, arm_rpy, stamp)});
    RCLCPP_WARN(
      get_logger(), "C11 camera mounting TF uses provisional geometry; "
      "arm_base is 0.030406 m ahead of the stage by default, with zero Z offset");
  }
  publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(output, rclcpp::QoS(1));
  subscription_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    input, rclcpp::QoS(1),
    std::bind(&TargetTransformNode::targets_callback, this, std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(), "C11 ready: source-time point transformation to %s",
    target_frame_.c_str());
}

void TargetTransformNode::targets_callback(
  vision_msgs::msg::Detection3DArray::ConstSharedPtr message)
{
  if (!valid_header(message->header)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "C11 invalid source frame or timestamp");
    return;
  }
  try {
    // BufferCore performs a source-time lookup without a blocking/thread check.
    const auto transform = static_cast<const tf2::BufferCore &>(*buffer_).lookupTransform(
      target_frame_, message->header.frame_id,
      tf2_ros::fromRclcpp(rclcpp::Time(message->header.stamp, get_clock()->get_clock_type())));
    vision_msgs::msg::Detection3DArray output;
    if (!transform_targets(*message, transform, output)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 5000, "C11 invalid transform or source header");
      return;
    }
    if (output.detections.size() != message->detections.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 5000, "C11 skipped invalid target positions/headers");
    }
    publisher_->publish(std::move(output));
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "C11 TF unavailable: %s <- %s at %d.%09u: %s",
      target_frame_.c_str(), message->header.frame_id.c_str(), message->header.stamp.sec,
      message->header.stamp.nanosec, error.what());
  }
}

}  // namespace k1muse_perception
