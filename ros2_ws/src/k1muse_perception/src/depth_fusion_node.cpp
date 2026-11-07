#include "k1muse_perception/depth_fusion_node.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <nlohmann/json.hpp>

namespace k1muse_perception
{
namespace
{

int64_t source_stamp(const std_msgs::msg::Header & header)
{
  return static_cast<int64_t>(header.stamp.sec) * 1000000000LL + header.stamp.nanosec;
}

bool same_calibration(
  const sensor_msgs::msg::CameraInfo & a, const sensor_msgs::msg::CameraInfo & b)
{
  return a.width == b.width && a.height == b.height && a.header.frame_id == b.header.frame_id &&
         a.k == b.k && a.d == b.d && a.distortion_model == b.distortion_model;
}

}  // namespace

DepthFusionNode::DepthFusionNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("depth_fusion", options)
{
  declare_parameter<std::string>("detections_topic", "/k1muse/perception/detections");
  declare_parameter<std::string>(
    "depth_topic",
    "/k1muse/front_camera/aligned_depth_to_color/image_raw");
  declare_parameter<std::string>(
    "camera_info_topic",
    "/k1muse/front_camera/aligned_depth_to_color/camera_info");
  declare_parameter<std::string>("output_topic", "/k1muse/perception/detections_3d");
  declare_parameter<std::string>("qr_requests_topic", "/k1muse/perception/qr_depth/requests");
  declare_parameter<std::string>("qr_output_topic", "/k1muse/perception/qr_depth/points");
  declare_parameter<std::string>("algorithm", "roi_median");
  declare_parameter<int>("center_window_size", 5);
  declare_parameter<int>("min_valid_count", 3);
  declare_parameter<double>("min_depth_m", 0.1);
  declare_parameter<double>("max_depth_m", 2.0);
  input_mode_ = declare_parameter<std::string>("input_mode", "both");
  if (input_mode_ != "off" && input_mode_ != "marker" && input_mode_ != "both") {
    throw std::invalid_argument("invalid depth input_mode");
  }
  using Control = k1muse_interfaces::srv::WebRequest;
  camera_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/k1muse/front_camera");
  control_ = create_service<Control>("/k1muse/perception/depth/control",
    [this](std::shared_ptr<rmw_request_id_t> header, Control::Request::SharedPtr request) {
      auto reply = [this, header, request](const std::string & error) {
          Control::Response response; response.operation_id = request->request_id;
          response.accepted = error.empty(); response.error = error;
          response.result = nlohmann::json{{"mode", input_mode_}}.dump();
          control_->send_response(*header, response);
        };
      try {
        const auto mode = nlohmann::json::parse(request->arguments).at("mode").get<std::string>();
        if (request->operation != "set_mode" ||
          (mode != "off" && mode != "marker" && mode != "both")) {
          throw std::runtime_error("invalid_depth_mode");
        }
        if (mode_pending_) {throw std::runtime_error("depth_mode_unresolved");}
        if (mode == input_mode_) {reply(""); return;}
        if (!camera_parameters_->service_is_ready()) {throw std::runtime_error("camera_parameters_unavailable");}
        // Stop consuming frames before changing alignment; ACK only after the driver replies.
        stop_inputs(); mode_pending_ = true; input_mode_ = "unresolved";
        camera_parameters_->set_parameters({rclcpp::Parameter("align_depth.enable", mode != "off")},
          [this, mode, reply](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
            try {
              const auto results = future.get(); mode_pending_ = false;
              if (results.size() != 1 || !results[0].successful) {
                reply("camera_alignment_rejected"); return;
              }
              input_mode_ = mode;
              if (inputs_active_) {start_inputs();}
              reply("");
            } catch (const std::exception & error) {mode_pending_ = false; reply(error.what());}
          });
      } catch (const std::exception & error) {reply(error.what());}
    });
  declare_parameter<int>("sync_queue_size", 32);
  declare_parameter<double>("sync_tolerance_sec", 0.03);
}

DepthFusionNode::CallbackReturn DepthFusionNode::on_configure(const rclcpp_lifecycle::State &)
{
  const auto window = get_parameter("center_window_size").as_int();
  const auto count = get_parameter("min_valid_count").as_int();
  const auto queue = get_parameter("sync_queue_size").as_int();
  tolerance_ = get_parameter("sync_tolerance_sec").as_double();
  const auto name = get_parameter("algorithm").as_string();
  if (window <= 0 || window > std::numeric_limits<int>::max() || count <= 0 ||
    queue <= 0 || queue > std::numeric_limits<int>::max() ||
    !std::isfinite(tolerance_) || tolerance_ <= 0 ||
    tolerance_ > std::numeric_limits<int32_t>::max() ||
    (name != "center_mean" && name != "roi_median" && name != "yolo_weighted"))
  {
    RCLCPP_ERROR(
      get_logger(), "Invalid C10 algorithm, window, sample count, or synchronization parameters");
    return CallbackReturn::FAILURE;
  }
  config_ = {static_cast<int>(window), static_cast<size_t>(count),
    get_parameter("min_depth_m").as_double(), get_parameter("max_depth_m").as_double()};
  std::string error;
  if (!valid_depth_config(config_, error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  detections_topic_ = get_parameter("detections_topic").as_string();
  depth_topic_ = get_parameter("depth_topic").as_string();
  info_topic_ = get_parameter("camera_info_topic").as_string();
  const auto output = get_parameter("output_topic").as_string();
  if (detections_topic_.empty() || depth_topic_.empty() || info_topic_.empty() || output.empty()) {
    RCLCPP_ERROR(get_logger(), "C10 topic names must not be empty");
    return CallbackReturn::FAILURE;
  }
  qr_topic_ = get_parameter("qr_requests_topic").as_string();
  qr_output_topic_ = get_parameter("qr_output_topic").as_string();
  const auto topics = get_node_topics_interface();
  const std::string resolved[] = {topics->resolve_topic_name(detections_topic_),
    topics->resolve_topic_name(depth_topic_), topics->resolve_topic_name(info_topic_),
    topics->resolve_topic_name(output), topics->resolve_topic_name(qr_topic_),
    topics->resolve_topic_name(qr_output_topic_)};
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = i + 1; j < 6; ++j) {
      if (resolved[i] == resolved[j]) {
        RCLCPP_ERROR(get_logger(), "C10 input/output topics must resolve to distinct names");
        return CallbackReturn::FAILURE;
      }
    }
  }
  if (qr_topic_.empty() || qr_output_topic_.empty()) {return CallbackReturn::FAILURE;}
  queue_size_ = static_cast<int>(queue);
  algorithm_ = name == "roi_median" ? depth_roi_median :
    name == "center_mean" ? depth_center_mean : depth_yolo_weighted;
  publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    output, rclcpp::QoS(1).reliable().durability_volatile());
  qr_publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    qr_output_topic_, rclcpp::QoS(1).reliable().durability_volatile());
  RCLCPP_INFO(
    get_logger(), "Configured C10 algorithm=%s, queue=%d, source tolerance=%.3f s",
    name.c_str(), queue_size_, tolerance_);
  return CallbackReturn::SUCCESS;
}

void DepthFusionNode::reset_matching()
{
  Policy policy(queue_size_);
  policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(tolerance_));
  synchronizer_ = std::make_unique<Synchronizer>(policy);
  synchronizer_->registerCallback(
    std::bind(
      &DepthFusionNode::matched_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  recent_depths_.clear(); qr_request_.reset(); qr_stamp_ = 0;
  have_detection_ = false;
  have_depth_ = false;
}

DepthFusionNode::CallbackReturn DepthFusionNode::on_activate(const rclcpp_lifecycle::State &)
{
  publisher_->on_activate(); qr_publisher_->on_activate();
  inputs_active_ = true; start_inputs();
  return CallbackReturn::SUCCESS;
}

void DepthFusionNode::start_inputs()
{
  if (input_mode_ != "marker" && input_mode_ != "both") {return;}
  reset_matching();
  qr_subscription_ = create_subscription<Detections>(qr_topic_, rclcpp::QoS(1),
    std::bind(&DepthFusionNode::qr_callback, this, std::placeholders::_1));
  if (input_mode_ == "both") {
    detections_subscription_ = create_subscription<Detections>(
      detections_topic_, rclcpp::QoS(1),
      std::bind(&DepthFusionNode::detections_callback, this, std::placeholders::_1));
  }
  depth_subscription_ = create_subscription<Image>(
    depth_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&DepthFusionNode::depth_callback, this, std::placeholders::_1));
  info_subscription_ = create_subscription<CameraInfo>(
    info_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&DepthFusionNode::info_callback, this, std::placeholders::_1));
}

void DepthFusionNode::stop_inputs()
{
  qr_subscription_.reset(); qr_request_.reset(); recent_depths_.clear();
  detections_subscription_.reset();
  depth_subscription_.reset();
  info_subscription_.reset();
  synchronizer_.reset();
  camera_info_.reset();
  scratch_ = {};
  have_detection_ = false;
  have_depth_ = false;
  detection_count_ = depth_count_ = pair_count_ = 0;
}

DepthFusionNode::CallbackReturn DepthFusionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  inputs_active_ = false;
  if (publisher_) {publisher_->on_deactivate();}
  if (qr_publisher_) {qr_publisher_->on_deactivate();}
  stop_inputs();
  return CallbackReturn::SUCCESS;
}

DepthFusionNode::CallbackReturn DepthFusionNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  inputs_active_ = false;
  stop_inputs();
  publisher_.reset(); qr_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

DepthFusionNode::CallbackReturn DepthFusionNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

DepthFusionNode::CallbackReturn DepthFusionNode::on_error(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

void DepthFusionNode::log_pairing()
{
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    *get_clock(), 5000,
    "C10 pairing: detections=%llu depth=%llu pairs=%llu source_ns=(%lld,%lld) queue=%d tolerance=%.3f",
    static_cast<unsigned long long>(detection_count_), static_cast<unsigned long long>(depth_count_),
    static_cast<unsigned long long>(pair_count_), static_cast<long long>(detection_stamp_),
    static_cast<long long>(depth_stamp_), queue_size_, tolerance_);
}

void DepthFusionNode::detections_callback(Detections::ConstSharedPtr message)
{
  if (!synchronizer_) {return;}
  const int64_t stamp = source_stamp(message->header);
  if (have_detection_ &&
    (stamp < detection_stamp_ || message->header.frame_id != detection_frame_))
  {
    reset_matching();
  }
  have_detection_ = true;
  detection_stamp_ = stamp;
  detection_frame_ = message->header.frame_id;
  ++detection_count_;
  synchronizer_->add<0>(message);
  log_pairing();
}

void DepthFusionNode::depth_callback(Image::ConstSharedPtr message)
{
  if (!synchronizer_) {return;}
  const int64_t stamp = source_stamp(message->header);
  if (have_depth_ && (stamp < depth_stamp_ || message->header.frame_id != depth_frame_ ||
    message->width != depth_width_ || message->height != depth_height_ ||
    message->encoding != depth_encoding_))
  {
    reset_matching();
  }
  have_depth_ = true;
  depth_stamp_ = stamp;
  depth_frame_ = message->header.frame_id;
  depth_width_ = message->width;
  depth_height_ = message->height;
  depth_encoding_ = message->encoding;
  ++depth_count_;
  recent_depths_.push_back(message);
  while (recent_depths_.size() > static_cast<size_t>(queue_size_)) {recent_depths_.pop_front();}
  try_qr();
  if (input_mode_ == "both") {synchronizer_->add<1>(message);}
  log_pairing();
}

void DepthFusionNode::info_callback(CameraInfo::ConstSharedPtr message)
{
  std::string error;
  if (!valid_depth_calibration(*message, error)) {
    camera_info_.reset();
    reset_matching();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "C10 %s", error.c_str());
    return;
  }
  if (camera_info_ && !same_calibration(*camera_info_, *message)) {reset_matching();}
  camera_info_ = std::move(message);
}

void DepthFusionNode::matched_callback(
  Detections::ConstSharedPtr detections,
  Image::ConstSharedPtr depth)
{
  fuse(detections, depth, publisher_);
}

void DepthFusionNode::qr_callback(Detections::ConstSharedPtr message)
{
  if (message->detections.empty()) {qr_request_.reset(); return;}
  const auto stamp = source_stamp(message->header);
  if (stamp <= qr_stamp_) {return;}
  qr_stamp_ = stamp; qr_request_ = message; try_qr();
}

void DepthFusionNode::try_qr()
{
  if (!qr_request_ || recent_depths_.empty() || !camera_info_) {return;}
  Image::ConstSharedPtr best;
  int64_t delta = std::numeric_limits<int64_t>::max();
  for (const auto & depth : recent_depths_) {
    const auto difference = std::abs(source_stamp(depth->header) - qr_stamp_);
    if (difference < delta) {delta = difference; best = depth;}
  }
  if (delta > static_cast<int64_t>(tolerance_ * 1e9)) {return;}
  auto request = std::move(qr_request_);
  fuse(request, best, qr_publisher_, true);
}

void DepthFusionNode::fuse(
  Detections::ConstSharedPtr detections, Image::ConstSharedPtr depth,
  const rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr & publisher,
  bool qr)
{
  if (!publisher || !publisher->is_activated()) {return;}
  if (!qr) {++pair_count_;} // Keep the existing pairing counter scoped to YOLO.
  if (!camera_info_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "C10 camera_info_unavailable");
    return;
  }
  if (depth->width != camera_info_->width || depth->height != camera_info_->height ||
    depth->header.frame_id != camera_info_->header.frame_id ||
    detections->header.frame_id != camera_info_->header.frame_id)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "C10 incompatible_geometry_or_frame");
    return;
  }
  DepthImageView image;
  std::string error;
  if (!make_depth_view(*depth, image, error)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "C10 %s encoding=%s",
      error.c_str(), depth->encoding.c_str());
    return;
  }
  vision_msgs::msg::Detection3DArray output;
  output.header = detections->header;
  output.detections.reserve(detections->detections.size());
  for (const auto & detection : detections->detections) {
    DepthRoi roi;
    if (detection.header != detections->header || detection.results.empty() ||
      !make_depth_roi(detection.bbox, depth->width, depth->height, roi))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "C10 invalid_detection_or_empty_roi");
      continue;
    }
    const auto estimate = (qr ? depth_center_mean : algorithm_)(image, roi, config_, scratch_);
    if (!estimate.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "C10 %s encoding=%s roi=%ux%u sampled=%zu valid=%zu",
        estimate.reason, depth->encoding.c_str(), roi.x2 - roi.x1, roi.y2 - roi.y1,
        estimate.sampled_count, estimate.valid_count);
      continue;
    }
    geometry_msgs::msg::Point point;
    if (!project_depth(*camera_info_, estimate.u, estimate.v, estimate.depth_m, point)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "C10 invalid_projection");
      continue;
    }
    vision_msgs::msg::Detection3D result;
    result.header = output.header;
    result.id = detection.id;
    result.results = detection.results;
    for (auto & hypothesis : result.results) {
      hypothesis.pose = geometry_msgs::msg::PoseWithCovariance();
      hypothesis.pose.pose.position = point;
      hypothesis.pose.pose.orientation.w = 1.0;
    }
    result.bbox.center.position = point;
    result.bbox.center.orientation.w = 1.0;
    output.detections.push_back(std::move(result));
  }
  publisher->publish(std::move(output));
}

}  // namespace k1muse_perception
