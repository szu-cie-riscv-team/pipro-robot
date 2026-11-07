#include <algorithm>
#include "k1muse_perception/yolo_detector_node.hpp"
#include "k1muse_perception/ocr_node.hpp"

#include <cmath>
#include <cinttypes>
#include <exception>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <sys/syscall.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

#include <cv_bridge/cv_bridge.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rcutils/logging.h>

namespace k1muse_perception
{
namespace
{
int64_t source_age_ns(const std_msgs::msg::Header & h,int64_t received)
{
  const int64_t source=int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;
  return h.stamp.sec>=0 && h.stamp.nanosec<1000000000u && source>0?received-source:-1;
}
}

YoloDetectorNode::YoloDetectorNode(const rclcpp::NodeOptions & options, std::shared_ptr<OcrNode> ocr)
: rclcpp_lifecycle::LifecycleNode("yolo_detector", options), ocr_(std::move(ocr))
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  this->declare_parameter<bool>("enable_perf_stats", false, startup);
  this->declare_parameter<std::string>("image_topic", "/k1muse/front_camera/color/image_raw");
  this->declare_parameter<std::string>("detections_topic", "/k1muse/perception/yolo/detections");
  this->declare_parameter<std::string>(
    "perception_detections_topic", "/k1muse/perception/detections");
  this->declare_parameter<bool>("use_tracker", true);
  this->declare_parameter<double>("tracker.track_threshold", 0.4);
  this->declare_parameter<double>("tracker.new_track_threshold", 0.4);
  this->declare_parameter<double>("tracker.match_threshold", 0.8);
  this->declare_parameter<int>("tracker.lost_buffer_frames", 30);
  this->declare_parameter<std::string>("model_path", "");
  this->declare_parameter<std::string>("labels_path", "");
  this->declare_parameter<std::string>("output_format", "NEED_MODEL_SCHEMA_CONFIRMATION");
  this->declare_parameter<int>("input_width", 320);
  this->declare_parameter<int>("input_height", 320);
  this->declare_parameter<std::string>("input_tensor_type", "float32");
  this->declare_parameter<std::string>("input_color_order", "rgb");
  this->declare_parameter<int>("letterbox_padding_value", 114);
  this->declare_parameter<double>("normalize_scale", 1.0 / 255.0);
  this->declare_parameter<double>("confidence_threshold", 0.25);
  this->declare_parameter<double>("nms_iou_threshold", 0.45);
  this->declare_parameter<int>("max_detections", 100);
  this->declare_parameter<int>("ort_intra_threads", 1);
  this->declare_parameter<int>("ort_inter_threads", 1);
  this->declare_parameter<int>("ep_intra_threads", 3);
  this->declare_parameter<int>("ep_inter_threads", 1);
  this->declare_parameter<std::string>("ep_intra_thread_affinity", "1;2;3");
  instance_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  if (ocr_) {ocr_control_ = create_client<Control>("/k1muse/perception/ocr/control");}
  inference_control_ = create_client<Control>("/k1muse/task/inference/control");
  prepare_service_ = create_service<Control>("/k1muse/perception/yolo/control",
    [this](Control::Request::SharedPtr request, Control::Response::SharedPtr response) {
      prepare(*request, *response);
    });
  set_enabled_service_ = create_service<std_srvs::srv::SetBool>(
    "/k1muse/perception/yolo/set_enabled",
    [this](Header header, std_srvs::srv::SetBool::Request::SharedPtr request) {
      set_enabled_callback(header, request);
    });
}

void YoloDetectorNode::prepare(const Control::Request & request, Control::Response & response)
{
  using Json = nlohmann::json;
  response.operation_id = request.request_id;
  try {
    if (request.client_id == "ocr_detector" && request.operation == "ocr_fault" && ocr_) {
      const auto args = Json::parse(request.arguments);
      if (args.at("work_id").get<std::string>() != inference_work_ ||
        args.at("generation").get<uint64_t>() != vision_generation_ || inference_work_.empty())
      {throw std::runtime_error("stale_ocr_fault");}
      ready_ = false; stop_requested_ = true;
      RCLCPP_ERROR(get_logger(), "Vision stopped after OCR fault: %s",
        args.value("error", std::string("ocr_failed")).c_str());
      if (!switching_) {stop_group();} else {disable_inference();}
      response.accepted = true; return;
    }
    if (request.client_id != "task_manager" || request.operation != "prepare") {
      throw std::runtime_error("unsupported_operation");
    }
    const auto id = Json::parse(request.arguments).at("work_id").get<std::string>();
    if (id.empty()) {throw std::runtime_error("invalid_work_id");}
    if (id == prepare_work_) {
      response.accepted = true; response.result = "{\"status\":\"running\"}"; return;
    }
    if (!prepare_work_.empty()) {throw std::runtime_error("already_prepared");}
    prepare_work_ = id; preparing_ = true; preparation_cancelled_ = false;
    prepare_timer_ = create_wall_timer(std::chrono::milliseconds(1), [this, id] {
      prepare_timer_->cancel(); prepare_timer_.reset();
      if (preparation_cancelled_) {
        finish_prepare(id, false, "vision_prepare_cancelled"); return;
      }
      RCLCPP_INFO(get_logger(), "PREPARE stage=YOLO event=configure_warmup_begin pid=%ld tid=%ld work=%s",
        static_cast<long>(::getpid()), static_cast<long>(::syscall(SYS_gettid)), id.c_str());
      bool success = configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      RCLCPP_INFO(get_logger(), "PREPARE stage=YOLO event=configure_warmup_end success=%d", success);
      if (success) {success = activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;}
      if (!success) {finish_prepare(id, false, "yolo_prepare_failed"); return;}
      call_ocr("prepare", id, [this, id](bool ok, const std::string & error) {
        finish_prepare(id, ok && !preparation_cancelled_, error);
      });
    });
    response.accepted = true; response.result = "{\"status\":\"running\"}";
  } catch (const std::exception & error) {response.error = error.what();}
}

YoloDetectorNode::~YoloDetectorNode()
{
  stop_local();
}

void YoloDetectorNode::finish_prepare(
  const std::string & work, bool success, const std::string & error)
{
  if (prepare_work_ != work || !preparing_) {return;}
  preparing_ = false;
  ready_ = success && !preparation_cancelled_ &&
    get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  auto done = std::make_shared<Control::Request>();
  done->client_id = "yolo_detector"; done->request_id = instance_ + "_prepared";
  done->operation = "release";
  done->arguments = nlohmann::json{{"work_id", work},
    {"outcome", ready_ ? "succeeded" : "failed"},
    {"error", ready_ ? "" : (error.empty() ? "vision_prepare_failed" : error)}}.dump();
  if (inference_control_->service_is_ready()) {inference_control_->async_send_request(done);}
}

void YoloDetectorNode::call_ocr(const std::string & operation, const std::string & work,
  std::function<void(bool, const std::string &)> completion)
{
  if (!ocr_) {completion(true, ""); return;}
  if (!ocr_control_->service_is_ready()) {completion(false, "ocr_control_unavailable"); return;}
  auto request = std::make_shared<Control::Request>();
  request->operation = operation; request->client_id = "yolo_detector";
  request->request_id = instance_ + "_ocr_" + std::to_string(++sequence_);
  request->arguments = nlohmann::json{{"work_id", work}, {"generation", vision_generation_}}.dump();
  try {
    request->token = ocr_->authorize(operation, work, vision_generation_);
    const auto id = request->request_id;
    auto unresolved = std::make_shared<bool>(false);
    ocr_deadline_ = create_wall_timer(std::chrono::milliseconds(1500),
      [this, completion, unresolved, operation, work] {
        ocr_deadline_->cancel(); ocr_deadline_.reset(); *unresolved = true;
        stop_requested_ = true; disable_inference();
        RCLCPP_ERROR(get_logger(), "OCR %s ACK unresolved; work=%s; retaining permit",
          operation.c_str(), work.c_str());
        // Prepare can terminate as failed; enable/disable retain their original permit
        // until the original ACK resolves. Never start a competing transition here.
        if (operation == "prepare") {completion(false, "ocr_prepare_ack_unresolved");}
      });
    ocr_control_->async_send_request(request,
      [this, completion, id, unresolved, operation](rclcpp::Client<Control>::SharedFuture future) {
        if (ocr_deadline_) {ocr_deadline_->cancel(); ocr_deadline_.reset();}
        bool ok = false; std::string error;
        try {
          const auto response = future.get();
          ok = response->accepted && response->operation_id == id;
          error = ok ? "" : response->error;
        } catch (const std::exception & exception) {error = exception.what();}
        if (!*unresolved || operation != "prepare") {completion(ok, error);}
      });
  } catch (const std::exception & exception) {
    if (ocr_deadline_) {ocr_deadline_->cancel(); ocr_deadline_.reset();}
    completion(false, exception.what());
  }
}

void YoloDetectorNode::stop_local()
{
  stop_requested_ = true;
  if (preparing_) {preparation_cancelled_ = true;}
  disable_inference();
  if (ocr_) {ocr_->stop_and_wait();}
}

YoloDetectorNode::CallbackReturn YoloDetectorNode::on_configure(const rclcpp_lifecycle::State &)
{
  if (!preparing_ || prepare_work_.empty()) {
    RCLCPP_ERROR(get_logger(), "YOLO configure requires manager preparation permit");
    return CallbackReturn::FAILURE;
  }
  YoloEngineConfig config;
  std::string error;
  if (!read_engine_config(config, error)) {
    RCLCPP_ERROR(get_logger(), "Invalid C08 configuration: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }

  image_topic_ = this->get_parameter("image_topic").as_string();
  enable_perf_stats_ = this->get_parameter("enable_perf_stats").as_bool();
  const auto detections_topic = this->get_parameter("detections_topic").as_string();
  const auto perception_topic = this->get_parameter("perception_detections_topic").as_string();
  if (image_topic_.empty() || config.postprocess.max_detections <= 0 || detections_topic.empty() ||
    perception_topic.empty() ||
    get_node_topics_interface()->resolve_topic_name(detections_topic) ==
    get_node_topics_interface()->resolve_topic_name(perception_topic))
  {
    RCLCPP_ERROR(
      get_logger(), "C08/C09 require an image topic, positive max_detections and distinct outputs");
    return CallbackReturn::FAILURE;
  }
  const ByteTrackerConfig tracker_config{
    static_cast<float>(this->get_parameter("tracker.track_threshold").as_double()),
    static_cast<float>(this->get_parameter("tracker.new_track_threshold").as_double()),
    static_cast<float>(this->get_parameter("tracker.match_threshold").as_double()),
    static_cast<int>(this->get_parameter("tracker.lost_buffer_frames").as_int()),
    static_cast<size_t>(config.postprocess.max_detections)};
  if (!tracker_.configure(tracker_config, error)) {
    RCLCPP_ERROR(get_logger(), "Invalid C09 configuration: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  use_tracker_ = this->get_parameter("use_tracker").as_bool();

  auto engine = std::make_unique<YoloEngine>();
  if (!engine->configure(config, error)) {
    RCLCPP_ERROR(get_logger(), "Could not configure YOLO engine: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }

  detections_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    detections_topic, rclcpp::QoS(1).reliable().durability_volatile());
  perception_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    perception_topic, rclcpp::QoS(1).reliable().durability_volatile());
  engine_ = std::move(engine);
  inference_enabled_ = false;
  RCLCPP_INFO(get_logger(), "YOLO model configured and warmed; inference is disabled");
  return CallbackReturn::SUCCESS;
}

YoloDetectorNode::CallbackReturn YoloDetectorNode::on_activate(const rclcpp_lifecycle::State &)
{
  detections_publisher_->on_activate();
  perception_publisher_->on_activate();
  return CallbackReturn::SUCCESS;
}

YoloDetectorNode::CallbackReturn YoloDetectorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_local();
  release_inference();
  if (detections_publisher_) {
    detections_publisher_->on_deactivate();
  }
  if (perception_publisher_) {
    perception_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

YoloDetectorNode::CallbackReturn YoloDetectorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_local();
  release_inference(); ready_ = false;
  release_ros_resources();
  engine_.reset();
  return CallbackReturn::SUCCESS;
}

YoloDetectorNode::CallbackReturn YoloDetectorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_local();
  release_inference(); ready_ = false;
  release_ros_resources();
  engine_.reset();
  return CallbackReturn::SUCCESS;
}

YoloDetectorNode::CallbackReturn YoloDetectorNode::on_error(const rclcpp_lifecycle::State &)
{
  stop_local();
  release_inference(); ready_ = false;
  release_ros_resources(); engine_.reset();
  return CallbackReturn::SUCCESS;
}

void YoloDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
{
  if (!inference_enabled_ || !engine_ || !detections_publisher_ ||
    !detections_publisher_->is_activated() || !perception_publisher_ ||
    !perception_publisher_->is_activated())
  {
    return;
  }
  if (message->encoding != sensor_msgs::image_encodings::RGB8) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring image with encoding '%s'; C08 only accepts rgb8", message->encoding.c_str());
    return;
  }

  using Clock = std::chrono::steady_clock;
  const auto callback_started = enable_perf_stats_ ? Clock::now() : Clock::time_point{};
  const auto input_age = enable_perf_stats_ ? source_age_ns(message->header,now().nanoseconds()) : -1;
  try {
    const auto image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::RGB8);
    if (ocr_) {ocr_->try_submit(message);}
    std::vector<vision_msgs::msg::Detection2D> detections;
    std::string error;
    if (enable_perf_stats_) {
      YoloFrameTiming timing;
      timing.input_age_ns=input_age;
      const bool success = engine_->infer_with_timing(image->image, detections, timing, error);
      if (success) {
        publish_detections(message, std::move(detections), &timing, callback_started);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "YOLO inference failed: %s", error.c_str());
        if (record_perf(timing, false)) {
          // Only a due failure report needs a source header; no result cache.
          vision_msgs::msg::Detection2DArray current;
          current.header = message->header;
          report_perf(timing.finished_at, false, &current, error);
        }
      }
    } else {
      if (!engine_->infer(image->image, detections, error)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "YOLO inference failed: %s", error.c_str());
        return;
      }
      publish_detections(message, std::move(detections));
    }
  } catch (const cv_bridge::Exception & exception) {
    RCLCPP_WARN(get_logger(), "Could not share RGB image: %s", exception.what());
  } catch (const std::exception & exception) {
    ready_ = false;
    RCLCPP_ERROR(get_logger(), "Vision algorithm failed: %s", exception.what());
    stop_group({}, std::string("vision_algorithm_failed: ") + exception.what());
  }
}

void YoloDetectorNode::reply_enabled(Header header, bool success, const std::string & message)
{
  if (!header) {return;}
  std_srvs::srv::SetBool::Response response;
  response.success = success; response.message = message;
  set_enabled_service_->send_response(*header, response);
}

void YoloDetectorNode::release_inference(Header header, const std::string & failure)
{
  if (inference_work_.empty()) {switching_ = false; reply_enabled(header, failure.empty(),
      failure.empty() ? (ocr_ ? "Vision inference disabled" : "YOLO inference disabled") : failure); return;}
  switching_ = true;
  if (!inference_control_->service_is_ready()) {
    reply_enabled(header, false, "manager_unavailable"); return;
  }
  auto request = std::make_shared<Control::Request>();
  const auto work = inference_work_;
  request->operation = "release"; request->client_id = "yolo_detector";
  request->request_id = instance_ + "_release_" + std::to_string(++sequence_);
  request->arguments = nlohmann::json{{"work_id", inference_work_},
    {"outcome", failure.empty() ? "succeeded" : "failed"}, {"error", failure}}.dump();
  inference_control_->async_send_request(request,
    [this, header, work, failure](rclcpp::Client<Control>::SharedFuture future) {
      const auto response = future.get();
      if (response->accepted && inference_work_ == work) {
        inference_work_.clear(); switching_ = false;
      }
      const auto message = failure.empty() ?
        (ocr_ ? "Vision inference disabled" : "YOLO inference disabled") : failure;
      reply_enabled(header, response->accepted && failure.empty(),
        response->accepted ? message : response->error);
    });
}

void YoloDetectorNode::stop_group(Header header, const std::string & failure)
{
  stop_requested_ = true;
  disable_inference();
  if (inference_work_.empty()) {release_inference(header, failure); return;}
  switching_ = true;
  const auto work = inference_work_;
  call_ocr("disable", work, [this, header, work, failure](bool ok, const std::string & error) {
    if (work != inference_work_) {reply_enabled(header, false, "vision_changed"); return;}
    if (!ok) {
      // An uncertain stop must retain the permit. No new model may start.
      RCLCPP_ERROR(get_logger(), "OCR stop not acknowledged: %s", error.c_str());
      reply_enabled(header, false, error); return;
    }
    release_inference(header, failure);
  });
}

void YoloDetectorNode::open_input(Header header)
{
  auto qos = rclcpp::SensorDataQoS(); qos.keep_last(1);
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(image_topic_, qos,
    std::bind(&YoloDetectorNode::image_callback, this, std::placeholders::_1));
  if (enable_perf_stats_) {
    perf_window_ = {}; perf_window_.started = std::chrono::steady_clock::now();
  }
  inference_enabled_ = true; switching_ = false;
  reply_enabled(header, true, ocr_ ? "Vision inference enabled" : "YOLO inference enabled");
}

void YoloDetectorNode::set_enabled_callback(
  Header header, const std::shared_ptr<std_srvs::srv::SetBool::Request> request)
{
  if (switching_) {reply_enabled(header, false, "transition_in_progress"); return;}
  if (!request->data) {stop_group(header); return;}
  if (!ready_ || !engine_ ||
    get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    reply_enabled(header, false, ocr_ ? "vision_not_ready" : "yolo_not_ready"); return;
  }
  if (inference_enabled_) {reply_enabled(header, true, ocr_ ? "Vision inference enabled" : "YOLO inference enabled"); return;}
  if (!inference_control_->service_is_ready()) {
    reply_enabled(header, false, "manager_unavailable"); return;
  }
  switching_ = true; stop_requested_ = false; ++vision_generation_;
  const auto generation = vision_generation_;
  const auto work = instance_ + "_vision_" + std::to_string(++sequence_);
  auto acquire = std::make_shared<Control::Request>();
  acquire->operation = "acquire"; acquire->client_id = "yolo_detector"; acquire->request_id = work;
  acquire->arguments = nlohmann::json{{"work_id", work}, {"stage", "vision"}}.dump();
  auto abandoned = std::make_shared<bool>(false);
  control_deadline_ = create_wall_timer(std::chrono::milliseconds(1500), [this, header, abandoned] {
    control_deadline_->cancel(); control_deadline_.reset(); *abandoned = true;
    stop_requested_ = true;
    reply_enabled(header, false, "permit_timeout");
  });
  inference_control_->async_send_request(acquire,
    [this, work, generation, header, abandoned](rclcpp::Client<Control>::SharedFuture future) {
      if (control_deadline_) {control_deadline_->cancel(); control_deadline_.reset();}
      const auto response = future.get();
      const auto result = nlohmann::json::parse(response->result, nullptr, false);
      const bool granted = response->accepted && result.is_object() && result.value("granted", false);
      if (!granted) {
        switching_ = false;
        if (!*abandoned) {reply_enabled(header, false, response->error);}
        return;
      }
      inference_work_ = work;
      if (*abandoned || stop_requested_ || !ready_ || generation != vision_generation_ ||
        get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        stop_group(*abandoned ? Header{} : header, "vision_start_cancelled"); return;
      }
      call_ocr("enable", work, [this, work, generation, header](bool ok, const std::string & error) {
        if (work != inference_work_) {reply_enabled(header, false, "vision_changed"); return;}
        if (!ok || stop_requested_ || !ready_ || generation != vision_generation_ ||
          get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          stop_group(header, ok ? "vision_start_cancelled" : error); return;
        }
        try {open_input(header);}
        catch (const std::exception & exception) {stop_group(header, exception.what());}
      });
    });
}

void YoloDetectorNode::publish_detections(
  const sensor_msgs::msg::Image::ConstSharedPtr & message,
  std::vector<vision_msgs::msg::Detection2D> detections, YoloFrameTiming * timing,
  std::chrono::steady_clock::time_point callback_started)
{
  vision_msgs::msg::Detection2DArray output;
  output.header = message->header;
  for (auto & detection : detections) {
    detection.header = message->header;
  }
  output.detections = std::move(detections);
  // Publish an immutable raw snapshot before assigning IDs to the perception result.
  detections_publisher_->publish(output);
  if (use_tracker_) {
    const auto track_started = timing ? std::chrono::steady_clock::now() :
      std::chrono::steady_clock::time_point{};
    tracker_.update(output, message->width, message->height);
    if (timing) {
      timing->track_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - track_started).count();
    }
  } else {
    // (source stamp, id) associates 2D ROI and filtered 3D output; this is not a tracking ID.
    // Preserve same-frame pairing while making non-tracked identities explicit
    // to consumers that require actual cross-frame tracks (warehouse counting).
    for (size_t i = 0; i < output.detections.size(); ++i) {output.detections[i].id = "frame/" + std::to_string(i);}
  }
  perception_publisher_->publish(output);
  if (timing) {
    timing->finished_at = std::chrono::steady_clock::now();
    timing->callback_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      timing->finished_at - callback_started).count();
    timing->output_age_ns=source_age_ns(message->header,now().nanoseconds());
    if (record_perf(*timing, true)) {report_perf(timing->finished_at, false, &output);}
  }
}

bool YoloDetectorNode::record_perf(const YoloFrameTiming & timing, bool success)
{
  if (success) {
    ++perf_window_.frames;
    perf_window_.pre_ns += timing.pre_ns;
    perf_window_.infer_ns += timing.infer_ns;
    perf_window_.post_ns += timing.post_ns;
    perf_window_.track_ns += timing.track_ns;
    perf_window_.callback_ns += timing.callback_ns;
    const auto total = timing.pre_ns + timing.infer_ns + timing.post_ns + timing.track_ns;
    perf_window_.total_max_ns = std::max(perf_window_.total_max_ns, total);
    perf_window_.callback_max_ns = std::max(perf_window_.callback_max_ns, timing.callback_ns);
    if(timing.input_age_ns>=0 && timing.output_age_ns>=0) {
      ++perf_window_.age_samples;
      perf_window_.input_age_ns+=timing.input_age_ns;perf_window_.output_age_ns+=timing.output_age_ns;
      perf_window_.input_age_max_ns=std::max(perf_window_.input_age_max_ns,timing.input_age_ns);
      perf_window_.output_age_max_ns=std::max(perf_window_.output_age_max_ns,timing.output_age_ns);
    }
  } else {
    ++perf_window_.failed;
  }
  return timing.finished_at - perf_window_.started >= std::chrono::seconds(5);
}

void YoloDetectorNode::report_perf(
  std::chrono::steady_clock::time_point finished_at, bool partial,
  const vision_msgs::msg::Detection2DArray * current, const std::string & error)
{
  const double seconds = std::chrono::duration<double>(finished_at - perf_window_.started).count();
  if (seconds <= 0.0) {
    return;
  }
  const auto window = perf_window_;
  perf_window_ = {}; perf_window_.started = finished_at;
  if (!rcutils_logging_logger_is_enabled_for(get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO)) {
    return;
  }
  const double fps = static_cast<double>(window.frames) / seconds;
  std::ostringstream log;
  log << std::fixed << std::setprecision(3) << "[YOLO perf] window=" << seconds
      << "s frames=" << window.frames << " failed=" << window.failed
      << " partial=" << (partial ? "true" : "false");
  if (window.frames > 0) {
    const double divisor = static_cast<double>(window.frames) * 1e6;
    log << " pre=" << window.pre_ns / divisor << "ms infer=" << window.infer_ns / divisor
        << "ms post=" << window.post_ns / divisor << "ms process="
        << (window.pre_ns + window.infer_ns + window.post_ns) / divisor << "ms fps=" << fps
        << " track=" << window.track_ns / divisor << "ms total="
        << (window.pre_ns + window.infer_ns + window.post_ns + window.track_ns) / divisor
        << "ms total_max=" << window.total_max_ns / 1e6 << "ms callback="
        << window.callback_ns / divisor << "ms callback_max=" << window.callback_max_ns / 1e6
        << "ms";
  } else {
    log << " pre=N/A infer=N/A post=N/A process=N/A fps=0.000 track=N/A total=N/A total_max=N/A callback=N/A callback_max=N/A";
  }
  log << " age_samples=" << window.age_samples;
  if(window.age_samples) {
    const double divisor=static_cast<double>(window.age_samples)*1e6;
    log << " input_age=" << window.input_age_ns/divisor << "ms input_age_max=" << window.input_age_max_ns/1e6
        << "ms output_age=" << window.output_age_ns/divisor << "ms output_age_max=" << window.output_age_max_ns/1e6 << "ms";
  } else {log << " input_age=N/A input_age_max=N/A output_age=N/A output_age_max=N/A";}
  if (current && !partial) {
    log << "\n  result: stamp=" << current->header.stamp.sec << '.' << std::setw(9)
        << std::setfill('0') << current->header.stamp.nanosec << std::setfill(' ');
    if (!error.empty()) {log << " error=" << nlohmann::json(error).dump();}
    else {
      log << " objects=" << current->detections.size();
      if (current->detections.empty()) {log << " none";}
      for (size_t i = 0; i < current->detections.size(); ++i) {
        const auto & detection = current->detections[i];
        log << "\n  #" << i;
        if (detection.results.empty()) {log << " class=unknown score=N/A";}
        else {
          const auto & hypothesis = detection.results.front().hypothesis;
          log << " class=" << nlohmann::json(hypothesis.class_id).dump(
            -1, ' ', false, nlohmann::json::error_handler_t::replace)
              << std::setprecision(3) << " score=" << hypothesis.score;
        }
        const auto & box = detection.bbox;
        log << std::setprecision(1) << " box_px=("
            << box.center.position.x - box.size_x / 2.0 << ','
            << box.center.position.y - box.size_y / 2.0 << ','
            << box.size_x << ',' << box.size_y << ')';
      }
    }
  }
  RCLCPP_INFO(get_logger(), "%s", log.str().c_str());
}

void YoloDetectorNode::disable_inference()
{
  const bool was_enabled = inference_enabled_;
  inference_enabled_ = false;
  image_subscription_.reset();
  tracker_.clear();
  if (enable_perf_stats_ && was_enabled) {
    if (perf_window_.frames > 0 || perf_window_.failed > 0) {
      report_perf(std::chrono::steady_clock::now(), true);
    }
    perf_window_ = {};
  }
}

bool YoloDetectorNode::read_engine_config(YoloEngineConfig & config, std::string & error) const
{
  const int padding_value = this->get_parameter("letterbox_padding_value").as_int();
  const auto tensor_type = this->get_parameter("input_tensor_type").as_string();
  const auto color_order = this->get_parameter("input_color_order").as_string();
  config.model_path = this->get_parameter("model_path").as_string();
  config.labels_path = this->get_parameter("labels_path").as_string();
  config.output_format = this->get_parameter("output_format").as_string();
  config.preprocess.input_width = this->get_parameter("input_width").as_int();
  config.preprocess.input_height = this->get_parameter("input_height").as_int();
  config.preprocess.padding_value = static_cast<uint8_t>(padding_value);
  config.preprocess.normalize_scale =
    static_cast<float>(this->get_parameter("normalize_scale").as_double());
  config.postprocess.confidence_threshold =
    static_cast<float>(this->get_parameter("confidence_threshold").as_double());
  config.postprocess.nms_iou_threshold =
    static_cast<float>(this->get_parameter("nms_iou_threshold").as_double());
  config.postprocess.max_detections = this->get_parameter("max_detections").as_int();
  config.ort_intra_threads = this->get_parameter("ort_intra_threads").as_int();
  config.ort_inter_threads = this->get_parameter("ort_inter_threads").as_int();
  config.ep_intra_threads = this->get_parameter("ep_intra_threads").as_int();
  config.ep_inter_threads = this->get_parameter("ep_inter_threads").as_int();
  config.ep_intra_thread_affinity = this->get_parameter("ep_intra_thread_affinity").as_string();

  if (config.model_path.empty() || config.labels_path.empty() || tensor_type != "float32" ||
    color_order != "rgb" || padding_value < 0 || padding_value > 255 ||
    !std::isfinite(config.preprocess.normalize_scale))
  {
    error =
      "model_path and labels_path are required; C08 requires float32 RGB input and padding in [0, 255]";
    return false;
  }
  return true;
}

void YoloDetectorNode::release_ros_resources()
{
  image_subscription_.reset();
  detections_publisher_.reset();
  perception_publisher_.reset();
}

}  // namespace k1muse_perception
