#include "k1muse_perception/ocr_node.hpp"

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <utility>
#include <pthread.h>
#include <sched.h>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rcutils/logging.h>

namespace k1muse_perception
{
OcrNode::OcrNode(const rclcpp::NodeOptions & options) : Node("ocr_detector", options)
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  enable_perf_stats_ = declare_parameter<bool>("enable_perf_stats", false, startup);
  config_.det_model_path = declare_parameter<std::string>("det_model_path", "", startup);
  config_.rec_model_path = declare_parameter<std::string>("rec_model_path", "", startup);
  config_.dictionary_path = declare_parameter<std::string>("dictionary_path", "", startup);
  config_.det_width = declare_parameter<int>("det_width", 640, startup);
  config_.det_height = declare_parameter<int>("det_height", 640, startup);
  config_.rec_width = declare_parameter<int>("rec_width", 320, startup);
  config_.rec_height = declare_parameter<int>("rec_height", 48, startup);
  config_.det_threshold = declare_parameter<double>("det_threshold", 0.3, startup);
  config_.box_threshold = declare_parameter<double>("box_threshold", 0.6, startup);
  config_.unclip_ratio = declare_parameter<double>("unclip_ratio", 2.0, startup);
  config_.nms_threshold = declare_parameter<double>("nms_threshold", 0.5, startup);
  const int candidates = declare_parameter<int>("max_candidates", 256, startup);
  const int lines = declare_parameter<int>("max_text_lines", 32, startup);
  if (candidates <= 0 || lines <= 0) {throw std::invalid_argument("OCR budgets must be positive");}
  config_.max_candidates = static_cast<std::size_t>(candidates);
  config_.max_text_lines = static_cast<std::size_t>(lines);
  config_.rec_output_logits = declare_parameter<bool>("rec_output_logits", false, startup);
  const auto topic = declare_parameter<std::string>(
    "observations_topic", "/k1muse/perception/ocr/observations", startup);
  auto qos = rclcpp::SensorDataQoS(); qos.keep_last(1);
  publisher_ = create_publisher<std_msgs::msg::String>(topic, qos);
  parent_ = create_client<Control>("/k1muse/perception/yolo/control");
  service_ = create_service<Control>("/k1muse/perception/ocr/control",
    [this](Header header, Control::Request::SharedPtr request) {control(header, request);});
  worker_ = std::thread(&OcrNode::worker_loop, this);
}

OcrNode::~OcrNode() {shutdown();}

std::string OcrNode::authorize(
  const std::string & operation, const std::string & work, uint64_t generation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (quitting_) {throw std::runtime_error("OCR is shutting down");}
  expected_operation_ = operation; expected_work_ = work; expected_generation_ = generation;
  return std::to_string(++authorization_);
}

void OcrNode::respond(
  Header header, const Control::Request & request, bool ok, const std::string & error)
{
  Control::Response response;
  response.operation_id = request.request_id; response.accepted = ok; response.error = error;
  response.result = ok ? "{\"status\":\"completed\"}" : "{\"status\":\"failed\"}";
  try {service_->send_response(*header, response);}
  catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "OCR control response failed: %s", exception.what());
  }
}

void OcrNode::control(Header header, std::shared_ptr<Control::Request> request)
{
  std::string error;
  std::optional<Frame> retired;
  try {
    const auto args = nlohmann::json::parse(request->arguments);
    const auto work = args.at("work_id").get<std::string>();
    const auto generation = args.at("generation").get<uint64_t>();
    std::lock_guard<std::mutex> lock(mutex_);
    if (quitting_ || request->client_id != "yolo_detector" || work.empty() ||
      request->token != std::to_string(authorization_) ||
      request->operation != expected_operation_ || work != expected_work_ ||
      generation != expected_generation_)
    {
      error = "stale_or_unauthorized_ocr_operation";
    } else if (command_) {error = "transition_in_progress";}
    else if (request->operation == "prepare" || request->operation == "enable" ||
      request->operation == "disable")
    {
      if (request->operation == "disable") {
        enabled_ = false; stop_.store(true, std::memory_order_relaxed);
        retired.swap(pending_);
      }
      command_ = Command{header, request, work, request->token, generation};
      expected_operation_.clear();
    } else {error = "unsupported_operation";}
  } catch (const std::exception & exception) {error = exception.what();}
  if (!error.empty()) {respond(header, *request, false, error);}
  else {wake_.notify_one();}
}

void OcrNode::try_submit(sensor_msgs::msg::Image::ConstSharedPtr image)
{
  std::optional<Frame> retired;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || !ready_ || quitting_) {return;}
    retired.swap(pending_);
    pending_ = Frame{std::move(image), work_, generation_};
  }
  wake_.notify_one();
}

void OcrNode::stop_and_wait()
{
  std::optional<Frame> retired;
  std::unique_lock<std::mutex> lock(mutex_);
  ++authorization_; expected_operation_.clear();
  enabled_ = false; stop_.store(true, std::memory_order_relaxed);
  retired.swap(pending_);
  wake_.notify_one();
  idle_.wait(lock, [this] {return !busy_ && !command_;});
}

void OcrNode::shutdown()
{
  if (!worker_.joinable()) {return;}
  stop_and_wait();
  {std::lock_guard<std::mutex> lock(mutex_); quitting_ = true;}
  wake_.notify_one(); worker_.join();
}

void OcrNode::worker_loop()
{
  cpu_set_t cpus; CPU_ZERO(&cpus); CPU_SET(0, &cpus);
  const int affinity = pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);
  const std::string affinity_error = affinity == 0 ? "" :
    std::string("OCR CPU0 affinity failed: ") + std::strerror(affinity);
  // The engine and its sessions are created and destroyed only by this worker.
  std::unique_ptr<OcrEngine> engine;
  while (true) {
    std::optional<Command> command;
    std::optional<Frame> frame;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] {return quitting_ || command_ || pending_;});
      if (quitting_) {break;}
      busy_ = true;
      if (command_) {command.swap(command_);}
      else {frame.swap(pending_);}
    }
    if (command) {
      std::string error;
      try {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (command->authorization != std::to_string(authorization_)) {
            throw std::runtime_error("ocr_operation_cancelled");
          }
        }
        const auto & op = command->request->operation;
        if (op == "prepare") {
          if (!affinity_error.empty()) {throw std::runtime_error(affinity_error);}
          if (!engine) {engine = std::make_unique<OcrEngine>(config_);}
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (command->authorization != std::to_string(authorization_)) {
            throw std::runtime_error("ocr_operation_cancelled");
          }
          if (op == "prepare") {ready_ = true;}
          else if (op == "enable") {
            if (!engine || !ready_) {throw std::runtime_error("ocr_not_ready");}
            work_ = command->work; generation_ = command->generation;
            if (enable_perf_stats_) {
              perf_started_ = std::chrono::steady_clock::now();
              perf_frames_ = 0; perf_failed_ = 0;
            }
            stop_.store(false, std::memory_order_relaxed); enabled_ = true;
          }
          // disable was gated immediately in control(); reaching here drains Run and publication.
        }
      } catch (const std::exception & exception) {
        error = exception.what();
        if (command->request->operation == "prepare") {engine.reset();}
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = false; stop_.store(true, std::memory_order_relaxed);
        if (command->request->operation == "prepare") {ready_ = false;}
      } catch (...) {
        error = "unknown_ocr_control_failure";
        std::lock_guard<std::mutex> lock(mutex_);
        ready_ = false; enabled_ = false; stop_.store(true, std::memory_order_relaxed);
      }
      respond(command->header, *command->request, error.empty(), error);
      command.reset();
    } else if (frame) {
      try {process_frame(*engine, *frame);}
      catch (const std::exception & exception) {report_fault(*frame, exception.what());}
      catch (...) {report_fault(*frame, "unknown_ocr_inference_failure");}
      frame.reset();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
    }
    idle_.notify_all();
  }
  engine.reset();
}

void OcrNode::process_frame(OcrEngine & engine, const Frame & frame)
{
  if (stop_.load(std::memory_order_relaxed)) {return;}
  const auto & image = *frame.image;
  nlohmann::json result{{"version", 1}, {"vision_work_id", frame.work},
    {"generation", frame.generation}, {"image_width", image.width}, {"image_height", image.height},
    {"header", {{"frame_id", image.header.frame_id}, {"stamp", {
      {"sec", image.header.stamp.sec}, {"nanosec", image.header.stamp.nanosec}}}}},
    {"error", ""}, {"truncated", false}, {"candidates", 0}, {"recognized", 0},
    {"texts", nlohmann::json::array()}};
  if (image.encoding != sensor_msgs::image_encodings::RGB8 || image.width == 0 || image.height == 0 ||
    image.width > static_cast<uint32_t>(std::numeric_limits<int>::max() / 3) ||
    image.height > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    image.step < static_cast<uint64_t>(image.width) * 3 ||
    image.data.size() < static_cast<uint64_t>(image.step) * image.height)
  {
    result["error"] = "invalid_rgb8_frame";
  } else {
    const cv::Mat rgb(static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3,
      const_cast<uint8_t *>(image.data.data()), image.step);
    const auto output = engine.infer(rgb, stop_);
    if (output.cancelled || stop_.load(std::memory_order_relaxed)) {return;}
    result["truncated"] = output.truncated;
    result["candidates"] = output.candidates; result["recognized"] = output.recognized;
    for (const auto & text : output.texts) {
      auto polygon = nlohmann::json::array();
      for (const auto & p : text.polygon) {polygon.push_back({p.x, p.y});}
      result["texts"].push_back({{"raw_text", text.raw_text}, {"polygon", std::move(polygon)},
        {"rec_score", text.score}});
    }
  }
  if (stop_.load(std::memory_order_relaxed)) {return;}
  std_msgs::msg::String message; message.data = result.dump();
  publisher_->publish(message);
  if (enable_perf_stats_) {record_perf(image, result);}
}

void OcrNode::record_perf(const sensor_msgs::msg::Image & image, const nlohmann::json & result)
{
  const auto finished = std::chrono::steady_clock::now();
  const auto & error = result.at("error").get_ref<const std::string &>();
  if (error.empty()) {++perf_frames_;} else {++perf_failed_;}
  if (finished - perf_started_ < std::chrono::seconds(5)) {return;}
  const double seconds = std::chrono::duration<double>(finished - perf_started_).count();
  const auto frames = perf_frames_, failed = perf_failed_;
  perf_started_ = finished; perf_frames_ = 0; perf_failed_ = 0;
  if (stop_.load(std::memory_order_relaxed) ||
    !rcutils_logging_logger_is_enabled_for(get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO))
  {return;}
  std::ostringstream log;
  log << std::fixed << std::setprecision(3) << "[OCR perf] window=" << seconds
      << "s frames=" << frames << " fps=" << frames / seconds << " failed=" << failed
      << "\n  result: stamp=" << image.header.stamp.sec << '.' << std::setw(9)
      << std::setfill('0') << image.header.stamp.nanosec << std::setfill(' ');
  if (!error.empty()) {log << " error=" << result.at("error").dump();}
  else {
    const auto & texts = result.at("texts");
    log << " texts=" << texts.size() << " truncated="
        << (result.at("truncated").get<bool>() ? "true" : "false");
    if (texts.empty()) {log << " none";}
    for (size_t i = 0; i < texts.size(); ++i) {
      const auto & text = texts[i];
      log << "\n  #" << i << " text=" << text.at("raw_text").dump() << std::setprecision(3)
          << " score=" << text.at("rec_score").get<double>() << " quad_px=["
          << std::setprecision(1);
      const auto & polygon = text.at("polygon");
      for (size_t j = 0; j < polygon.size(); ++j) {
        if (j) {log << ',';}
        log << '(' << polygon[j][0].get<double>() << ',' << polygon[j][1].get<double>() << ')';
      }
      log << ']';
    }
  }
  if (!stop_.load(std::memory_order_relaxed)) {RCLCPP_INFO(get_logger(), "%s", log.str().c_str());}
}

void OcrNode::report_fault(const Frame & frame, const std::string & error)
{
  if (enable_perf_stats_) {++perf_failed_;}
  std::optional<Frame> retired;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = false; enabled_ = false; stop_.store(true, std::memory_order_relaxed);
    retired.swap(pending_);
  }
  RCLCPP_ERROR(get_logger(), "OCR engine failed: %s", error.c_str());
  try {
    auto request = std::make_shared<Control::Request>();
    request->operation = "ocr_fault"; request->client_id = "ocr_detector";
    request->request_id = frame.work + "_ocr_fault_" + std::to_string(frame.generation);
    request->arguments = nlohmann::json{{"work_id", frame.work},
      {"generation", frame.generation}, {"error", error}}.dump();
    parent_->async_send_request(request, [logger = get_logger()](rclcpp::Client<Control>::SharedFuture future) {
      try {
        if (!future.get()->accepted) {RCLCPP_ERROR(logger, "OCR fault was not acknowledged by parent");}
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(logger, "OCR fault acknowledgement: %s", exception.what());
      }
    });
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "OCR fault report failed: %s", exception.what());
  }
}
}  // namespace k1muse_perception
