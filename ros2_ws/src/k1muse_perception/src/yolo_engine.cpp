#include "k1muse_perception/yolo_engine.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_map>
#include <utility>

#include <onnxruntime_cxx_api.h>
#include <spacemit_ort_env.h>

namespace k1muse_perception
{
namespace
{

constexpr char kRawYoloV8Format[] = "yolov8_raw_1x84xn";
constexpr char kDflYoloV8Format[] = "yolov8_dfl_9outputs";

bool load_labels(const std::string & path, std::vector<std::string> & labels, std::string & error)
{
  std::ifstream input(path);
  if (!input) {
    error = "cannot open labels file: " + path;
    return false;
  }
  labels.clear();
  std::string label;
  while (std::getline(input, label)) {
    if (!label.empty() && label.back() == '\r') {
      label.pop_back();
    }
    if (label.empty()) {
      error = "labels file contains an empty label";
      return false;
    }
    labels.push_back(label);
  }
  if (labels.empty()) {
    error = "labels file is empty";
    return false;
  }
  return true;
}

bool validate_affinity(const std::string & affinity, int expected_count, std::string & error)
{
  if (affinity.empty()) {
    return true;
  }
  std::istringstream stream(affinity);
  std::string token;
  std::set<unsigned long> cpu_ids;
  while (std::getline(stream, token, ';')) {
    if (token.empty() || !std::all_of(
        token.begin(), token.end(), [](unsigned char value) {
          return std::isdigit(value) != 0;
        }))
    {
      error = "ep_intra_thread_affinity must contain semicolon-separated CPU IDs";
      return false;
    }
    unsigned long cpu_id = 0;
    try {
      cpu_id = std::stoul(token);
    } catch (const std::exception &) {
      error = "ep_intra_thread_affinity contains an invalid CPU ID";
      return false;
    }
    if (!cpu_ids.insert(cpu_id).second) {
      error = "ep_intra_thread_affinity contains a duplicate CPU ID";
      return false;
    }
    const auto cpu_directory = std::filesystem::path("/sys/devices/system/cpu") /
      ("cpu" + std::to_string(cpu_id));
    if (!std::filesystem::exists(cpu_directory)) {
      error = "ep_intra_thread_affinity contains a CPU ID that is unavailable on this target";
      return false;
    }
    const auto online_path = cpu_directory / "online";
    if (std::filesystem::exists(online_path)) {
      std::ifstream online(online_path);
      std::string value;
      std::getline(online, value);
      if (value != "1") {
        error = "ep_intra_thread_affinity contains an offline CPU ID";
        return false;
      }
    }
  }
  if (static_cast<int>(cpu_ids.size()) != expected_count) {
    error = "ep_intra_thread_affinity CPU-ID count must equal ep_intra_threads";
    return false;
  }
  return true;
}

}  // namespace

class YoloEngine::Impl
{
public:
  bool run(
    const std::vector<float> & tensor, std::vector<Ort::Value> & outputs,
    std::string & error)
  {
    if (!session || !memory_info ||
      tensor.size() != static_cast<size_t>(config.preprocess.input_width) *
      static_cast<size_t>(config.preprocess.input_height) * 3U)
    {
      error = "YOLO engine is not configured";
      return false;
    }
    try {
      std::array<int64_t, 4> shape{
        1, 3, config.preprocess.input_height, config.preprocess.input_width};
      auto input = Ort::Value::CreateTensor<float>(
        *memory_info, const_cast<float *>(tensor.data()), tensor.size(), shape.data(),
        shape.size());
      const char * input_names[] = {input_name.c_str()};
      std::vector<const char *> output_name_pointers;
      for (const auto & name : output_names) {
        output_name_pointers.push_back(name.c_str());
      }
      outputs = session->Run(
        Ort::RunOptions{nullptr}, input_names, &input, 1,
        output_name_pointers.data(), output_name_pointers.size());
      return true;
    } catch (const Ort::Exception & exception) {
      error = std::string("ONNX Runtime inference failed: ") + exception.what();
      return false;
    }
  }

  YoloEngineConfig config;
  bool postprocess(
    const std::vector<Ort::Value> & outputs,
    std::vector<vision_msgs::msg::Detection2D> & detections, std::string & error);

  PreprocessedImage preprocessed;
  std::unique_ptr<Ort::Env> env;
  std::unique_ptr<Ort::Session> session;
  std::unique_ptr<Ort::MemoryInfo> memory_info;
  std::string input_name;
  std::vector<std::string> output_names;
};

YoloEngine::YoloEngine()
: impl_(std::make_unique<Impl>())
{
}

YoloEngine::~YoloEngine() = default;

bool YoloEngine::configure(const YoloEngineConfig & config, std::string & error)
{
  reset();
  if (!std::filesystem::is_regular_file(config.model_path)) {
    error = "model_path is not a regular file: " + config.model_path;
    return false;
  }
  if (config.output_format != kRawYoloV8Format && config.output_format != kDflYoloV8Format) {
    error =
      "output_format must explicitly select yolov8_raw_1x84xn or yolov8_dfl_9outputs";
    return false;
  }
  if (config.preprocess.input_width != config.preprocess.input_height ||
    (config.preprocess.input_width != 320 && config.preprocess.input_width != 416) ||
    config.ort_intra_threads <= 0 || config.ort_inter_threads <= 0 ||
    config.ep_intra_threads <= 0 || config.ep_inter_threads <= 0 ||
    config.postprocess.max_detections <= 0 || config.postprocess.confidence_threshold < 0.0F ||
    config.postprocess.confidence_threshold > 1.0F ||
    config.postprocess.nms_iou_threshold <= 0.0F ||
    config.postprocess.nms_iou_threshold > 1.0F)
  {
    error = "invalid YOLO engine parameter";
    return false;
  }
  if (!validate_affinity(config.ep_intra_thread_affinity, config.ep_intra_threads, error))
  {
    return false;
  }
  impl_->config = config;
  if (!load_labels(config.labels_path, impl_->config.postprocess.labels, error)) {
    return false;
  }
  const int64_t class_count = static_cast<int64_t>(impl_->config.postprocess.labels.size());
  if (config.output_format == kRawYoloV8Format && class_count != 80) {
    error = "yolov8_raw_1x84xn requires exactly 80 class labels";
    return false;
  }

  try {
    impl_->env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "k1muse_yolo");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(config.ort_intra_threads);
    options.SetInterOpNumThreads(config.ort_inter_threads);
    std::unordered_map<std::string, std::string> provider_options{
      {"SPACEMIT_EP_INTRA_THREAD_NUM", std::to_string(config.ep_intra_threads)},
      {"SPACEMIT_EP_INTER_THREAD_NUM", std::to_string(config.ep_inter_threads)},
      {"SPACEMIT_EP_USE_GLOBAL_INTRA_THREAD", "0"}};
    if (!config.ep_intra_thread_affinity.empty()) {
      provider_options.emplace(
        "SPACEMIT_EP_INTRA_THREAD_AFFINITY",
        config.ep_intra_thread_affinity);
    }
    // SpaceMIT is registered by this API; the global ORT list can contain only CPU.
    const auto ep_status = SessionOptionsSpaceMITEnvInit(options, provider_options);
    if (!ep_status.IsOK()) {
      error = "SpaceMIT EP initialization failed: " + ep_status.GetErrorMessage();
      reset();
      return false;
    }
    impl_->session =
      std::make_unique<Ort::Session>(*impl_->env, config.model_path.c_str(), options);
    impl_->memory_info = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    const size_t expected_outputs = config.output_format == kDflYoloV8Format ? 9U : 1U;
    if (impl_->session->GetInputCount() != 1 ||
      impl_->session->GetOutputCount() != expected_outputs)
    {
      error = "model input/output count does not match the selected YOLOv8 contract";
      reset();
      return false;
    }
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name = impl_->session->GetInputNameAllocated(0, allocator);
    impl_->input_name = input_name.get();
    for (size_t index = 0; index < expected_outputs; ++index) {
      const auto output_name = impl_->session->GetOutputNameAllocated(index, allocator);
      impl_->output_names.emplace_back(output_name.get());
    }

    const auto input_type = impl_->session->GetInputTypeInfo(0);
    const auto input_info = input_type.GetTensorTypeAndShapeInfo();
    const auto input_shape = input_info.GetShape();
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      input_shape.size() != 4U || input_shape[0] != 1 || input_shape[1] != 3 ||
      input_shape[2] != config.preprocess.input_height ||
      input_shape[3] != config.preprocess.input_width)
    {
      error = "model input must be static float [1, 3, configured_height, configured_width]";
      reset();
      return false;
    }
    for (size_t index = 0; index < expected_outputs; ++index) {
      const auto output_type = impl_->session->GetOutputTypeInfo(index);
      const auto output_info = output_type.GetTensorTypeAndShapeInfo();
      const auto shape = output_info.GetShape();
      bool valid = output_info.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
      if (config.output_format == kDflYoloV8Format) {
        const int stride = 8 << (index / 3U);
        const int64_t channels = index % 3U == 0U ? 64 :
          (index % 3U == 1U ? class_count : 1);
        const std::vector<int64_t> expected{
          1, channels, config.preprocess.input_height / stride,
          config.preprocess.input_width / stride};
        valid = valid && config.preprocess.input_width % 32 == 0 && shape == expected;
      } else {
        valid = valid && shape.size() == 3U && shape[0] == 1 && shape[1] == 84 &&
          shape[2] != 0;
      }
      if (!valid) {
        error = "model output " + std::to_string(index) + " does not match " + config.output_format;
        reset();
        return false;
      }
    }
    const cv::Mat warmup(
      config.preprocess.input_height, config.preprocess.input_width, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<vision_msgs::msg::Detection2D> warmup_detections;
    if (!infer(warmup, warmup_detections, error))
    {
      reset();
      return false;
    }
  } catch (const Ort::Exception & exception) {
    error = std::string("ONNX Runtime setup failed: ") + exception.what();
    reset();
    return false;
  }
  return true;
}

void YoloEngine::reset()
{
  impl_->preprocessed = {};
  impl_->output_names.clear();
  impl_->input_name.clear();
  impl_->memory_info.reset();
  impl_->session.reset();
  impl_->env.reset();
}

bool YoloEngine::infer(
  const cv::Mat & rgb8, std::vector<vision_msgs::msg::Detection2D> & detections,
  std::string & error)
{
  auto & preprocessed = impl_->preprocessed;
  if (!preprocess_rgb8(rgb8, impl_->config.preprocess, preprocessed, error)) {
    return false;
  }
  std::vector<Ort::Value> outputs;
  if (!impl_->run(preprocessed.tensor, outputs, error)) {
    return false;
  }
  return impl_->postprocess(outputs, detections, error);
}

bool YoloEngine::infer_with_timing(
  const cv::Mat & rgb8, std::vector<vision_msgs::msg::Detection2D> & detections,
  YoloFrameTiming & timing, std::string & error)
{
  using Clock = std::chrono::steady_clock;
  std::vector<Ort::Value> outputs;
  const auto t0 = Clock::now();
  const bool pre_ok = preprocess_rgb8(
    rgb8, impl_->config.preprocess, impl_->preprocessed, error);
  const auto t1 = Clock::now();
  timing.finished_at = t1;
  if (!pre_ok) {
    return false;
  }
  const bool infer_ok = impl_->run(impl_->preprocessed.tensor, outputs, error);
  const auto t2 = Clock::now();
  timing.finished_at = t2;
  if (!infer_ok) {
    return false;
  }
  const bool post_ok = impl_->postprocess(outputs, detections, error);
  const auto t3 = Clock::now();
  timing.finished_at = t3;
  if (!post_ok) {
    return false;
  }
  timing.pre_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
  timing.infer_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
  timing.post_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count();
  return true;
}

bool YoloEngine::Impl::postprocess(
  const std::vector<Ort::Value> & outputs,
  std::vector<vision_msgs::msg::Detection2D> & detections, std::string & error)
{
  const bool dfl = config.output_format == kDflYoloV8Format;
  if (outputs.size() != (dfl ? 9U : 1U)) {
    error = "model output count changed after configuration";
    return false;
  }
  for (const auto & output : outputs) {
    if (!output.IsTensor() ||
      output.GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    {
      error = "model output must be a float tensor";
      return false;
    }
  }
  std::vector<DetectionCandidate> candidates;
  if (dfl) {
    std::array<YoloDflOutput, 9> views;
    for (size_t index = 0; index < views.size(); ++index) {
      views[index] = YoloDflOutput{
        outputs[index].GetTensorData<float>(),
        outputs[index].GetTensorTypeAndShapeInfo().GetShape()};
    }
    if (!decode_yolov8_dfl_9outputs(
        views, config.preprocess.input_width, config.preprocess.input_height,
        config.postprocess.confidence_threshold, candidates, error))
    {
      return false;
    }
  } else if (!decode_yolov8_raw_1x84xn(
      outputs.front().GetTensorData<float>(), outputs.front().GetTensorTypeAndShapeInfo().GetShape(),
      config.postprocess.confidence_threshold, candidates, error))
  {
    return false;
  }
  detections = make_detections(candidates, preprocessed.letterbox, config.postprocess);
  return true;
}

}  // namespace k1muse_perception
