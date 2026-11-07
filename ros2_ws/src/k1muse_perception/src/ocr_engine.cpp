// PPOCR geometry/CTC adapted from SpacemiT model-zoo-vision (Apache-2.0).
// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// See OCR_ATTRIBUTION.md for the source snapshot and local adaptations.
#include "k1muse_perception/ocr_engine.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <onnxruntime_cxx_api.h>
#include <spacemit_ort_env.h>
#include <opencv2/imgproc.hpp>
#include "k1muse_perception/ocr_rvv_pack.hpp"

namespace k1muse_perception
{
namespace
{
void require(bool value, const char * error)
{
  if (!value) {throw std::runtime_error(error);}
}

struct Model
{
  std::unique_ptr<Ort::Session> session;
  std::string input_name, output_name;
  std::array<int64_t, 4> shape;
  std::vector<float> input;

  void load(Ort::Env & env, const std::string & path, int width, int height)
  {
    require(!path.empty(), "OCR model path is empty");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1); options.SetInterOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    options.AddConfigEntry("session.intra_op.allow_spinning", "0");
    options.AddConfigEntry("session.inter_op.allow_spinning", "0");
    const auto status = Ort::SessionOptionsSpaceMITEnvInit(options, {
      {"SPACEMIT_EP_INTRA_THREAD_NUM", "1"}, {"SPACEMIT_EP_INTER_THREAD_NUM", "1"},
      {"SPACEMIT_EP_INTRA_THREAD_AFFINITY", "0"},
      {"SPACEMIT_EP_USE_GLOBAL_INTRA_THREAD", "0"}});
    if (!status.IsOK()) {
      throw std::runtime_error(std::string("OCR EP initialization: ") + status.GetErrorMessage());
    }
    session = std::make_unique<Ort::Session>(env, path.c_str(), options);
    require(session->GetInputCount() == 1 && session->GetOutputCount() == 1,
      "OCR requires one input and one output per model");
    // Keep the parent TypeInfo alive while reading its non-owning tensor view.
    const auto type = session->GetInputTypeInfo(0);
    const auto info = type.GetTensorTypeAndShapeInfo();
    const auto dims = info.GetShape();
    shape = {1, 3, height, width};
    require(info.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT && dims.size() == 4,
      "OCR currently supports external float32 NCHW inputs only");
    for (std::size_t i = 0; i < shape.size(); ++i) {
      require(dims[i] == shape[i] || dims[i] < 0, "OCR input dimensions do not match config");
    }
    const auto output_type = session->GetOutputTypeInfo(0);
    require(output_type.GetTensorTypeAndShapeInfo().GetElementType() ==
      ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, "OCR output must be float32");
    Ort::AllocatorWithDefaultOptions allocator;
    input_name = session->GetInputNameAllocated(0, allocator).get();
    output_name = session->GetOutputNameAllocated(0, allocator).get();
    input.resize(static_cast<std::size_t>(3) * width * height);
  }

  Ort::Value run(Ort::MemoryInfo & memory)
  {
    auto tensor = Ort::Value::CreateTensor<float>(
      memory, input.data(), input.size(), shape.data(), shape.size());
    const char * inputs[] = {input_name.c_str()};
    const char * outputs[] = {output_name.c_str()};
    Ort::Value result{nullptr};
    session->Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, &result, 1);
    return result;
  }
};

void pack_rgb(const cv::Mat & resized, std::vector<float> & tensor, int width, int height,
  const std::array<float, 3> & mean, const std::array<float, 3> & scale,
  const std::array<float, 3> & padding,
  const std::array<float, 3> & input_scale = {1.0F, 1.0F, 1.0F})
{
  const std::size_t plane = static_cast<std::size_t>(width) * height;
  for (int c = 0; c < 3; ++c) {
    auto * channel = tensor.data() + c * plane;
    for (int y = 0; y < resized.rows; ++y) {
      std::fill(channel + static_cast<std::size_t>(y) * width + resized.cols,
        channel + static_cast<std::size_t>(y + 1) * width, padding[c]);
    }
    std::fill(channel + static_cast<std::size_t>(resized.rows) * width,
      channel + plane, padding[c]);
  }
  for (int y = 0; y < resized.rows; ++y) {
    const auto * source = resized.ptr<uint8_t>(y);
    auto * r = tensor.data() + static_cast<std::size_t>(y) * width;
    auto * g = r + plane; auto * b = g + plane;
#if defined(__riscv_vector)
    vision_operators::detail::pack_u8c3_to_f32_planes_rvv(
      source, resized.cols, false, r, g, b, mean, scale, input_scale);
#else
    for (int x = 0; x < resized.cols; ++x) {
      r[x] = (source[3 * x] * input_scale[0] - mean[0]) * scale[0];
      g[x] = (source[3 * x + 1] * input_scale[1] - mean[1]) * scale[1];
      b[x] = (source[3 * x + 2] * input_scale[2] - mean[2]) * scale[2];
    }
#endif
  }
}

std::array<cv::Point2f, 4> corners(const cv::RotatedRect & rect)
{
  std::array<cv::Point2f, 4> p;
  rect.points(p.data());
  std::sort(p.begin(), p.end(), [](const auto & a, const auto & b) {return a.x < b.x;});
  const auto tl = p[0].y <= p[1].y ? p[0] : p[1];
  const auto bl = p[0].y <= p[1].y ? p[1] : p[0];
  const auto tr = p[2].y <= p[3].y ? p[2] : p[3];
  const auto br = p[2].y <= p[3].y ? p[3] : p[2];
  return {tl, tr, br, bl};
}

cv::RotatedRect expand(const cv::RotatedRect & rect, float ratio)
{
  // Offset each edge outward, then recover a rectangle from the expanded hull.
  const auto p = corners(rect);
  const float area = rect.size.area();
  const float perimeter = 2.0F * (rect.size.width + rect.size.height);
  if (area <= 0 || perimeter <= 0) {return rect;}
  const float distance = area * ratio / perimeter;
  std::array<cv::Point2f, 8> hull;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto edge = p[(i + 1) % 4] - p[i];
    const auto length = static_cast<float>(cv::norm(edge));
    if (length < 1e-6F) {return rect;}
    cv::Point2f normal(-edge.y / length, edge.x / length);
    if (normal.dot((p[i] + p[(i + 1) % 4]) * 0.5F - rect.center) < 0) {normal = -normal;}
    hull[2 * i] = p[i] + normal * distance;
    hull[2 * i + 1] = p[(i + 1) % 4] + normal * distance;
  }
  return cv::minAreaRect(hull);
}

struct Candidate {std::size_t index; double area;};

struct Box
{
  std::array<cv::Point2f, 4> p;
  cv::Rect bounds;
  float score;
};
}  // namespace

struct OcrEngine::Impl
{
  explicit Impl(const OcrConfig & value)
  : config(value), env(ORT_LOGGING_LEVEL_WARNING, "k1muse_ocr"),
    memory(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
  {
    require(config.det_width > 0 && config.det_width <= 2048 && config.det_height > 0 &&
      config.det_height <= 2048 && config.rec_width > 0 && config.rec_width <= 2048 &&
      config.rec_height > 0 && config.rec_height <= 256 && config.max_candidates > 0 &&
      config.max_candidates <= 4096 && config.max_text_lines > 0 &&
      config.max_text_lines <= config.max_candidates, "invalid OCR dimensions or budgets");
    require(std::isfinite(config.det_threshold) && config.det_threshold > 0 &&
      config.det_threshold < 1 && std::isfinite(config.box_threshold) &&
      config.box_threshold > 0 && config.box_threshold <= 1 &&
      std::isfinite(config.unclip_ratio) && config.unclip_ratio > 0 &&
      config.unclip_ratio <= 10 && std::isfinite(config.nms_threshold) &&
      config.nms_threshold > 0 && config.nms_threshold <= 1, "invalid OCR thresholds");
    std::ifstream file(config.dictionary_path);
    require(file.is_open(), "cannot open OCR dictionary");
    std::string line;
    while (std::getline(file, line)) {
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      if (dictionary.empty() && line.compare(0, 3, "\xEF\xBB\xBF") == 0) {line.erase(0, 3);}
      require(!line.empty(), "OCR dictionary contains an empty entry");
      dictionary.push_back(line);
    }
    require(!dictionary.empty(), "empty OCR dictionary");
    if (dictionary.front() != "blank") {dictionary.insert(dictionary.begin(), "blank");}
    dictionary.push_back(" ");
    det.load(env, config.det_model_path, config.det_width, config.det_height);
    rec.load(env, config.rec_model_path, config.rec_width, config.rec_height);
    // Both sessions warm under the parent's preparation permit, even for blank scenes.
    auto det_output = det.run(memory);
    validate_detection(det_output);
    auto rec_output = rec.run(memory);
    validate_recognition(rec_output);
    boxes.reserve(config.max_candidates);
  }

  std::vector<int64_t> validate_detection(const Ort::Value & output) const
  {
    const auto info = output.GetTensorTypeAndShapeInfo();
    const auto dims = info.GetShape();
    require(info.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT && dims.size() == 4 &&
      dims[0] == 1 && dims[1] == 1 && dims[2] > 0 && dims[2] <= config.det_height &&
      dims[3] > 0 && dims[3] <= config.det_width, "OCR det output must be float [1,1,H,W]");
    return dims;
  }
  std::vector<int64_t> validate_recognition(const Ort::Value & output) const
  {
    const auto info = output.GetTensorTypeAndShapeInfo();
    const auto dims = info.GetShape();
    require(info.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT && dims.size() == 3 &&
      dims[0] == 1 && dims[1] > 0 && dims[1] <= 4096 && dims[2] > 1 &&
      static_cast<std::size_t>(dims[2]) == dictionary.size(),
      "OCR rec output must be float [1,T,blank+dictionary+space]");
    return dims;
  }

  float box_score(const cv::Mat & prob, const std::array<cv::Point2f, 4> & points)
  {
    std::array<cv::Point, 4> polygon;
    for (std::size_t i = 0; i < points.size(); ++i) {
      polygon[i] = {std::clamp(cvRound(points[i].x), 0, prob.cols - 1),
        std::clamp(cvRound(points[i].y), 0, prob.rows - 1)};
    }
    const auto bbox = cv::boundingRect(polygon) & cv::Rect(0, 0, prob.cols, prob.rows);
    if (bbox.empty()) {return 0;}
    if (mask_storage.size() < prob.total()) {mask_storage.resize(prob.total());}
    // Reinterpret reusable capacity at each ROI size; Mat::create would
    // reallocate whenever two consecutive text boxes have different dimensions.
    cv::Mat mask(bbox.height, bbox.width, CV_8UC1, mask_storage.data());
    mask.setTo(0);
    for (auto & p : polygon) {p -= bbox.tl();}
    cv::fillConvexPoly(mask, polygon, cv::Scalar(1));
    return static_cast<float>(cv::mean(prob(bbox), mask)[0]);
  }

  void detect(const cv::Mat & prob, int content_w, int content_h,
    const cv::Size & source, OcrFrame & result, const std::atomic_bool & stop)
  {
    cv::compare(prob, config.det_threshold, binary, cv::CMP_GT);
    const int valid_w = std::min(prob.cols, static_cast<int>(std::ceil(
      static_cast<double>(content_w) * prob.cols / config.det_width)));
    const int valid_h = std::min(prob.rows, static_cast<int>(std::ceil(
      static_cast<double>(content_h) * prob.rows / config.det_height)));
    auto clear_padding = [&] {
        if (valid_w < binary.cols) {binary.colRange(valid_w, binary.cols).setTo(0);}
        if (valid_h < binary.rows) {binary.rowRange(valid_h, binary.rows).setTo(0);}
      };
    clear_padding();
    cv::dilate(binary, binary, dilation_kernel);
    clear_padding();
    contours.clear();
    cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    candidates.clear();
    candidates.reserve(contours.size());
    for (std::size_t i = 0; i < contours.size(); ++i) {
      if (stop.load(std::memory_order_relaxed)) {return;}
      if (contours[i].size() < 4) {continue;}
      const auto area = std::abs(cv::contourArea(contours[i]));
      if (area >= 4) {candidates.push_back({i, area});}
    }
    result.candidates = candidates.size();
    const auto count = std::min(config.max_candidates, candidates.size());
    result.truncated = candidates.size() > count;
    std::partial_sort(candidates.begin(), candidates.begin() + count, candidates.end(),
      [](const auto & a, const auto & b) {return a.area > b.area;});
    boxes.clear();
    const float sx = static_cast<float>(source.width) * config.det_width /
      (content_w * static_cast<float>(prob.cols));
    const float sy = static_cast<float>(source.height) * config.det_height /
      (content_h * static_cast<float>(prob.rows));
    for (std::size_t i = 0; i < count; ++i) {
      if (stop.load(std::memory_order_relaxed)) {return;}
      const auto rect = cv::minAreaRect(contours[candidates[i].index]);
      if (std::min(rect.size.width, rect.size.height) < 3) {continue;}
      const float score = box_score(prob, corners(rect));
      if (!std::isfinite(score) || score < config.box_threshold) {continue;}
      auto points = corners(expand(rect, config.unclip_ratio));
      for (auto & p : points) {
        p.x = std::clamp(p.x * sx, 0.0F, static_cast<float>(source.width - 1));
        p.y = std::clamp(p.y * sy, 0.0F, static_cast<float>(source.height - 1));
      }
      const auto bounds = cv::boundingRect(points);
      if (bounds.width > 1 && bounds.height > 1) {boxes.push_back({points, bounds, score});}
    }
    std::sort(boxes.begin(), boxes.end(),
      [](const Box & a, const Box & b) {return a.score > b.score;});
    std::size_t kept = 0;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
      if (stop.load(std::memory_order_relaxed)) {return;}
      const auto candidate = boxes[i];
      const bool overlap = std::any_of(boxes.begin(), boxes.begin() + kept, [&](const Box & other) {
          const float intersection = static_cast<float>((candidate.bounds & other.bounds).area());
          const float total = static_cast<float>(candidate.bounds.area() + other.bounds.area()) - intersection;
          return total > 0 && intersection / total > config.nms_threshold;
        });
      if (!overlap) {boxes[kept++] = candidate;}
    }
    boxes.resize(kept);
    if (boxes.size() > config.max_text_lines) {
      result.truncated = true; boxes.resize(config.max_text_lines);
    }
  }

  OcrText recognize(const cv::Mat & rgb, const Box & box, const std::atomic_bool & stop)
  {
    const auto & p = box.p;
    const int w = std::max(2, static_cast<int>(std::max(cv::norm(p[1] - p[0]), cv::norm(p[2] - p[3]))));
    const int h = std::max(2, static_cast<int>(std::max(cv::norm(p[3] - p[0]), cv::norm(p[2] - p[1]))));
    const std::array<cv::Point2f, 4> dst{{
      {0, 0}, {static_cast<float>(w - 1), 0},
      {static_cast<float>(w - 1), static_cast<float>(h - 1)}, {0, static_cast<float>(h - 1)}}};
    const auto matrix = cv::getPerspectiveTransform(p.data(), dst.data());
    cv::warpPerspective(rgb, crop, matrix, {w, h});
    const cv::Mat * text_image = &crop;
    if (h > w * 1.5F) {
      cv::rotate(crop, rotated, cv::ROTATE_90_COUNTERCLOCKWISE); text_image = &rotated;
    }
    const int width = std::clamp(static_cast<int>(
      static_cast<double>(text_image->cols) / text_image->rows * config.rec_height), 1, config.rec_width);
    cv::resize(*text_image, resized_rec, {width, config.rec_height}, 0, 0, cv::INTER_LINEAR);
    constexpr float norm = 1.0F / 127.5F;
    constexpr float pad = (127.0F - 127.5F) / 127.5F;
    pack_rgb(resized_rec, rec.input, config.rec_width, config.rec_height,
      {127.5F, 127.5F, 127.5F}, {norm, norm, norm}, {pad, pad, pad});
    OcrText result; result.polygon = box.p;
    if (stop.load(std::memory_order_relaxed)) {return result;}
    auto output = rec.run(memory);
    if (stop.load(std::memory_order_relaxed)) {return result;}
    const auto dims = validate_recognition(output);
    const auto * values = output.GetTensorData<float>();
    result.raw_text.reserve(static_cast<std::size_t>(dims[1]) * 4);
    double sum = 0; int chars = 0, last = -1;
    for (int64_t t = 0; t < dims[1]; ++t) {
      const auto * step = values + t * dims[2];
      if (stop.load(std::memory_order_relaxed)) {return result;}
      int best = 0;
      for (int64_t c = 0; c < dims[2]; ++c) {
        require(std::isfinite(step[c]), "non-finite OCR rec output");
        if (step[c] > step[best]) {best = static_cast<int>(c);}
      }
      if (best != 0 && best != last) {
        double score = step[best];
        if (config.rec_output_logits) {
          double denom = 0;
          for (int64_t c = 0; c < dims[2]; ++c) {denom += std::exp(step[c] - step[best]);}
          score = 1.0 / denom;
        }
        require(std::isfinite(score) && score >= 0 && score <= 1,
          "OCR rec score is not a probability; check rec_output_logits");
        result.raw_text += dictionary[static_cast<std::size_t>(best)];
        sum += score; ++chars;
      }
      last = best;
    }
    result.score = chars > 0 ? static_cast<float>(sum / chars) : 0;
    return result;
  }

  OcrFrame infer(const cv::Mat & rgb, const std::atomic_bool & stop)
  {
    if (rgb.empty() || rgb.type() != CV_8UC3) {throw std::invalid_argument("OCR expects RGB8 image");}
    OcrFrame result;
    const auto cancelled = [&] {
        result.cancelled = stop.load(std::memory_order_relaxed); return result.cancelled;
      };
    if (cancelled()) {return result;}
    const double scale = std::min(static_cast<double>(config.det_width) / rgb.cols,
      static_cast<double>(config.det_height) / rgb.rows);
    const int w = std::clamp(cvRound(rgb.cols * scale), 1, config.det_width);
    const int h = std::clamp(cvRound(rgb.rows * scale), 1, config.det_height);
    cv::Mat content = rgb;
    if (rgb.cols != w || rgb.rows != h) {
      cv::resize(rgb, resized_det, {w, h}, 0, 0, cv::INTER_LINEAR); content = resized_det;
    }
    // Reference CPU pipeline: scale u8 -> subtract RGB mean -> multiply std^-1.
    // Black (u8=0) padding is normalized too, rather than filling tensor padding with zero.
    pack_rgb(content, det.input, config.det_width, config.det_height,
      {0.485F, 0.456F, 0.406F}, {1.0F / 0.229F, 1.0F / 0.224F, 1.0F / 0.225F},
      {-0.485F / 0.229F, -0.456F / 0.224F, -0.406F / 0.225F},
      {1.0F / 255, 1.0F / 255, 1.0F / 255});
    if (cancelled()) {return result;}
    {
      auto output = det.run(memory);
      if (cancelled()) {return result;}
      const auto dims = validate_detection(output);
      cv::Mat probability(static_cast<int>(dims[2]), static_cast<int>(dims[3]), CV_32F,
        const_cast<float *>(output.GetTensorData<float>()));
      detect(probability, w, h, rgb.size(), result, stop);
    }
    result.texts.reserve(boxes.size());
    for (const auto & box : boxes) {
      if (cancelled()) {return result;}
      auto text = recognize(rgb, box, stop);
      if (cancelled()) {return result;}
      ++result.recognized;
      if (!text.raw_text.empty()) {result.texts.push_back(std::move(text));}
    }
    cancelled();
    return result;
  }

  OcrConfig config;
  Ort::Env env;
  Ort::MemoryInfo memory;
  Model det, rec;
  std::vector<std::string> dictionary;
  cv::Mat resized_det, resized_rec, crop, rotated, binary;
  cv::Mat dilation_kernel = cv::Mat::ones(3, 3, CV_8UC1);
  std::vector<uint8_t> mask_storage;
  std::vector<Candidate> candidates;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Box> boxes;
};

OcrEngine::OcrEngine(const OcrConfig & config) : impl_(std::make_unique<Impl>(config)) {}
OcrEngine::~OcrEngine() = default;
OcrFrame OcrEngine::infer(const cv::Mat & rgb8, const std::atomic_bool & stop)
{
  return impl_->infer(rgb8, stop);
}
}  // namespace k1muse_perception
