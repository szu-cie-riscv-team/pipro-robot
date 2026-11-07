#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#include "k1muse_perception/yolo_postprocess.hpp"
#include "k1muse_perception/yolo_preprocess.hpp"

namespace k1muse_perception
{

struct YoloEngineConfig
{
  std::string model_path;
  std::string labels_path;
  std::string output_format;
  PreprocessConfig preprocess;
  PostprocessConfig postprocess;
  int ort_intra_threads{1};
  int ort_inter_threads{1};
  int ep_intra_threads{3};
  int ep_inter_threads{1};
  std::string ep_intra_thread_affinity{"1;2;3"};
};

struct YoloFrameTiming
{
  int64_t pre_ns{0};
  int64_t infer_ns{0};
  int64_t post_ns{0};
  std::chrono::steady_clock::time_point finished_at{};
  int64_t track_ns{0};
  int64_t callback_ns{0};
  int64_t input_age_ns{-1}, output_age_ns{-1};  // ROS source age, separate from steady-clock cost.
};

class YoloEngine
{
public:
  YoloEngine();
  ~YoloEngine();

  YoloEngine(const YoloEngine &) = delete;
  YoloEngine & operator=(const YoloEngine &) = delete;

  bool configure(const YoloEngineConfig & config, std::string & error);
  void reset();

  bool infer(
    const cv::Mat & rgb8, std::vector<vision_msgs::msg::Detection2D> & detections,
    std::string & error);
  bool infer_with_timing(
    const cv::Mat & rgb8, std::vector<vision_msgs::msg::Detection2D> & detections,
    YoloFrameTiming & timing, std::string & error);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_perception
