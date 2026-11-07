#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>

#include "k1muse_manager/voice_command_node.hpp"
#include "k1muse_perception/yolo_detector_node.hpp"
#include "k1muse_perception/ocr_node.hpp"
#include "k1muse_voice/voice_input_node.hpp"

namespace
{
struct Arguments
{
  bool enable_voice{true};
  bool enable_yolo{true};
  bool enable_ocr{false};
  std::string input_params;
  std::string command_params;
  std::string yolo_params;
  std::string ocr_params;
  std::string config_dir;
  std::string voice_startup_mode;
};

bool boolean(const std::string & value)
{
  if (value == "true") {return true;}
  if (value == "false") {return false;}
  throw std::invalid_argument("enable flags must be true or false");
}

Arguments read_arguments(int argc, char ** argv)
{
  Arguments result;
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  for (std::size_t i = 1; i < args.size(); i += 2) {
    if (i + 1 == args.size()) {throw std::invalid_argument("missing value for " + args[i]);}
    const auto & key = args[i];
    const auto & value = args[i + 1];
    if (key == "--enable-voice") {result.enable_voice = boolean(value);}
    else if (key == "--enable-yolo") {result.enable_yolo = boolean(value);}
    else if (key == "--enable-ocr") {result.enable_ocr = boolean(value);}
    else if (key == "--voice-input-params") {result.input_params = value;}
    else if (key == "--voice-command-params") {result.command_params = value;}
    else if (key == "--yolo-params") {result.yolo_params = value;}
    else if (key == "--ocr-params") {result.ocr_params = value;}
    else if (key == "--voice-config-dir") {result.config_dir = value;}
    else if (key == "--voice-startup-mode") {result.voice_startup_mode = value;}
    else {throw std::invalid_argument("unknown argument: " + key);}
  }
  if (result.enable_voice && (result.input_params.empty() || result.command_params.empty() ||
    result.config_dir.empty()))
  {
    throw std::invalid_argument("voice requires parameter files and config directory");
  }
  if (result.enable_yolo && result.yolo_params.empty()) {
    throw std::invalid_argument("YOLO requires its parameter file");
  }
  if (result.enable_ocr && (!result.enable_yolo || result.ocr_params.empty())) {
    throw std::invalid_argument("OCR requires enable_yolo=true and its parameter file");
  }
  return result;
}

rclcpp::NodeOptions node_options(const std::string & params)
{
  rclcpp::NodeOptions options;
  // A file (including wildcard overrides) belongs only to this node. Process-wide
  // launch remaps must not rename all three nodes or mix their configurations.
  options.use_global_arguments(false);
  options.arguments({"--ros-args", "--params-file", params});
  return options;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto args = read_arguments(argc, argv);
    if (!args.enable_voice && !args.enable_yolo) {
      rclcpp::shutdown();
      return 0;
    }
    // Process-global OpenCV setting: set once, before any model/compute worker.
    if (args.enable_ocr) {cv::setNumThreads(1);}
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Node::SharedPtr input;
    std::shared_ptr<k1muse_perception::OcrNode> ocr;
    std::shared_ptr<k1muse_perception::YoloDetectorNode> yolo;
    if (args.enable_ocr) {
      ocr = std::make_shared<k1muse_perception::OcrNode>(node_options(args.ocr_params));
      executor.add_node(ocr);
    }
    if (args.enable_voice) {
      input = k1muse_voice::make_voice_input_node(node_options(args.input_params));
      executor.add_node(input);
    }
    if (args.enable_yolo) {
      yolo = std::make_shared<k1muse_perception::YoloDetectorNode>(node_options(args.yolo_params), ocr);
      executor.add_node(yolo->get_node_base_interface());
    }

    if (args.enable_voice) {
      auto options = node_options(args.command_params);
      const std::filesystem::path configs(args.config_dir);
      options.parameter_overrides({
        rclcpp::Parameter("startup_mode_override", args.voice_startup_mode),
        rclcpp::Parameter("intent_config_file", (configs / "voice_intent.yaml").string()),
        rclcpp::Parameter("cloud_config_file", (configs / "cloud_api.yaml").string()),
        rclcpp::Parameter("cloud_prompt_file", (configs / "cloud_prompt.yaml").string()),
        rclcpp::Parameter("local_config_file", (configs / "local_llm.yaml").string()),
        rclcpp::Parameter("local_prompt_file", (configs / "local_prompt.yaml").string())});
      std::string prepare_work;
      nlohmann::json catalog;
      while (rclcpp::ok()) {
        bool restart = false;
        auto handoff = std::exchange(prepare_work, {});
        auto command = k1muse_manager::make_voice_command_node(options, {},
          [&](const std::string & work,const nlohmann::json & snapshot) {
            prepare_work = work;
            catalog = snapshot;
            restart = true;
            executor.cancel();
          }, std::move(handoff),catalog);
        executor.add_node(command);
        executor.spin();
        executor.remove_node(command);
        // Preserve the existing restart handoff. Input/YOLO stay alive while the
        // old intent worker joins; the manager retains its restart permit.
        command.reset();
        if (!restart) {break;}
      }
    } else {
      executor.spin();
    }
    if (yolo) {executor.remove_node(yolo->get_node_base_interface());}
    if (ocr) {executor.remove_node(ocr); ocr->shutdown();}
    if (input) {executor.remove_node(input);}
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(rclcpp::get_logger("ai_inference"), "AI startup failed: %s", error.what());
      rclcpp::shutdown();
      return 1;
    }
  }
  rclcpp::shutdown();
  return 0;
}
