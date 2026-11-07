#include "k1muse_voice/voice_input_node.hpp"

#include "k1muse_voice/voice_input_core.hpp"
#include "k1muse_voice/voice_kws.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <alsa/asoundlib.h>
#include <asr_service.h>
#include <vad_service.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <k1muse_interfaces/srv/web_request.hpp>
#include <nlohmann/json.hpp>

namespace k1muse_voice
{

namespace
{

using Control = k1muse_interfaces::srv::WebRequest;
using Listening = std_srvs::srv::SetBool;
using Json = nlohmann::json;

constexpr int kSampleRate = 16000;
constexpr unsigned int kChannels = 1U;
constexpr std::size_t kFrameSamples = 512U;
constexpr std::size_t kPreRollFrames = 10U;
constexpr std::size_t kMaxUtteranceSamples = 10U * kSampleRate;

void require_file(const std::filesystem::path & path, const std::string & description)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error(description + " is missing: " + path.string());
  }
}

class VoiceInputNode final : public rclcpp::Node
{
public:
  explicit VoiceInputNode(const rclcpp::NodeOptions & options)
  : Node("voice_input", options),
    utterance_buffer_(kFrameSamples, kPreRollFrames, kMaxUtteranceSamples)
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    enable_perf_stats_ = declare_parameter<bool>("enable_perf_stats", false, startup);
    capture_device_ = declare_parameter<std::string>("capture_device", "plughw:0,0");
    const auto transcript_topic = declare_parameter<std::string>(
      "transcript_topic", "/k1muse/voice/transcript");
    vad_model_dir_ = declare_parameter<std::string>(
      "vad_model_dir", "/home/bianbu/.cache/models/vad/silero");
    trigger_threshold_ = declare_parameter<double>("vad_trigger_threshold", 0.50);
    stop_threshold_ = declare_parameter<double>("vad_stop_threshold", 0.35);
    asr_model_dir_ = declare_parameter<std::string>(
      "asr_model_dir", "/home/bianbu/.cache/models/asr/sensevoice");
    const std::string kws_dir = "/home/bianbu/.cache/models/kws/wenetspeech-3.3m/";
    kws_config_.encoder = declare_parameter<std::string>("kws_encoder_model",
      kws_dir + "encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx");
    kws_config_.decoder = declare_parameter<std::string>("kws_decoder_model",
      kws_dir + "decoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx");
    kws_config_.joiner = declare_parameter<std::string>("kws_joiner_model",
      kws_dir + "joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx");
    kws_config_.tokens = declare_parameter<std::string>("kws_tokens_file", kws_dir + "tokens.txt");
    kws_config_.keywords = declare_parameter<std::string>("kws_keywords_file", "");
    if (kws_config_.keywords.empty()) {
      kws_config_.keywords = ament_index_cpp::get_package_share_directory("k1muse_voice") +
        "/config/wake_keywords.txt";
    }
    kws_config_.threshold = declare_parameter<double>("kws_threshold", 0.25);
    const auto kws_cpu = declare_parameter<int>("kws_cpu", 1);
    const auto command_ms = declare_parameter<int>("wake_command_timeout_ms", 5000);
    if (kws_cpu < 0 || kws_cpu > 3 || command_ms <= 0 ||
      !(kws_config_.threshold > 0.0 && kws_config_.threshold <= 1.0))
    {throw std::invalid_argument("invalid KWS CPU, threshold or command timeout");}
    command_timeout_ = std::chrono::milliseconds(command_ms);
    CPU_ZERO(&kws_affinity_); CPU_SET(kws_cpu, &kws_affinity_);
    const auto asr_cpus = declare_parameter<std::vector<int64_t>>("asr_cpu_ids", {2, 3});
    if (asr_cpus.size() != 2 || asr_cpus[0] == asr_cpus[1]) {
      throw std::invalid_argument("asr_cpu_ids must contain two distinct CPU IDs");
    }
    CPU_ZERO(&asr_affinity_);
    for (const auto cpu : asr_cpus) {
      if (cpu < 0 || cpu > 3 || cpu == kws_cpu) {
        throw std::invalid_argument("ASR must select CPUs 0-3 separately from KWS");
      }
      CPU_SET(cpu, &asr_affinity_);
      if (!asr_ep_affinity_.empty()) {asr_ep_affinity_ += ";";}
      asr_ep_affinity_ += std::to_string(cpu);
    }
    const auto vad_cpu = declare_parameter<int>("vad_cpu", 4);
    if (vad_cpu != 4 && vad_cpu != 5) {
      throw std::invalid_argument("vad_cpu must be 4 or 5");
    }
    CPU_ZERO(&vad_affinity_);
    CPU_SET(vad_cpu, &vad_affinity_);

    validate_parameters(transcript_topic, trigger_threshold_, stop_threshold_);
    validate_models();
    publisher_ = create_publisher<std_msgs::msg::String>(
      transcript_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

    inference_ = create_client<Control>("/k1muse/task/inference/control");
    input_control_ = create_service<Control>("/k1muse/voice/input/control",
      [this](Control::Request::SharedPtr request, Control::Response::SharedPtr response) {
        response->operation_id = request->request_id;
        try {
          if (request->operation == "get_status") {
            std::lock_guard<std::mutex> lock(completion_mutex_);
            response->accepted = true; response->result = Json{{"completion", completion_}}.dump();
            return;
          }
          if (request->operation != "prepare" || request->client_id != "task_manager") {
            throw std::runtime_error("unsupported_operation");
          }
          const auto id = Json::parse(request->arguments).at("work_id").get<std::string>();
          if (id.empty()) {throw std::runtime_error("invalid_work_id");}
          std::lock_guard<std::mutex> lock(handoff_mutex_);
          if (id != prepare_work_) {
            if (!prepare_work_.empty()) {throw std::runtime_error("already_prepared");}
            prepare_work_ = id; prepare_vad_ = true;
          }
          response->accepted = true; response->result = "{\"status\":\"running\"}";
        } catch (const std::exception & error) {response->error = error.what();}
      });
    listening_control_ = create_service<Listening>("/k1muse/voice/input/set_listening",
      [this](std::shared_ptr<rmw_request_id_t> header, Listening::Request::SharedPtr request) {
        std::lock_guard<std::mutex> lock(handoff_mutex_);
        if (listening_header_ || (request->data && !prepared_)) {
          Listening::Response response; response.success = false; response.message = "input_not_ready";
          listening_control_->send_response(*header, response); return;
        }
        listening_header_ = header; listening_target_ = request->data;
      });
    instance_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    try {
      open_capture();

      running_.store(true);
      asr_thread_ = std::thread(&VoiceInputNode::asr_loop, this);
      capture_thread_ = std::thread(&VoiceInputNode::capture_loop, this);
    } catch (...) {
      running_.store(false);
      handoff_cv_.notify_all();
      if (capture_thread_.joinable()) {
        capture_thread_.join();
      }
      if (asr_thread_.joinable()) {
        asr_thread_.join();
      }
      close_capture();
      throw;
    }

    RCLCPP_INFO(
      get_logger(),
      "C14 paused, awaiting manager initialization on %s (16 kHz mono S16_LE); transcript=%s; VAD CPU=%d; ASR EP CPUs=%s; KWS CPU=%d",
      capture_device_.c_str(), transcript_topic.c_str(), static_cast<int>(vad_cpu),
      asr_ep_affinity_.c_str(), static_cast<int>(kws_cpu));
  }

  ~VoiceInputNode() override
  {
    running_.store(false);
    if (capture_handle_ != nullptr) {
      (void)snd_pcm_drop(capture_handle_);
    }
    handoff_cv_.notify_all();
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (asr_thread_.joinable()) {
      asr_thread_.join();
    }
    close_capture();
  }

private:
  void validate_parameters(
    const std::string & transcript_topic, const double trigger_threshold,
    const double stop_threshold) const
  {
    if (capture_device_.empty()) {
      throw std::invalid_argument("capture_device must not be empty");
    }
    if (transcript_topic.empty()) {
      throw std::invalid_argument("transcript_topic must not be empty");
    }
    if (vad_model_dir_.empty() || asr_model_dir_.empty()) {
      throw std::invalid_argument("VAD and ASR model directories must not be empty");
    }
    if (!valid_vad_thresholds(trigger_threshold, stop_threshold)) {
      throw std::invalid_argument(
              "VAD thresholds must satisfy 0 < stop < trigger <= 1");
    }
  }

  void validate_models() const
  {
    require_file(kws_config_.encoder, "KWS encoder");
    require_file(kws_config_.decoder, "KWS decoder");
    require_file(kws_config_.joiner, "KWS joiner");
    require_file(kws_config_.tokens, "KWS tokens");
    require_file(kws_config_.keywords, "KWS keywords");
    require_file(
      std::filesystem::path(vad_model_dir_) / "silero_vad.onnx",
      "Silero VAD model");
    const auto asr_path = std::filesystem::path(asr_model_dir_);
    require_file(asr_path / "model_quant_optimized.onnx", "SenseVoice ONNX model");
    require_file(asr_path / "tokens.txt", "SenseVoice token file");
    require_file(asr_path / "am.mvn", "SenseVoice normalization file");
  }

  void open_capture()
  {
    int result = snd_pcm_open(
      &capture_handle_, capture_device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (result < 0) {
      throw std::runtime_error(
              "failed to open ALSA capture device " + capture_device_ + ": " +
              snd_strerror(result));
    }

    snd_pcm_hw_params_t * parameters = nullptr;
    snd_pcm_hw_params_alloca(&parameters);
    result = snd_pcm_hw_params_any(capture_handle_, parameters);
    if (result >= 0) {
      result = snd_pcm_hw_params_set_access(
        capture_handle_, parameters, SND_PCM_ACCESS_RW_INTERLEAVED);
    }
    if (result >= 0) {
      result = snd_pcm_hw_params_set_format(
        capture_handle_, parameters, SND_PCM_FORMAT_S16_LE);
    }
    if (result >= 0) {
      result = snd_pcm_hw_params_set_channels(capture_handle_, parameters, kChannels);
    }

    unsigned int actual_rate = kSampleRate;
    if (result >= 0) {
      result = snd_pcm_hw_params_set_rate_near(
        capture_handle_, parameters, &actual_rate, nullptr);
    }
    snd_pcm_uframes_t period = kFrameSamples;
    if (result >= 0) {
      result = snd_pcm_hw_params_set_period_size_near(
        capture_handle_, parameters, &period, nullptr);
    }
    if (result >= 0) {
      result = snd_pcm_hw_params(capture_handle_, parameters);
    }
    if (result < 0) {
      throw std::runtime_error(
              std::string("failed to configure ALSA capture: ") + snd_strerror(result));
    }
    if (actual_rate != static_cast<unsigned int>(kSampleRate)) {
      throw std::runtime_error(
              "ALSA capture did not negotiate the required 16000 Hz sample rate");
    }
    result = snd_pcm_prepare(capture_handle_);
    if (result < 0) {
      throw std::runtime_error(
              std::string("failed to prepare ALSA capture: ") + snd_strerror(result));
    }
  }

  void close_capture()
  {
    if (capture_handle_ != nullptr) {
      (void)snd_pcm_close(capture_handle_);
      capture_handle_ = nullptr;
    }
  }

  void initialize_vad()
  {
    auto vad_config = SpacemiT::VadConfig::Preset("silero");
    vad_config.model_dir = vad_model_dir_;
    vad_config.sample_rate = kSampleRate;
    vad_config.window_size = static_cast<int>(kFrameSamples);
    vad_config.trigger_threshold = static_cast<float>(trigger_threshold_);
    vad_config.stop_threshold = static_cast<float>(stop_threshold_);
    vad_config.num_threads = 1;
    vad_ = std::make_unique<SpacemiT::VadEngine>(vad_config);
    if (!vad_ || !vad_->IsInitialized()) {
      throw std::runtime_error("failed to initialize SpaceMIT Silero VAD");
    }

  }

  void initialize_asr()
  {
    auto asr_config = SpacemiT::AsrConfig::Preset("sensevoice");
    asr_config.engine = "sensevoice";
    asr_config.model_dir = asr_model_dir_;
    asr_config.language = "zh";
    asr_config.punctuation = true;
    asr_config.sample_rate = kSampleRate;
    asr_config.ep_intra_thread_affinity = asr_ep_affinity_;
    asr_ = std::make_unique<SpacemiT::AsrEngine>(asr_config);
    if (!asr_ || !asr_->IsInitialized()) {
      throw std::runtime_error("failed to initialize SpaceMIT SenseVoice");
    }
  }

  bool reset_utterance()
  {
    utterance_buffer_.reset();
    try {
      if (vad_) {vad_->Reset();}
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "VAD reset failed: %s", error.what());
      running_.store(false);
      handoff_cv_.notify_all();
      rclcpp::shutdown();
      return false;
    }
  }

  bool read_frame(std::vector<int16_t> * frame)
  {
    std::size_t offset = 0U;
    while (running_.load() && offset < frame->size()) {
      const auto count = snd_pcm_readi(
        capture_handle_, frame->data() + offset, frame->size() - offset);
      if (count > 0) {
        offset += static_cast<std::size_t>(count);
        continue;
      }
      if (count == 0) {
        continue;
      }
      const int recovered = snd_pcm_recover(capture_handle_, static_cast<int>(count), 1);
      if (recovered >= 0) {
        if (!capture_work_.empty()) {end_speech("capture_recovered");}
        else if (!reset_utterance()) {return false;}
        if (kws_ && listening_enabled_.load() && capture_work_.empty()) {kws_->begin();}
        offset = 0U;
        continue;
      }
      if (!running_.load()) {
        return false;
      }
      RCLCPP_ERROR(
        get_logger(), "ALSA capture failed: %s", snd_strerror(recovered));
      running_.store(false);
      handoff_cv_.notify_all();
      rclcpp::shutdown();
      return false;
    }
    return running_.load() && offset == frame->size();
  }

  void bind_capture(const cpu_set_t & mask)
  {
    const int error = pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
    if (error != 0) {throw std::runtime_error(std::strerror(error));}
  }

  void begin_wake()
  {
    bind_capture(kws_affinity_);
    kws_->begin();
    listening_enabled_.store(true);
  }

  void end_speech(const std::string & outcome)
  {
    listening_enabled_.store(false);
    const auto work = std::exchange(capture_work_, {});
    if (!reset_utterance()) {return;}
    if (!work.empty()) {
      send_control("release", {{"work_id", work}, {"outcome", outcome}});
    }
  }

  void capture_loop()
  {
    std::vector<int16_t> frame(kFrameSamples);
    std::vector<float> normalized(kFrameSamples);
    try {
      bind_capture(kws_affinity_);
      while (read_frame(&frame)) {
        bool prepare = false;
        {
          std::lock_guard<std::mutex> lock(handoff_mutex_);
          prepare = prepare_vad_; prepare_vad_ = false;
        }
        if (prepare) {
          try {
            RCLCPP_INFO(get_logger(), "PREPARE stage=KWS event=load_warmup_begin pid=%ld tid=%ld",
              static_cast<long>(::getpid()), static_cast<long>(::syscall(SYS_gettid)));
            kws_ = std::make_unique<VoiceKws>(kws_config_);
            kws_->begin();
            for (int n = 0; n < 32; ++n) {kws_->detect(normalized.data(), normalized.size());}
            kws_->stop();
            RCLCPP_INFO(get_logger(), "PREPARE stage=KWS event=warmup_end");
            bind_capture(vad_affinity_);
            RCLCPP_INFO(get_logger(), "PREPARE stage=VAD event=load_begin");
            initialize_vad();
            RCLCPP_INFO(get_logger(), "PREPARE stage=VAD event=warmup_begin");
            vad_->Detect(normalized, kSampleRate);
            RCLCPP_INFO(get_logger(), "PREPARE stage=VAD event=warmup_end");
            if (!reset_utterance()) {break;}
            std::lock_guard<std::mutex> lock(handoff_mutex_);
            prepare_asr_ = true; handoff_cv_.notify_one();
          } catch (const std::exception & error) {complete_prepare(false, error.what());}
          continue;
        }
        std::shared_ptr<rmw_request_id_t> header;
        bool target = false, replied = false;
        std::string granted_work;
        {
          std::lock_guard<std::mutex> lock(handoff_mutex_);
          header = std::move(listening_header_); target = listening_target_;
          replied = std::exchange(speech_reply_ready_, false);
          if (replied) {granted_work = std::exchange(speech_reply_work_, {});}
        }
        if (header) {
          listening_enabled_.store(false);
          if (kws_) {kws_->stop();}
          if (!capture_work_.empty()) {end_speech("interrupted");}
          const bool reset = reset_utterance();
          wake_allowed_ = target && reset;
          if (wake_allowed_) {begin_wake();}
          Listening::Response response; response.success = reset;
          listening_control_->send_response(*header, response);
          // This frame predates the transition. Resume only with newly read audio.
          if (!replied) {continue;}
        }
        if (replied) {
          if (!granted_work.empty()) {
            capture_work_ = std::move(granted_work);
            if (!reset_utterance()) {break;}
            bind_capture(vad_affinity_);
            waiting_silence_ = true; command_started_ = false;
            command_deadline_ = std::chrono::steady_clock::now() + command_timeout_;
            listening_enabled_.store(true);
          } else if (wake_allowed_) {begin_wake();}
          continue;
        }
        // Paused audio is drained without conversion, KWS or VAD.
        if (!listening_enabled_.load()) {continue;}
        for (std::size_t i = 0; i < frame.size(); ++i) {
          normalized[i] = static_cast<float>(frame[i]) / 32768.0F;
        }
        if (capture_work_.empty()) {
          std::string keyword;
          std::chrono::steady_clock::duration detect_time{};
          if (enable_perf_stats_) {
            const auto started = std::chrono::steady_clock::now();
            keyword = kws_->detect(normalized.data(), normalized.size());
            detect_time = std::chrono::steady_clock::now() - started;
          } else {keyword = kws_->detect(normalized.data(), normalized.size());}
          if (!keyword.empty()) {
            kws_->stop(); listening_enabled_.store(false);
            {
              std::lock_guard<std::mutex> lock(handoff_mutex_);
              pending_work_ = instance_ + "_" + std::to_string(++work_sequence_);
              wake_pending_ = true; handoff_cv_.notify_one();
            }
            if (enable_perf_stats_) {
              RCLCPP_INFO(get_logger(), "[KWS] detect_ms=%.3f keyword=%s",
                std::chrono::duration<double, std::milli>(detect_time).count(),
                Json(keyword).dump(-1, ' ', false, Json::error_handler_t::replace).c_str());
            } else {RCLCPP_INFO(get_logger(), "KWS keyword=\"%s\"", keyword.c_str());}
          }
          continue;
        }
        if (!command_started_ && std::chrono::steady_clock::now() >= command_deadline_) {
          end_speech("no_speech"); continue;
        }
        std::shared_ptr<SpacemiT::VadResult> result;
        try {result = vad_->Detect(normalized, kSampleRate);}
        catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "VAD failed: %s", error.what());
          end_speech("vad_failed"); continue;
        }
        if (!result || !result->IsSuccess()) {end_speech("vad_failed"); continue;}
        const auto state = result->GetState();
        const bool starts = result->IsSpeechStart() || state == SpacemiT::VadState::SPEECH_START;
        const bool active = result->IsSpeech() || state == SpacemiT::VadState::SPEECH;
        const bool ends = result->IsSpeechEnd() || state == SpacemiT::VadState::SPEECH_END;
        if (waiting_silence_) {
          if (!starts && !active && !ends) {waiting_silence_ = false;}
          continue;
        }
        if (starts || active) {command_started_ = true;}
        auto update = utterance_buffer_.push(frame, starts, active, ends);
        if (update.event == SegmentEvent::kDroppedTooLong) {
          end_speech("too_long");
          RCLCPP_WARN(get_logger(), "Dropped speech longer than 10 seconds");
        } else if (update.event == SegmentEvent::kReady) {
          listening_enabled_.store(false);
          if (!reset_utterance()) {break;}
          std::lock_guard<std::mutex> lock(handoff_mutex_);
          pending_work_ = std::exchange(capture_work_, {});
          pending_utterance_ = std::move(update.samples);
          handoff_cv_.notify_one();
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Voice input stopped: %s", error.what());
      running_.store(false); handoff_cv_.notify_all(); rclcpp::shutdown();
    }
  }

  void asr_loop()
  {
    // Only this worker and the SDK threads it creates inherit the ASR mask.
    // The shared ROS executor and capture worker retain their own affinities.
    const int affinity_error = pthread_setaffinity_np(
      pthread_self(), sizeof(asr_affinity_), &asr_affinity_);
    while (true) {
      std::vector<int16_t> samples;
      std::string work;
      {
        std::unique_lock<std::mutex> lock(handoff_mutex_);
        handoff_cv_.wait(
          lock, [this]() {
            return !running_.load() || prepare_asr_ || wake_pending_ || pending_utterance_.has_value();
          });
        if (!running_.load()) {
          pending_utterance_.reset();
          return;
        }
        if (prepare_asr_) {
          prepare_asr_ = false;
          lock.unlock();
          try {
            if (affinity_error != 0) {
              throw std::runtime_error(
                      std::string("cannot set ASR CPU affinity: ") +
                      std::strerror(affinity_error));
            }
            RCLCPP_INFO(get_logger(), "PREPARE stage=ASR event=load_begin pid=%ld tid=%ld",
              static_cast<long>(::getpid()), static_cast<long>(::syscall(SYS_gettid)));
            initialize_asr();
            // One real SDK warmup, excluded from user ASR timing/output.
            RCLCPP_INFO(get_logger(), "PREPARE stage=ASR event=warmup_begin");
            asr_->Recognize(std::vector<int16_t>(kSampleRate, 0), kSampleRate);
            RCLCPP_INFO(get_logger(), "PREPARE stage=ASR event=warmup_end");
            complete_prepare(true, "");
          } catch (const std::exception & error) {complete_prepare(false, error.what());}
          continue;
        }
        work = std::exchange(pending_work_, {});
        if (wake_pending_) {
          wake_pending_ = false;
          lock.unlock();
          const bool granted = request_grant("acquire", work, {{"stage", "speech"}});
          lock.lock();
          speech_reply_work_ = granted ? work : ""; speech_reply_ready_ = true;
          continue;
        }
        samples = std::move(*pending_utterance_);
        pending_utterance_.reset();
      }

      if (!request_grant("speech_done", work)) {
        send_control("release", {{"work_id", work}, {"outcome", "handoff_failed"}});
        continue;
      }
      std_msgs::msg::String message;
      std::string recognition_error;
      double asr_ms = 0.0;
      const auto asr_started = std::chrono::steady_clock::now();
      try {
        const auto result = asr_->Recognize(samples, kSampleRate);
        asr_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - asr_started).count();
        if (result && !result->IsEmpty()) {
          message.data = clean_sensevoice_text(result->GetText());
        }
      } catch (const std::exception & error) {
        asr_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - asr_started).count();
        recognition_error = error.what();
      }

      if (running_.load() && !message.data.empty()) {
        publisher_->publish(message);
      }
      send_control("asr_done", {{"work_id", work}, {"text", message.data}});
      if (!recognition_error.empty()) {
        RCLCPP_ERROR(
          get_logger(), "ASR %.2f ms | error=\"%s\"", asr_ms, recognition_error.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "ASR %.2f ms | result=\"%s\"", asr_ms, message.data.c_str());
      }
    }
  }

  void send_control(const std::string & operation, const Json & args)
  {
    if (!running_.load() || !rclcpp::ok(get_node_base_interface()->get_context())) {return;}
    auto request = std::make_shared<Control::Request>();
    request->client_id = "voice_input"; request->operation = operation;
    request->request_id = instance_ + "_rpc_" + std::to_string(++rpc_sequence_);
    request->arguments = args.dump();
    const auto id = request->request_id;
    {
      std::lock_guard<std::mutex> lock(completion_mutex_);
      // Keep only the latest completion callback. A newer manager-granted work
      // supersedes old diagnostics; do not accumulate callbacks after lost ACKs.
      if (completion_sequence_) {inference_->remove_pending_request(completion_sequence_);completion_sequence_=0;}
      completion_ = {{"request_id", id}, {"work_id", args.value("work_id", "")},
        {"operation", operation}, {"arguments", args}, {"status", "pending"}};
    }
    auto record = [this, id](bool ok, const std::string & error) {
      std::lock_guard<std::mutex> lock(completion_mutex_);
      if (completion_.value("request_id", "") == id) {
        completion_["status"] = ok ? "acknowledged" : "unresolved";
        completion_["error"] = error;
        if (ok) {completion_sequence_=0;}
      }
      if (!ok) {RCLCPP_ERROR(get_logger(), "Inference completion %s unresolved: %s", id.c_str(), error.c_str());}
    };
    if (!inference_->service_is_ready()) {record(false, "manager_unavailable"); return;}
    // One deadline per completion, no polling or automatic resend. The original
    // response callback remains valid after timeout and can resolve a late ACK.
    auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    std::weak_ptr<rclcpp::TimerBase::SharedPtr> weak_timer = timer;
    rclcpp::TimerBase::SharedPtr deadline = create_wall_timer(std::chrono::milliseconds(1500), [weak_timer, record] {
      auto timer = weak_timer.lock();if (!timer) {return;}
      auto deadline = std::atomic_exchange(timer.get(), rclcpp::TimerBase::SharedPtr{});
      if (deadline) {deadline->cancel();}
      record(false, "completion_ack_timeout");
    });
    std::atomic_store(timer.get(), deadline);
    try {
      std::lock_guard<std::mutex> lock(completion_mutex_);
      auto sent = inference_->async_send_request(request, [timer, record, id](rclcpp::Client<Control>::SharedFuture f) {
        auto deadline = std::atomic_exchange(timer.get(), rclcpp::TimerBase::SharedPtr{});
        if (deadline) {deadline->cancel();}
        try {
          const auto response = f.get();
          const bool ok = response->accepted && response->operation_id == id;
          record(ok, ok ? "" : (response->error.empty() ? "completion_ack_mismatch" : response->error));
        } catch (const std::exception & error) {record(false, error.what());}
      });
      completion_sequence_=sent.request_id;
    } catch (const std::exception & error) {
      auto deadline = std::atomic_exchange(timer.get(), rclcpp::TimerBase::SharedPtr{});
      if (deadline) {deadline->cancel();} record(false, error.what());
    }
  }

  void complete_prepare(bool success, const std::string & error)
  {
    std::string work;
    {
      std::lock_guard<std::mutex> lock(handoff_mutex_);
      prepared_ = success; work = prepare_work_;
    }
    send_control("release", {{"work_id", work}, {"outcome", success ? "succeeded" : "failed"},
      {"error", error}});
  }

  bool request_grant(const std::string & operation, const std::string & work, Json args = Json::object())
  {
    if (!running_.load() || !rclcpp::ok(get_node_base_interface()->get_context()) ||
      !inference_->service_is_ready()) {return false;}
    struct Grant
    {
      std::mutex mutex;
      std::condition_variable cv;
      bool ready{false}, abandoned{false}, granted{false};
    };
    auto grant = std::make_shared<Grant>();
    auto request = std::make_shared<Control::Request>();
    request->client_id = "voice_input"; request->operation = operation;
    args["work_id"] = work; request->request_id = work; request->arguments = args.dump();
    inference_->async_send_request(request, [this, grant, work](rclcpp::Client<Control>::SharedFuture f) {
      std::lock_guard<std::mutex> lock(grant->mutex);
      const auto response = f.get();
      const auto result = Json::parse(response->result, nullptr, false);
      grant->granted = response->accepted && result.is_object() && result.value("granted", false);
      grant->ready = true;
      if (grant->abandoned && grant->granted) {
        send_control("release", {{"work_id", work}, {"outcome", "failed"}, {"error", "unused_grant"}});
      }
      grant->cv.notify_one();
    });
    std::unique_lock<std::mutex> lock(grant->mutex);
    if (!grant->cv.wait_for(lock, std::chrono::milliseconds(1500), [&] {return grant->ready;})) {
      grant->abandoned = true; return false;
    }
    return running_.load() && grant->granted;
  }

  bool enable_perf_stats_{false};
  std::string capture_device_;
  std::string vad_model_dir_;
  std::string asr_model_dir_;
  cpu_set_t vad_affinity_{}, kws_affinity_{}, asr_affinity_{};
  KwsConfig kws_config_;
  std::unique_ptr<VoiceKws> kws_;
  std::string asr_ep_affinity_, capture_work_, pending_work_, speech_reply_work_;
  std::chrono::milliseconds command_timeout_{5000};
  std::chrono::steady_clock::time_point command_deadline_;
  bool wake_pending_{false}, speech_reply_ready_{false};
  bool wake_allowed_{false}, command_started_{false};
  snd_pcm_t * capture_handle_{nullptr};
  std::unique_ptr<SpacemiT::VadEngine> vad_;
  std::unique_ptr<SpacemiT::AsrEngine> asr_;
  UtteranceBuffer utterance_buffer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::thread asr_thread_;
  std::mutex handoff_mutex_;
  std::condition_variable handoff_cv_;
  std::optional<std::vector<int16_t>> pending_utterance_;
  bool prepared_{false}, prepare_vad_{false}, prepare_asr_{false};
  bool listening_target_{false}, waiting_silence_{true};
  double trigger_threshold_{0.5}, stop_threshold_{0.35};
  std::atomic<bool> listening_enabled_{false};
  std::string prepare_work_, instance_;
  uint64_t work_sequence_{0};
  std::atomic<uint64_t> rpc_sequence_{0};
  std::mutex completion_mutex_;
  Json completion_ = Json::object();
  int64_t completion_sequence_{0};
  std::shared_ptr<rmw_request_id_t> listening_header_;
  rclcpp::Client<Control>::SharedPtr inference_;
  rclcpp::Service<Control>::SharedPtr input_control_;
  rclcpp::Service<Listening>::SharedPtr listening_control_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_voice_input_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<VoiceInputNode>(options);
}

}  // namespace k1muse_voice
