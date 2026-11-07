#ifndef K1MUSE_MANAGER__VOICE_INTENT_HPP_
#define K1MUSE_MANAGER__VOICE_INTENT_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>
#include "k1muse_manager/voice_local.hpp"

namespace k1muse_manager
{
using IntentJson = nlohmann::json;

struct VoiceIntent
{
  std::string action;
  std::string drug;
  int64_t quantity{0};
  IntentJson json() const;
};

VoiceIntent parse_voice_intent(const std::string & text);

struct CloudConfig
{
  std::string api_type{"deepseek"};
  std::string base_url{"https://api.deepseek.com"};
  std::string model{"deepseek-flash"};
  std::string api_key;
  int connect_timeout_ms{2000};
  int request_timeout_ms{8000};
  int max_tokens{256};
  IntentJson public_json() const;
  bool operator==(const CloudConfig & other) const;
};

struct VoiceLocalConfig
{
  VoiceLocalConfig(const std::string & config_path, const std::string & prompt_path);
  LocalConfig config;
  std::string prompt;
  std::string error;
};

class VoiceStartupConfig
{
public:
  explicit VoiceStartupConfig(std::string path);
  std::string mode{"rules"};
  std::string error;
  void save_mode(const std::string & value);

private:
  std::string path_;
};

// Runtime loads once. Only the web's one-shot file helper calls the save methods.
class VoiceCloudConfig
{
public:
  VoiceCloudConfig(std::string api_path, std::string prompt_path);
  CloudConfig api;
  std::string prompt;
  std::string api_error;
  std::string prompt_error;
  void save_api(const IntentJson & fields);
  void save_prompt(const IntentJson & fields);
  bool ready() const;

private:
  std::string api_path_;
  std::string prompt_path_;
};

struct CloudJob
{
  uint64_t generation{0};
  std::string id;
  std::string text;
  std::string system_prompt;
  CloudConfig config;
  bool probe{false};
  bool activate{false};
  std::string backend{"cloud"};
  LocalConfig local;
};

struct CloudReply
{
  long http_status{0};
  std::string body;
  std::string error;
};

// A callable seam permits isolated tests without a test-only ROS backend mode.
using CloudTransport = std::function<CloudReply(const CloudJob &, const std::atomic_bool &)>;
IntentJson cloud_request_json(const CloudJob & job);
const std::string & voice_intent_schema();

class CloudHttpClient
{
public:
  static bool initialize();
  explicit CloudHttpClient(std::string ca_file = {});
  ~CloudHttpClient();
  CloudHttpClient(const CloudHttpClient &) = delete;
  CloudHttpClient & operator=(const CloudHttpClient &) = delete;
  CloudReply perform(const CloudJob & job, const std::atomic_bool & canceled);

private:
  void * handle_{nullptr};
  std::string ca_file_;
};

struct CloudResult
{
  CloudJob job;
  CloudReply reply;
  double duration_ms{0};
  std::optional<VoiceIntent> intent;  // Local body parsed once before resource handoff.
};

// One request/result slot. Only take_result() releases the admission slot.
class VoiceCloudWorker
{
public:
  explicit VoiceCloudWorker(CloudTransport transport = {});
  ~VoiceCloudWorker();
  bool start(CloudJob job);
  void cancel(bool release_transport = false);
  void stop();
  std::optional<CloudResult> take_result();
  bool busy() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace k1muse_manager
#endif  // K1MUSE_MANAGER__VOICE_INTENT_HPP_
