#include "k1muse_manager/voice_intent.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace k1muse_manager
{
namespace
{
void require(bool value, const char * error = "invalid_config")
{
  if (!value) {throw std::invalid_argument(error);}
}

bool nonblank(const std::string & value)
{
  return value.find_first_not_of(" \t\r\n") != std::string::npos;
}

int64_t integer(const IntentJson & value, const char * error)
{
  require(value.is_number_integer(), error);
  require(!value.is_number_unsigned() ||
    value.get<uint64_t>() <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max()), error);
  return value.get<int64_t>();
}

void validate(const CloudConfig & c)
{
  require(c.api_type == "deepseek" || c.api_type == "openai_compatible");
  require(c.base_url.compare(0, 8, "https://") == 0 && c.base_url.size() > 8 &&
    c.base_url.find_first_of("?#\r\n ") == std::string::npos);
  require(nonblank(c.model) && c.api_key.find_first_of("\r\n") == std::string::npos);
  require(c.connect_timeout_ms >= 1000 && c.connect_timeout_ms <= 5000 &&
    c.request_timeout_ms >= c.connect_timeout_ms && c.request_timeout_ms <= 30000);
  require(c.max_tokens >= 64 && c.max_tokens <= 1024);
  // Validate UTF-8 at the configuration boundary, not on every audio frame.
  (void)IntentJson{{"model", c.model}, {"key", c.api_key}}.dump();
}

void validate_prompt(const std::string & prompt)
{
  require(nonblank(prompt) && prompt.size() <= 16384);
  (void)IntentJson(prompt).dump();
}

void atomic_write(const std::string & path, const std::string & text)
{
  require(!path.empty(), "config_write_failed");
  const auto parent = std::filesystem::path(path).parent_path();
  std::error_code error;
  if (!parent.empty()) {std::filesystem::create_directories(parent, error);}
  if (error) {throw std::runtime_error("config_write_failed");}
  std::string temporary = path + ".tmp.XXXXXX";
  int fd = ::mkstemp(temporary.data());  // 0600, including the credentials file.
  if (fd < 0) {throw std::runtime_error("config_write_failed");}
  std::size_t offset = 0;
  while (offset < text.size()) {
    const auto count = ::write(fd, text.data() + offset, text.size() - offset);
    if (count < 0 && errno == EINTR) {continue;}
    if (count <= 0) {
      ::close(fd); ::unlink(temporary.c_str());
      throw std::runtime_error("config_write_failed");
    }
    offset += static_cast<std::size_t>(count);
  }
  if (::close(fd) != 0 || ::rename(temporary.c_str(), path.c_str()) != 0) {
    ::unlink(temporary.c_str());
    throw std::runtime_error("config_write_failed");
  }
}

std::string template_path(const std::string & path)
{
  auto result = std::filesystem::path(path);
  const auto name = result.stem().string() + ".default.yaml";
  const auto sibling = result.parent_path() / name;
  if (std::filesystem::exists(sibling)) {return sibling.string();}
  return ament_index_cpp::get_package_share_directory("k1muse_voice") + "/config/" + name;
}

YAML::Node load_config(const std::string & path)
{
  require(!path.empty(), "unconfigured");
  return YAML::LoadFile(std::filesystem::exists(path) ? path : template_path(path));
}

CloudConfig from_yaml(const YAML::Node & value)
{
  require(value.IsMap());
  CloudConfig c;
  c.api_type = value["api_type"].as<std::string>();
  c.base_url = value["base_url"].as<std::string>();
  c.model = value["model"].as<std::string>();
  c.api_key = value["api_key"].as<std::string>();
  c.connect_timeout_ms = value["connect_timeout_ms"].as<int>();
  c.request_timeout_ms = value["request_timeout_ms"].as<int>();
  c.max_tokens = value["max_tokens"].as<int>();
  validate(c);
  return c;
}

LocalConfig local_from_yaml(const YAML::Node & node)
{
  require(node.IsMap());
  LocalConfig c;
  c.model_path = node["model_path"].as<std::string>();
  // Legacy process settings are ignored. The combined process owns CPU0-3.
  if (node["cpu_ids"]) {
    require(node["cpu_ids"].as<std::vector<int>>() == std::vector<int>({0, 1, 2, 3}));
  }
  c.threads = node["threads"].as<int>();
  c.context_size = node["context_size"].as<int>();
  c.batch_size = node["batch_size"].as<int>();
  c.ubatch_size = node["ubatch_size"].as<int>();
  c.mmap = node["mmap"].as<bool>();
  c.flash_attention = node["flash_attention"].as<std::string>();
  c.poll_level = node["poll_level"].as<int>();
  c.max_tokens = node["max_tokens"].as<int>();
  c.startup_timeout_ms = node["startup_timeout_ms"].as<int>();
  c.request_timeout_ms = node["request_timeout_ms"].as<int>();
  require(!c.model_path.empty() && c.model_path.front() == '/' && c.threads >= 1 &&
    c.threads <= 4 && c.context_size >= 512 &&
    c.batch_size >= 1 && c.ubatch_size >= 1 && c.ubatch_size <= c.batch_size &&
    c.max_tokens >= 32 && c.max_tokens <= 256 && c.poll_level >= 0 &&
    c.poll_level <= 1 && c.startup_timeout_ms >= 1000 &&
    c.request_timeout_ms >= 1000 &&
    (c.flash_attention == "auto" || c.flash_attention == "on" || c.flash_attention == "off"));
  return c;
}
}  // namespace

IntentJson VoiceIntent::json() const
{
  return {{"action", action}, {"drug", drug}, {"quantity", quantity}};
}

VoiceIntent parse_voice_intent(const std::string & text)
{
  const auto j = IntentJson::parse(text, nullptr, false);
  require(j.is_object() && j.size() == 3 && j.contains("action") && j.contains("drug") &&
    j.contains("quantity") && j["action"].is_string() && j["drug"].is_string(),
    "invalid_intent_format");
  VoiceIntent intent{j["action"].get<std::string>(), j["drug"].get<std::string>(),
    integer(j["quantity"], "invalid_intent_format")};
  require((intent.action == "fetch" && nonblank(intent.drug) && intent.quantity > 0) ||
    (intent.action == "survey" && (intent.drug.empty() || nonblank(intent.drug)) && intent.quantity == 0) ||
    (intent.action == "unknown" && intent.drug.empty() && intent.quantity == 0), "invalid_intent_format");
  return intent;
}

IntentJson CloudConfig::public_json() const
{
  return {{"api_type", api_type}, {"base_url", base_url}, {"model", model},
    {"api_key_set", !api_key.empty()}, {"connect_timeout_ms", connect_timeout_ms},
    {"request_timeout_ms", request_timeout_ms}, {"max_tokens", max_tokens}};
}

bool CloudConfig::operator==(const CloudConfig & other) const
{
  return std::tie(api_type, base_url, model, api_key, connect_timeout_ms, request_timeout_ms,
           max_tokens) ==
         std::tie(other.api_type, other.base_url, other.model, other.api_key,
           other.connect_timeout_ms, other.request_timeout_ms, other.max_tokens);
}

VoiceStartupConfig::VoiceStartupConfig(std::string path) : path_(std::move(path))
{
  // Legacy direct rule-only callers need no writable configuration directory.
  if (path_.empty()) {return;}
  try {
    mode = load_config(path_)["startup_mode"].as<std::string>();
    require(mode == "rules" || mode == "cloud" || mode == "local");
  } catch (...) {mode = "rules"; error = "invalid_config";}
}

void VoiceStartupConfig::save_mode(const std::string & value)
{
  require(value == "rules" || value == "cloud" || value == "local");
  if (value == mode && error.empty() && !path_.empty() && std::filesystem::exists(path_)) {
    return;
  }
  YAML::Node node; node["startup_mode"] = value;
  YAML::Emitter out; out << node;
  atomic_write(path_, out.c_str());
  mode = value; error.clear();
}

VoiceCloudConfig::VoiceCloudConfig(std::string api_path, std::string prompt_path)
: prompt("将用户指令转换为 action、drug、quantity 三字段 JSON。"),
  api_path_(std::move(api_path)), prompt_path_(std::move(prompt_path))
{
  try {
    api = from_yaml(load_config(api_path_)["cloud"]);
  } catch (...) {
    api_error = api_path_.empty() ? "unconfigured" : "invalid_config";
    try {api = from_yaml(YAML::LoadFile(template_path(api_path_))["cloud"]);} catch (...) {}
  }
  try {
    auto value = load_config(prompt_path_)["system_prompt"].as<std::string>();
    validate_prompt(value);
    prompt = std::move(value);
  } catch (...) {
    prompt_error = prompt_path_.empty() ? "unconfigured" : "invalid_config";
    try {
      auto value = YAML::LoadFile(template_path(prompt_path_))["system_prompt"].as<std::string>();
      validate_prompt(value); prompt = std::move(value);
    } catch (...) {}
  }
}

VoiceLocalConfig::VoiceLocalConfig(
  const std::string & config_path, const std::string & prompt_path)
{
  try {
    config = local_from_yaml(load_config(config_path)["local"]);
    prompt = load_config(prompt_path)["system_prompt"].as<std::string>();
    validate_prompt(prompt);
  } catch (...) {error = config_path.empty() || prompt_path.empty() ? "unconfigured" : "invalid_config";}
}

bool VoiceCloudConfig::ready() const
{
  return api_error.empty() && prompt_error.empty() && !api.api_key.empty();
}

void VoiceCloudConfig::save_api(const IntentJson & fields)
{
  require(fields.is_object() && (api_error.empty() || fields.contains("api_key")));
  CloudConfig candidate = api;
  try {
    candidate.api_type = fields.at("api_type").get<std::string>();
    candidate.base_url = fields.at("base_url").get<std::string>();
    candidate.model = fields.at("model").get<std::string>();
    const auto connect = integer(fields.at("connect_timeout_ms"), "invalid_config");
    const auto total = integer(fields.at("request_timeout_ms"), "invalid_config");
    const auto tokens = integer(fields.at("max_tokens"), "invalid_config");
    require(connect >= 1000 && connect <= 5000 && total >= connect && total <= 30000 &&
      tokens >= 64 && tokens <= 1024);
    candidate.connect_timeout_ms = static_cast<int>(connect);
    candidate.request_timeout_ms = static_cast<int>(total);
    candidate.max_tokens = static_cast<int>(tokens);
    if (fields.contains("api_key")) {candidate.api_key = fields.at("api_key").get<std::string>();}
    validate(candidate);
  } catch (...) {throw std::invalid_argument("invalid_config");}
  YAML::Node value;
  value["cloud"]["api_type"] = candidate.api_type;
  value["cloud"]["base_url"] = candidate.base_url;
  value["cloud"]["model"] = candidate.model;
  value["cloud"]["api_key"] = candidate.api_key;
  value["cloud"]["connect_timeout_ms"] = candidate.connect_timeout_ms;
  value["cloud"]["request_timeout_ms"] = candidate.request_timeout_ms;
  value["cloud"]["max_tokens"] = candidate.max_tokens;
  YAML::Emitter out; out << value;
  atomic_write(api_path_, out.c_str());
  api = std::move(candidate);
  api_error.clear();
}

void VoiceCloudConfig::save_prompt(const IntentJson & fields)
{
  std::string value;
  try {value = fields.at("system_prompt").get<std::string>(); validate_prompt(value);}
  catch (...) {throw std::invalid_argument("invalid_config");}
  YAML::Node node; node["system_prompt"] = value;
  YAML::Emitter out; out << node;
  atomic_write(prompt_path_, out.c_str());
  prompt = std::move(value);
  prompt_error.clear();
}

IntentJson cloud_request_json(const CloudJob & job)
{
  IntentJson body{{"model", job.config.model},
    {"messages", {{{"role", "system"}, {"content", job.system_prompt}},
      {{"role", "user"}, {"content", job.text}}}},
    {"stream", false}, {"temperature", 0}, {"max_tokens", job.config.max_tokens},
    {"response_format", {{"type", "json_object"}}}};
  if (job.config.api_type == "deepseek") {body["thinking"] = {{"type", "disabled"}};}
  return body;
}

const std::string & voice_intent_schema()
{
  static const IntentJson schema = {{"type", "object"}, {"properties", {
    {"action", {{"type", "string"}, {"enum", {"fetch", "survey", "unknown"}}}},
    {"drug", {{"type", "string"}}},
    {"quantity", {{"type", "integer"}, {"minimum", 0}}}}},
    {"required", {"action", "drug", "quantity"}}, {"additionalProperties", false}};
  static const std::string text = schema.dump();
  return text;
}
}  // namespace k1muse_manager
