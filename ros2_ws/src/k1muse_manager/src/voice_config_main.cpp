// One explicit file operation. No ROS initialization, worker, model or network.
#include "k1muse_manager/voice_intent.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>

int main(int argc, char ** argv)
{
  using namespace k1muse_manager;
  try {
    if (argc != 3) {throw std::invalid_argument("invalid_config");}
    std::string input;
    char buffer[4096];
    while (std::cin.read(buffer, sizeof(buffer)) || std::cin.gcount()) {
      input.append(buffer, static_cast<std::size_t>(std::cin.gcount()));
      if (input.size() > 65536) {throw std::invalid_argument("invalid_config");}
    }
    std::vector<std::set<std::string>> keys;
    const auto fields = IntentJson::parse(input, [&keys](int, IntentJson::parse_event_t event, IntentJson & item) {
      if (event == IntentJson::parse_event_t::object_start) {keys.emplace_back();}
      if (event == IntentJson::parse_event_t::key && !keys.back().insert(item.get<std::string>()).second) {
        throw std::invalid_argument("invalid_config");
      }
      if (event == IntentJson::parse_event_t::object_end) {keys.pop_back();}
      return true;
    });
    if (!fields.is_object()) {throw std::invalid_argument("invalid_config");}
    const std::filesystem::path directory(argv[1]);
    const std::string operation(argv[2]);
    VoiceStartupConfig startup((directory / "voice_intent.yaml").string());
    VoiceCloudConfig cloud((directory / "cloud_api.yaml").string(), (directory / "cloud_prompt.yaml").string());
    if (operation == "voice_intent_save_cloud") {cloud.save_api(fields);}
    else if (operation == "voice_intent_save_prompt") {cloud.save_prompt(fields);}
    else if (operation == "voice_intent_set_mode") {startup.save_mode(fields.at("mode").get<std::string>());}
    else if (operation == "voice_intent_set_cloud_enabled") {
      startup.save_mode(fields.at("enabled").get<bool>() ? "cloud" :
        startup.mode == "cloud" ? "rules" : startup.mode);
    } else if (operation != "voice_intent_get" || !fields.empty()) {
      throw std::invalid_argument("invalid_config");
    }
    const IntentJson result{{"startup_mode", startup.mode}, {"cloud", cloud.api.public_json()},
      {"system_prompt", cloud.prompt}, {"saved", operation != "voice_intent_get"},
      {"config_errors", {{"startup_mode", startup.error}, {"cloud", cloud.api_error}, {"system_prompt", cloud.prompt_error}}}};
    std::cout << result.dump() << '\n';
    return 0;
  } catch (const std::exception & error) {
    // YAML/JSON exceptions may include input (including secrets). Return codes only.
    const std::string code = std::string(error.what()) == "config_write_failed" ?
      "config_write_failed" : "invalid_config";
    std::cout << IntentJson{{"error", code}}.dump() << '\n';
    return 1;
  }
}
