#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace k1muse_manager
{
struct VoiceDrug
{
  std::string name;
  std::vector<std::string> aliases;
};

std::vector<VoiceDrug> parse_drug_catalog(const nlohmann::json & document);
std::vector<VoiceDrug> load_drug_catalog(const std::string & path);
nlohmann::json drug_catalog_json(const std::vector<VoiceDrug> & drugs);
}  // namespace k1muse_manager
