#include "k1muse_manager/drug_catalog.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>

namespace k1muse_manager
{
namespace
{
using Json = nlohmann::json;
std::string name(const Json & value)
{
  if (!value.is_string()) {throw std::invalid_argument("drug_catalog_invalid: string required");}
  const auto s=value.get<std::string>();
  const auto first=s.find_first_not_of(" \t\r\n\f\v");
  if(first==std::string::npos) {throw std::invalid_argument("drug_catalog_invalid: empty name");}
  return s.substr(first,s.find_last_not_of(" \t\r\n\f\v")-first+1);
}
void keys(const YAML::Node & node,const std::set<std::string> & allowed)
{
  if(!node.IsMap()) {throw std::invalid_argument("drug_catalog_invalid: object required");}
  std::set<std::string> seen;
  for(const auto & entry:node) {
    const auto key=entry.first.as<std::string>();
    if(!allowed.count(key) || !seen.insert(key).second) {
      throw std::invalid_argument("drug_catalog_invalid: unknown or duplicate key");
    }
  }
}
std::string scalar(const YAML::Node & node)
{
  if(!node.IsScalar()) {throw std::invalid_argument("drug_catalog_invalid: string required");}
  if(node.Tag()!="!" && node.Tag()!="tag:yaml.org,2002:str") {
    bool b;double d;
    if(YAML::convert<bool>::decode(node,b) || YAML::convert<double>::decode(node,d)) {
      throw std::invalid_argument("drug_catalog_invalid: string required");
    }
  }
  return node.as<std::string>();
}
}

std::vector<VoiceDrug> parse_drug_catalog(const Json & document)
{
  if(!document.is_object() || document.size()!=1 || !document.contains("drugs") || !document.at("drugs").is_array()) {
    throw std::invalid_argument("drug_catalog_invalid: drugs array required");
  }
  std::vector<VoiceDrug> drugs;
  std::map<std::string,std::string> owners;
  for(const auto & item:document.at("drugs")) {
    if(!item.is_object() || !item.contains("name") || item.size()!=(item.contains("aliases")?2u:1u)) {
      throw std::invalid_argument("drug_catalog_invalid: name/aliases required");
    }
    VoiceDrug drug{name(item.at("name")),{}};
    if(owners.count(drug.name)) {throw std::invalid_argument("drug_catalog_invalid: name conflict");}
    owners.emplace(drug.name,drug.name);
    if(item.contains("aliases")) {
      if(!item.at("aliases").is_array()) {throw std::invalid_argument("drug_catalog_invalid: aliases array required");}
      for(const auto & value:item.at("aliases")) {
        auto alias=name(value);const auto found=owners.find(alias);
        if(found!=owners.end()) {
          if(found->second!=drug.name) {throw std::invalid_argument("drug_catalog_invalid: alias conflict");}
          continue;
        }
        owners.emplace(alias,drug.name);drug.aliases.push_back(std::move(alias));
      }
    }
    drugs.push_back(std::move(drug));
  }
  return drugs;
}

std::vector<VoiceDrug> load_drug_catalog(const std::string & path)
{
  auto source=std::filesystem::path(path);
  if(path.empty() || !std::filesystem::exists(source)) {
    source=std::filesystem::path(ament_index_cpp::get_package_share_directory("k1muse_manager"))/"config/drug_catalog.default.yaml";
  }
  std::ifstream file(source,std::ios::binary);
  if(!file) {throw std::runtime_error("drug_catalog_read_failed");}
  std::string raw(65537,'\0');file.read(raw.data(),raw.size());raw.resize(file.gcount());
  if(file.bad() || raw.size()>65536) {throw std::runtime_error("drug_catalog_read_failed_or_too_large");}
  const auto doc=YAML::Load(raw);keys(doc,{"drugs"});
  if(!doc["drugs"].IsSequence()) {throw std::invalid_argument("drug_catalog_invalid: drugs array required");}
  Json list=Json::array();
  for(const auto & item:doc["drugs"]) {
    keys(item,{"name","aliases"});
    if(!item["name"].IsScalar()) {throw std::invalid_argument("drug_catalog_invalid: name required");}
    Json aliases=Json::array();
    if(item["aliases"]) {
      if(!item["aliases"].IsSequence()) {throw std::invalid_argument("drug_catalog_invalid: aliases array required");}
      for(const auto & alias:item["aliases"]) {
        if(!alias.IsScalar()) {throw std::invalid_argument("drug_catalog_invalid: alias string required");}
        aliases.push_back(scalar(alias));
      }
    }
    list.push_back({{"name",scalar(item["name"])},{"aliases",aliases}});
  }
  return parse_drug_catalog({{"drugs",list}});
}

nlohmann::json drug_catalog_json(const std::vector<VoiceDrug> & drugs)
{
  Json list=Json::array();
  for(const auto & drug:drugs) {list.push_back({{"name",drug.name},{"aliases",drug.aliases}});}
  return list;
}
}  // namespace k1muse_manager
