#pragma once
#include "k1muse_manager/warehouse_config.hpp"
#include <nlohmann/json.hpp>
#include <map>
#include <optional>
#include <string>

namespace k1muse_manager
{
class InventoryStore
{
public:
  using Json=nlohmann::json;
  void load_inventory(const std::string & warehouse_path,const WarehouseTable & warehouse);
  Json inventory_state(const WarehouseTable &) const;
  std::optional<int> effective_quantity(const WarehouseTarget &) const;
  Json commit_scan_layer(const std::string & shelf,int layer,const std::string & drug,int count);
  Json record_incomplete_layer(const std::string & shelf,int layer,const std::string & drug,const std::string & reason);
  Json set_inventory_quantity(const std::string & shelf,int layer,const std::string & drug,int count);
  Json apply_fetch_inventory_delta(const WarehouseTarget &);
  bool save_inventory();
  bool persisted() const {return !dirty_ && load_error_.empty();}
  const std::string & error() const {return load_error_.empty() ? save_error_ : load_error_;}

private:
  Json & row(const std::string &,int,const std::string &);
  Json update_result(const Json &) const;
  std::string path_,map_id_,load_error_,save_error_;
  uint64_t revision_{0};
  bool dirty_{false};
  std::map<std::pair<std::string,int>,Json> rows_;
};
}  // namespace k1muse_manager
