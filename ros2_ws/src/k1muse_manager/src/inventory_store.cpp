#include "k1muse_manager/inventory_store.hpp"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <unistd.h>

namespace k1muse_manager
{
namespace
{
using Json=nlohmann::json;
std::string utc()
{
  const auto now=std::time(nullptr);std::tm tm{};gmtime_r(&now,&tm);
  char result[32];std::strftime(result,sizeof(result),"%Y-%m-%dT%H:%M:%SZ",&tm);return result;
}
bool integer(const Json & value,int64_t maximum)
{return value.is_number_integer() && value.get<double>()>=0 && value.get<double>()<=maximum;}
Json empty_row(const std::string & shelf,int layer,const std::string & drug)
{
  return {{"shelf_id",shelf},{"layer",layer},{"drug",drug},{"warehouse_revision",nullptr},
    {"quantity",nullptr},{"source",nullptr},{"updated_at",nullptr},{"observed_at",nullptr},{"last_scan",nullptr}};
}
void validate_row(const Json & r)
{
  const auto fields={"shelf_id","layer","drug","warehouse_revision","quantity","source","updated_at","observed_at","last_scan"};
  if (!r.is_object() || r.size()!=fields.size()) {throw std::runtime_error("inventory_invalid_fields");}
  for (const auto field:fields) {if (!r.contains(field)) {throw std::runtime_error("inventory_missing_field");}}
  if (!r.at("shelf_id").is_string() || r.at("shelf_id").get<std::string>().empty() ||
    !integer(r.at("layer"),INT32_MAX) || r.at("layer")==0 || !r.at("drug").is_string() ||
    !(r.at("quantity").is_null() || integer(r.at("quantity"),INT32_MAX)) ||
    !(r.at("warehouse_revision").is_null() || integer(r.at("warehouse_revision"),9007199254740991LL)))
  {throw std::runtime_error("inventory_invalid_identity_or_quantity");}
  for (const auto field:{"updated_at","observed_at"}) {
    if (!r.at(field).is_null() && !r.at(field).is_string()) {throw std::runtime_error("inventory_invalid_time");}
  }
  const auto & source=r.at("source");
  if (!(source.is_null() || source=="scan" || source=="manual" || source=="fetch")) {throw std::runtime_error("inventory_invalid_source");}
  const auto & scan=r.at("last_scan");
  if (!scan.is_null() && (!scan.is_object() || scan.size()!=3 ||
    !(scan.at("status")=="complete" || scan.at("status")=="incomplete") ||
    !scan.at("attempted_at").is_string() || !scan.at("reason").is_string())) {throw std::runtime_error("inventory_invalid_scan");}
}
}
void InventoryStore::load_inventory(const std::string & warehouse_path,const WarehouseTable & warehouse)
{
  map_id_=warehouse.map_id;revision_=warehouse.revision;rows_.clear();dirty_=false;load_error_.clear();save_error_.clear();
  auto p=std::filesystem::path(warehouse_path);p.replace_extension(".inventory.json");
  // replace the complete suffix, not only the final .json.
  const std::string suffix=".warehouse.json";
  path_=warehouse_path.size()>=suffix.size() && warehouse_path.compare(warehouse_path.size()-suffix.size(),suffix.size(),suffix)==0 ?
    warehouse_path.substr(0,warehouse_path.size()-suffix.size())+".inventory.json" : p.string();
  try {
    if (warehouse_path.empty() || map_id_.empty()) {throw std::runtime_error("inventory_context_unavailable");}
    if (!std::filesystem::exists(path_)) {return;}
    std::ifstream file(path_,std::ios::binary);
    if (!file) {throw std::runtime_error("inventory_read_failed");}
    std::string text(1024*1024+1,'\0');file.read(text.data(),text.size());text.resize(file.gcount());
    if (file.bad() || text.size()>1024*1024) {throw std::runtime_error("inventory_read_failed_or_too_large");}
    std::vector<std::set<std::string>> keys;
    const auto doc=Json::parse(text,[&](int,Json::parse_event_t event,Json & item) {
      if(event==Json::parse_event_t::object_start) {keys.emplace_back();}
      if(event==Json::parse_event_t::key && !keys.back().insert(item.get<std::string>()).second) {throw std::runtime_error("inventory_duplicate_key");}
      if(event==Json::parse_event_t::object_end) {keys.pop_back();}return true;
    });
    if (!doc.is_object() || doc.size()!=3 || doc.at("schema_version")!=1 || doc.at("map_id")!=map_id_ || !doc.at("layers").is_array())
    {throw std::runtime_error("inventory_schema_or_map_mismatch");}
    for (const auto & r:doc.at("layers")) {
      validate_row(r);const auto key=std::make_pair(r.at("shelf_id").get<std::string>(),r.at("layer").get<int>());
      if (!rows_.emplace(key,r).second) {throw std::runtime_error("inventory_duplicate_layer");}
    }
  } catch (const std::exception & e) {rows_.clear();load_error_=e.what();}
}
InventoryStore::Json & InventoryStore::row(const std::string & shelf,int layer,const std::string & drug)
{
  auto & r=rows_[{shelf,layer}];
  if (r.is_null() || r.at("drug")!=drug) {r=empty_row(shelf,layer,drug);}return r;
}
InventoryStore::Json InventoryStore::inventory_state(const WarehouseTable & warehouse) const
{
  Json layers=Json::array();
  for (const auto & t:warehouse.targets) {
    auto r=empty_row(t.shelf_id,t.layer,t.drug);const auto it=rows_.find({t.shelf_id,t.layer});
    if (!t.drug.empty() && it!=rows_.end() && it->second.at("drug")==t.drug) {r=it->second;}
    r["stale"]=!r.at("quantity").is_null() && r.at("warehouse_revision")!=warehouse.revision;layers.push_back(std::move(r));
  }
  return {{"map_id",map_id_},{"loaded_warehouse_revision",warehouse.revision},{"persisted",persisted()},
    {"load_error",load_error_},{"save_error",save_error_},{"layers",std::move(layers)}};
}
InventoryStore::Json InventoryStore::update_result(const Json & r) const
{return {{"layer",r},{"quantity",r.at("quantity")},{"persisted",persisted()},{"error",error()},{"save_error",save_error_}};}
std::optional<int> InventoryStore::effective_quantity(const WarehouseTarget & t) const
{
  const auto it=rows_.find({t.shelf_id,t.layer});
  if(it==rows_.end()) {return {};}
  const auto & r=it->second;
  if(r.at("drug")!=t.drug || r.at("warehouse_revision")!=revision_ ||
    !r.at("quantity").is_number_integer()) {return {};}
  return r.at("quantity").get<int>();
}
InventoryStore::Json InventoryStore::commit_scan_layer(const std::string & shelf,int layer,const std::string & drug,int count)
{
  if(count<0) {throw std::invalid_argument("quantity_invalid");}
  auto & r=row(shelf,layer,drug);const auto time=utc();r["quantity"]=count;r["source"]="scan";
  r["warehouse_revision"]=revision_;r["updated_at"]=time;r["observed_at"]=time;
  r["last_scan"]={{"status","complete"},{"attempted_at",time},{"reason",""}};
  dirty_=true;save_inventory();return update_result(r);
}
InventoryStore::Json InventoryStore::record_incomplete_layer(const std::string & shelf,int layer,const std::string & drug,const std::string & reason)
{
  auto & r=row(shelf,layer,drug);r["last_scan"]={{"status","incomplete"},{"attempted_at",utc()},{"reason",reason}};
  dirty_=true;save_inventory();return update_result(r);
}
InventoryStore::Json InventoryStore::set_inventory_quantity(const std::string & shelf,int layer,const std::string & drug,int count)
{
  if(count<0) {throw std::invalid_argument("quantity_invalid");}
  auto & r=row(shelf,layer,drug);
  if(r.at("quantity")!=count || r.at("source")!="manual" || r.at("warehouse_revision")!=revision_) {
    r["quantity"]=count;r["source"]="manual";r["warehouse_revision"]=revision_;r["updated_at"]=utc();dirty_=true;
  }
  if(dirty_) {save_inventory();}return update_result(r);
}
InventoryStore::Json InventoryStore::apply_fetch_inventory_delta(const WarehouseTarget & t)
{
  auto & r=row(t.shelf_id,t.layer,t.drug);
  if(effective_quantity(t).value_or(0)>0) {
    r["quantity"]=r.at("quantity").get<int>()-1;r["source"]="fetch";r["updated_at"]=utc();dirty_=true;save_inventory();
  }
  auto result=update_result(r);result["handled"]=true;return result;
}
bool InventoryStore::save_inventory()
{
  if(!load_error_.empty()) {save_error_="inventory_load_error: original file retained";return false;}
  std::string temporary;int fd=-1;
  try {
    Json layers=Json::array();for(const auto & pair:rows_) {layers.push_back(pair.second);}
    const auto text=Json{{"schema_version",1},{"map_id",map_id_},{"layers",std::move(layers)}}.dump(2)+"\n";
    temporary=path_+".tmp.XXXXXX";fd=mkstemp(temporary.data());
    if(fd<0) {throw std::runtime_error(std::strerror(errno));}
    size_t written=0;while(written<text.size()) {
      const auto n=write(fd,text.data()+written,text.size()-written);
      if(n<0 && errno==EINTR) {continue;}if(n<=0) {throw std::runtime_error("inventory_write_failed");}written+=n;
    }
    if(fsync(fd)) {throw std::runtime_error("inventory_fsync_failed");}
    const auto closed=close(fd);fd=-1;if(closed) {throw std::runtime_error("inventory_close_failed");}
    if(std::rename(temporary.c_str(),path_.c_str())) {throw std::runtime_error("inventory_replace_failed");}
    dirty_=false;save_error_.clear();return true;
  } catch(const std::exception & e) {
    if(fd>=0) {close(fd);}if(!temporary.empty()) {std::remove(temporary.c_str());}
    save_error_=e.what();return false;
  }
}
}  // namespace k1muse_manager
