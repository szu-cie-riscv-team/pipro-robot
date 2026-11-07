#include "k1muse_manager/warehouse_config.hpp"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace k1muse_manager
{
namespace
{
using Json = nlohmann::json;
constexpr double pi = 3.14159265358979323846;
constexpr double max_safe_integer = 9007199254740991.0;

[[noreturn]] void invalid(const std::string & field)
{
  throw std::invalid_argument("warehouse_invalid: " + field);
}
void keys(const Json & value, std::initializer_list<const char *> names)
{
  if (!value.is_object() || value.size() != names.size()) {invalid("object fields");}
  for (const auto name : names) {if (!value.contains(name)) {invalid(name);}}
}
std::string string(const Json & value)
{
  if (!value.is_string()) {invalid("string");}
  return value.get<std::string>();
}
std::string identity(const Json & value)
{
  auto result = string(value);
  if (result.empty() || result.size() > 32 || !std::all_of(result.begin(), result.end(),
    [](unsigned char c) {return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9') || c == '_' || c == '-';})) {invalid("identity");}
  return result;
}
double number(const Json & value, bool positive = false)
{
  if (!value.is_number()) {invalid("number");}
  const double result = value.get<double>();
  if (!std::isfinite(result) || (positive && result <= 0)) {invalid("finite number/range");}
  return result;
}
double integer(const Json & value, double minimum, double maximum)
{
  const auto result = number(value);
  if (std::floor(result) != result || result < minimum || result > maximum) {invalid("integer/range");}
  return result;
}
WarehousePose pose(const Json & value)
{
  keys(value, {"x_m", "y_m", "yaw_rad"});
  return {number(value.at("x_m")), number(value.at("y_m")), number(value.at("yaw_rad"))};
}
std::optional<std::string> nullable_string(const Json & value)
{
  return value.is_null() ? std::nullopt : std::make_optional(string(value));
}
std::optional<double> nullable_number(const Json & value)
{
  return value.is_null() ? std::nullopt : std::make_optional(number(value));
}
std::array<double, 2> rotate(double yaw, double x, double y)
{
  return {std::cos(yaw) * x - std::sin(yaw) * y, std::sin(yaw) * x + std::cos(yaw) * y};
}
bool filled(const std::optional<std::string> & value) {return value && !value->empty();}
}  // namespace

WarehouseDocument parse_warehouse_document(const std::string & text)
{
  try {
    // nlohmann callback events preserve keys before object insertion overwrites duplicates.
    std::vector<std::set<std::string>> object_keys;
    const auto json = Json::parse(text, [&object_keys](int, Json::parse_event_t event, Json & item) {
      if (event == Json::parse_event_t::object_start) {object_keys.emplace_back();}
      if (event == Json::parse_event_t::key &&
        !object_keys.back().insert(item.get<std::string>()).second) {invalid("duplicate key");}
      if (event == Json::parse_event_t::object_end) {object_keys.pop_back();}
      return true;
    });
    keys(json, {"schema_version", "revision", "map_id", "layout"});
    if (json.at("schema_version") == 1) {invalid("schema_v1_requires_web_migration");}
    integer(json.at("schema_version"), 2, 2);
    WarehouseDocument result;
    result.revision = static_cast<std::uint64_t>(integer(json.at("revision"), 0, max_safe_integer));
    result.map_id = string(json.at("map_id"));
    if (result.map_id.size() != 64 || !std::all_of(result.map_id.begin(), result.map_id.end(),
      [](char c) {return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');})) {invalid("map_id");}
    const auto & layout = json.at("layout");
    keys(layout, {"anchor", "shelves"});
    if (!layout.at("shelves").is_array()) {invalid("shelves");}
    std::set<std::string> shelf_ids;
    for (const auto & s : layout.at("shelves")) {
      keys(s, {"id", "name", "width_m", "depth_m", "pose", "levels"});
      WarehouseShelf shelf;
      shelf.id = identity(s.at("id"));
      if (!shelf_ids.insert(shelf.id).second) {invalid("duplicate shelf id");}
      shelf.name = string(s.at("name"));
      shelf.width_m = number(s.at("width_m"), true);
      shelf.depth_m = number(s.at("depth_m"), true);
      shelf.pose = pose(s.at("pose"));
      if (!s.at("levels").is_array()) {invalid("levels");}
      std::set<int> layers;
      for (const auto & l : s.at("levels")) {
        keys(l, {"layer", "height_m", "drug", "qr", "fetch"});
        WarehouseLevel level;
        level.layer = static_cast<int>(integer(l.at("layer"), 1, 2147483647));
        if (!layers.insert(level.layer).second) {invalid("duplicate layer");}
        level.height_m = number(l.at("height_m"), true);
        level.drug = string(l.at("drug"));
        const auto & q = l.at("qr");
        keys(q, {"left", "right"});
        for (const auto side : {"left", "right"}) {
          if (q.at(side).is_null()) {continue;}
          const auto & marker = q.at(side);
          if (marker.contains("marker_id")) {keys(marker, {"x_m", "z_m", "marker_id"});}
          else {keys(marker, {"x_m", "z_m"});}
          auto value = WarehouseQr{number(marker.at("x_m")), number(marker.at("z_m"), true), std::nullopt};
          if (marker.contains("marker_id") && !marker.at("marker_id").is_null()) {
            if (!marker.at("marker_id").is_number_integer()) {invalid("marker_id integer");}
            value.marker_id = static_cast<int>(integer(marker.at("marker_id"), 0, 49));
          }
          (std::string(side) == "left" ? level.left_qr : level.right_qr) = value;
        }
        if (level.left_qr && level.right_qr && level.left_qr->x_m >= level.right_qr->x_m) {
          invalid("QR left/right order");
        }
        if (!l.at("fetch").is_null()) {
          const auto & f = l.at("fetch");
          keys(f, {"class_id", "grasp", "roi", "search_min_m", "search_max_m"});
          WarehouseFetch fetch;
          fetch.class_id = nullable_string(f.at("class_id"));
          fetch.grasp = nullable_string(f.at("grasp"));
          if (filled(fetch.grasp) && *fetch.grasp != "front" && *fetch.grasp != "top") {invalid("grasp");}
          if (!f.at("roi").is_null()) {
            const auto & roi = f.at("roi");
            if (!roi.is_array() || roi.size() != 4) {invalid("roi");}
            std::array<double, 4> r{};
            for (std::size_t i = 0; i < 4; ++i) {
              r[i] = number(roi[i]);
              if (r[i] < 0 || r[i] > 1) {invalid("roi range");}
            }
            if (r[0] >= r[2] || r[1] >= r[3]) {invalid("roi order");}
            fetch.roi = r;
          }
          fetch.search_min_m = nullable_number(f.at("search_min_m"));
          fetch.search_max_m = nullable_number(f.at("search_max_m"));
          for (const auto bound : {fetch.search_min_m, fetch.search_max_m}) {
            if (bound && (*bound < 0 || *bound > shelf.width_m)) {invalid("search range");}
          }
          if (fetch.search_min_m && fetch.search_max_m &&
            *fetch.search_min_m > *fetch.search_max_m) {invalid("search order");}
          level.fetch = std::move(fetch);
        }
        shelf.levels.push_back(std::move(level));
      }
      result.shelves.push_back(std::move(shelf));
    }
    if (!layout.at("anchor").is_null()) {
      const auto & a = layout.at("anchor");
      keys(a, {"shelf_id", "layer", "point_name", "point_pose", "camera_distance_m"});
      WarehouseAnchor anchor;
      anchor.shelf_id = identity(a.at("shelf_id"));
      anchor.layer = static_cast<int>(integer(a.at("layer"), 1, 2147483647));
      anchor.point_name = string(a.at("point_name"));
      if (anchor.point_name.empty()) {invalid("point_name");}
      anchor.point_pose = pose(a.at("point_pose"));
      anchor.camera_distance_m = number(a.at("camera_distance_m"), true);
      bool found = false;
      for (const auto & shelf : result.shelves) {
        for (const auto & level : shelf.levels) {
          found |= shelf.id == anchor.shelf_id && level.layer == anchor.layer && level.left_qr.has_value();
        }
      }
      if (!found) {invalid("anchor QR reference");}
      result.anchor = std::move(anchor);
    }
    return result;
  } catch (const Json::exception & e) {
    invalid(std::string("JSON: ") + e.what());
  }
}

double warehouse_wrap(double angle)
{
  auto value = std::fmod(angle + pi, 2 * pi);
  if (value < 0) {value += 2 * pi;}
  return value - pi;
}

WarehousePose warehouse_layout_to_map(
  const WarehouseShelf & shelf, const WarehouseQr & qr, const WarehouseAnchor & anchor,
  const std::array<double, 2> & camera_xy)
{
  const auto theta = warehouse_wrap(anchor.point_pose.yaw_rad - pi / 2);
  const auto yaw = warehouse_wrap(theta - shelf.pose.yaw_rad);
  const auto camera = rotate(anchor.point_pose.yaw_rad, camera_xy[0], camera_xy[1]);
  const auto offset = rotate(theta, qr.x_m, -anchor.camera_distance_m);
  const auto origin = rotate(yaw, shelf.pose.x_m, shelf.pose.y_m);
  return {anchor.point_pose.x_m + camera[0] - offset[0] - origin[0],
    anchor.point_pose.y_m + camera[1] - offset[1] - origin[1], yaw};
}

WarehousePose warehouse_observation_pose(
  const WarehousePose & layout_to_map, const WarehouseShelf & shelf, const WarehouseQr & qr,
  const std::array<double, 2> & camera_xy, double observation_distance_m)
{
  const auto origin = rotate(layout_to_map.yaw_rad, shelf.pose.x_m, shelf.pose.y_m);
  const auto theta = warehouse_wrap(layout_to_map.yaw_rad + shelf.pose.yaw_rad);
  const auto yaw = warehouse_wrap(theta + pi / 2);
  const auto offset = rotate(theta, qr.x_m, -observation_distance_m);
  const auto camera = rotate(yaw, camera_xy[0], camera_xy[1]);
  return {layout_to_map.x_m + origin[0] + offset[0] - camera[0],
    layout_to_map.y_m + origin[1] + offset[1] - camera[1], yaw};
}

double warehouse_initial_lift(double height_m, double observation_drop_m, double zero_height_m)
{
  return height_m - observation_drop_m - zero_height_m;
}

namespace
{
std::array<int, 50> marker_counts(const WarehouseDocument & document)
{
  std::array<int, 50> counts{};
  for (const auto & shelf : document.shelves) for (const auto & level : shelf.levels) {
    for (const auto & marker : {level.left_qr, level.right_qr}) {
      if (marker && marker->marker_id) {++counts[*marker->marker_id];}
    }
  }
  return counts;
}
std::vector<std::string> marker_reasons(const WarehouseLevel & level, const std::array<int, 50> & counts)
{
  std::vector<std::string> reasons;
  for (const auto & pair : {std::make_pair("left", level.left_qr), std::make_pair("right", level.right_qr)}) {
    if (!pair.second || !pair.second->marker_id) {reasons.push_back(std::string(pair.first) + "_marker_id_unconfigured");}
    else if (counts[*pair.second->marker_id] != 1) {reasons.push_back(std::string(pair.first) + "_marker_id_conflict");}
  }
  return reasons;
}
}  // namespace

WarehouseTable build_warehouse_table(const WarehouseDocument & document, const WarehouseInputs & inputs)
{
  WarehouseTable result;
  result.map_id = document.map_id;
  result.revision = document.revision;
  const auto counts = marker_counts(document);
  const bool camera_valid = inputs.camera_xy_in_base_m &&
    std::isfinite((*inputs.camera_xy_in_base_m)[0]) && std::isfinite((*inputs.camera_xy_in_base_m)[1]);
  const bool dock_valid = std::isfinite(inputs.observation_distance_m) && inputs.observation_distance_m > 0;
  std::optional<WarehousePose> transform;
  if (document.anchor && camera_valid) {
    for (const auto & shelf : document.shelves) {
      if (shelf.id != document.anchor->shelf_id) {continue;}
      for (const auto & level : shelf.levels) {
        if (level.layer == document.anchor->layer && level.left_qr) {
          transform = warehouse_layout_to_map(shelf, *level.left_qr, *document.anchor, *inputs.camera_xy_in_base_m);
        }
      }
    }
  }
  for (const auto & shelf : document.shelves) {
    for (const auto & level : shelf.levels) {
      WarehouseTarget target;
      target.shelf_id = shelf.id;
      target.shelf_name = shelf.name;
      target.layer = level.layer;
      target.height_m = level.height_m;
      target.drug = level.drug;
      target.fetch = level.fetch;
      if (level.left_qr) {target.left_marker_id = level.left_qr->marker_id;}
      if (level.right_qr) {target.right_marker_id = level.right_qr->marker_id;}
      target.marker_reasons = marker_reasons(level, counts);
      auto & reasons = target.unavailable_reasons;
      if (level.drug.empty()) {reasons.push_back("drug_unconfigured");}
      else if (inputs.canonical_drugs.count(level.drug) == 0) {reasons.push_back("unknown_drug");}
      if (!level.left_qr) {reasons.push_back("left_qr_unconfigured");}
      if (!level.right_qr) {reasons.push_back("right_qr_unconfigured");}
      if (!document.anchor) {reasons.push_back("anchor_unconfigured");}
      if (!camera_valid) {reasons.push_back("camera_geometry_unconfigured");}
      if (!dock_valid) {reasons.push_back("observation_distance_invalid");}
      if (transform && level.left_qr && dock_valid) {
        const auto p = warehouse_observation_pose(*transform, shelf, *level.left_qr,
          *inputs.camera_xy_in_base_m, inputs.observation_distance_m);
        if (std::isfinite(p.x_m) && std::isfinite(p.y_m) && std::isfinite(p.yaw_rad)) {
          target.observation_pose = p;
        } else {reasons.push_back("observation_pose_nonfinite");}
      }
      if (!level.fetch) {reasons.push_back("fetch_unconfigured");}
      else {
        const auto & fetch = *level.fetch;
        if (!filled(fetch.class_id)) {reasons.push_back("class_id_unconfigured");}
        if (!fetch.roi) {reasons.push_back("roi_unconfigured");}
        if (!fetch.search_min_m) {reasons.push_back("search_min_unconfigured");}
        if (!fetch.search_max_m) {reasons.push_back("search_max_unconfigured");}
        if (!filled(fetch.grasp)) {reasons.push_back("grasp_unconfigured");}
        else {
          const auto & grasp = *fetch.grasp == "front" ? inputs.front : inputs.top;
          if (!grasp.calibrated) {reasons.push_back("grasp_uncalibrated");}
          if (!grasp.observation_drop_m || !std::isfinite(*grasp.observation_drop_m) ||
            *grasp.observation_drop_m < 0) {reasons.push_back("observation_drop_unconfigured");}
          else if (!inputs.lift_zero_height_m || !std::isfinite(*inputs.lift_zero_height_m) ||
            *inputs.lift_zero_height_m < 0) {reasons.push_back("lift_geometry_unconfigured");}
          else {
            auto z = warehouse_initial_lift(level.height_m,
              *grasp.observation_drop_m, *inputs.lift_zero_height_m);
            if(z>=kLiftMinM-inputs.position_tolerance_m && z<=kLiftMaxM+inputs.position_tolerance_m) {
              z=std::clamp(z,kLiftMinM,kLiftMaxM);
            }
            target.initial_lift_z_m = z;
            if (!std::isfinite(z) || z < kLiftMinM || z > kLiftMaxM) {reasons.push_back("lift_unreachable");}
          }
        }
      }
      const auto index = result.targets.size();
      if (!target.drug.empty()) {result.drug_index[target.drug].push_back(index);}
      result.targets.push_back(std::move(target));
    }
  }
  return result;
}

std::vector<SurveyLayer> build_survey_layers(const WarehouseDocument & document,const WarehouseInputs & inputs)
{
  std::optional<WarehousePose> transform;
  if(document.anchor && inputs.camera_xy_in_base_m) {
    for(const auto & shelf:document.shelves) {
      for(const auto & level:shelf.levels) {
        if(shelf.id==document.anchor->shelf_id && level.layer==document.anchor->layer && level.left_qr) {
          transform=warehouse_layout_to_map(shelf,*level.left_qr,*document.anchor,*inputs.camera_xy_in_base_m);
        }
      }
    }
  }
  std::vector<SurveyLayer> result;
  const auto counts = marker_counts(document);
  for(const auto & shelf:document.shelves) {
    for(const auto & level:shelf.levels) {
      SurveyLayer row;row.shelf_id=shelf.id;row.layer=level.layer;row.drug=level.drug;row.height_m=level.height_m;
      row.left_qr=level.left_qr;row.right_qr=level.right_qr;
      row.unavailable_reasons = marker_reasons(level, counts);
      if(!level.left_qr || !level.right_qr) {row.unavailable_reasons.push_back("qr_unconfigured");}
      if(!transform) {row.unavailable_reasons.push_back("anchor_or_camera_unconfigured");}
      if(!std::isfinite(inputs.observation_distance_m) || inputs.observation_distance_m<=0) {row.unavailable_reasons.push_back("observation_distance_invalid");}
      else if(transform) {
        if(level.left_qr) {row.left_pose=warehouse_observation_pose(*transform,shelf,*level.left_qr,*inputs.camera_xy_in_base_m,inputs.observation_distance_m);}
        if(level.right_qr) {row.right_pose=warehouse_observation_pose(*transform,shelf,*level.right_qr,*inputs.camera_xy_in_base_m,inputs.observation_distance_m);}
      }
      if(!inputs.lift_zero_height_m || !std::isfinite(*inputs.lift_zero_height_m)) {row.unavailable_reasons.push_back("lift_geometry_unconfigured");}
      else {
        row.lift_z_m=warehouse_initial_lift(level.height_m,0,*inputs.lift_zero_height_m);
        if(*row.lift_z_m>=kLiftMinM-inputs.position_tolerance_m && *row.lift_z_m<=kLiftMaxM+inputs.position_tolerance_m) {
          row.lift_z_m=std::clamp(*row.lift_z_m,kLiftMinM,kLiftMaxM);
        }
        if(*row.lift_z_m<kLiftMinM || *row.lift_z_m>kLiftMaxM) {row.unavailable_reasons.push_back("lift_unreachable");}
      }
      result.push_back(std::move(row));
    }
  }
  std::stable_sort(result.begin(),result.end(),[](const auto & a,const auto & b) {
    if(a.shelf_id!=b.shelf_id) {return a.shelf_id<b.shelf_id;}
    if(a.height_m!=b.height_m) {return a.height_m<b.height_m;}return a.layer<b.layer;
  });
  return result;
}

const WarehouseTarget * WarehouseTable::select(const std::string & canonical_drug, bool require_markers, const std::string & grasp) const
{
  const auto found = drug_index.find(canonical_drug);
  if (found == drug_index.end()) {return nullptr;}
  for (const auto index : found->second) {
    const auto & t=targets[index];
    if (t.executable() && (!require_markers || t.markers_available()) &&
      (grasp.empty() || (t.fetch && t.fetch->grasp==grasp))) {return &t;}
  }
  return nullptr;
}
}  // namespace k1muse_manager
