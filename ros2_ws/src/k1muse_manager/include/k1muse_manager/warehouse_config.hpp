#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace k1muse_manager
{
constexpr double kLiftMinM = .005, kLiftMaxM = .495;
struct WarehousePose
{
  double x_m{0}, y_m{0}, yaw_rad{0};
};
struct WarehouseQr
{
  double x_m{0}, z_m{0};  // Symbol centre; shelf-local Y is always zero.
  std::optional<int> marker_id;
};
struct WarehouseFetch
{
  std::optional<std::string> class_id, grasp;
  std::optional<std::array<double, 4>> roi;
  std::optional<double> search_min_m, search_max_m;
};
struct WarehouseLevel
{
  int layer{0};
  double height_m{0};
  std::string drug;
  std::optional<WarehouseQr> left_qr, right_qr;
  std::optional<WarehouseFetch> fetch;
};
struct WarehouseShelf
{
  std::string id, name;
  double width_m{0}, depth_m{0};
  WarehousePose pose;
  std::vector<WarehouseLevel> levels;
};
struct WarehouseAnchor
{
  std::string shelf_id, point_name;
  int layer{0};
  WarehousePose point_pose;  // Saved base_link pose, NOT the shelf origin.
  double camera_distance_m{0};
};
struct WarehouseDocument
{
  std::uint64_t revision{0};
  std::string map_id;
  std::optional<WarehouseAnchor> anchor;
  std::vector<WarehouseShelf> shelves;
};
struct WarehouseGraspSettings
{
  bool calibrated{false};
  std::optional<double> observation_drop_m;
};
struct WarehouseInputs
{
  std::optional<std::array<double, 2>> camera_xy_in_base_m;
  std::optional<double> lift_zero_height_m;
  double observation_distance_m{0.30};
  double position_tolerance_m{.005};
  WarehouseGraspSettings front, top;
  std::unordered_set<std::string> canonical_drugs;
};
struct WarehouseTarget
{
  std::string shelf_id, shelf_name, drug;
  int layer{0};
  double height_m{0};
  std::optional<WarehouseFetch> fetch;
  std::optional<int> left_marker_id, right_marker_id;
  std::vector<std::string> marker_reasons;
  bool markers_available() const {return marker_reasons.empty();}
  std::optional<WarehousePose> observation_pose;
  std::optional<double> initial_lift_z_m;
  std::vector<std::string> unavailable_reasons;
  bool executable() const {return unavailable_reasons.empty();}
};
struct WarehouseTable
{
  std::string map_id;
  std::uint64_t revision{0};
  std::vector<WarehouseTarget> targets;
  std::unordered_map<std::string, std::vector<std::size_t>> drug_index;
  const WarehouseTarget * select(const std::string & canonical_drug, bool require_markers = false,
    const std::string & grasp = "") const;
};
struct SurveyLayer
{
  std::string shelf_id, drug;
  int layer{0};
  double height_m{0};
  std::optional<WarehouseQr> left_qr, right_qr;
  std::optional<WarehousePose> left_pose, right_pose;
  std::optional<double> lift_z_m;
  std::vector<std::string> unavailable_reasons;
  bool scannable() const {return !drug.empty() && unavailable_reasons.empty();}
};
std::vector<SurveyLayer> build_survey_layers(const WarehouseDocument &,const WarehouseInputs &);

// Pure functions. Parsing throws std::invalid_argument for malformed documents;
// incomplete models remain valid. No file I/O, ROS entities or installation defaults.
WarehouseDocument parse_warehouse_document(const std::string & text);
double warehouse_wrap(double angle);
WarehousePose warehouse_layout_to_map(
  const WarehouseShelf & shelf, const WarehouseQr & qr, const WarehouseAnchor & anchor,
  const std::array<double, 2> & camera_xy);
WarehousePose warehouse_observation_pose(
  const WarehousePose & layout_to_map, const WarehouseShelf & shelf, const WarehouseQr & qr,
  const std::array<double, 2> & camera_xy, double observation_distance_m);
double warehouse_initial_lift(double height_m, double observation_drop_m, double zero_height_m);
WarehouseTable build_warehouse_table(const WarehouseDocument & document, const WarehouseInputs & inputs);
}  // namespace k1muse_manager
