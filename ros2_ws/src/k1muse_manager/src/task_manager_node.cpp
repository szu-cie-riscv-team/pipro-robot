#include "k1muse_manager/task_manager_node_impl.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>

namespace k1muse_manager
{
namespace
{
double number(const YAML::Node & n, const char * key, double fallback)
{
  double v = n[key] ? n[key].as<double>() : fallback;
  if (!std::isfinite(v)) {throw std::runtime_error(std::string("nonfinite config: ") + key);}
  return v;
}
template<size_t N> std::array<double, N> array(const YAML::Node & n)
{
  if (!n.IsSequence() || n.size() != N) {throw std::runtime_error("invalid pose/ROI length");}
  std::array<double, N> result{};
  for (size_t i = 0; i < N; ++i) {
    result[i] = n[i].as<double>();
    if (!std::isfinite(result[i])) {throw std::runtime_error("nonfinite pose/ROI");}
  }
  return result;
}
GraspProfile load_profile(const YAML::Node & n, bool top, GraspProfile p)
{
  p.surface_to_tcp = array<3>(n["surface_to_tcp_xyz"]);
  if (top) {
    p.preferred_tcp=array<3>(n["preferred_tcp_xyz"]);
    return p;
  }
  p.minimum = array<3>(n["pose_min_xyz"]); p.maximum = array<3>(n["pose_max_xyz"]);
  p.approach_m = number(n, "approach_m", .05); p.lift_m = number(n, "lift_m", .05);
  p.roll = number(n, "roll", 1.5707963267948966);
  p.pitch = number(n, "pitch", 0); p.open_rad = number(n, "open_rad", 1.3);
  p.align_lift = n["align_lift"].as<bool>(true);
  for (const auto & it : n["poses"]) {
    const auto name=it.first.as<std::string>();
    if(name!="ready") {p.poses[name]=array<5>(it.second);}
  }
  for (const auto & key : {"pick", "exit"}) {
    auto & seq = std::string(key) == "pick" ? p.pick : p.exit;
    for (const auto & step : n[key]) {
      const auto type = step["type"].as<std::string>();
      if (type != "pose" && type != "open" && type != "grip") {throw std::runtime_error("unknown ARM step type");}
      seq.push_back({step["id"].as<std::string>(), type == "pose" ? ArmStepKind::Pose :
        type == "open" ? ArmStepKind::Open : ArmStepKind::Grip, step["target"].as<std::string>("")});
    }
  }
  std::string error;
  if(!validate_grasp_profile(p,error)) {throw std::runtime_error(error);}
  return p;
}
}  // namespace

FetchConfig load_fetch_config(const std::string & path,std::string * profile_error)
{
  const auto n = YAML::LoadFile(path); FetchConfig c;
  c.enabled = n["enabled"].as<bool>(false); c.calibrated = n["calibration_confirmed"].as<bool>(false);
  const auto route = n["route"], search = n["search"], limits = n["limits"], geometry = n["geometry"];
  if (n["variant"] || n["observation_z_m"] || n["cells"] || route["map_id"] || route["shelf_point"] ||
    search["step_m"] || search["limit_m"] || search["align_base"] || search["preferred_tcp_x"] ||
    search["align_tolerance_m"] || search["max_adjustment_m"] || n["dock"])
  {throw std::runtime_error("legacy_fetch_config: migrate to warehouse JSON and v2 warehouse JSON and navigation/framing");}
  c.return_point = route["return_point"].as<std::string>("");
  c.search_timeout = number(search, "timeout_sec", 20);
  c.withdraw = number(search, "withdraw_m", 0); c.speed = number(search, "speed_mps", .04);
  c.yaw_speed = number(search, "yaw_speed_radps", .15);
  c.position_tolerance = number(search, "position_tolerance_m", .005);
  c.yaw_tolerance = number(search, "yaw_tolerance_rad", .03);
  c.max_align = limits["max_align_attempts"].as<unsigned>(8);
  c.status_age = number(limits, "status_max_age_sec", .5);
  c.observation_age = number(limits, "observation_max_age_sec", 1.0);
  c.observation_timeout = number(limits, "observation_timeout_sec", 5);
  c.step_timeout = number(limits, "step_timeout_sec", 60);
  c.navigation_timeout = number(limits, "navigation_timeout_sec", 120);
  c.service_timeout = number(limits, "service_timeout_sec", 3);
  c.local_timeout = number(limits, "local_timeout_sec", 10);
  c.confidence = number(limits, "min_confidence", .2);
  if (c.search_timeout <= 0 || c.withdraw < 0 ||
    c.speed <= 0 || c.speed > .20 || c.yaw_speed <= 0 || c.yaw_speed > .6 ||
    c.position_tolerance <= 0 || c.yaw_tolerance <= 0 ||
    c.status_age <= 0 || c.observation_age <= 0 || c.observation_timeout <= 0 ||
    c.step_timeout <= 0 || c.navigation_timeout <= 0 || c.service_timeout <= 0 ||
    c.local_timeout <= 0 || c.confidence < 0 || c.confidence > 1 || c.max_align > 10)
  {throw std::runtime_error("invalid fetch limits");}
  if (geometry["lift_zero_height_m"] && !geometry["lift_zero_height_m"].IsNull()) {
    c.geometry.lift_zero_height_m = number(geometry, "lift_zero_height_m", 0);
    if (*c.geometry.lift_zero_height_m < 0) {throw std::runtime_error("invalid lift zero height");}
  }
  if (geometry["color_optical_origin_xy_in_base_m"] && geometry["color_optical_origin_xy_in_base_m"].size()) {
    c.geometry.camera_xy = array<2>(geometry["color_optical_origin_xy_in_base_m"]);
  }
  if (geometry["color_optical_to_base_quaternion_xyzw"] && geometry["color_optical_to_base_quaternion_xyzw"].size()) {
    c.geometry.camera_rotation = array<4>(geometry["color_optical_to_base_quaternion_xyzw"]);
    double norm = 0;
    for (const auto value : *c.geometry.camera_rotation) {norm += value * value;}
    if (std::abs(norm - 1) > 1e-6) {throw std::runtime_error("camera rotation must be a unit quaternion");}
  }
  const auto framing = n["framing"];
  c.observation_distance = number(n["navigation"], "observation_distance_m", .30);
  c.no_qr = n["qr"]["no_qr"].as<bool>(false);
  c.cv_frame_age = number(n["qr"], "max_frame_age_sec", 1.0);
  c.qr_lateral_tolerance = number(n["qr"], "lateral_tolerance_m", .03);
  c.qr_max_travel = number(n["qr"], "max_lateral_travel_m", .30);
  c.qr_align_timeout = number(n["qr"], "alignment_timeout_sec", 15);
  if(c.qr_lateral_tolerance<=c.position_tolerance || c.qr_max_travel<=0 ||
    c.qr_max_travel>.5 || c.qr_align_timeout<=0) {throw std::runtime_error("invalid QR alignment configuration");}
  c.framing.target_u = number(framing, "target_u_ratio", .60);
  c.framing.tolerance_u = number(framing, "tolerance_u_ratio", .05);
  c.framing.timeout = number(framing, "timeout_sec", 10);
  c.framing.max_frame_age = number(framing, "max_frame_age_sec", 1.0);
  const auto approach=n["approach"] ? n["approach"] : YAML::Node(YAML::NodeType::Map);
  c.approach.enabled=approach["enabled"].as<bool>(false);
  c.approach.target_distance=number(approach,"target_distance_m",.25);
  c.approach.min_distance=number(approach,"min_distance_m",.20);
  c.approach.max_distance=number(approach,"max_distance_m",.28);
  c.approach.lateral_tolerance=number(approach,"lateral_tolerance_m",.04);
  c.approach.max_step=number(approach,"max_step_m",.05);
  c.approach.max_travel=number(approach,"max_travel_m",.30);
  c.approach.shelf_clearance=number(approach,"min_shelf_clearance_m",.12);
  if(c.approach.min_distance<=0 || c.approach.target_distance<=c.approach.min_distance ||
    c.approach.target_distance>=c.approach.max_distance || c.approach.lateral_tolerance<=c.position_tolerance ||
    c.approach.max_step<std::max(.01,2*c.position_tolerance) || c.approach.max_step>.10 ||
    c.approach.max_travel<c.approach.max_step || c.approach.max_travel>.50 || c.approach.shelf_clearance<=0)
  {throw std::runtime_error("invalid depth approach configuration");}
  if (c.observation_distance <= 0 || c.cv_frame_age <= 0 ||
    c.framing.target_u <= 0 || c.framing.target_u >= 1 || c.framing.tolerance_u <= 0 ||
    c.framing.tolerance_u > 1 || c.framing.timeout <= 0 || c.framing.max_frame_age <= 0)
  {throw std::runtime_error("invalid navigation/QR/framing configuration");}
  c.grasp_mode=n["grasp"]["mode"].as<std::string>("front");
  if(c.grasp_mode!="front" && c.grasp_mode!="top") {
    throw std::runtime_error("grasp.mode must be front or top");
  }
  auto & selected=c.grasp_mode=="front"?c.front:c.top;
  const auto profile=n["grasp"][c.grasp_mode];
  // Common preparation geometry remains available when only the fetch profile is invalid.
  selected.calibrated=profile["calibration_confirmed"].as<bool>(false);
  selected.poses["ready"]=array<5>(profile["poses"]["ready"]);
  if(profile_error) {
    try {selected=load_profile(profile,c.grasp_mode=="top",selected);profile_error->clear();}
    catch(const std::exception & error) {*profile_error=error.what();}
  } else {selected=load_profile(profile,c.grasp_mode=="top",selected);}
  // Demo2 searches at the layer height. Legacy observation_drop_m and fixed
  // preferred_tcp_z/height_tolerance keys no longer lower the platform.
  auto optional_json = [](const auto & value) -> Json {return value ? Json(*value) : Json(nullptr);};
  c.startup_fields = {{"enabled", c.enabled}, {"calibration_confirmed", c.calibrated},
    {"return_point", c.return_point},
    {"geometry", {{"lift_zero_height_m", optional_json(c.geometry.lift_zero_height_m)},
      {"color_optical_origin_xy_in_base_m", optional_json(c.geometry.camera_xy)},
      {"color_optical_to_base_quaternion_xyzw", optional_json(c.geometry.camera_rotation)}}},
    {"navigation", {{"observation_distance_m", c.observation_distance}}},
    {"approach", {{"enabled",c.approach.enabled},{"target_distance_m",c.approach.target_distance},
      {"min_distance_m",c.approach.min_distance},{"max_distance_m",c.approach.max_distance},
      {"lateral_tolerance_m",c.approach.lateral_tolerance},{"max_step_m",c.approach.max_step},{"max_travel_m",c.approach.max_travel},
      {"min_shelf_clearance_m",c.approach.shelf_clearance}}},
    {"qr", {{"max_frame_age_sec", c.cv_frame_age}, {"no_qr", c.no_qr},
      {"lateral_tolerance_m",c.qr_lateral_tolerance}, {"max_lateral_travel_m",c.qr_max_travel},
      {"alignment_timeout_sec",c.qr_align_timeout}}},
    {"framing", {{"target_u_ratio", c.framing.target_u}, {"tolerance_u_ratio", c.framing.tolerance_u},
      {"timeout_sec", c.framing.timeout},
      {"max_frame_age_sec", c.framing.max_frame_age}}},
    {"grasp", {{"mode", c.grasp_mode}, {"front", {{"observation_drop_m", optional_json(c.front_observation_drop)},
      {"calibration_confirmed", c.front.calibrated}}},
      {"top", {{"observation_drop_m", optional_json(c.top_observation_drop)},
      {"calibration_confirmed", c.top.calibrated}}}}},
    {"motion", {c.search_timeout, c.withdraw, c.speed, c.yaw_speed, c.position_tolerance, c.yaw_tolerance}},
    {"limits", {c.max_align, c.status_age, c.observation_age, c.observation_timeout, c.step_timeout,
      c.navigation_timeout, c.service_timeout, c.local_timeout, c.confidence}}};
  return c;
}

void reload_fetch_profiles(FetchConfig & current, const FetchConfig & candidate)
{
  if (current.startup_fields != candidate.startup_fields) {throw std::runtime_error("restart_required");}
  current.front = candidate.front;
  current.top = candidate.top;
}

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options) : Node("task_manager", options)
{
  instance_ = std::to_string(std::random_device{}()) + "-" + std::to_string(std::random_device{}());
  const auto timeout = declare_parameter<int>("navigation_control_timeout_ms", 1000);
  if (timeout < 1 || timeout > 2000) {throw std::invalid_argument("navigation_control_timeout_ms must be 1..2000");}
  timeout_ = std::chrono::milliseconds(timeout);
  inference_ = std::make_unique<InferenceControl>(*this);
  nav_ = rclcpp_action::create_client<Nav>(this, "/navigate_to_pose");
  gate_ = create_client<Control>("/k1muse/web/navigation/control");
  upper_cancel_ = create_client<action_msgs::srv::CancelGoal>("/k1muse/task/navigate/_action/cancel_goal");
  control_ = create_service<Control>("/k1muse/task/navigation/control",
    [this](Control::Request::SharedPtr req, Control::Response::SharedPtr res) {
      res->operation_id = req->request_id;
      try {configure(*req); res->accepted = true;} catch (const std::exception & e) {res->error = e.what();}
      res->result = context_state().dump();
    });
  cancel_ = create_service<Trigger>("/k1muse/task/cancel",
    [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      try {res->success = cancel_current(); res->message = res->success ?
        "Cancellation requested; await the real terminal result" : "No active task";}
      catch (const std::exception & e) {res->message = e.what();}
    });
  server_ = rclcpp_action::create_server<Task>(this, "/k1muse/task/navigate",
    [this](const rclcpp_action::GoalUUID & id, std::shared_ptr<const Task::Goal> goal) {return admit(id, *goal);},
    [this](std::shared_ptr<TaskHandle> handle) {
      if (!active_ || active_->handle != handle) {return rclcpp_action::CancelResponse::REJECT;}
      request_cancel(); return rclcpp_action::CancelResponse::ACCEPT;
    }, [this](std::shared_ptr<TaskHandle> handle) {
      active_ = std::move(prepared_goal_); active_->handle = handle; inference_->business(true); advance();
    });
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor(); descriptor.read_only = true;
  config_file_ = declare_parameter<std::string>("fetch_config_file", "", descriptor);
  warehouse_file_ = declare_parameter<std::string>("warehouse_config_file", "", descriptor);
  survey_file_=declare_parameter<std::string>("survey_config_file","",descriptor);
  if (!config_file_.empty()) {
    try {config_=load_fetch_config(config_file_,&config_error_);}
    catch(const std::exception & error) {config_error_=motion_config_error_=error.what();}
  } else {config_error_=motion_config_error_="fetch_config_unconfigured";}
  const auto catalog=declare_parameter<std::string>("drug_catalog_file",
    "/home/bianbu/k1muse_config/drug_catalog.yaml");
  try {
    const auto entries=load_drug_catalog(catalog);
    drug_catalog_["drugs"]=drug_catalog_json(entries);
    for(const auto & drug:entries) {drugs_.insert(drug.name);}
  } catch(const std::exception & e) {drug_catalog_["error"]=e.what();}
  inference_->set_drug_catalog(drug_catalog_);
  inference_->on_idle([this] {
    if (active_ && active_->waiting_permit) {
      advance();
    }
  });
  if(!config_file_.empty()) {setup_fetch();}
  if(!survey_file_.empty()) {
    try {survey_config_=load_survey_config(survey_file_);}
    catch(const std::exception & e) {survey_error_=e.what();}
  }
  setup_survey();
  load_warehouse();
  inventory_.load_inventory(warehouse_file_,warehouse_);
  // Inspection remains available in ordinary-navigation and unconfigured modes.
  debug_ = create_service<Control>("/k1muse/task/control",
    [this](Control::Request::SharedPtr req, Control::Response::SharedPtr res) {
      res->operation_id = req->request_id;
      try {task_control(*req, *res); res->accepted = true;}
      catch (const std::exception & e) {res->error = e.what(); res->result = inspect().dump();}
    });
}

void TaskManagerNode::load_warehouse()
{
  for (const auto key : {"geometry", "navigation", "qr", "framing", "grasp", "approach"}) {
    if (config_.startup_fields.contains(key)) {warehouse_capabilities_[key] = config_.startup_fields.at(key);}
  }
  warehouse_capabilities_["canonical_drugs"] = drugs_;
  if (warehouse_file_.empty()) {warehouse_error_ = "warehouse_unconfigured"; return;}
  try {
    std::ifstream file(warehouse_file_, std::ios::binary);
    if (!file) {throw std::runtime_error("warehouse_io_error: cannot open configured file");}
    std::string raw(65537, '\0'); file.read(raw.data(), raw.size());
    if (file.bad()) {throw std::runtime_error("warehouse_io_error: read failed");}
    raw.resize(static_cast<std::size_t>(file.gcount()));
    if (raw.size() > 65536) {throw std::runtime_error("warehouse_too_large");}
    auto document = parse_warehouse_document(raw);
    // Runtime grasp selection is owned by YAML; leave the stored geometry/revision unchanged.
    for(auto & shelf:document.shelves) for(auto & level:shelf.levels) {
      if(level.fetch) {level.fetch->grasp=config_.grasp_mode;}
    }
    WarehouseInputs inputs;
    if (config_.geometry.camera_rotation) {inputs.camera_xy_in_base_m = config_.geometry.camera_xy;}
    inputs.lift_zero_height_m = config_.geometry.lift_zero_height_m;
    inputs.observation_distance_m = config_.observation_distance;
    inputs.front = {config_.front.calibrated, config_.front_observation_drop};
    inputs.position_tolerance_m=config_.position_tolerance;
    inputs.top = {config_.top.calibrated, config_.top_observation_drop};
    inputs.canonical_drugs.insert(drugs_.begin(), drugs_.end());
    warehouse_ = build_warehouse_table(document, inputs);
    survey_layers_=build_survey_layers(document,inputs);
    warehouse_anchor_ = document.anchor; warehouse_loaded_ = true;
  } catch (const std::exception & e) {
    warehouse_error_ = e.what();
    RCLCPP_WARN(get_logger(), "Warehouse unavailable: %s", warehouse_error_.c_str());
  }
}

std::string TaskManagerNode::warehouse_context_error() const
{
  if (!motion_config_error_.empty()) {return motion_config_error_;}
  if (!warehouse_loaded_) {return warehouse_error_;}
  if (!ready_) {return "navigation_not_prepared";}
  if (map_ != warehouse_.map_id) {return "map_changed: sync warehouse and restart";}
  if (!warehouse_anchor_) {return "anchor_unconfigured";}
  const auto & anchor = *warehouse_anchor_;
  if (!points_.contains(anchor.point_name)) {return "warehouse_anchor_changed: sync and restart";}
  const auto & point = points_.at(anchor.point_name);
  if (std::abs(point.at("x").get<double>() - anchor.point_pose.x_m) > 1e-9 ||
    std::abs(point.at("y").get<double>() - anchor.point_pose.y_m) > 1e-9 ||
    std::abs(warehouse_wrap(point.at("yaw").get<double>() - anchor.point_pose.yaw_rad)) > 1e-9)
  {return "warehouse_anchor_changed: sync and restart";}
  if (config_.return_point.empty() || !points_.contains(config_.return_point)) {return "return_point_unconfigured";}
  return "";
}

Json TaskManagerNode::warehouse_state() const
{
  return {{"config_file", warehouse_file_},
    {"loaded_map_id", warehouse_loaded_ ? Json(warehouse_.map_id) : Json(nullptr)},
    {"loaded_revision", warehouse_loaded_ ? Json(warehouse_.revision) : Json(nullptr)},
    {"load_error", warehouse_error_.empty() ? config_error_ : warehouse_error_}, {"fetch_config_error", config_error_},
    {"context_error", warehouse_context_error()}, {"target_count", warehouse_.targets.size()},
    {"capabilities", warehouse_capabilities_}};
}
TaskManagerNode::~TaskManagerNode()
{
  if (local_pub_) {geometry_msgs::msg::TwistStamped zero; zero.header.stamp = now(); zero.header.frame_id = "base_link"; local_pub_->publish(zero);}
}
Json TaskManagerNode::context_state() const
{
  return {{"manager_instance", instance_}, {"context_id", context_}, {"map_id", map_},
    {"ready", ready_}, {"busy", static_cast<bool>(active_)},
    {"business_busy", active_ && active_->business()}, {"motion_inhibited", motion_inhibited_},
    {"goal_id", active_ ? active_->id : ""}, {"goal_context_id", active_ ? active_->context : ""}};
}
void TaskManagerNode::configure(const Control::Request & req)
{
  const auto a = Json::parse(req.arguments.empty() ? "{}" : req.arguments);
  if (req.operation == "get_context") {return;}
  if (active_ && active_->business()) {throw std::runtime_error("business_busy: cancel the complete task first");}
  if (motion_inhibited_) {throw std::runtime_error("arm posture/holding unresolved; prepare hardware first");}
  if (req.operation == "suspend") {
    const auto next = a.at("next_context_id").get<std::string>();
    const auto expected = a.at("expected_context_id").get<std::string>();
    const auto web = a.at("web_boot_id").get<std::string>();
    if (next.empty() || web.empty()) {throw std::runtime_error("missing context identity");}
    if (next == context_ && web == web_) {return;}
    if (expected != context_) {throw std::runtime_error("navigation_context_changed");}
    if (active_ && !a.value("cancel_navigation", false)) {throw std::runtime_error("navigation_busy");}
    if (active_) {cancel_current();}
    ready_ = false; configuring_ = true; context_ = next; web_ = web; return;
  }
  if (req.operation != "configure") {throw std::runtime_error("unknown navigation operation");}
  if (a.at("manager_instance") != instance_ || a.at("context_id") != context_ || a.at("web_boot_id") != web_) {
    throw std::runtime_error("navigation_context_changed");
  }
  if (!configuring_) {
    if (a.at("ready") == ready_ && a.at("map_id") == map_ && a.at("tasks") == points_) {return;}
    throw std::runtime_error("suspend navigation before configuring");
  }
  if (active_) {throw std::runtime_error("navigation_busy");}
  bool ready = a.at("ready").get<bool>(); Json points = a.at("tasks");
  const auto map = a.at("map_id").get<std::string>();
  if (!points.is_object() || points.size() > 100 || (ready && map.empty())) {throw std::runtime_error("invalid navigation context");}
  for (auto it = points.begin(); it != points.end(); ++it) {
    if (it.key().empty() || it.key().size() > 256) {throw std::runtime_error("invalid point name");}
    for (const auto * key : {"x", "y", "yaw"}) {
      if (!it.value().at(key).is_number() || !std::isfinite(it.value().at(key).get<double>())) {throw std::runtime_error("invalid point pose");}
    }
  }
  if (ready && (!nav_->action_server_is_ready() || !gate_->service_is_ready())) {throw std::runtime_error("navigation backend unavailable");}
  map_ = ready ? map : ""; points_ = ready ? std::move(points) : Json::object(); ready_ = ready; configuring_ = false;
}
Pose TaskManagerNode::resolve(const Task::Goal & goal) const
{
  if ((!goal.map_id.empty() && goal.map_id != map_) || (goal.use_pose && goal.map_id.empty())) {throw std::runtime_error("map_changed");}
  Pose p;
  if (goal.use_pose) {
    if (!goal.task_name.empty()) {throw std::runtime_error("choose name or pose");}
    p = goal.target_pose; const auto & v = p.pose.position; const auto & q = p.pose.orientation;
    if (p.header.frame_id != "map" || !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
      std::abs(v.z) > 1e-6 || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w) || std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || std::abs(q.z*q.z + q.w*q.w - 1) > 1e-3)
    {throw std::runtime_error("expected a finite planar map pose with a unit quaternion");}
  } else {
    if (goal.task_name.empty() || !points_.contains(goal.task_name)) {throw std::runtime_error("unknown task point");}
    const auto & v = points_.at(goal.task_name); p.header.frame_id = "map";
    p.pose.position.x = v.at("x"); p.pose.position.y = v.at("y"); const double yaw = v.at("yaw");
    p.pose.orientation.z = std::sin(yaw/2); p.pose.orientation.w = std::cos(yaw/2);
  }
  return p;
}
rclcpp_action::GoalResponse TaskManagerNode::admit(const rclcpp_action::GoalUUID & id, const Task::Goal & goal)
{
  prepared_goal_.reset();
  try {
    if (active_ || !ready_ || motion_inhibited_) {throw std::runtime_error("navigation busy or not prepared");}
    if (!nav_->action_server_is_ready() || !gate_->service_is_ready()) {throw std::runtime_error("navigation backend unavailable");}
    auto t = std::make_shared<Active>(); t->pose = resolve(goal); t->id = rclcpp_action::to_string(id);
    t->leg_id = t->id; t->context = context_; prepared_goal_ = t;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } catch (const std::exception & e) {RCLCPP_WARN(get_logger(), "Navigation rejected: %s", e.what()); return rclcpp_action::GoalResponse::REJECT;}
}
void TaskManagerNode::request_cancel()
{
  active_->upper_cancel_pending = false; active_->cancel = true; active_->pause_requested = false;
  if (active_->business()) {event("cancel_requested"); if (active_->pending == Pending::Local) {stop_local();}}
  if (active_ && (!active_->business() || active_->nav_active)) {cancel_nav(active_);}
  advance();
}
bool TaskManagerNode::cancel_current()
{
  const auto t = active_; if (!t) {return false;}
  if (t->cancel || t->upper_cancel_pending) {return true;}
  auto client = t->is_survey() ? survey_cancel_ : t->is_fetch ? fetch_cancel_ : upper_cancel_;
  if (!client->service_is_ready()) {throw std::runtime_error("cancel endpoint unavailable");}
  if ((!t->handle && !t->fetch && !t->survey)) {throw std::runtime_error("goal admission pending; retry cancel");}
  auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  req->goal_info.goal_id.uuid = t->is_survey() ? t->survey->get_goal_id() : t->is_fetch ? t->fetch->get_goal_id() : t->handle->get_goal_id();
  t->upper_cancel_pending = true;
  try {client->async_send_request(req, [this,t](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture f) {
    if (active_ != t) {return;} t->upper_cancel_pending = false;
    if (f.get()->return_code != action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
      t->blocked = true; t->error = "Task cancellation rejected; await terminal result";
    }
    advance();
  });} catch (...) {t->upper_cancel_pending = false; throw;}
  return true;
}
void TaskManagerNode::begin_navigation(const Pose & pose, Phase next)
{
  auto t = active_; t->pose = pose; t->next_phase = next; t->nav_active = true; t->pending = Pending::Navigation;
  t->leg_id = t->id + ":" + std::to_string(++t->leg); t->nav.reset();
  t->started = t->sent = t->running = t->cancel_sent = t->stopped = t->terminal = t->close_failed = false;
  t->navigation_compute_ready = false;
  t->outcome = Code::UNKNOWN; advance();
}
void TaskManagerNode::navigation_done(const std::shared_ptr<Active> & t)
{
  if (!t->business()) {
    auto result = std::make_shared<Task::Result>(); result->message = t->error.empty() ? t->message : t->error;
    active_.reset(); inference_->business(false);
    if (t->outcome == Code::SUCCEEDED) {t->handle->succeed(result);}
    else if (t->outcome == Code::CANCELED) {t->handle->canceled(result);} else {t->handle->abort(result);}
    return;
  }
  cancel_deadline(); t->nav_active = false; t->pending = Pending::None; t->nav.reset();
  if (t->outcome != Code::SUCCEEDED && !t->cancel) {t->error = t->message;}
  if(t->is_survey()) {
    auto & s=*t->survey_run;
    if(s.phase==SurveyPhase::Return) {s.return_status=t->outcome==Code::SUCCEEDED?"succeeded":t->cancel && t->error.empty()?"canceled":"failed";}
    if(!t->error.empty() || t->cancel) {advance_survey();return;}
    settle_survey(s.next);return;
  }
  if (!t->error.empty() || t->cancel) {advance_fetch(); return;}
  settle(t->next_phase);
}
std::shared_ptr<rclcpp::Node> make_task_manager_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<TaskManagerNode>(options);
}
}  // namespace k1muse_manager
