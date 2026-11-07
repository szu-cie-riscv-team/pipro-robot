#include "k1muse_manager/task_manager_node_impl.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace k1muse_manager
{
namespace
{
int64_t stamp(const std_msgs::msg::Header & h)
{
  return h.stamp.sec < 0 || h.stamp.nanosec >= 1000000000u ? 0 :
    int64_t(h.stamp.sec) * 1000000000LL + h.stamp.nanosec;
}
bool pose_near(const ArmStatus & s, const Pose5 & p)
{
  return std::isfinite(s.current_x) && std::isfinite(s.current_y) && std::isfinite(s.current_z) &&
    std::hypot(std::hypot(s.current_x-p[0], s.current_y-p[1]), s.current_z-p[2]) <= .010;
}
}
const char * TaskManagerNode::phase_name(Phase p)
{
  switch (p) {
    case Phase::Prepare: return "prepare"; case Phase::ToShelf: return "to_shelf";
    case Phase::Lift: return "lift"; case Phase::Settle: return "settle";
    case Phase::Scan: return "scan"; case Phase::Detect: return "detect";
    case Phase::Align: return "align"; case Phase::Grasp: return "grasp";
    case Phase::Exit: return "exit"; case Phase::Recover: return "empty_recovery";
    case Phase::Withdraw: return "withdraw"; case Phase::Return: return "return";
    case Phase::Finish: return "finish";
  }
  return "unknown";
}
void TaskManagerNode::setup_fetch()
{
  arm_client_ = rclcpp_action::create_client<Arm>(this, "/k1muse/arm/command");
  lift_client_ = rclcpp_action::create_client<Lift>(this, "/k1muse/lift/command");
  yolo_ = create_client<Toggle>("/k1muse/perception/yolo/set_enabled");
  depth_control_ = create_client<Control>("/k1muse/perception/depth/control");
  fetch_cancel_ = create_client<action_msgs::srv::CancelGoal>("/k1muse/task/fetch_drug/_action/cancel_goal");
  arm_sub_ = create_subscription<ArmStatus>("/k1muse/arm/status", rclcpp::QoS(1).reliable(),
    [this](ArmStatus::ConstSharedPtr s) {arm_status_ = s; check_hardware();});
  lift_sub_ = create_subscription<LiftStatus>("/k1muse/lift/status", rclcpp::QoS(1).reliable(),
    [this](LiftStatus::ConstSharedPtr s) {lift_status_ = s; check_hardware();});
  odom_sub_ = create_subscription<Odom>("/odom", rclcpp::SensorDataQoS().keep_last(1),
    [this](Odom::ConstSharedPtr s) {on_odom(s);});
  local_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/k1muse/task/cmd_vel", rclcpp::QoS(1).reliable());
  fetch_server_ = rclcpp_action::create_server<Fetch>(this, "/k1muse/task/fetch_drug",
    [this](const auto & id, std::shared_ptr<const Fetch::Goal> g) {return admit_fetch(id, *g);},
    [this](std::shared_ptr<FetchHandle> h) {
      if (!active_ || active_->fetch != h) {return rclcpp_action::CancelResponse::REJECT;}
      request_cancel(); return rclcpp_action::CancelResponse::ACCEPT;
    }, [this](std::shared_ptr<FetchHandle> h) {
      active_ = std::move(prepared_goal_); active_->fetch = h; inference_->business(true);
      event("task_begin", "drug=" + active_->drug); advance_fetch();
    });
  inference_->on_vision_failure([this](const std::string & error) {
    if(!active_ || !active_->business()) {return;}
    // The validated release confirms actual visual stop. Do not race a redundant
    // SetBool close against the provider's still-in-flight release ACK.
    auto t=active_;t->yolo_maybe=false;detections_sub_.reset();targets_sub_.reset();
    fail("vision failed: "+error); // Snapshot the last observations before cleanup.
    t->candidates.clear();t->targets.reset();t->candidate={};
  });
}
bool TaskManagerNode::fresh(int64_t s, double age) const
{
  const double elapsed = (now().nanoseconds() - s)*1e-9;
  return s > 0 && elapsed >= 0 && elapsed <= age;
}
bool TaskManagerNode::hardware_ok(bool idle) const
{
  return arm_status_ && lift_status_ && fresh(stamp(arm_status_->header), config_.status_age) &&
    fresh(stamp(lift_status_->header), config_.status_age) && !arm_status_->has_fault &&
    !lift_status_->has_fault && lift_status_->is_homed &&
    std::isfinite(lift_status_->current_z) && (!idle || (!arm_status_->is_busy && !lift_status_->is_busy));
}
bool TaskManagerNode::stopped() const
{
  if(!odom_) {return false;}
  const auto & p=odom_->pose.pose.position; const auto & q=odom_->pose.pose.orientation;
  return odom_->header.frame_id=="odom" && odom_->child_frame_id=="base_link" &&
    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(q.x) && std::isfinite(q.y) &&
    std::isfinite(q.z) && std::isfinite(q.w) && std::abs(q.x)<1e-3 && std::abs(q.y)<1e-3 &&
    std::abs(q.z*q.z+q.w*q.w-1)<1e-3 && fresh(stamp(odom_->header), config_.status_age) &&
    std::isfinite(odom_->twist.twist.linear.x) && std::isfinite(odom_->twist.twist.linear.y) &&
    std::isfinite(odom_->twist.twist.angular.z) &&
    std::hypot(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y) < .02 &&
    std::abs(odom_->twist.twist.angular.z) < .05;
}
Json TaskManagerNode::feedback_state() const
{
  const auto current=now().nanoseconds();
  const auto age=[&](const auto & message) -> Json {
      return message ? Json((current-stamp(message->header))*1e-9) : Json(nullptr);
    };
  Json r={{"status_max_age_sec",config_.status_age},{"hardware_ok",hardware_ok()},{"stopped",stopped()},
    {"arm",nullptr},{"lift",nullptr},{"odom",nullptr}};
  if(arm_status_) {r["arm"]={{"age_sec",age(arm_status_)},{"busy",bool(arm_status_->is_busy)},
    {"fault",bool(arm_status_->has_fault)},
    {"xyz",{arm_status_->current_x,arm_status_->current_y,arm_status_->current_z}}};}
  if(lift_status_) {r["lift"]={{"age_sec",age(lift_status_)},{"busy",bool(lift_status_->is_busy)},
    {"fault",bool(lift_status_->has_fault)},{"homed",bool(lift_status_->is_homed)},{"z_m",lift_status_->current_z}};}
  if(odom_) {const auto & v=odom_->twist.twist;const auto & q=odom_->pose.pose.orientation;
    r["odom"]={{"age_sec",age(odom_)},{"frame",odom_->header.frame_id},{"child_frame",odom_->child_frame_id},
      {"velocity",{v.linear.x,v.linear.y,v.angular.z}},{"quaternion",{q.x,q.y,q.z,q.w}}};}
  return r;
}
const GraspProfile & TaskManagerNode::profile() const {return active_ && active_->is_fetch && active_->target.fetch && active_->target.fetch->grasp == "top" ? config_.top : config_.front;}
bool TaskManagerNode::matches(const std::shared_ptr<Active> & t, uint64_t operation) const
{
  return active_ == t && t->operation == operation;
}
std::string TaskManagerNode::fetch_admission_error(const WarehouseTarget * target) const
{
  if(!config_error_.empty()) {return config_error_;}
  const auto context_error=warehouse_context_error();
  if(!context_error.empty()) {return context_error;}
  if(!config_.enabled) {return "fetch_disabled";}
  if(!config_.calibrated) {return "calibration_unconfirmed";}
  if(active_) {return "task_busy";}
  if(recovery_pending_) {return "recovery_pending";}
  if(motion_inhibited_) {return "motion_inhibited";}
  if(holding_!=Holding::Empty) {return holding_==Holding::Confirmed?"holding_confirmed":"holding_unknown";}
  if(left_holding_!=Holding::Empty || right_holding_!=Holding::Empty) {return "transport_not_empty";}
  if(!hardware_ok()) {return "hardware_not_ready_or_stale";}
  if(!stopped()) {return "base_not_stopped";}
  if(!target || !target->executable() || (!config_.no_qr && !target->markers_available())) {return "no_executable_warehouse_target";}
  const auto & p=*target->fetch->grasp=="top" ? config_.top : config_.front;
  if(!pose_near(*arm_status_,p.poses.at("ready"))) {return "arm_not_at_selected_ready_pose";}
  if(!nav_->action_server_is_ready() || !gate_->service_is_ready() || !arm_client_->action_server_is_ready() ||
    !lift_client_->action_server_is_ready()) {return "fetch_backend_unavailable";}
  return "";
}
rclcpp_action::GoalResponse TaskManagerNode::admit_fetch(const rclcpp_action::GoalUUID & id, const Fetch::Goal & goal)
{
  prepared_goal_.reset();
  const WarehouseTarget * target=nullptr;
  try {
    if(goal.quantity<1 || goal.quantity>3) {throw std::runtime_error("quantity_invalid");}
    target=warehouse_.select(goal.drug_name, !config_.no_qr, config_.grasp_mode);
    const auto error=fetch_admission_error(target);
    if(!error.empty()) {throw std::runtime_error(error);}
    auto t=std::make_shared<Active>();t->is_fetch=true;t->kind=TaskKind::Fetch;t->target=*target;
    t->id=rclcpp_action::to_string(id);t->context=context_;t->drug=target->drug;
    t->requested_quantity=goal.quantity;t->planned_quantity=goal.quantity;
    if(const auto stock=inventory_.effective_quantity(*target)) {
      if(*stock==0) {throw std::runtime_error("inventory_empty");}
      t->planned_quantity=std::min<int>(goal.quantity,*stock);
    }
    RCLCPP_INFO(get_logger(),"Fetch %s requested=%u planned=%u",goal.drug_name.c_str(),t->requested_quantity,t->planned_quantity);
    t->expected_class=*target->fetch->class_id;
    const auto & p0=*target->observation_pose;t->pose.header.frame_id="map";
    t->pose.pose.position.x=p0.x_m;t->pose.pose.position.y=p0.y_m;
    t->pose.pose.orientation.z=std::sin(p0.yaw_rad/2);t->pose.pose.orientation.w=std::cos(p0.yaw_rad/2);
    Task::Goal back;back.task_name=config_.return_point;t->return_pose=resolve(back);prepared_goal_=t;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } catch(const std::exception & e) {
    last_fetch_rejection_={{"goal_id",rclcpp_action::to_string(id)},{"drug",goal.drug_name},
      {"stamp_ns",now().nanoseconds()},{"reason",e.what()},{"busy",bool(active_)},
      {"shelf_id",target?Json(target->shelf_id):Json(nullptr)},{"layer",target?Json(target->layer):Json(nullptr)},
      {"recovery_pending",recovery_pending_},{"motion_inhibited",motion_inhibited_},
      {"holding",holding_==Holding::Empty?"empty":holding_==Holding::Confirmed?"confirmed":"unknown"},
      {"feedback",feedback_state()}};
    RCLCPP_WARN(get_logger(),"Fetch rejected: %s",last_fetch_rejection_.dump().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
}
Json TaskManagerNode::inspect() const
{
  Json r = context_state();
  r["drug_catalog"]=drug_catalog_;
  r["holding"] = holding_ == Holding::Empty ? "empty" : holding_ == Holding::Confirmed ? "confirmed" : "unknown";
  const auto held=[](Holding h) {return h==Holding::Empty?"empty":h==Holding::Confirmed?"loaded":"unknown";};
  r["transport"]={{"left",held(left_holding_)},{"right",held(right_holding_)},{"hand",r["holding"]}};
  r["recovery_pending"] = recovery_pending_; r["config_file"] = config_file_; r["fetch_enabled"] = config_.enabled;
  r["no_qr"] = config_.no_qr;
  r["feedback"] = feedback_state();
  r["last_fetch_rejection"] = last_fetch_rejection_;
  r["warehouse"] = warehouse_state();
  const auto context_error=warehouse_context_error();
  const bool executable=std::any_of(warehouse_.targets.begin(),warehouse_.targets.end(),[this](const auto & target){return target.executable() && target.fetch && target.fetch->grasp==config_.grasp_mode && (config_.no_qr || target.markers_available());});
  r["fetch_available"] = config_.enabled && config_.calibrated && context_error.empty() && executable;
  r["fetch_flow_error"] = !context_error.empty() ? context_error :
    !config_.enabled ? "fetch_disabled" : !config_.calibrated ? "calibration_unconfirmed" : !executable ? "no_executable_warehouse_target" : "";
  r["fetch_admission"]=Json::object();
  bool fetch_ready=false;std::string blocked_reason=fetch_admission_error(nullptr);
  for(const auto & target:warehouse_.targets) {
    if(target.drug.empty() || r["fetch_admission"].contains(target.drug)) {continue;}
    const auto * selected=warehouse_.select(target.drug,!config_.no_qr,config_.grasp_mode);
    const auto error=fetch_admission_error(selected);
    const auto stock=selected?inventory_.effective_quantity(*selected):std::optional<int>{};
    const std::string admission=error.empty() && stock && *stock==0?"inventory_empty":error;
    r["fetch_admission"][target.drug]={{"ready",admission.empty()},{"reason",admission}};
    fetch_ready=fetch_ready || error.empty();
    if(!error.empty()) {blocked_reason=error;}
  }
  r["fetch_ready"]=fetch_ready;r["fetch_blocked_reason"]=fetch_ready?"":blocked_reason;
  r["last_result"] = last_result_;
  r["survey"]=survey_state();
  r["navigation"]=active_?navigation_diagnostics(*active_):last_navigation_;
  if (active_ && active_->is_fetch) {
    const auto & t = *active_;
    r["requested_quantity"]=t.requested_quantity;r["planned_quantity"]=t.planned_quantity;r["fetched_quantity"]=t.fetched_quantity;
    r["phase"] = phase_name(t.phase); r["step_id"] = t.step_id;
    const char * pending_names[]={"none","service","navigation","arm","lift","arm_status","lift_status","local"};
    r["pending"] = pending_names[static_cast<size_t>(t.pending)]; r["cancel_requested"] = t.cancel;
    r["pause_requested"] = t.pause_requested; r["blocked"] = t.blocked; r["error"] = t.error;
    r["shelf_id"] = t.target.shelf_id; r["layer"] = t.target.layer; r["drug"] = t.drug; r["search_offset_m"] = t.search_offset;
    r["search_ready"]=t.search_ready;r["left_confirmed"]=t.left_confirmed;r["right_reached"]=t.right_reached;
    r["qr_context_error"]=t.qr_context_error;
    r["cv_source_ns"]=t.cv_frame_stamp;r["boundary_wait"]=t.boundary_wait;
    r["left_marker_q"]=t.left_marker.stamp ? Json(t.left_marker.q) : Json(nullptr);
    r["right_marker_q"]=t.right_marker.stamp ? Json(t.right_marker.q) : Json(nullptr);
    r["right_limit_m"]=t.right_limit_m;r["search_start_offset_m"]=t.search_start_offset;
    r["shelf_reference_odom"]={t.shelf_odom_x,t.shelf_odom_y};
    r["candidate_q"]=t.candidate.valid ? Json(t.candidate.q) : Json(nullptr);
    r["candidate_source_ns"]=t.candidate.stamp;r["point_barrier_ns"]=t.barrier;
    r["shelf_inward_odom"]={t.inward_x,t.inward_y};r["shelf_right_odom"]={t.right_x,t.right_y};
    r["yaw_hold_rad"]=t.shelf_yaw;r["camera_right_sign"]=t.camera_right_sign;
    r["waiting_point"]=t.waiting_point;r["reacquisitions"]=t.reacquisitions;
    r["locked_id"]=t.locked_id;r["tracking_lost"]=t.tracking_lost;
    r["vision"]=vision_diagnostics(t);r["failure_snapshot"]=t.failure_snapshot;r["hardware_result"]=t.hardware_diag;
    r["target_lost_sec"]=t.tracking_lost?std::chrono::duration<double>(Clock::now()-t.lost_since).count():0.0;
    r["scan_elapsed_sec"]=t.scan_elapsed;r["search_elapsed_sec"]=t.search_elapsed;r["framing_elapsed_sec"]=t.framing_elapsed;
    r["grasp"]=*t.target.fetch->grasp;r["loaded_revision"]=warehouse_.revision;
    r["tcp"] = t.plan.tcp; r["approach"] = t.plan.approach; r["target_stamp_ns"] = t.target_stamp;
    r["camera_forward_depth_m"]=t.camera_distance?Json(*t.camera_distance):Json(nullptr);
    r["camera_right_offset_m"]=t.camera_lateral?Json(*t.camera_lateral):Json(nullptr);
    r["approach_steps"]=t.approach_steps;r["approach_inward_m"]=t.inward_offset;
    r["initial_shelf_clearance_m"]=t.initial_shelf_clearance;r["approach_limit_m"]=t.approach_limit;
    r["surface_arm_base"]=t.target_stamp?Json(t.surface):Json(nullptr);
  }
  return r;
}
void TaskManagerNode::task_control(const Control::Request & req, Control::Response & res)
{
  const auto a = Json::parse(req.arguments.empty() ? "{}" : req.arguments);
  if(active_ && active_->is_survey() && (req.operation=="pause" || req.operation=="next" ||
    req.operation=="continue" || req.operation=="breakpoints")) {throw std::runtime_error("unsupported_for_survey");}
  if(req.operation=="inventory_get" || req.operation=="inventory_set") {
    if(!warehouse_loaded_ || a.at("map_id")!=warehouse_.map_id) {throw std::runtime_error("map_changed");}
    if(req.operation=="inventory_get") {res.result=inventory_.inventory_state(warehouse_).dump();return;}
    if(active_ || prepared_goal_) {throw std::runtime_error("business_busy");}
    const auto & q=a.at("quantity");
    if(!q.is_number_integer() || q.get<double>()<0 || q.get<double>()>INT32_MAX) {throw std::runtime_error("quantity_invalid");}
    const auto it=std::find_if(warehouse_.targets.begin(),warehouse_.targets.end(),[&](const auto & t) {
      return t.shelf_id==a.at("shelf_id") && t.layer==a.at("layer");
    });
    if(it==warehouse_.targets.end()) {throw std::runtime_error("layer_not_found");}
    if(it->drug.empty() || it->drug!=a.at("drug")) {throw std::runtime_error("drug_changed");}
    res.result=inventory_.set_inventory_quantity(it->shelf_id,it->layer,it->drug,q.get<int>()).dump();return;
  }
  if (req.operation == "inspect") {}
  else if (req.operation == "preview") {
    Json r = inspect(); r["pick"] = Json::array(); r["exit"] = Json::array();
    const auto & grasp=config_.grasp_mode;
    if(a.contains("drug_name")) {
      const auto * target=warehouse_.select(a.at("drug_name").get<std::string>(), !config_.no_qr, config_.grasp_mode);
      if(!target) {
        const auto name=a.at("drug_name").get<std::string>();
        const auto found=std::find_if(warehouse_.targets.begin(),warehouse_.targets.end(),[&](const auto & v){return v.drug==name;});
        if(found!=warehouse_.targets.end()) {target=&*found;}
      }
      if(!target) {throw std::runtime_error("no warehouse drug target");}
      r["selected_shelf_id"]=target->shelf_id;r["selected_layer"]=target->layer;r["selected_target_available"]=target->executable() && target->fetch && target->fetch->grasp==config_.grasp_mode && (config_.no_qr || target->markers_available());
    }
    const auto & p = grasp == "front" ? config_.front : config_.top; r["grasp"] = grasp;
    for (const auto & step : p.pick) {r["pick"].push_back({{"id",step.id},{"target",step.target},{"kind",int(step.kind)}});}
    for (const auto & step : p.exit) {r["exit"].push_back({{"id",step.id},{"target",step.target},{"kind",int(step.kind)}});}
    {
      const auto quantity=a.value("quantity",1);
      if(quantity<1 || quantity>3) {throw std::runtime_error("quantity_invalid");}
      int planned=quantity;
      if(a.contains("drug_name")) {
        if(const auto * target=warehouse_.select(a.at("drug_name").get<std::string>(),!config_.no_qr,config_.grasp_mode)) {
          if(const auto stock=inventory_.effective_quantity(*target)) {planned=std::min(planned,*stock);}
        }
      }
      r["requested_quantity"]=quantity;r["planned_quantity"]=planned;
      r["placement"]=Json::array();
      for(int i=0;i<planned;++i) {r["placement"].push_back({{"destination",i==planned-1?"hand":i==0?"left":"right"},
        {"command",grasp=="top"?Json(i==planned-1?ArmGoal::TOP_PICK_HOLD:i==0?ArmGoal::TOP_PICK_PLACE_LEFT:ArmGoal::TOP_PICK_PLACE_RIGHT):
          i==planned-1?Json(nullptr):Json(i==0?ArmGoal::PLACE_LEFT:ArmGoal::PLACE_RIGHT)}});}
    }
    r["grasp_profiles"]=Json::object();
    for(const auto name:{"front","top"}) {
      const auto & summary=std::string(name)=="front" ? config_.front : config_.top;
      Json pick=Json::array(),exit=Json::array();
      for(const auto & step:summary.pick) {pick.push_back({{"id",step.id},{"target",step.target},{"kind",int(step.kind)}});}
      for(const auto & step:summary.exit) {exit.push_back({{"id",step.id},{"target",step.target},{"kind",int(step.kind)}});}
      r["grasp_profiles"][name]={{"calibrated",summary.calibrated},{"pick",pick},{"exit",exit}};
    }
    r["warehouse_targets"] = Json::array();
    for (const auto & target : warehouse_.targets) {
      Json row = {{"shelf_id", target.shelf_id}, {"layer", target.layer},
        {"drug", target.drug}, {"executable", target.executable()},
        {"unavailable_reasons", target.unavailable_reasons}, {"marker_reasons", target.marker_reasons}, {"grasp", target.fetch && target.fetch->grasp ? Json(*target.fetch->grasp) : Json(nullptr)}, {"initial_lift_z_m", nullptr}, {"observation_pose", nullptr}};
      if (target.initial_lift_z_m) {row["initial_lift_z_m"] = *target.initial_lift_z_m;}
      if (target.observation_pose) {
        const auto & pose = *target.observation_pose;
        row["observation_pose"] = {{"x_m", pose.x_m}, {"y_m", pose.y_m}, {"yaw_rad", pose.yaw_rad}};
      }
      r["warehouse_targets"].push_back(std::move(row));
    }
    res.result = r.dump(); return;
  } else if (req.operation == "reload") {
    if (active_ || motion_inhibited_ || holding_ != Holding::Empty || left_holding_!=Holding::Empty || right_holding_!=Holding::Empty) {throw std::runtime_error("reload requires confirmed idle/empty hardware");}
    auto c = load_fetch_config(config_file_);
    reload_fetch_profiles(config_, c);
  } else if (req.operation == "confirm_empty") {
    if (active_ || recovery_pending_ || !a.value("confirmed", false) || !hardware_ok() || !stopped() ||
      !ready_pose_ok())
    {throw std::runtime_error("operator confirmation and fresh idle hardware at ready pose required");}
    if (!motion_inhibited_) {holding_ = left_holding_ = right_holding_ = Holding::Empty;}
    else {
      if (!gate_->service_is_ready()) {throw std::runtime_error("motion owner unavailable for recovery");}
      recovery_pending_ = true; const auto op = ++recovery_sequence_;
      auto request = std::make_shared<Control::Request>(); request->operation = "confirm_motion_ready";
      request->client_id = instance_; request->request_id = std::to_string(++gate_sequence_);
      request->arguments = Json{{"manager_instance",instance_},{"context_id",context_},{"sequence",gate_sequence_}}.dump();
      task_deadline_ = create_wall_timer(std::chrono::duration<double>(config_.service_timeout),[this,op] {
        if (op != recovery_sequence_) {return;} cancel_deadline(); recovery_pending_=false;
        ++recovery_sequence_; last_result_["recovery_error"]="motion recovery acknowledgement timeout";
      });
      gate_->async_send_request(request,[this,op](rclcpp::Client<Control>::SharedFuture f) {
        if (op != recovery_sequence_) {return;} cancel_deadline(); recovery_pending_=false;
        if (f.get()->accepted && hardware_ok() && stopped() && ready_pose_ok()) {
          holding_=left_holding_=right_holding_=Holding::Empty; motion_inhibited_=false;
        } else {last_result_["recovery_error"]="motion recovery rejected or hardware changed";}
      });
    }
  } else if (req.operation == "breakpoints") {
    std::set<std::string> next;
    for (const auto & v : a.at("steps")) {
      const auto s = v.get<std::string>();
      if (s.size() > 128 || (s.rfind("before:",0) && s.rfind("after:",0))) {throw std::runtime_error("expected before:step or after:step");}
      next.insert(s);
    }
    breakpoints_ = std::move(next);
  } else if (req.operation == "cancel") {cancel_current();}
  else if (req.operation == "pause" || req.operation == "next" || req.operation == "continue") {
    if (!active_ || !active_->is_fetch || active_->blocked || active_->cancel) {throw std::runtime_error("no controllable fetch task");}
    auto t = active_;
    if (req.operation == "pause") {
      update_budgets(); t->pause_requested = true;
      if (t->pending == Pending::Local) {stop_local();} else {advance_fetch();}
    } else {
      if (!t->pause_requested || t->pending != Pending::None || t->nav_active) {throw std::runtime_error("wait for the current operation to finish pausing");}
      if(req.operation=="next" && !t->paused_boundary.empty()) {throw std::runtime_error("stage breakpoints require continue; next advances one ARM step only");}
      if(t->paused_boundary.rfind("before:",0)==0) {t->boundary_skip=t->paused_boundary;}
      t->paused_boundary.clear();t->budget_tick=Clock::now();
      t->pause_requested = false; t->one_step = req.operation == "next";
      // A pause before contact invalidates the snapshot. Do not grip an old point.
      if (t->phase == Phase::Grasp && holding_ != Holding::Confirmed && !fresh(t->target_stamp, config_.observation_age)) {
        if (t->step_index == 0) {enter(t->search_ready ? Phase::Detect : Phase::Scan,"reobserve");}
        else {
          enter(Phase::Recover,"paused_target_recovery"); t->step_index = 0;
          ArmGoal open; open.command = ArmGoal::GRIPPER; open.target_rad = profile().open_rad;
          ArmGoal retreat = t->plan.pick.at(0).goal; retreat.command = ArmGoal::MOVE_POSE;
          retreat.x = t->plan.approach[0]; retreat.y = t->plan.approach[1]; retreat.z = t->plan.approach[2];
          retreat.roll = profile().roll; retreat.pitch = profile().pitch;
          ArmGoal ready = retreat; const auto & p = profile().poses.at("ready");
          ready.x=p[0]; ready.y=p[1]; ready.z=p[2]; ready.roll=p[3]; ready.pitch=p[4];
          t->recovery = {{"recovery_open",open},{"recovery_retreat",retreat},{"recovery_ready",ready}};
        }
      }
      // Clearing this step's breakpoint once permits stepping past it.
      if (!t->step_id.empty()) {t->skip_before=t->step_id;}
      advance();
    }
  } else {throw std::runtime_error("unknown task control operation");}
  res.result = inspect().dump();
}
void TaskManagerNode::event(const std::string & name, const std::string & detail)
{
  if (!active_ || !active_->business()) {return;}
  auto t = active_;
  if(t->is_survey()) {RCLCPP_INFO(get_logger(),"SURVEY id=%s event=%s %s",t->id.c_str(),name.c_str(),detail.c_str());survey_feedback();return;}
  RCLCPP_INFO(get_logger(), "TASK id=%s phase=%s step=%s op=%llu event=%s %s", t->id.c_str(),
    phase_name(t->phase), t->step_id.c_str(), static_cast<unsigned long long>(t->operation), name.c_str(), detail.c_str());
  if (t->fetch && t->fetch->is_active()) {
    auto feedback = std::make_shared<Fetch::Feedback>(); feedback->stage = std::string(phase_name(t->phase))+"/"+name;
    feedback->planned_quantity=t->planned_quantity;feedback->fetched_quantity=t->fetched_quantity;
    t->fetch->publish_feedback(feedback);
  }
}
void TaskManagerNode::enter(Phase phase, const std::string & step)
{
  update_budgets(); cancel_deadline(); auto t = active_; t->phase = phase; t->step_id = step;
  t->waiting_permit = false; t->candidates.clear();t->targets.reset(); t->step_started = Clock::now();
  event("enter");
}
void TaskManagerNode::cancel_deadline()
{
  if (task_deadline_) {task_deadline_->cancel(); task_deadline_.reset();}
}
void TaskManagerNode::deadline(double seconds, const std::string & error)
{
  cancel_deadline(); auto t = active_; const auto op = t->operation;
  task_deadline_ = create_wall_timer(std::chrono::duration<double>(seconds), [this,t,op,error] {
    if (!matches(t,op)) {return;} cancel_deadline();
    fail(error, t->pending != Pending::None);
  });
}
void TaskManagerNode::fail(const std::string & error, bool uncertain)
{
  if (!active_) {return;} auto t = active_;
  if (t->error.empty()) {t->error = error;}
  capture_failure();
  event("failed", "reason="+error);
  if(t->local_open) {zero_local();}
  if (t->nav_active) {t->cancel = true; cancel_nav(t); advance(); return;}
  if (t->pending == Pending::Local) {stop_local(); return;}
  if (uncertain) {t->blocked = true; event("blocked"); return;}
  t->blocked = false; advance_business();
}
void TaskManagerNode::capture_failure()
{
  auto t=active_;if(!t || t->error.empty() || !t->failure_snapshot.is_null()) {return;}
  const auto & g=t->arm_goal;
  t->failure_snapshot={{"reason",t->error},{"phase",phase_name(t->phase)},{"step",t->step_id},
    {"operation",t->operation},{"stamp_ns",now().nanoseconds()},
    {"step_elapsed_ms",std::chrono::duration<double,std::milli>(Clock::now()-t->step_started).count()},
    {"vision",vision_diagnostics(*t)},{"hardware_result",t->hardware_diag},{"feedback",feedback_state()},
    {"navigation",navigation_diagnostics(*t)},
    {"last_arm_request",{{"command",g.command},{"xyz",{g.x,g.y,g.z}},{"roll",g.roll},{"pitch",g.pitch},{"target_rad",g.target_rad}}},
    {"last_lift_goal_m",t->lift_goal},{"target_stamp_ns",t->target_stamp},{"surface_arm_base",t->surface},
    {"search_offset_m",t->search_offset},{"framing_elapsed_sec",t->framing_elapsed}};
  event("failure_snapshot",t->failure_snapshot.dump());
}
void TaskManagerNode::web_call(const std::string & op, Json args, std::function<void(const Json &)> done)
{
  auto t = active_; t->pending = Pending::Service; auto sequence = ++t->operation;
  args["manager_instance"] = instance_; args["context_id"] = t->context;
  args["business_id"] = t->id; args["sequence"] = ++gate_sequence_;
  auto req = std::make_shared<Control::Request>(); req->client_id = instance_;
  req->operation = op; req->request_id = std::to_string(gate_sequence_); req->arguments = args.dump();
  const bool opening = op == "set_business" && args.value("state",std::string()) == "starting";
  if (!gate_->service_is_ready()) {
    t->pending=Pending::None;
    // A missing close/stop service cannot release an existing owner. Re-entering
    // finish_fetch/stop_local here would recursively retry the same unavailable RPC.
    fail("web motion owner unavailable", !opening); return;
  }
  deadline(op == "set_navigation_compute" ? 16.0 : config_.service_timeout, "web boundary acknowledgement timeout");
  try {gate_->async_send_request(req,[this,t,sequence,done,opening](rclcpp::Client<Control>::SharedFuture f) {
    if (!matches(t,sequence)) {return;} cancel_deadline(); t->pending=Pending::None; t->blocked=false;
    try {
      auto r=f.get(); if (!r->accepted) {fail("web boundary rejected: "+r->error,!opening); return;}
      done(Json::parse(r->result.empty()?"{}":r->result));
    } catch (const std::exception & e) {fail(e.what(),true);}
  });} catch (const std::exception & e) {fail(e.what(),true);}
}
void TaskManagerNode::depth_mode(const std::string & mode, std::function<void()> done)
{
  auto t = active_;
  if (t->depth_mode == mode) {done(); return;}
  t->pending = Pending::Service; const auto sequence = ++t->operation;
  // Keep ownership on an uncertain alignment/mode transition, like the visual permits.
  t->depth_mode = "unresolved";
  if (!depth_control_ || !depth_control_->service_is_ready()) {
    fail("depth control unavailable", true); return;
  }
  auto request = std::make_shared<Control::Request>();
  request->client_id = "task_manager"; request->operation = "set_mode";
  request->request_id = t->id + ":depth:" + std::to_string(sequence);
  request->arguments = Json{{"mode", mode}}.dump();
  deadline(std::max(5.0, config_.service_timeout), "depth mode acknowledgement timeout");
  try {depth_control_->async_send_request(request,
    [this,t,sequence,mode,done](rclcpp::Client<Control>::SharedFuture future) {
      if (!matches(t,sequence)) {return;}
      cancel_deadline();
      const auto response = future.get();
      if (!response->accepted) {fail("depth mode rejected: " + response->error, true); return;}
      t->depth_mode = mode; t->pending = Pending::None; done();
    });
  } catch (const std::exception & error) {fail(error.what(),true);}
}
void TaskManagerNode::toggle_yolo(bool enabled, std::function<void()> done)
{
  auto t=active_;
  const auto mode = t->is_survey() ? "marker" : "both";
  if (enabled && t->depth_mode != mode) {
    depth_mode(mode, [this,enabled,done] {toggle_yolo(enabled,done);}); return;
  }
  t->pending=Pending::Service; auto op=++t->operation;
  if (enabled) {t->yolo_maybe=true;t->tracking_lost=false;}
  if (!yolo_->service_is_ready()) {
    if(enabled) {t->yolo_maybe=false;t->pending=Pending::None;}
    fail("YOLO control unavailable",!enabled);return;
  }
  auto req=std::make_shared<Toggle::Request>(); req->data=enabled;
  deadline(config_.service_timeout,"YOLO acknowledgement timeout");
  try {yolo_->async_send_request(req,[this,t,op,enabled,done](rclcpp::Client<Toggle>::SharedFuture f) {
    if (!matches(t,op)) {return;} cancel_deadline();
    const auto r=f.get();
    if (!r->success) {
      t->blocked=true;if(t->error.empty()) {t->error=r->message;}capture_failure();event("blocked","reason="+r->message);return;
    }
    t->pending=Pending::None; t->blocked=false;
    if (!enabled) {t->yolo_maybe=false;detections_sub_.reset();targets_sub_.reset();t->candidates.clear();t->targets.reset();t->candidate={};
      if(t->pause_requested) {t->tracking_lost=false;}} // Preserve terminal loss evidence for last_result.
    done();
  });} catch (const std::exception & e) {fail(e.what(),true);}
}
void TaskManagerNode::stop_vision(std::function<void()> done)
{
  auto t=active_;
  if(local_timer_) {local_timer_->cancel();local_timer_.reset();}
  if(t->is_survey()) {t->survey_run->qr_request=0;}
  if(qr_request_pub_) {qr_request_pub_->publish(D2{});}qr_point_sub_.reset();
  camera_info_sub_.reset();tf_listener_.reset();tf_buffer_.reset();
  if (t->yolo_maybe) {toggle_yolo(false,[this,done]{stop_vision(done);});}
  else if (t->cv_maybe) {
    t->pending=Pending::Service; auto op=++t->operation;
    deadline(config_.service_timeout,"CV close timeout");
    inference_->scan(false,t->id+":cv",[this,t,op,done](bool ok) {
      if (!matches(t,op)) {return;} cancel_deadline();
      if (!ok) {t->blocked=true; if(t->error.empty()) {t->error="CV close not acknowledged";} capture_failure();event("blocked"); return;}
      t->cv_maybe=false; cv_sub_.reset(); tf_listener_.reset(); tf_buffer_.reset(); t->pending=Pending::None; t->blocked=false; stop_vision(done);
    });
  } else if (t->depth_mode != "off") {depth_mode("off", [this,done] {stop_vision(done);});}
  else {camera_info_sub_.reset();tf_listener_.reset();tf_buffer_.reset();done();}
}
void TaskManagerNode::settle(Phase next)
{
  auto t=active_; enter(Phase::Settle,"wait_stopped"); t->next_phase=next;
  web_call("wait_stopped",Json::object(),[this,t](const Json &) {
    if (!hardware_ok() || !stopped()) {fail("hardware not idle/stopped after positioning"); return;}
    const auto next=t->next_phase;enter(next,phase_name(next));t->barrier=now().nanoseconds();
    if(next==Phase::Detect && t->boundary_wait) {t->candidate={};}
    if(next==Phase::Align) {if(t->tracking_lost) {t->lost_since=Clock::now();}t->waiting_point=true;t->candidate={};t->candidates.clear();t->targets.reset();stage_boundary("after:framing_stop");}
    advance_fetch();
  });
}
const std::vector<ArmStep> & TaskManagerNode::steps() const
{
  return active_->phase==Phase::Recover ? active_->recovery : active_->phase==Phase::Exit ? active_->plan.exit : active_->plan.pick;
}
bool TaskManagerNode::pause_boundary(const std::string & id)
{
  auto t=active_; t->step_id=id;
  if(t->skip_before==id) {t->skip_before.clear();}
  else if(!t->one_step && breakpoints_.count("before:"+id)) {t->pause_requested=true;}
  if(t->pause_requested) {cancel_deadline(); return true;} return false;
}
void TaskManagerNode::send_arm(const ArmStep & step)
{
  auto t=active_;
  if(!hardware_ok() || !stopped()) {fail("ARM precondition failed"); return;}
  t->pending=Pending::Arm; t->arm_goal=step.goal; t->sent_stamp=now().nanoseconds(); t->hardware_result=false;t->grip_succeeded=false;
  t->hardware_diag=nullptr;
  if(step.goal.command==ArmGoal::MOVE_POSE || step.goal.command==ArmGoal::TOP_PICK_HOLD ||
    step.goal.command==ArmGoal::TOP_PICK_PLACE_LEFT || step.goal.command==ArmGoal::TOP_PICK_PLACE_RIGHT ||
    step.goal.command==ArmGoal::PLACE_LEFT || step.goal.command==ArmGoal::PLACE_RIGHT) {motion_inhibited_=true;}
  const auto op=++t->operation; event("step_begin","command="+std::to_string(step.goal.command)+
    " xyz="+std::to_string(step.goal.x)+","+std::to_string(step.goal.y)+","+std::to_string(step.goal.z));
  deadline(config_.step_timeout,"ARM result/status timeout");
  rclcpp_action::Client<Arm>::SendGoalOptions opts;
  opts.goal_response_callback=[this,t,op](auto h) {
    if(matches(t,op) && !h) {t->pending=Pending::None; t->blocked=false; fail("ARM goal rejected");}
  };
  opts.result_callback=[this,t,op](const rclcpp_action::ClientGoalHandle<Arm>::WrappedResult & r) {
    if(!matches(t,op)) {return;}
    t->blocked=false; const auto v=r.result;
    t->hardware_diag={{"kind","arm"},{"step",t->step_id},{"operation",op},{"action",int(r.code)},
      {"result",v?Json(v->result):Json(nullptr)},{"reason",v?Json(v->reason):Json(nullptr)},
      {"fault",v?Json(v->fault_source):Json(nullptr)},{"diag_code",v?Json(v->diag_code):Json(nullptr)},
      {"demo_valid",v && v->demo_valid},{"holding",v && v->demo_valid?Json(v->holding):Json(nullptr)},
      {"demo_step",v && v->demo_valid?Json(v->demo_step):Json(nullptr)}};
    event("arm_result","action="+std::to_string(int(r.code))+" result="+std::to_string(v?v->result:0)+
      " reason="+std::to_string(v?v->reason:0)+" fault="+std::to_string(v?v->fault_source:0));
    if(t->arm_goal.command==ArmGoal::GRIP && v && r.code==Code::SUCCEEDED && v->result==3 && v->reason==0 && v->fault_source==0) {t->grip_succeeded=true;}
    const bool empty_grip=t->arm_goal.command==ArmGoal::GRIP && v && r.code==Code::ABORTED &&
      v->result==5 && v->reason==9 && v->fault_source==0;
    if(empty_grip &&
      !t->cancel && t->error.empty() && t->grip_attempts++==0) {
      holding_=Holding::Empty; t->pending=Pending::ArmStatus; cancel_deadline();
      ArmGoal open; open.command=ArmGoal::GRIPPER; open.target_rad=profile().open_rad;
      ArmGoal back=t->arm_goal; back.command=ArmGoal::MOVE_POSE; back.x=t->plan.approach[0]; back.y=t->plan.approach[1]; back.z=t->plan.approach[2]; back.roll=profile().roll;back.pitch=profile().pitch;
      ArmGoal ready=back; const auto & p=profile().poses.at("ready"); ready.x=p[0];ready.y=p[1];ready.z=p[2];ready.roll=p[3];ready.pitch=p[4];
      t->recovery={{"empty_open",open},{"empty_retreat",back},{"empty_ready",ready}};
      t->step_index=0; enter(Phase::Recover,"empty_recovery"); deadline(config_.step_timeout,"empty grip idle status timeout");check_hardware(); return;
    }
    if(r.code!=Code::SUCCEEDED || !v || v->result!=3 || v->reason!=0 || v->fault_source!=0) {
      t->pending=Pending::ArmStatus;
      if(t->arm_goal.command>=ArmGoal::TOP_PICK_HOLD && t->arm_goal.command<=ArmGoal::PLACE_RIGHT) {
        // DEMO holding describes the hand, not a sensor-confirmed tray state.
        holding_=v && v->demo_valid && v->holding==2?Holding::Confirmed:
          v && v->demo_valid && (v->holding==1 || v->holding==3)?Holding::Empty:Holding::Unknown;
        if(t->arm_goal.command==ArmGoal::TOP_PICK_PLACE_LEFT || t->arm_goal.command==ArmGoal::PLACE_LEFT) {left_holding_=Holding::Unknown;}
        if(t->arm_goal.command==ArmGoal::TOP_PICK_PLACE_RIGHT || t->arm_goal.command==ArmGoal::PLACE_RIGHT) {right_holding_=Holding::Unknown;}
      }
      if(t->arm_goal.command==ArmGoal::GRIP && holding_!=Holding::Confirmed) {holding_=empty_grip ? Holding::Empty : Holding::Unknown;}
      if(t->error.empty()) {t->error="ARM terminal failure step="+t->step_id+" details="+t->hardware_diag.dump();}
      capture_failure();
      check_hardware(); return;
    }
    t->pending=Pending::ArmStatus; t->hardware_result=true; check_hardware();
  };
  try {arm_client_->async_send_goal(step.goal,opts);} catch(const std::exception & e) {fail(e.what(),true);}
}
void TaskManagerNode::send_lift(double target, Phase next)
{
  auto t=active_;
  const float wire_target=static_cast<float>(target);
  if(!hardware_ok() || !stopped() || motion_inhibited_ || !std::isfinite(wire_target) || wire_target<static_cast<float>(kLiftMinM) || wire_target>static_cast<float>(kLiftMaxM)) {fail("LIFT precondition/range failed");return;}
  t->pending=Pending::Lift; t->lift_goal=target; t->next_phase=next; t->sent_stamp=now().nanoseconds(); t->hardware_result=false;
  t->hardware_diag=nullptr;
  const auto op=++t->operation; Lift::Goal goal; goal.command=Lift::Goal::MOVE_Z;goal.target_z_m=target;
  deadline(config_.step_timeout,"LIFT result/status timeout"); event("lift_begin","z="+std::to_string(target));
  rclcpp_action::Client<Lift>::SendGoalOptions opts;
  opts.goal_response_callback=[this,t,op](auto h) {if(matches(t,op) && !h) {t->pending=Pending::None; t->blocked=false;fail("LIFT goal rejected");}};
  opts.result_callback=[this,t,op](const rclcpp_action::ClientGoalHandle<Lift>::WrappedResult & r) {
    if(!matches(t,op)) {return;} t->blocked=false;
    t->hardware_diag={{"kind","lift"},{"step",t->step_id},{"operation",op},{"action",int(r.code)},
      {"result",r.result?Json(r.result->result):Json(nullptr)},
      {"reject",r.result?Json(r.result->reject_reason):Json(nullptr)},{"fault",r.result?Json(r.result->fault_reason):Json(nullptr)}};
    event("lift_result","action="+std::to_string(int(r.code))+" result="+std::to_string(r.result?r.result->result:0)+
      " reject="+std::to_string(r.result?r.result->reject_reason:0)+" fault="+std::to_string(r.result?r.result->fault_reason:0));
    if(r.code!=Code::SUCCEEDED || !r.result || r.result->result!=3 || r.result->reject_reason || r.result->fault_reason) {
      t->pending=Pending::LiftStatus; if(t->error.empty()) {t->error="LIFT terminal failure details="+t->hardware_diag.dump();}
      capture_failure();check_hardware(); return;
    }
    t->pending=Pending::LiftStatus; t->hardware_result=true; check_hardware();
  };
  try {lift_client_->async_send_goal(goal,opts);} catch(const std::exception & e) {fail(e.what(),true);}
}
void TaskManagerNode::check_hardware()
{
  auto t=active_; if(!t || !t->business()) {return;}
  if((t->pending!=Pending::ArmStatus && t->pending!=Pending::LiftStatus) || !hardware_ok() || !stopped()) {return;}
  if(t->pending==Pending::LiftStatus) {
    if(stamp(lift_status_->header)<t->sent_stamp) {return;}
  } else {
    if(stamp(arm_status_->header)<t->sent_stamp) {return;}
  }
  t->blocked=false; hardware_done();
}
void TaskManagerNode::hardware_done()
{
  auto t=active_; const bool lift=t->pending==Pending::LiftStatus;
  cancel_deadline(); t->pending=Pending::None;
  event("step_end","elapsed_ms="+std::to_string(std::chrono::duration<double,std::milli>(Clock::now()-t->step_started).count()));
  if(!lift && t->hardware_result && t->grip_succeeded && t->arm_goal.command==ArmGoal::GRIP) {holding_=Holding::Confirmed;}
  if(!lift && t->hardware_result && t->arm_goal.command>=ArmGoal::TOP_PICK_HOLD &&
    t->arm_goal.command<=ArmGoal::PLACE_RIGHT) {
    t->hardware_result=false;motion_inhibited_=false;complete_fetch_item();return;
  }
  if(lift && t->is_survey()) {if(t->cancel || !t->error.empty()) {advance_survey();return;}settle_survey(t->survey_run->next);return;}
  if(lift) {if(t->cancel || !t->error.empty()) {advance_fetch();return;} settle(t->next_phase);return;}
  if(t->hardware_result && t->arm_goal.command==ArmGoal::MOVE_POSE &&
    ([&] {
      for(const auto * name:{"ready","carry"}) {
        const auto found=profile().poses.find(name);if(found==profile().poses.end()) {continue;}
        const auto & p=found->second;const auto & g=t->arm_goal;
        if(std::abs(g.roll-p[3])<1e-3 && std::abs(g.pitch-p[4])<1e-3 &&
          std::abs(g.x-p[0])<1e-4 && std::abs(g.y-p[1])<1e-4 && std::abs(g.z-p[2])<1e-4) {return true;}
      }return false;
    })()) {motion_inhibited_=false;}
  if(!(t->phase==Phase::Recover && t->arm_goal.command==ArmGoal::GRIP)) {++t->step_index;}
  if(t->one_step || breakpoints_.count("after:"+t->step_id)) {t->one_step=false;t->pause_requested=true;}
  if(t->hardware_result && t->phase==Phase::Exit && t->target.fetch->grasp=="front" &&
    t->step_index>=steps().size() && t->fetched_quantity+1>=t->planned_quantity &&
    holding_==Holding::Confirmed && !motion_inhibited_) {
    complete_fetch_item();return;
  }
  advance_fetch();
}
void TaskManagerNode::complete_fetch_item()
{
  auto t=active_;
  if(t->arm_goal.command!=ArmGoal::TOP_PICK_PLACE_LEFT && t->arm_goal.command!=ArmGoal::TOP_PICK_PLACE_RIGHT &&
    t->arm_goal.command!=ArmGoal::PLACE_LEFT && t->arm_goal.command!=ArmGoal::PLACE_RIGHT) {holding_=Holding::Confirmed;}
  else {
    holding_=Holding::Empty;
    ((t->arm_goal.command==ArmGoal::TOP_PICK_PLACE_LEFT || t->arm_goal.command==ArmGoal::PLACE_LEFT)?left_holding_:right_holding_)=Holding::Confirmed;
  }
  ++t->fetched_quantity;
  t->inventory_handled=false;apply_fetch_inventory_delta();
  event("item_complete","requested="+std::to_string(t->requested_quantity)+" planned="+
    std::to_string(t->planned_quantity)+" fetched="+std::to_string(t->fetched_quantity));
  if(t->one_step || breakpoints_.count("after:"+t->step_id)) {t->one_step=false;t->pause_requested=true;}
  stage_boundary("after:grasp_complete");
  // Settle a physically completed item before honoring cancellation/pause.
  if(t->cancel || !t->error.empty()) {advance_fetch();return;}
  if(t->fetched_quantity>=t->planned_quantity) {begin_fetch_return();return;}
  t->candidate={};t->last_candidate={};t->locked_id.clear();t->tracking_lost=false;
  t->candidates.clear();t->targets.reset();t->target_stamp=0;t->waiting_point=false;
  t->align_attempts=0;t->approach_steps=0;t->grip_attempts=0;t->grip_succeeded=false;t->framing_started=false;t->framing_elapsed=0;
  t->barrier=now().nanoseconds();t->step_index=0;t->inventory_handled=false;
  enter(Phase::Lift,"restore_observation_height");advance_fetch();
}
void TaskManagerNode::begin_fetch_return()
{
  auto t=active_;
  if(t->local_open) {t->next_phase=Phase::Withdraw;stop_local();return;}
  enter(Phase::Withdraw,"withdraw");advance_fetch();
}
void TaskManagerNode::advance_fetch()
{
  auto t=active_; if(!t || !t->fetch || t->nav_active || t->blocked || t->pending!=Pending::None || t->upper_cancel_pending) {return;}
  // A cancellation before tray placement still returns a completed, safely held box.
  if((t->cancel || !t->error.empty()) && t->phase==Phase::Exit && !t->inventory_handled &&
    t->hardware_result && holding_==Holding::Confirmed && !motion_inhibited_) {
    complete_fetch_item();return;
  }
  if(t->cancel || !t->error.empty()) {finish_fetch();return;}
  if(t->local_open) {stop_local();return;}
  if(t->pause_requested) {
    cancel_deadline();update_budgets();
    if(local_timer_) {local_timer_->cancel();local_timer_.reset();}
    if(t->cv_maybe || t->yolo_maybe || camera_info_sub_) {stop_vision([this,t] {
      if(t->phase==Phase::Detect || t->phase==Phase::Align || t->phase==Phase::Scan) {
        const bool waiting=t->phase==Phase::Align && t->waiting_point;
        const auto resumed=t->search_ready && (config_.no_qr || t->left_confirmed) ? (t->framing_started ? Phase::Align : Phase::Detect) : Phase::Scan;
        enter(resumed,"paused_observation");t->waiting_point=waiting && resumed==Phase::Align;
        t->candidate={};
      }
    });}
    return;
  }
  switch(t->phase) {
    case Phase::Prepare:
      web_call("set_business",{{"state","starting"}},[this,t](const Json &) {
        t->business_reserved=true;
        if(t->cancel || !t->error.empty()) {advance_fetch();return;}
        enter(Phase::ToShelf,"to_shelf");begin_navigation(t->pose,Phase::Lift);
      });break;
    case Phase::Lift:
      if(!hardware_ok() || !stopped()) {fail("hardware not ready after approach");break;}
      if(!odom_ || odom_->header.frame_id!="odom") {fail("expected odom frame");break;}
      {const auto next=t->search_ready?Phase::Detect:Phase::Scan;
      if(std::abs(lift_status_->current_z-*t->target.initial_lift_z_m)>config_.position_tolerance) {send_lift(*t->target.initial_lift_z_m,next);}
      else {settle(next);}}break;
    case Phase::Scan: scan();break;
    case Phase::Detect: detect();break;
    case Phase::Align: detect();break;
    case Phase::Grasp: case Phase::Exit: case Phase::Recover:
      if(t->phase==Phase::Grasp && t->target.fetch->grasp=="top") {
        const auto step=t->plan.pick.at(0);
        if(pause_boundary(step.id)) {advance_fetch();break;}
        stop_vision([this,t,step] {
          if(t->cancel || !t->error.empty() || t->pause_requested) {advance_fetch();return;}
          t->step_started=Clock::now();send_arm(step);
        });break;
      }
      if(t->step_index<steps().size()) {
        const auto step=steps()[t->step_index];
        if(pause_boundary(step.id)) {advance_fetch();break;}
        stop_vision([this,t,step] {
          if(t->cancel || !t->error.empty() || t->pause_requested) {advance_fetch();return;}
          t->step_started=Clock::now();send_arm(step);
        });
      } else if(t->phase==Phase::Grasp) {
        if(holding_!=Holding::Confirmed) {fail("GRIP not confirmed");break;}
        stop_vision([this,t] {t->step_index=0;enter(Phase::Exit,"exit");advance_fetch();});
      } else if(t->phase==Phase::Recover) {motion_inhibited_=false;t->target_stamp=0;t->waiting_point=false;settle(t->framing_started ? Phase::Align : Phase::Detect);}
      else {
        if(holding_!=Holding::Confirmed || motion_inhibited_) {fail("carry posture/holding not confirmed");break;}
        if(t->fetched_quantity+1>=t->planned_quantity) {complete_fetch_item();break;}
        ArmGoal place;place.command=t->fetched_quantity==0?ArmGoal::PLACE_LEFT:ArmGoal::PLACE_RIGHT;
        place.expected_lift_z_m=lift_status_->current_z;
        t->plan.exit.push_back({"place_tray",place});advance_fetch();
      }break;
    case Phase::Withdraw:
      if(t->cv_maybe || t->yolo_maybe || t->depth_mode!="off") {stop_vision([this] {advance_fetch();});break;}
      if(config_.withdraw>0) {start_local(-config_.withdraw*t->inward_x,-config_.withdraw*t->inward_y,Phase::Return);}
      else {enter(Phase::Return,"return");advance_fetch();}break;
    case Phase::Return: begin_navigation(t->return_pose,Phase::Finish);break;
    case Phase::Finish: finish_fetch();break;
    default: break;
  }
}
void TaskManagerNode::finish_fetch()
{
  auto t=active_; if(!t || t->pending!=Pending::None || t->nav_active || t->blocked) {return;}
  capture_failure();
  cancel_deadline();
  if(t->local_open) {stop_local();return;}
  if(local_timer_) {local_timer_->cancel();local_timer_.reset();}
  if(t->cv_maybe || t->yolo_maybe || t->depth_mode != "off") {stop_vision([this]{finish_fetch();});return;}
  camera_info_sub_.reset(); tf_listener_.reset(); tf_buffer_.reset();
  if(t->business_reserved && !t->resources_parked && ready_) {
    web_call("set_navigation_compute",{{"state","parked"}},[this,t](const Json &) {
      t->resources_parked=true;finish_fetch();
    });return;
  }
  if(t->business_reserved) {
    web_call("set_business",{{"state","finished"},{"allow_motion",!motion_inhibited_}},[this,t](const Json &) {
      t->business_reserved=false;finish_fetch();
    });return;
  }
  const bool success=t->error.empty() && !t->cancel && t->phase==Phase::Finish && !motion_inhibited_;
  const std::string held=holding_==Holding::Confirmed?"confirmed":holding_==Holding::Empty?"empty":"unknown";
  auto result=std::make_shared<Fetch::Result>();
  result->requested_quantity=t->requested_quantity;result->planned_quantity=t->planned_quantity;result->fetched_quantity=t->fetched_quantity;
  result->message=(success?"Returned: requested="+std::to_string(t->requested_quantity)+" planned="+std::to_string(t->planned_quantity)+" fetched="+std::to_string(t->fetched_quantity)+"; end="+(t->fetched_quantity>=t->planned_quantity?"planned_quantity_reached":"shelf_end"):
    !t->error.empty()?t->error:"Task canceled")+std::string("; holding=")+held+(motion_inhibited_?"; arm posture inhibits base motion":"");
  event("task_end",result->message);
  Point3 requested;
  const bool has_target=t->target_stamp>0 && surface_to_tcp(t->surface,profile(),requested);
  last_result_={{"task_id",t->id},{"message",result->message},{"tcp",t->plan.tcp},{"approach",t->plan.approach},
    {"loaded_revision",warehouse_.revision},{"no_qr",config_.no_qr},{"scan_elapsed_sec",t->scan_elapsed},{"search_elapsed_sec",t->search_elapsed},
    {"left_confirmed",t->left_confirmed},{"right_reached",t->right_reached},{"reacquisitions",t->reacquisitions},
    {"locked_id",t->locked_id},{"tracking_lost",t->tracking_lost},
    {"search_offset_m",t->search_offset},{"framing_elapsed_sec",t->framing_elapsed},{"holding",held},
    {"candidate_valid",t->last_candidate.valid},{"candidate_q",t->last_candidate.valid?Json(t->last_candidate.q):Json(nullptr)},
    {"candidate_source_ns",t->last_candidate.stamp},
    {"candidate_age_sec",t->last_candidate.stamp?Json((now().nanoseconds()-t->last_candidate.stamp)*1e-9):Json(nullptr)},
    {"vision",vision_diagnostics(*t)},{"failure_snapshot",t->failure_snapshot},{"hardware_result",t->hardware_diag},
    {"navigation",last_navigation_},
    {"camera_right_sign",t->camera_right_sign},
    {"surface_arm_base",has_target?Json(t->surface):Json(nullptr)},{"requested_tcp",has_target?Json(requested):Json(nullptr)},
    {"camera_forward_depth_m",t->camera_distance?Json(*t->camera_distance):Json(nullptr)},
    {"camera_right_offset_m",t->camera_lateral?Json(*t->camera_lateral):Json(nullptr)},
    {"approach_steps",t->approach_steps},{"approach_inward_m",t->inward_offset},
    {"initial_shelf_clearance_m",t->initial_shelf_clearance},{"approach_limit_m",t->approach_limit},
    {"status",success?"succeeded":!t->error.empty()?"failed":"canceled"}};
  if(!t->inventory_handled && holding_!=Holding::Empty) {
    t->inventory_update={{"handled",false},{"quantity",nullptr},{"persisted",inventory_.persisted()},
      {"error","Exit not confirmed; review quantity by resurvey or manual correction"}};
  }
  last_result_["requested_quantity"]=t->requested_quantity;last_result_["planned_quantity"]=t->planned_quantity;
  last_result_["fetched_quantity"]=t->fetched_quantity;
  const auto tray=[](Holding h) {return h==Holding::Empty?"empty":h==Holding::Confirmed?"loaded":"unknown";};
  last_result_["transport"]={{"left",tray(left_holding_)},{"right",tray(right_holding_)},{"hand",held}};
  last_result_["end_reason"]=t->fetched_quantity>=t->planned_quantity?"planned_quantity_reached":(success || t->right_reached || t->boundary_wait)?"shelf_end":"interrupted";
  last_result_["inventory_update"]=t->inventory_update;
  if (t->cancel && t->error.empty() && !t->fetch->is_canceling()) {
    task_deadline_=create_wall_timer(std::chrono::milliseconds(1),[this,t] {
      cancel_deadline(); if(active_==t) {finish_fetch();}
    });return;
  }
  active_.reset(); inference_->business(false);
  if(success) {t->fetch->succeed(result);}
  else if(t->error.empty() && t->fetch->is_canceling()) {t->fetch->canceled(result);}
  else {t->fetch->abort(result);}
}
}  // namespace k1muse_manager
