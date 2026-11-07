#include "k1muse_web/control.hpp"
#include "k1muse_web/child.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <k1muse_interfaces/action/navigate_to_task.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <k1muse_interfaces/msg/chassis_status.hpp>
#include <k1muse_interfaces/msg/web_state.hpp>
#include <k1muse_interfaces/msg/web_teleop.hpp>
#include <k1muse_interfaces/srv/web_request.hpp>
#include <k1muse_interfaces/action/fetch_drug.hpp>
#include <k1muse_interfaces/action/survey_warehouse.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <map>
#include <set>
#include <random>
#include <optional>
#include <functional>
#include <filesystem>
#include <utility>

using Json = nlohmann::json;
using Twist = geometry_msgs::msg::Twist;
using Pose = geometry_msgs::msg::PoseStamped;
using Map = nav_msgs::msg::OccupancyGrid;
using Odom = nav_msgs::msg::Odometry;
using Path = nav_msgs::msg::Path;
using Scan = sensor_msgs::msg::LaserScan;
using Imu = sensor_msgs::msg::Imu;
using Chassis = k1muse_interfaces::msg::ChassisStatus;
using State = k1muse_interfaces::msg::WebState;
using Request = k1muse_interfaces::srv::WebRequest;
using Navigate = k1muse_interfaces::action::NavigateToTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
using FetchDrug = k1muse_interfaces::action::FetchDrug;
using FetchGoalHandle = rclcpp_action::ClientGoalHandle<FetchDrug>;
using SurveyWarehouse = k1muse_interfaces::action::SurveyWarehouse;
using SurveyGoalHandle = rclcpp_action::ClientGoalHandle<SurveyWarehouse>;
using Trigger = std_srvs::srv::Trigger;
using GetState = lifecycle_msgs::srv::GetState;
using Manage = nav2_msgs::srv::ManageLifecycleNodes;
using SetInitialPose = nav2_msgs::srv::SetInitialPose;
using Empty = std_srvs::srv::Empty;
using k1muse_web::Child;

class SurveyClient : public rclcpp_action::Client<SurveyWarehouse> {
 public:
  explicit SurveyClient(rclcpp::Node * n):rclcpp_action::Client<SurveyWarehouse>(
    n->get_node_base_interface(),n->get_node_graph_interface(),n->get_node_logging_interface(),"/k1muse/task/survey") {}
  rclcpp_action::GoalUUID last_id{};
 protected:
  rclcpp_action::GoalUUID generate_goal_id() override {
    last_id=rclcpp_action::Client<SurveyWarehouse>::generate_goal_id();return last_id;
  }
};

static double steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static std::string identifier() {
  std::random_device rd;
  return std::to_string(rd()) + "-" + std::to_string(rd());
}
static geometry_msgs::msg::Quaternion orientation(double yaw) {
  geometry_msgs::msg::Quaternion q; q.z = std::sin(yaw / 2); q.w = std::cos(yaw / 2); return q;
}

class WebOperator : public rclcpp::Node {
  friend class NavigationWebTest;
  friend class IntentWebTest;
  friend class WarehouseWebTest;
 public:
  explicit WebOperator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("web_operator", options) {
    boot_ = identifier();
    maps_dir_ = declare_parameter<std::string>("maps_directory", "/home/bianbu/maps");
    serial_ = declare_parameter<std::string>("serial_device", "/dev/mymcu");
    laser_ = declare_parameter<std::string>("laser_device", "/dev/mylidar");
    use_ekf_ = declare_parameter<bool>("use_ekf", false);
    navigation_use_composition_ = declare_parameter<bool>("navigation_use_composition", true,
      rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true));
    imu_xyz_ = declare_parameter<std::string>("imu_xyz", "");
    imu_rpy_ = declare_parameter<std::string>("imu_rpy", "");
    helper_ = ament_index_cpp::get_package_share_directory("k1muse_web") + "/scripts/map_helper.py";
    const auto manager=ament_index_cpp::get_package_share_directory("k1muse_manager");
    const auto configs=declare_parameter<std::string>("config_dir","/home/bianbu/k1muse_config");
    voice_configs_=declare_parameter<std::string>("voice_config_dir",configs);
    voice_helper_=ament_index_cpp::get_package_prefix("k1muse_manager")+"/lib/k1muse_manager/voice_config";
    catalog_file_=declare_parameter<std::string>("drug_catalog_file",configs+"/drug_catalog.yaml");
    catalog_default_=manager+"/config/drug_catalog.default.yaml";
    const auto command_qos = rclcpp::QoS(1).reliable().durability_volatile();
    const auto latch = rclcpp::QoS(1).reliable().transient_local();
    const auto display = rclcpp::QoS(1).best_effort().durability_volatile();
    // Display readers keep one DDS sample and are taken by the 2 Hz timer.
    // Excluding them from the executor avoids decoding every source message.
    display_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    output_ = create_publisher<Twist>("/cmd_vel", command_qos);
    state_pub_ = create_publisher<State>("/k1muse/web/state", latch);
    initial_pose_client_ = create_client<SetInitialPose>("/set_initial_pose");
    nomotion_client_ = create_client<Empty>("/request_nomotion_update");
    if (declare_parameter<bool>("debug_sequence_echo", false))
      echo_ = create_publisher<std_msgs::msg::UInt32>("/k1muse/web/received_sequence", display);
    map_pub_ = create_publisher<Map>("/k1muse/web/map", latch);
    pose_pub_ = create_publisher<Pose>("/k1muse/web/pose", display);
    scan_pose_pub_ = create_publisher<Pose>("/k1muse/web/scan_pose", display);
    odom_pub_ = create_publisher<Odom>("/k1muse/web/odom", display);
    imu_pub_ = create_publisher<Imu>("/k1muse/web/imu", display);
    chassis_pub_ = create_publisher<Chassis>("/k1muse/web/chassis", display);
    global_pub_ = create_publisher<Path>("/k1muse/web/global_path", display);
    local_pub_ = create_publisher<Path>("/k1muse/web/local_path", display);
    scan_pub_ = create_publisher<Scan>("/k1muse/web/scan", display);
    teleop_sub_ = create_subscription<k1muse_interfaces::msg::WebTeleop>(
      "/k1muse/web/teleop", command_qos, [this](const k1muse_interfaces::msg::WebTeleop::SharedPtr msg) {
        if (token_.empty() || msg->token != token_ || !manual_mode() ||
            (busy() && !voice_file_operation() && !catalog_operation())) return;
        if (input_.accept(msg->sequence, msg->velocity.linear.x, msg->velocity.linear.y,
                          msg->velocity.angular.z, steady())) {
          Twist command; command.linear.x = input_.x; command.linear.y = input_.y;
          command.angular.z = input_.yaw; output_->publish(command);
          if (echo_) { std_msgs::msg::UInt32 seq; seq.data = msg->sequence; echo_->publish(seq); }
        }
      });
    intent_client_ = create_client<Request>("/k1muse/voice/intent/control");
    warehouse_client_ = create_client<Request>("/k1muse/task/control");
    service_ = create_service<Request>("/k1muse/web/request",
      [this](rclcpp::Service<Request>::SharedPtr service,
             std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Request::Request> req) {
        if(req->operation=="inventory_get") {forward_inventory(service,header,req);return;}
        if (!intent_operation(req->operation).empty()) {
          forward_intent(service, header, req); return;
        }
        Request::Response res;
        try { request(*req, res); }
        catch (const std::exception &e) {
          res.accepted = false; res.error = (voice_file_operation(req->operation) && dynamic_cast<const Json::exception *>(&e)) ? "invalid_config" : e.what();
          if (req->request_id == request_id_ && busy()) finish({}, res.error);
        }
        publish_state(); update_fast_timer();
        service->send_response(*header, res);
      });
    nav_client_ = rclcpp_action::create_client<Navigate>(this, "/k1muse/task/navigate");
    navigation_client_ = create_client<Request>("/k1muse/task/navigation/control");
    const auto nav_timeout = declare_parameter<int>("navigation_control_timeout_ms", 1000);
    if (nav_timeout < 1 || nav_timeout > 2000)
      throw std::invalid_argument("navigation_control_timeout_ms must be 1..2000");
    navigation_timeout_ = nav_timeout / 1000.0;
    navigation_service_ = create_service<Request>("/k1muse/web/navigation/control",
      [this](rclcpp::Service<Request>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
             Request::Request::SharedPtr req) {
        Request::Response res; res.operation_id = req->request_id;
        try {
          if (req->operation == "set_navigation_compute") {
            set_navigation_compute(header, *req); update_fast_timer(); return;
          }
          if (req->operation == "wait_stopped") {
            const auto a = Json::parse(req->arguments);
            check_business(a);
            if (!motion_goal_.empty() || !gated_ || stop_wait_) throw std::runtime_error("motion not closed or stop wait busy");
            if (!stopped()) {
              stop_wait_ = StopWait{header, req->request_id, steady() + 2.0};
              update_fast_timer(); return;
            }
          } else navigation_control(*req);
          res.accepted = true; res.result = Json{{"stopped", stopped()}}.dump();
        } catch (const std::exception &e) { res.error = e.what(); }
        publish_state(); update_fast_timer(); service->send_response(*header, res);
      });
    fetch_client_ = rclcpp_action::create_client<FetchDrug>(this, "/k1muse/task/fetch_drug");
    survey_client_=std::make_shared<SurveyClient>(this);
    get_node_waitables_interface()->add_waitable(survey_client_,nullptr);
    cancel_task_client_ = create_client<Trigger>("/k1muse/task/cancel");
    prepare_timeout_ = declare_parameter<double>("mode_prepare_timeout_sec", 60.0);
    if(!std::isfinite(prepare_timeout_) || prepare_timeout_ <= 0) {
      throw std::invalid_argument("mode_prepare_timeout_sec must be positive and finite");
    }
    manage_ = create_client<Manage>("/lifecycle_manager_navigation/manage_nodes");
    localization_manage_ = create_client<Manage>("/lifecycle_manager_localization/manage_nodes");
#ifdef __riscv
    resource_affinity_ = declare_parameter<bool>("resource_affinity", true);
#else
    resource_affinity_ = declare_parameter<bool>("resource_affinity", false);
#endif
    for (const auto &name : {"map_server", "amcl", "controller_server", "planner_server",
                            "behavior_server", "bt_navigator", "velocity_smoother"})
      lifecycle_[name] = create_client<GetState>("/" + std::string(name) + "/get_state");
    health_ = create_wall_timer(std::chrono::seconds(1), [this] { health(); });
    publish_state();
  }
  ~WebOperator() override { zero(); }

 private:
  std::string boot_, token_, client_, mode_{"IDLE"}, map_id_, maps_dir_, serial_, laser_;
  std::string catalog_file_,catalog_default_,voice_configs_,voice_helper_;
  std::string helper_, imu_xyz_, imu_rpy_, nav_state_{"idle"};
  bool use_ekf_{false}, ready_{true}, initial_available_{false}, navigation_ready_{false};
  bool navigation_use_composition_{true};
  std::string navigation_compute_{"unknown"};
  bool localization_paused_{false}, resource_affinity_{false};
  struct ComputePending {
    std::shared_ptr<rmw_request_id_t> header;
    std::string request_id, target;
    uint64_t serial, epoch;
    double started;
    bool sent{false}, replied{false}, affinity_done{false}, wait_localization{false};
  };
  std::optional<ComputePending> compute_pending_;
  uint64_t compute_serial_{0};
  bool map_seen_{false}, scan_seen_{false}, gated_{true}, nav_starting_{false};
  bool mapping_origin_ready_{false};
  rclcpp::Time mapping_started_ros_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mapping_pose_sub_;
  double lease_{0}, odom_at_{0}, stopped_since_{0}, scan_at_{0}, map_at_{0};
  // AMCL observation stamp used for TF confirmation, not the RPC send time.
  double initial_at_{0}; rclcpp::Time initial_ros_{0, 0, RCL_ROS_TIME};
  bool relocalizing_{false}, localization_pose_received_{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_sub_;
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
  std::optional<int64_t> localization_request_;
  std::optional<int64_t> startup_request_;
  k1muse_web::ManualInput input_;
  std::string op_, request_id_, op_name_, stage_, target_mode_;
  Json args_, result_{{"status", "idle"}};
  rclcpp::Client<Request>::SharedPtr warehouse_client_;
  int64_t warehouse_sequence_{0};
  double warehouse_deadline_{0};
  Json known_maps_ = Json::array(), points_ = Json::object();
  double deadline_{0}, prepare_timeout_{60.0};
  std::vector<std::string> stopping_nodes_;
  Child base_, lidar_, capability_, file_;
  GoalHandle::SharedPtr goal_; bool goal_pending_{false};
  FetchGoalHandle::SharedPtr fetch_goal_;
  SurveyGoalHandle::SharedPtr survey_goal_;
  bool fetch_cancel_requested_{false};
  double action_sent_at_{0};
  uint64_t nav_epoch_{0}, mode_epoch_{0};
  std::string manager_instance_, navigation_context_, motion_context_, motion_goal_, closed_goal_;
  std::string business_, motion_source_{"nav"};
  bool business_motion_inhibited_{false};
  double local_open_ros_{0}, local_received_{0};
  struct StopWait {std::shared_ptr<rmw_request_id_t> header; std::string request_id; double deadline;};
  std::optional<StopWait> stop_wait_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr task_motion_sub_;
  uint64_t motion_sequence_{0}, navigation_rpc_{0};
  bool navigation_prepared_{false}, context_pending_{false}, cancel_latched_{false};
  bool points_navigation_ready_{false};
  double navigation_timeout_{1};
  using NavigationReply = std::function<void(bool, const Json &, const std::string &)>;
  struct NavigationPending {
    uint64_t epoch;
    double deadline;
    NavigationReply done;
    int64_t sequence{0};
  };
  std::optional<NavigationPending> navigation_pending_;
  rclcpp::Client<Request>::SharedPtr navigation_client_;
  rclcpp::Service<Request>::SharedPtr navigation_service_;
  rclcpp::Publisher<Twist>::SharedPtr output_;
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  State last_state_;
  rclcpp::Client<SetInitialPose>::SharedPtr initial_pose_client_;
  rclcpp::Client<Empty>::SharedPtr nomotion_client_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr echo_;
  rclcpp::Subscription<k1muse_interfaces::msg::WebTeleop>::SharedPtr teleop_sub_;
  rclcpp::Service<Request>::SharedPtr service_;
  rclcpp::Client<Request>::SharedPtr intent_client_;
  struct IntentPending {
    rclcpp::Service<Request>::SharedPtr service;
    std::shared_ptr<rmw_request_id_t> header;
    std::string request_id;
    uint64_t epoch;
    double deadline;
    int64_t sequence{0};
  };
  std::optional<IntentPending> intent_pending_,inventory_pending_;
  uint64_t inventory_epoch_{0};
  uint64_t intent_epoch_{0}, intent_view_epoch_{0};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr intent_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr intent_pub_;
  std::string intent_cache_;
  rclcpp::TimerBase::SharedPtr health_, fast_, view_timer_, telemetry_timer_;
  rclcpp::CallbackGroup::SharedPtr display_group_;
  bool odom_polled_{false};
  rclcpp::SerializedMessage telemetry_sample_{2048};
  rclcpp_action::Client<Navigate>::SharedPtr nav_client_;
  rclcpp_action::Client<FetchDrug>::SharedPtr fetch_client_;
  std::shared_ptr<SurveyClient> survey_client_;
  rclcpp::Client<Trigger>::SharedPtr cancel_task_client_;
  rclcpp::Client<Manage>::SharedPtr manage_, localization_manage_;
  std::map<std::string, rclcpp::Client<GetState>::SharedPtr> lifecycle_;
  std::map<std::string, bool> active_, pending_;
  std::map<std::string, double> active_at_, pending_at_, last_sent_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Chassis>::SharedPtr chassis_sub_;
  rclcpp::Subscription<Map>::SharedPtr map_sub_;
  rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<Path>::SharedPtr global_sub_, local_sub_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<Chassis>::SharedPtr chassis_pub_;
  rclcpp::Publisher<Map>::SharedPtr map_pub_;
  rclcpp::Publisher<Pose>::SharedPtr pose_pub_, scan_pose_pub_;
  rclcpp::Publisher<Scan>::SharedPtr scan_pub_;
  rclcpp::Publisher<Path>::SharedPtr global_pub_, local_pub_;
  Map::SharedPtr map_cache_;
  bool map_dirty_{false}, telemetry_{false}, map_view_{false}, paths_{false}, laser_view_{false};
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;

  bool manual_mode() const { return mode_ == "MANUAL" || mode_ == "MAPPING"; }
  bool busy() const { return result_.value("status", "") == "running"; }
  void zero() { if (output_) output_->publish(Twist{}); }
  void disable() { if (manual_mode() && input_.enabled) zero(); input_.disable(); }
  void expire_session() {
    disable(); token_.clear(); client_.clear(); views(false, false, false, false);
    intent_view(false); publish_state();
  }
  bool stopped() const {
    return odom_at_ > 0 && steady() - odom_at_ <= .250 &&
      stopped_since_ > 0 && steady() - stopped_since_ >= .300;
  }
  bool rate(const std::string &name, double interval) {
    double now = steady();
    if (now - last_sent_[name] < interval) return false;
    last_sent_[name] = now; return true;
  }
  void publish_state(bool heartbeat = false) {
    State s; s.boot_id = boot_; s.mode = mode_; s.ready = ready_;
    s.initial_pose_available = initial_available_; s.navigation_ready = navigation_ready_;
    s.teleop_enabled = input_.enabled; s.map_id = map_id_; s.client_id = client_;
    s.navigation_state = nav_state_; s.operation_id = op_; s.operation_result = operation_state().dump();
    if (heartbeat || s != last_state_) { last_state_ = s; state_pub_->publish(s); }
  }
  bool warehouse_operation() const {
    return op_name_ == "warehouse_get" || op_name_ == "warehouse_sync" || op_name_ == "warehouse_status";
  }
  bool catalog_operation() const {
    return op_name_=="drug_catalog_get" || op_name_=="drug_catalog_save";
  }
  static bool voice_file_operation(const std::string &operation) {
    return operation=="voice_intent_get" || operation=="voice_intent_save_cloud" ||
      operation=="voice_intent_save_prompt" || operation=="voice_intent_set_mode" ||
      operation=="voice_intent_set_cloud_enabled";
  }
  bool voice_file_operation() const {return voice_file_operation(op_name_);}
  Json operation_state() const {
    Json brief;
    if(op_name_=="survey_warehouse") {
      brief={{"status",result_.value("status","idle")},{"phase",result_.value("phase","")},
        {"error",result_.value("error","")},{"feedback",result_.value("feedback",Json::object())}};
      if(result_.contains("result")) {
        const auto & r=result_.at("result");brief["result"]=Json::object();
        for(const auto key:{"completion","persisted","return_status","message"}) {if(r.contains(key)) {brief["result"][key]=r.at(key);}}
      }
    } else if(catalog_operation() || voice_file_operation()) {
      brief={{"status",result_.value("status","idle")},{"error",result_.value("error","")}};
    } else if (!warehouse_operation()) brief = result_;
    else {
      // Read small fields directly. Never copy/dump the full document for the heartbeat.
      brief = {{"status", result_.value("status", "idle")},
                    {"error", result_.value("error", "")}, {"result", Json::object()}};
      if (result_.contains("result")) {
        const auto & value = result_.at("result");
        for (const auto key : {"map_id", "synced_revision", "restart_required"})
          if (value.contains(key)) brief["result"][key] = value.at(key);
        if (value.contains("manager")) {
          const auto & manager = value.at("manager");
          brief["result"]["manager"] = {{"available", manager.value("available", false)},
            {"loaded_revision", manager.value("loaded_revision", Json(nullptr))}};
        }
      }
    }
    brief["operation"] = op_name_; brief["stage"] = stage_;
    brief["web_busy"] = busy(); brief["motion_busy"] = motion_busy();
    return brief;
  }
  void begin(const Request::Request &req, const std::string &stage, double timeout) {
    if (busy()) throw std::runtime_error("busy: 当前操作尚未完成");
    op_ = identifier(); request_id_ = req.request_id; op_name_ = req.operation;
    stage_ = stage; args_ = Json::parse(req.arguments.empty() ? "{}" : req.arguments);
    deadline_ = steady() + timeout; result_ = {{"status", "running"}};
  }
  void finish(Json value = Json::object(), const std::string &error = "",
              const std::string &status = "") {
    mapping_pose_sub_.reset();
    localization_pose_sub_.reset();
    if (localization_request_) {
      if (stage_ == "set_initial_pose") initial_pose_client_->remove_pending_request(*localization_request_);
      else nomotion_client_->remove_pending_request(*localization_request_);
      localization_request_.reset();
    }
    if (startup_request_) {
      auto client = stage_ == "resume_localization" ? localization_manage_ : manage_;
      client->remove_pending_request(*startup_request_); startup_request_.reset();
    }
    if (stage_ == "warehouse_inspect") warehouse_client_->remove_pending_request(warehouse_sequence_);
    result_ = {{"status", status.empty() ? (error.empty() ? "succeeded" : "failed") : status},
               {"result", std::move(value)}, {"error", error}};
    if (warehouse_operation() || catalog_operation() || voice_file_operation()) args_ = Json::object();
    stage_.clear(); ensure_task_subscriptions(); publish_state(); update_fast_timer();
  }
  void file_start(const std::string &operation, const Json &arguments) {
    auto text = arguments.dump();
    if (voice_file_operation()) {
      if (text.size()>65536) throw std::runtime_error("invalid_config");
      args_=Json::object();
      file_.start({voice_helper_,voice_configs_,operation},true,text);return;
    }
    if(catalog_operation()) {
      if(text.size()>65536) throw std::runtime_error("drug_catalog_too_large");
      const auto helper=(std::filesystem::path(helper_).parent_path()/"drug_catalog_store.py").string();
      file_.start({"python3",helper,catalog_file_,catalog_default_,operation,std::move(text)},true);return;
    }
    if ((operation == "warehouse_get" || operation == "warehouse_sync") && text.size() > 65536)
      throw std::runtime_error("warehouse_too_large");
    file_.start({"python3", helper_, maps_dir_, operation, std::move(text)}, true);
  }
  void warehouse_inspect(Json value) {
    value["manager"] = {{"available", false}, {"instance_id", nullptr},
      {"loaded_map_id", nullptr}, {"loaded_revision", nullptr},
      {"load_error", nullptr}, {"capabilities", nullptr}, {"fetch_available", false}, {"fetch_flow_error", "manager_unavailable"},
      {"fetch_ready", false}, {"fetch_blocked_reason", "manager_unavailable"}, {"fetch_admission", Json::object()}};
    if (!warehouse_client_->service_is_ready()) { finish(std::move(value)); return; }
    args_ = std::move(value); stage_ = "warehouse_inspect";
    warehouse_deadline_ = steady() + navigation_timeout_;
    const auto operation = op_;
    auto request = std::make_shared<Request::Request>();
    request->operation = "inspect"; request->client_id = boot_;
    request->request_id = operation; request->arguments = "{}";
    try {
      auto pending = warehouse_client_->async_send_request(request,
        [this, operation](rclcpp::Client<Request>::SharedFuture future) {
          if (operation != op_ || stage_ != "warehouse_inspect") return;
          try {
            const auto reply = future.get();
            if (reply->accepted) {
              const auto info = Json::parse(reply->result);
              const auto & warehouse = info.at("warehouse");
              args_["manager"] = {{"available", true}, {"instance_id", info.at("manager_instance")},
                {"loaded_map_id", warehouse.at("loaded_map_id")},
                {"loaded_revision", warehouse.at("loaded_revision")},
                {"load_error", warehouse.at("load_error")}, {"capabilities", warehouse.at("capabilities")},
                {"fetch_available", info.value("fetch_available", false)}, {"fetch_flow_error", info.value("fetch_flow_error", "")},
                {"fetch_ready", info.value("fetch_ready", false)}, {"fetch_blocked_reason", info.value("fetch_blocked_reason", "status_unknown")},
                {"fetch_admission", info.value("fetch_admission", Json::object())},
                {"drug_catalog",info.value("drug_catalog",Json())}};
            }
          } catch (const std::exception &) { /* File result remains useful without a manager. */ }
          finish(std::move(args_));
        });
      warehouse_sequence_ = pending.request_id;
    } catch (const std::exception &) { finish(std::move(args_)); }
  }
  bool motion_busy() const {
    return !business_.empty() || business_motion_inhibited_ || !motion_goal_.empty() ||
      (compute_pending_ && !compute_pending_->replied) || nav_state_ == "stopping" || nav_state_ == "blocked";
  }
  void reply_navigation_compute(bool success, const std::string &error = "", bool completed = true) {
    if (!compute_pending_) return;
    if (!success) {
      navigation_compute_ = "unknown";
      navigation_ready_ = navigation_prepared_ = false;
    }
    if (!compute_pending_->replied) {
      Request::Response reply; reply.operation_id = compute_pending_->request_id;
      reply.accepted = success; reply.error = error;
      reply.result = Json{{"navigation_compute",navigation_compute_}, {"localization_paused",localization_paused_},
        {"elapsed_ms",(steady()-compute_pending_->started)*1000}}.dump();
      navigation_service_->send_response(*compute_pending_->header, reply);
      compute_pending_->replied = true;
    }
    if (completed) compute_pending_.reset();
    update_fast_timer();
  }
  void advance_navigation_compute() {
    if (!compute_pending_ || compute_pending_->replied) return;
    if (steady()-compute_pending_->started >= 15.0) {
      reply_navigation_compute(false,"navigation compute timeout: transition result unknown",!compute_pending_->sent);
      return;
    }
    if (compute_pending_->sent || (compute_pending_->target != "active" && !stopped())) return;
    const auto target = compute_pending_->target;
    if (!compute_pending_->affinity_done) {
      try {if (resource_affinity_) capability_.affinity(target == "active");}
      catch (const std::exception &error) {reply_navigation_compute(false,error.what());return;}
      compute_pending_->affinity_done = true;
    }
    if (compute_pending_->wait_localization) {
      if (!localized()) return;
      compute_pending_->wait_localization = false;
    }
    const bool nav_active = target == "active";
    // Resume localization before planning; pause localization only after navigation stops.
    const bool localization = (localization_paused_ && target != "parked") ||
      (target == "parked" && navigation_compute_ == "paused" && !localization_paused_);
    const bool enable = localization ? target != "parked" : nav_active;
    if (!localization && navigation_compute_ == (nav_active ? "active" : "paused")) {
      RCLCPP_INFO(get_logger(),"NAV compute=%s success=1 elapsed_ms=%.1f",target.c_str(),
        (steady()-compute_pending_->started)*1000);
      reply_navigation_compute(true);return;
    }
    auto client = localization ? localization_manage_ : manage_;
    if (!client->service_is_ready()) {reply_navigation_compute(false,"Nav2 lifecycle manager unavailable");return;}
    compute_pending_->sent = true;
    const auto serial=compute_pending_->serial, epoch=compute_pending_->epoch;
    if (!localization) navigation_compute_ = enable ? "resuming" : "pausing";
    auto request=std::make_shared<Manage::Request>();
    request->command=enable ? Manage::Request::RESUME : Manage::Request::PAUSE;
    try {client->async_send_request(request,[this,serial,epoch,localization,enable](rclcpp::Client<Manage>::SharedFuture future) {
      if (!compute_pending_ || compute_pending_->serial != serial || epoch != mode_epoch_) return;
      bool success=false;try {success=future.get()->success;}catch (const std::exception &) {}
      compute_pending_->sent=false;
      if (!success) {reply_navigation_compute(false,"Nav2 computation transition failed");return;}
      if (localization) {
        localization_paused_=!enable;compute_pending_->wait_localization=enable;
      } else navigation_compute_=enable ? "active" : "paused";
      for (const auto &name : localization ? std::vector<std::string>{"map_server","amcl"} :
          std::vector<std::string>{"controller_server","planner_server","behavior_server","bt_navigator","velocity_smoother"}) {
        lifecycle_.at(name)->prune_pending_requests();pending_[name]=false;
        active_[name]=enable;active_at_[name]=steady();
      }
      if(compute_pending_->replied) {compute_pending_.reset();update_fast_timer();return;}
      advance_navigation_compute();
    });}catch (const std::exception &error) {reply_navigation_compute(false,error.what());}
  }
  void set_navigation_compute(std::shared_ptr<rmw_request_id_t> header, const Request::Request &request) {
    const auto a = Json::parse(request.arguments);
    const auto target = a.at("state").get<std::string>();
    if (target != "active" && target != "paused" && target != "parked") throw std::runtime_error("invalid navigation compute state");
    if (a.at("manager_instance") != manager_instance_) throw std::runtime_error("navigation compute owner changed");
    // A mode teardown owns the group now; do not pause it again for an old canceled leg.
    if (target != "active" && (mode_ != "NAVIGATION" || cancel_latched_)) {
      Request::Response reply; reply.operation_id = request.request_id; reply.accepted = true;
      reply.result = Json{{"navigation_compute",navigation_compute_},{"skipped",true}}.dump();
      navigation_service_->send_response(*header,reply); return;
    }
    if (mode_ != "NAVIGATION" || !navigation_prepared_ || a.at("context_id") != navigation_context_)
      throw std::runtime_error("navigation compute context not prepared");
    const auto business = a.value("business_id",std::string());
    if (business != business_) throw std::runtime_error("navigation compute business changed");
    if (compute_pending_ || startup_request_) throw std::runtime_error("navigation compute transition unresolved");
    if (!motion_goal_.empty() || !gated_) throw std::runtime_error("navigation motion source still open");
    if (navigation_compute_ != "active" && navigation_compute_ != "paused")
      throw std::runtime_error("navigation compute state unknown: reload navigation");
    if (navigation_compute_ == target && !localization_paused_) {
      if(resource_affinity_) capability_.affinity(target == "active");
      Request::Response reply; reply.operation_id = request.request_id; reply.accepted = true;
      reply.result = Json{{"navigation_compute",navigation_compute_},{"elapsed_ms",0}}.dump();
      navigation_service_->send_response(*header,reply); return;
    }
    compute_pending_ = ComputePending{header,request.request_id,target,++compute_serial_,mode_epoch_,steady()};
    advance_navigation_compute();
  }
  void navigation_unavailable(const std::string &error) {
    gated_ = true; navigation_ready_ = navigation_prepared_ = false; zero();
    nav_state_ = motion_busy() || goal_ || goal_pending_ ? "blocked" : "failed";
    if (busy()) finish({}, error);
    else publish_state();
  }
  void navigation_call(const std::string &operation, Json arguments, NavigationReply done) {
    if (navigation_pending_) throw std::runtime_error("navigation control busy");
    if (!navigation_client_->service_is_ready()) {
      done(false, Json::object(), "task_manager navigation control unavailable"); return;
    }
    const auto epoch = ++navigation_rpc_;
    auto request = std::make_shared<Request::Request>();
    request->operation = operation; request->client_id = boot_;
    request->request_id = std::to_string(epoch); request->arguments = arguments.dump();
    navigation_pending_ = NavigationPending{epoch, steady() + navigation_timeout_, std::move(done)};
    try {
      auto future = navigation_client_->async_send_request(request,
        [this, epoch](rclcpp::Client<Request>::SharedFuture future) {
          if (!navigation_pending_ || navigation_pending_->epoch != epoch) return;
          auto pending = std::move(*navigation_pending_); navigation_pending_.reset();
          bool accepted = false; Json value; std::string error;
          try {
            const auto reply = future.get(); accepted = reply->accepted; error = reply->error;
            value = Json::parse(reply->result);
          } catch (const std::exception &e) { error = e.what(); }
          try { pending.done(accepted, value, error); }
          catch (const std::exception &e) { navigation_unavailable(e.what()); }
          update_fast_timer();
        });
      navigation_pending_->sequence = future.request_id;
    } catch (const std::exception &e) {
      auto pending = std::move(*navigation_pending_); navigation_pending_.reset();
      pending.done(false, Json::object(), e.what());
    }
    update_fast_timer();
  }
  void suspend_navigation(bool cancel, std::function<void()> next, bool retried = false) {
    const auto operation = op_;
    if (manager_instance_.empty()) {
      navigation_call("get_context", Json::object(),
        [this, operation, cancel, next, retried](bool ok, const Json &value, const std::string &error) {
          if (operation != op_ || !busy()) return;
          if (!ok) { navigation_unavailable(error); return; }
          if (!motion_busy() && !goal_ && !goal_pending_) {
            motion_sequence_ = 0; closed_goal_.clear();
          }
          manager_instance_ = value.at("manager_instance"); navigation_context_ = value.at("context_id");
          suspend_navigation(cancel, next, retried);
        });
      return;
    }
    const auto context = boot_ + ":" + operation;
    const auto previous = navigation_context_;
    navigation_call("suspend", {{"expected_context_id", previous}, {"next_context_id", context},
        {"web_boot_id", boot_}, {"cancel_navigation", cancel}},
      [this, operation, context, previous, cancel, next, retried](bool ok, const Json &value, const std::string &error) {
        if (operation != op_ || !busy()) return;
        if (!ok) {
          if (value.is_object() && value.contains("manager_instance")) {
            if (value.at("manager_instance") != manager_instance_ || error == "navigation_context_changed") {
              manager_instance_.clear();
              // Only an explicit, idle rejection may be re-handshaken once.
              // No replay after a timeout, pending goal or active business task.
              if (!retried && error == "navigation_context_changed" && !value.value("busy", true) &&
                !motion_busy() && !goal_ && !goal_pending_) {
                suspend_navigation(cancel, next, true); return;
              }
              navigation_unavailable(error);
            } else finish({}, error);  // Explicit busy rejection leaves the old navigation intact.
          } else navigation_unavailable(error);
          return;
        }
        manager_instance_ = value.at("manager_instance");
        if (motion_goal_.empty()) motion_context_ = previous;
        navigation_context_ = context; context_pending_ = true;
        navigation_ready_ = navigation_prepared_ = false; gated_ = true; zero();
        if (cancel) {
          const auto goal = value.value("goal_id", std::string{});
          if (value.value("busy", false) && goal != closed_goal_) {
            motion_goal_ = goal; motion_context_ = value.at("goal_context_id"); nav_state_ = "stopping";
          } else if (motion_goal_.empty()) nav_state_ = "idle";
        }
        next();
      });
  }
  void configure_navigation(bool ready, Json result = Json::object()) {
    const auto operation = op_;
    stage_ = "nav_configure"; navigation_prepared_ = ready;
    navigation_call("configure", {{"manager_instance", manager_instance_},
        {"context_id", navigation_context_}, {"web_boot_id", boot_},
        {"map_id", ready ? map_id_ : ""}, {"tasks", ready ? points_ : Json::object()}, {"ready", ready}},
      [this, operation, ready, result](bool ok, const Json &, const std::string &error) {
        if (operation != op_ || !busy()) return;
        if (!ok) { navigation_unavailable("导航配置未确认: " + error); return; }
        context_pending_ = false; navigation_ready_ = ready;
        if (ready) cancel_latched_ = false;
        finish(result);
      });
  }
  void check_business(const Json &a) const {
    if (a.at("manager_instance") != manager_instance_ || a.at("context_id") != navigation_context_ ||
        business_.empty() || a.at("business_id") != business_)
      throw std::runtime_error("business owner/context changed");
  }
  void navigation_control(const Request::Request &request) {
    const auto a = Json::parse(request.arguments);
    // Catalog I/O leaves the manager's running snapshot and motion owner intact.
    const bool reading_warehouse = op_name_ == "warehouse_get" || op_name_ == "warehouse_status" || catalog_operation() || voice_file_operation();
    if (request.operation == "confirm_motion_ready") {
      if (a.at("manager_instance") != manager_instance_ || a.at("context_id") != navigation_context_ ||
          !business_.empty() || !motion_goal_.empty() || !stopped()) throw std::runtime_error("hardware recovery cannot release motion yet");
      business_motion_inhibited_ = false; nav_state_ = "idle"; return;
    }
    if (request.operation == "set_business") {
      if (a.at("manager_instance") != manager_instance_ || a.at("context_id") != navigation_context_)
        throw std::runtime_error("business context changed");
      const auto id = a.at("business_id").get<std::string>();
      if (id.empty()) throw std::runtime_error("missing business identity");
      const auto state = a.at("state").get<std::string>();
      if (state == "starting") {
        if ((!business_.empty() && business_ != id) || business_motion_inhibited_ || !motion_goal_.empty() ||
            mode_ != "NAVIGATION" || !navigation_prepared_ || cancel_latched_ ||
            (busy() && stage_ != "fetch_drug" && stage_ != "survey_warehouse" && !reading_warehouse)) throw std::runtime_error("business motion context unavailable");
        business_ = id; gated_ = true; disable(); zero();
      } else if (state == "finished") {
        check_business(a);
        if (!motion_goal_.empty() || !stopped()) throw std::runtime_error("business motion not settled");
        business_motion_inhibited_ = !a.value("allow_motion", false); business_.clear();
        gated_ = true; zero(); nav_state_ = business_motion_inhibited_ ? "blocked" : "idle";
      } else throw std::runtime_error("invalid business state");
      return;
    }
    if (request.operation != "set_navigation" && request.operation != "set_motion")
      throw std::runtime_error("unknown navigation operation");
    const auto source = a.value("source", std::string("nav"));
    if (source != "nav" && source != "local") throw std::runtime_error("invalid motion source");
    const auto business = a.value("business_id", std::string());
    if (!business.empty()) check_business(a);
    else if (!business_.empty() || business_motion_inhibited_) throw std::runtime_error("business_busy");
    if (source == "local" && business.empty()) throw std::runtime_error("local motion requires business owner");
    const auto id = a.at("goal_id").get<std::string>();
    const auto context = a.at("context_id").get<std::string>();
    const auto state = a.at("state").get<std::string>();
    const auto sequence = a.at("sequence").get<uint64_t>();
    if (manager_instance_.empty() || a.at("manager_instance") != manager_instance_ || id.empty())
      throw std::runtime_error("navigation owner changed");
    if (sequence <= motion_sequence_) {
      if (sequence == motion_sequence_ && state == nav_state_ &&
          (id == motion_goal_ || id == closed_goal_)) return;
      throw std::runtime_error("stale navigation control");
    }
    const bool terminal = state == "succeeded" || state == "canceled" || state == "failed";
    if (!terminal && state != "starting" && state != "running" && state != "stopping")
      throw std::runtime_error("invalid navigation state");
    if (state == "starting" || state == "running") {
      if (compute_pending_ || (source == "nav" && navigation_compute_ != "active"))
        throw std::runtime_error("navigation computation not active or transitioning");
      const bool changing = busy() && !reading_warehouse && stage_ != "navigate" && stage_ != "nav_configure" && !((stage_ == "fetch_drug" || stage_ == "survey_warehouse") && business == business_ && !business.empty());
      if (mode_ != "NAVIGATION" || !navigation_prepared_ || cancel_latched_ || changing ||
          context != navigation_context_ || id == closed_goal_)
        throw std::runtime_error("navigation not prepared or changing");
      if (state == "starting") {
        if (source == "local" && !stopped()) throw std::runtime_error("local motion requires fresh stopped odometry");
        if (!motion_goal_.empty()) throw std::runtime_error("navigation already owned");
        motion_goal_ = id; motion_context_ = context; motion_source_ = source; disable(); gated_ = true;
      } else {
        if (motion_goal_ != id || motion_context_ != context || motion_source_ != source || nav_state_ != "starting")
          throw std::runtime_error("navigation is not starting");
        gated_ = false;
        if (source == "local") {local_open_ros_ = now().seconds(); local_received_ = steady();}
      }
    } else {
      if ((!motion_goal_.empty() && (motion_goal_ != id || motion_context_ != context)) ||
          (motion_goal_.empty() && context != navigation_context_ && context != motion_context_))
        throw std::runtime_error("navigation goal changed");
      if (!terminal && id == closed_goal_) throw std::runtime_error("navigation already closed");
      gated_ = true; zero();
      if (!a.value("ready", true)) navigation_ready_ = navigation_prepared_ = false;
      if (terminal) { closed_goal_ = id; motion_goal_.clear(); }
      else { motion_goal_ = id; motion_context_ = context; }
    }
    motion_sequence_ = sequence; nav_state_ = state;
  }
  void begin_mode_change() {
    if (compute_pending_) reply_navigation_compute(false,"navigation mode changed");
    disable(); gated_ = true; zero(); ready_ = false;
    initial_available_ = navigation_ready_ = navigation_prepared_ = false; ++mode_epoch_;
    if (target_mode_ == "NAVIGATION") { stage_ = "select_map"; file_start("select_map", args_); }
    else {
      stage_ = "handover";
      if (mode_ == "IDLE" && odom_at_ == 0 && !motion_busy() && !goal_ && !goal_pending_) stop_old();
    }
  }
  void complete_mode() {
    ready_ = true;
    if (context_pending_) configure_navigation(false);
    else finish();
  }
  static std::string intent_operation(const std::string &operation) {
    static const std::map<std::string, std::string> operations{
      {"voice_intent_check_cloud", "check_cloud"},
      {"voice_intent_restart", "restart_node"}};
    const auto found = operations.find(operation);
    return found == operations.end() ? "" : found->second;
  }
  void forward_intent(const rclcpp::Service<Request>::SharedPtr &service,
      const std::shared_ptr<rmw_request_id_t> &header,
      const std::shared_ptr<Request::Request> &req) {
    Request::Response failure; failure.operation_id = req->request_id;
    bool started = false;
    try {
      if (req->client_id.empty() || req->request_id.empty())
        throw std::runtime_error("missing request/client ID");
      if (token_.empty() || req->token != token_ || req->client_id != client_)
        throw std::runtime_error("session_expired: 请重新连接");
      if (intent_pending_) throw std::runtime_error("intent_control_busy");
      if (!intent_client_->service_is_ready()) throw std::runtime_error("voice_unavailable");
      auto forwarded = std::make_shared<Request::Request>(*req);
      forwarded->operation = intent_operation(req->operation);
      const auto epoch = ++intent_epoch_;
      intent_pending_ = IntentPending{service, header, req->request_id, epoch, steady() + 2};
      started = true;
      auto future = intent_client_->async_send_request(forwarded,
        [this, epoch](rclcpp::Client<Request>::SharedFuture reply) {
          if (!intent_pending_ || intent_pending_->epoch != epoch) return;
          auto pending = std::move(*intent_pending_); intent_pending_.reset();
          auto response = *reply.get();
          pending.service->send_response(*pending.header, response);
        });
      intent_pending_->sequence = future.request_id;
      return;
    } catch (const std::exception &e) {
      // No navigation begin/finish state is touched by this separate RPC slot.
      if (started) intent_pending_.reset();
      failure.error = e.what();
    }
    service->send_response(*header, failure);
  }
  void forward_inventory(const rclcpp::Service<Request>::SharedPtr & service,
      const std::shared_ptr<rmw_request_id_t> & header,const std::shared_ptr<Request::Request> & req) {
    Request::Response failure;failure.operation_id=req->request_id;bool started=false;
    try {
      if(req->request_id.empty() || req->client_id!=client_ || token_.empty() || req->token!=token_) throw std::runtime_error("session_expired");
      if(inventory_pending_) throw std::runtime_error("inventory_read_busy");
      if(!warehouse_client_->service_is_ready()) throw std::runtime_error("manager_unavailable");
      const auto epoch=++inventory_epoch_;
      inventory_pending_=IntentPending{service,header,req->request_id,epoch,steady()+navigation_timeout_};
      started=true;
      const auto future=warehouse_client_->async_send_request(std::make_shared<Request::Request>(*req),
        [this,epoch](rclcpp::Client<Request>::SharedFuture reply) {
          if(!inventory_pending_ || inventory_pending_->epoch!=epoch) return;
          auto p=std::move(*inventory_pending_);inventory_pending_.reset();
          p.service->send_response(*p.header,*reply.get());update_fast_timer();
        });
      inventory_pending_->sequence=future.request_id;update_fast_timer();return;
    } catch(const std::exception & e) {failure.error=e.what();if(started) inventory_pending_.reset();}
    service->send_response(*header,failure);
  }
  void expire_inventory_read() {
    if(!inventory_pending_ || steady()<inventory_pending_->deadline) return;
    auto p=std::move(*inventory_pending_);inventory_pending_.reset();warehouse_client_->remove_pending_request(p.sequence);
    Request::Response response;response.operation_id=p.request_id;response.error="manager_unavailable";p.service->send_response(*p.header,response);
  }
  void intent_view(bool enabled) {
    if (enabled == static_cast<bool>(intent_sub_)) return;
    const auto epoch = ++intent_view_epoch_;
    if (!enabled) {
      intent_sub_.reset(); intent_pub_.reset(); intent_cache_.clear(); return;
    }
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    intent_pub_ = create_publisher<std_msgs::msg::String>("/k1muse/web/intent_status", qos);
    intent_sub_ = create_subscription<std_msgs::msg::String>("/k1muse/voice/intent/status", qos,
      [this, epoch](std_msgs::msg::String::ConstSharedPtr message) {
        if (epoch != intent_view_epoch_ || !intent_pub_ || message->data == intent_cache_) return;
        intent_cache_ = message->data; intent_pub_->publish(*message);
      });
  }
  void request(const Request::Request &req, Request::Response &res) {
    res.accepted = false;
    if (req.client_id.empty() || req.request_id.empty()) throw std::runtime_error("missing request/client ID");
    const bool voice_file=voice_file_operation(req.operation);
    if (voice_file && req.arguments.size()>65536) throw std::runtime_error("invalid_config");
    const bool warehouse = req.operation == "warehouse_get" || req.operation == "warehouse_sync";
    const bool catalog=req.operation=="drug_catalog_get" || req.operation=="drug_catalog_save";
    if (warehouse && req.arguments.size() > 65536) throw std::runtime_error("warehouse_too_large");
    if (catalog && req.arguments.size() > 65536) throw std::runtime_error("drug_catalog_too_large");
    std::vector<std::set<std::string>> keys;
    const auto a = Json::parse(req.arguments.empty() ? "{}" : req.arguments,
      [&keys, warehouse,catalog](int, Json::parse_event_t event, Json & item) {
        if (!warehouse && !catalog) return true;
        if (event == Json::parse_event_t::object_start) keys.emplace_back();
        if (event == Json::parse_event_t::key && !keys.back().insert(item.get<std::string>()).second)
          throw std::runtime_error(catalog?"drug_catalog_invalid: duplicate key":"warehouse_invalid: duplicate key");
        if (event == Json::parse_event_t::object_end) keys.pop_back();
        return true;
      });
    if (req.operation == "claim_control") {
      if (!client_.empty() && client_ != req.client_id) {
        if (steady() - lease_ < 3 || input_.enabled) throw std::runtime_error("busy: 已有操作会话");
        expire_session();  // Explicit stale-session takeover; business/motion owners stay intact.
      }
      if (client_.empty()) { client_ = req.client_id; token_ = identifier(); input_.sequence = 0; }
      lease_ = steady(); res.accepted = true; res.result = Json{{"token", token_}, {"boot_id", boot_}}.dump(); return;
    }
    if (token_.empty() || req.token != token_ || req.client_id != client_)
      throw std::runtime_error("session_expired: 请重新连接");
    if (req.operation == "get_operation") {
      if ((a.contains("operation_id") && a.at("operation_id") != op_) ||
          (a.contains("request_id") && a.at("request_id") != request_id_))
        throw std::runtime_error("unknown operation");
      res.accepted = true; res.operation_id = op_; res.result = result_.dump(); return;
    }
    if (req.request_id == request_id_) {
      res.accepted = true; res.operation_id = op_; res.result = result_.dump(); return;
    }
    if (req.operation == "release_control") { expire_session(); }
    else if (req.operation == "set_views") {
      lease_ = steady();
      views(a.value("telemetry", false), a.value("map", false),
            a.value("paths", false), a.value("laser", false));
      intent_view(a.value("intent", false));
    } else if (req.operation == "set_teleop_enabled") {
      if (a.value("enabled", false)) {
        if (!manual_mode() || !ready_ || busy() || motion_busy()) throw std::runtime_error("manual mode not ready");
        input_.enable(steady());
      } else disable();
    } else if(req.operation=="survey_warehouse") {
      if(mode_!="NAVIGATION" || !navigation_ready_ || motion_busy()) throw std::runtime_error("survey requires prepared idle navigation");
      if(!a.is_object() || a.size()!=1 || !a.at("map_id").is_string()) throw std::runtime_error("survey expects map_id");
      if(!survey_client_->action_server_is_ready()) throw std::runtime_error("SurveyWarehouse Action unavailable");
      begin(req,"survey_warehouse",0);send_survey(a.at("map_id").get<std::string>());
    } else if(req.operation=="inventory_set") {
      if(motion_busy()) throw std::runtime_error("business_busy");
      if(!warehouse_client_->service_is_ready()) throw std::runtime_error("manager_unavailable");
      begin(req,"inventory_set",navigation_timeout_);const auto operation=op_;
      auto forwarded=std::make_shared<Request::Request>(req);
      warehouse_client_->async_send_request(forwarded,[this,operation](rclcpp::Client<Request>::SharedFuture f) {
        if(operation!=op_ || stage_!="inventory_set" || !busy()) return;
        const auto r=f.get();if(!r->accepted) {finish({},r->error);return;}
        try {finish(Json::parse(r->result));}catch(const std::exception & e) {finish({},e.what());}
      });
    } else if (req.operation == "fetch_drug") {
      if (mode_ != "NAVIGATION" || !navigation_ready_ || motion_busy()) throw std::runtime_error("fetch_drug requires prepared idle navigation");
      if (!a.is_object() || a.size() > 2 || !a.contains("drug_name"))
        throw std::runtime_error("fetch_drug expects one canonical drug_name");
      const auto drug = a.at("drug_name").get<std::string>();
      if (drug.empty()) throw std::runtime_error("missing drug_name");
      if (!fetch_client_->action_server_is_ready()) throw std::runtime_error("FetchDrug Action unavailable");
      if(a.contains("quantity") && (!a.at("quantity").is_number_integer() || a.at("quantity").get<double>()<1 || a.at("quantity").get<double>()>3)) {
        throw std::runtime_error("quantity_invalid");
      }
      begin(req, "fetch_drug", 0); send_fetch(drug,a.value("quantity",1));
    } else if (req.operation == "cancel_task") {
      if (!cancel_task_client_->service_is_ready()) throw std::runtime_error("task cancellation unavailable");
      const bool standalone = !busy();
      if (standalone) begin(req, "cancel_task", 5);
      if (stage_ == "fetch_drug" || stage_ == "survey_warehouse") fetch_cancel_requested_ = true;
      const auto operation = op_;
      result_["cancel_request"] = "pending";
      cancel_task_client_->async_send_request(std::make_shared<Trigger::Request>(),
        [this, operation, standalone](rclcpp::Client<Trigger>::SharedFuture future) {
          if (operation != op_) return;
          const auto response = future.get();
          if (standalone && stage_ == "cancel_task") {
            finish({{"cancel_requested", response->success}, {"message", response->message}},
                   response->success ? "" : response->message);
          } else {
            result_["cancel_request"] = response->success ? "accepted" : "rejected";
            result_["cancel_message"] = response->message;
            publish_state();
          }
        });
      res.accepted = true; res.operation_id = op_;
      res.result = Json{{"cancel_request", "pending"}}.dump(); return;
    } else if (req.operation == "cancel_navigation") {
      if (stage_ == "cancel") { res.accepted = true; res.operation_id = op_; return; }
      if (mode_ != "NAVIGATION") { disable(); zero(); res.accepted = true; return; }
      if (!business_.empty() || business_motion_inhibited_) throw std::runtime_error("business_busy: cancel/prepare the complete task first");
      // A cancel replaces the navigation operation; no request queue.
      if (busy() && op_name_ != "navigate") throw std::runtime_error("busy");
      result_ = {{"status", "idle"}};
      begin(req, "suspend", 5); gated_ = true; cancel_latched_ = true; zero();
      if (motion_busy()) nav_state_ = "stopping";
      suspend_navigation(true, [this] { stage_ = "cancel"; update_fast_timer(); });
    } else if (req.operation == "set_mode") {
      if (!business_.empty() || business_motion_inhibited_) throw std::runtime_error("business_busy");
      std::string target = a.at("mode").get<std::string>();
      if (target != "IDLE" && target != "MANUAL" && target != "MAPPING" && target != "NAVIGATION")
        throw std::runtime_error("invalid mode");
      if (target == "NAVIGATION") {
        auto id = a.at("map_id").get<std::string>();
        bool found = false;
        for (const auto &m : known_maps_) if (m.at("map_id") == id) found = true;
        if (!found) throw std::runtime_error("refresh map list first");
      }
      begin(req, "suspend", prepare_timeout_); target_mode_ = target;
      if (mode_ == "NAVIGATION" || target == "NAVIGATION")
        suspend_navigation(false, [this] { begin_mode_change(); });
      else begin_mode_change();
    } else if(voice_file) {
      begin(req,"file",10);file_start(req.operation,a);
    } else if(catalog) {
      if(!a.is_object() || (req.operation=="drug_catalog_get"?!a.empty():(a.size()!=1 || !a.contains("drugs"))))
        throw std::runtime_error("drug_catalog_invalid: unexpected arguments");
      begin(req,"file",10);file_start(req.operation,a);
    } else if (req.operation == "warehouse_status") {
      begin(req, "warehouse_inspect", navigation_timeout_);
      warehouse_inspect(Json::object());
    } else if (warehouse) {
      if (req.operation == "warehouse_sync" && motion_busy())
        throw std::runtime_error("business_busy: 仓库同步需等待当前任务结束");
      const auto map = a.at("map_id").get<std::string>();
      bool found = false;
      for (const auto & saved : known_maps_) if (saved.at("map_id") == map) found = true;
      if (!found) throw std::runtime_error("map_changed: 请先刷新并选择地图");
      begin(req, "file", 10); file_start(req.operation, a);
    } else if (req.operation == "list_maps" || req.operation == "get_points" ||
               req.operation == "save_points" || req.operation == "save_map") {
      if (req.operation != "list_maps" && req.operation != "save_map" &&
          a.at("map_id").get<std::string>() != map_id_)
        throw std::runtime_error("map_changed: 当前地图不匹配");
      if (req.operation == "save_map" && (mode_ != "MAPPING" || !ready_))
        throw std::runtime_error("mapping not ready");
      if (req.operation == "save_map") disable();
      begin(req, req.operation == "save_map" ? "save_target" : "file", 10);
      if (req.operation == "save_points" && mode_ == "NAVIGATION") {
        points_navigation_ready_ = navigation_ready_;
        stage_ = "suspend";
        suspend_navigation(false, [this] { stage_ = "file"; file_start("save_points", args_); });
      } else file_start(req.operation == "save_map" ? "save_target" : req.operation, a);
    } else if (req.operation == "set_initial_pose") {
      if (mode_ != "NAVIGATION" || !initial_available_) throw std::runtime_error("initial_pose_unavailable: load navigation first");
      if (busy() || motion_busy() || goal_ || goal_pending_ || fetch_goal_ || survey_goal_)
        throw std::runtime_error("relocalization_busy: wait for the real task terminal");
      if (!stopped()) throw std::runtime_error("relocalization_requires_fresh_stopped_odometry");
      auto pose = target_pose(a);
      relocalizing_ = navigation_ready_;
      begin(req, "suspend", prepare_timeout_);
      suspend_navigation(false, [this, pose] {
        auto set_pose = [this,pose] {
          stage_ = "set_initial_pose";
          initial_pose_.header = pose.header; initial_pose_.pose.pose = pose.pose;
          initial_pose_.pose.covariance.fill(0);
          initial_pose_.pose.covariance[0] = .25; initial_pose_.pose.covariance[7] = .25;
          initial_pose_.pose.covariance[35] = .06854;
          initial_at_ = steady(); nav_starting_ = false;
          localization_pose_received_ = false;
        };
        if(!localization_paused_) {set_pose();return;}
        if(!localization_manage_->service_is_ready()) {finish({},"localization manager unavailable");return;}
        stage_="resume_localization";
        auto request=std::make_shared<Manage::Request>();request->command=Manage::Request::RESUME;
        const auto operation=op_;const auto epoch=mode_epoch_;
        auto pending=localization_manage_->async_send_request(request,
          [this,operation,epoch,set_pose](rclcpp::Client<Manage>::SharedFuture result) {
            if(operation!=op_ || epoch!=mode_epoch_ || stage_!="resume_localization") return;
            startup_request_.reset();
            if(!result.get()->success) {finish({},"localization resume rejected");return;}
            localization_paused_=false;
            for(const auto &name:{"map_server","amcl"}) {active_[name]=true;active_at_[name]=steady();}
            set_pose();
          });
        startup_request_=pending.request_id;
      });
    } else if (req.operation == "navigate") {
      if (mode_ != "NAVIGATION" || !navigation_ready_ || goal_ || goal_pending_ ||
          a.at("map_id").get<std::string>() != map_id_)
        throw std::runtime_error("navigation not ready or goal active");
      Navigate::Goal goal; goal.map_id = map_id_;
      if (a.contains("name")) goal.task_name = a.at("name").get<std::string>();
      else { goal.use_pose = true; goal.target_pose = target_pose(a); }
      if (motion_busy()) throw std::runtime_error("navigation already active");
      if (!nav_client_->action_server_is_ready()) throw std::runtime_error("task_manager unavailable");
      begin(req, "navigate", 0); send_goal(goal);
    } else throw std::runtime_error("unknown operation");
    res.accepted = true;
    if (request_id_ == req.request_id) { res.operation_id = op_; res.result = result_.dump(); }
    else res.result = "{}";
  }
  Pose target_pose(const Json &a) {
    double x = a.at("x"), y = a.at("y"), yaw = a.at("yaw");
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) throw std::runtime_error("invalid pose");
    Pose p; p.header.frame_id = "map"; p.header.stamp = now();
    p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation = orientation(yaw); return p;
  }
  void send_goal(const Navigate::Goal &goal) {
    const auto epoch = ++nav_epoch_;
    goal_pending_ = true;
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = [this, epoch](GoalHandle::SharedPtr handle) {
      if (epoch != nav_epoch_) return;
      goal_pending_ = false; goal_ = handle;
      result_["admission_ms"] = (steady() - action_sent_at_) * 1000;
      result_["phase"] = handle ? "accepted" : "rejected"; result_.erase("admission_error");
      if (!handle && stage_ == "navigate") finish({}, "task_manager rejected goal");
      publish_state();
    };
    options.result_callback = [this, epoch](const GoalHandle::WrappedResult &r) {
      if (epoch != nav_epoch_) return;
      goal_.reset(); goal_pending_ = false;
      // Global motion state was already committed by the manager's close ACK.
      if (stage_ == "navigate") {
        const auto state = r.code == rclcpp_action::ResultCode::SUCCEEDED ? "succeeded" :
          r.code == rclcpp_action::ResultCode::CANCELED ? "canceled" : "failed";
        finish({{"navigation_state", state}, {"message", r.result ? r.result->message : ""}},
          r.code == rclcpp_action::ResultCode::ABORTED ? "navigation failed" : "", state);
      }
      publish_state();
    };
    action_sent_at_ = steady(); result_["phase"] = "pending";
    try {nav_client_->async_send_goal(goal, options);}
    catch (const std::exception &e) {
      result_["phase"] = "admission_unresolved"; result_["admission_error"] = e.what();
      publish_state();  // A send exception cannot prove that the server saw no goal.
    }
  }
  void send_survey(const std::string & map) {
    const auto operation=op_;fetch_cancel_requested_=false;survey_goal_.reset();result_["phase"]="pending";
    SurveyWarehouse::Goal goal;goal.map_id=map;goal.drug_name.clear();
    rclcpp_action::Client<SurveyWarehouse>::SendGoalOptions options;
    options.goal_response_callback=[this,operation](SurveyGoalHandle::SharedPtr h) {
      if(operation!=op_) return;
      if(!h) {finish({},"Business rejected survey; inspect last_survey_rejection");return;}
      survey_goal_=h;result_["phase"]="accepted";result_.erase("admission_error");
      if(fetch_cancel_requested_) {survey_client_->async_cancel_goal(h);}
      publish_state();
    };
    options.feedback_callback=[this,operation](SurveyGoalHandle::SharedPtr,const std::shared_ptr<const SurveyWarehouse::Feedback> f) {
      if(operation!=op_ || stage_!="survey_warehouse") return;
      result_["phase"]=f->stage;result_["feedback"]={{"stage",f->stage},{"shelf_id",f->shelf_id},
        {"layer",f->layer},{"direction",f->direction},{"attempt",f->attempt},{"provisional_count",f->provisional_count}};publish_state();
    };
    options.result_callback=[this,operation](const SurveyGoalHandle::WrappedResult & r) {
      if(operation!=op_) {return;}
      survey_goal_.reset();fetch_cancel_requested_=false;
      const auto message=r.result?r.result->message:"Survey failed";
      Json summary;
      try {
        summary=Json::parse(r.result?r.result->summary_json:"");
        summary.get_ref<Json::object_t &>()["message"]=message;
      } catch(const Json::exception & error) {
        finish({{"message",message},{"action_status",int(r.code)}},
          std::string("survey_summary_invalid: ")+error.what());return;
      }
      if(r.code==rclcpp_action::ResultCode::SUCCEEDED) finish(summary);
      else if(r.code==rclcpp_action::ResultCode::CANCELED) finish(summary,"","canceled");else finish(summary,message);
    };
    action_sent_at_=steady();
    try {
      survey_client_->async_send_goal(goal,options);result_["goal_id"]=rclcpp_action::to_string(survey_client_->last_id);
    } catch(const std::exception & e) {result_["phase"]="admission_unresolved";result_["admission_error"]=e.what();}
  }
  void send_fetch(const std::string &drug, int quantity) {
    const auto operation = op_;
    fetch_cancel_requested_ = false; fetch_goal_.reset();
    result_["drug_name"] = drug; result_["phase"] = "pending";
    FetchDrug::Goal goal; goal.drug_name = drug;goal.quantity=quantity;result_["requested_quantity"]=quantity;
    rclcpp_action::Client<FetchDrug>::SendGoalOptions options;
    options.goal_response_callback = [this, operation](FetchGoalHandle::SharedPtr handle) {
      if (operation != op_) return;
      if (!handle) { finish({}, "Business rejected fetch"); return; }
      fetch_goal_ = handle; result_["phase"] = "accepted";
      result_["admission_ms"] = (steady() - action_sent_at_) * 1000;
      result_.erase("admission_error");
      if (fetch_cancel_requested_) fetch_client_->async_cancel_goal(handle);
      publish_state();
    };
    options.feedback_callback = [this, operation](FetchGoalHandle::SharedPtr,
        const std::shared_ptr<const FetchDrug::Feedback> feedback) {
      if (operation != op_ || stage_ != "fetch_drug") return;
      result_["phase"] = feedback->stage;result_["planned_quantity"]=feedback->planned_quantity;
      result_["fetched_quantity"]=feedback->fetched_quantity;publish_state();
    };
    options.result_callback = [this, operation](const FetchGoalHandle::WrappedResult &result) {
      if (operation != op_) return;
      fetch_goal_.reset(); fetch_cancel_requested_ = false;
      const std::string message = result.result ? result.result->message : "";
      Json summary={{"message",message}};
      if(result.result) {
        summary["requested_quantity"]=result.result->requested_quantity;
        summary["planned_quantity"]=result.result->planned_quantity;summary["fetched_quantity"]=result.result->fetched_quantity;
      }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) finish(summary);
      else if (result.code == rclcpp_action::ResultCode::CANCELED) finish(summary, "", "canceled");
      else finish(summary, message.empty() ? "Fetch failed" : message);
    };
    action_sent_at_ = steady();
    try {fetch_client_->async_send_goal(goal, options);}
    catch (const std::exception &e) {
      result_["phase"] = "admission_unresolved"; result_["admission_error"] = e.what(); publish_state();
    }
  }
  void update_fast_timer() {
    const bool needed = input_.enabled || navigation_pending_ || inventory_pending_ || stop_wait_ ||
      (compute_pending_ && !compute_pending_->replied) ||
      (motion_source_ == "local" && !gated_ && !motion_goal_.empty()) ||
      (busy() && stage_ != "navigate" && stage_ != "fetch_drug" && stage_ != "survey_warehouse");
    if (needed && !fast_) fast_ = create_wall_timer(std::chrono::milliseconds(25), [this] { tick(); });
    if (!needed) fast_.reset();
  }
  void tick() {
    expire_inventory_read();
    try {
      advance_navigation_compute();
      if (input_.expired(steady())) { disable(); publish_state(); }
      if (motion_source_ == "local" && !gated_ && !motion_goal_.empty() && steady() - local_received_ > .200) {
        gated_ = true; nav_state_ = "blocked"; zero(); publish_state();
      }
      if (stop_wait_ && (stopped() || steady() >= stop_wait_->deadline)) {
        Request::Response reply; reply.operation_id = stop_wait_->request_id; reply.accepted = stopped();
        reply.result = Json{{"stopped", stopped()}}.dump();
        if (!reply.accepted) reply.error = "fresh stopped odometry not confirmed";
        navigation_service_->send_response(*stop_wait_->header, reply); stop_wait_.reset();
      }
      if (navigation_pending_ && steady() >= navigation_pending_->deadline) {
        auto pending = std::move(*navigation_pending_); navigation_pending_.reset();
        navigation_client_->remove_pending_request(pending.sequence);
        pending.done(false, Json::object(), "navigation control timeout");
      }
      if (!busy()) { update_fast_timer(); return; }
      if (stage_ == "warehouse_inspect" && steady() >= warehouse_deadline_) {
        finish(std::move(args_)); return;
      }
      if (stage_ != "navigate" && stage_ != "fetch_drug" && stage_ != "survey_warehouse" && steady() > deadline_) {
        if (warehouse_operation() || catalog_operation() || voice_file_operation()) {
          file_.signal(SIGKILL);
          finish({}, "operation timeout: " + stage_); return;
        }
        gated_ = true; navigation_ready_ = navigation_prepared_ = false;
        disable(); zero(); file_.signal(SIGKILL);
        if (op_name_ == "set_mode") ready_ = false;
        ++mode_epoch_; finish({}, "operation timeout: " + stage_); return;
      }
      if (stage_ == "cancel" || stage_ == "handover") {
        if (!motion_busy() && !goal_ && !goal_pending_ && stopped()) {
          if (stage_ == "cancel") {
            target_mode_ = "MANUAL"; ready_ = false;
            navigation_ready_ = initial_available_ = false; ++mode_epoch_;
            stop_old();
          } else stop_old();
        }
      } else if (stage_ == "stop_old") {
        capability_.poll();
        const bool stop_lidar = target_mode_ == "MANUAL" || target_mode_ == "IDLE";
        if (stop_lidar) lidar_.poll();
        // The owned ROS launcher performs graceful shutdown and escalation.
        bool removed = true;
        for (const auto &name : stopping_nodes_) if (node_exists(name)) removed = false;
        if (capability_.pid <= 0 && (!stop_lidar || lidar_.pid <= 0) && removed) start_mode();
      } else if (stage_ == "prepare") {
        if (target_mode_ == "IDLE") complete_mode();
        else if (odom_at_ > 0 && steady() - odom_at_ < .250) {
          if (mode_ == "MANUAL") complete_mode();
          else if (mode_ == "MAPPING" && map_seen_ && scan_seen_ && mapping_origin_ready_ && stopped()) complete_mode();
          else if (mode_ == "NAVIGATION" && is_active("map_server") && is_active("amcl") && map_seen_) {
            ready_ = initial_available_ = true; finish({{"initial_pose_available", true}});
          }
        }
      } else if (stage_ == "file" || stage_ == "save_target" || stage_ == "save_process" ||
                 stage_ == "saved_map" || stage_ == "select_map") {
        poll_file();
      } else if (stage_ == "set_initial_pose") {
        if (!localization_request_ && initial_pose_client_->service_is_ready()) {
          auto request = std::make_shared<SetInitialPose::Request>(); request->pose = initial_pose_;
          const auto operation = op_; const auto epoch = mode_epoch_;
          auto pending = initial_pose_client_->async_send_request(request,
            [this,operation,epoch](rclcpp::Client<SetInitialPose>::SharedFuture) {
              if (operation != op_ || epoch != mode_epoch_ || stage_ != "set_initial_pose") return;
              // Subscribe after AMCL applied this pose. Volatile QoS excludes cached
              // observations; laser source stamps may legitimately precede this RPC.
              localization_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/amcl_pose", rclcpp::QoS(1).reliable().durability_volatile(),
                [this,operation,epoch](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr value) {
                  if (operation != op_ || epoch != mode_epoch_ ||
                      (stage_ != "refresh_localization" && stage_ != "localize") || value->header.frame_id != "map") return;
                  initial_ros_ = rclcpp::Time(value->header.stamp);
                  localization_pose_received_ = true;
                });
              localization_request_.reset(); stage_ = "refresh_localization";
            });
          localization_request_ = pending.request_id;
        }
      } else if (stage_ == "refresh_localization") {
        if (!localization_request_ && nomotion_client_->service_is_ready() &&
            localization_pose_sub_->get_publisher_count() > 0) {
          const auto operation = op_; const auto epoch = mode_epoch_;
          // The initial-pose service ACK orders this update after AMCL applied the pose.
          // A topic publication followed by a service call would not provide that order.
          auto pending = nomotion_client_->async_send_request(std::make_shared<Empty::Request>(),
            [this,operation,epoch](rclcpp::Client<Empty>::SharedFuture) {
              if (operation != op_ || epoch != mode_epoch_ || stage_ != "refresh_localization") return;
              localization_request_.reset(); stage_ = "localize";
            });
          localization_request_ = pending.request_id;
        }
      } else if (stage_ == "localize") {
        if (!nav_starting_ && !startup_request_ && localized() && navigation_lifecycle_known()) {
          if (all_navigation_active()) {navigation_compute_ = "active"; nav_starting_ = true;}
          else if (relocalizing_ && navigation_group_paused()) nav_starting_ = true;
          else if (relocalizing_) {finish({}, "Nav2 lifecycle changed: reload navigation before relocalizing"); return;}
          else {
            if (!manage_->service_is_ready()) return;
            auto req = std::make_shared<Manage::Request>(); req->command = Manage::Request::STARTUP;
            const auto epoch = mode_epoch_;
            const auto operation = op_;
            auto pending = manage_->async_send_request(req, [this, epoch, operation](rclcpp::Client<Manage>::SharedFuture future) {
              if (epoch != mode_epoch_ || operation != op_ || stage_ != "localize") return;
              startup_request_.reset();
              if (!future.get()->success) finish({}, "Nav2 startup not confirmed: inspect lifecycle before reloading navigation");
              else nav_starting_ = true;
            });
            startup_request_ = pending.request_id;
          }
        }
        if (nav_starting_ && (all_navigation_active() || (relocalizing_ && navigation_group_paused())) &&
            localized() && nav_client_->action_server_is_ready()) {
          if (all_navigation_active()) navigation_compute_ = "active";
          configure_navigation(true, {{"navigation_ready", true}});
        }
      }
    } catch (const std::exception &e) { gated_ = true; zero(); finish({}, e.what()); }
  }
  void poll_file() {
    if (!file_.poll()) return;
    Json value = Json::object();
    if (stage_ != "save_process") value = Json::parse(file_.output);
    if (warehouse_operation() || catalog_operation() || voice_file_operation()) file_.output.clear();
    if (file_.exit_code != 0) { finish({}, value.value("error", "map saver failed")); return; }
    if (stage_ == "save_target") {
      stage_ = "save_process";
      file_.start({"ros2", "run", "nav2_map_server", "map_saver_cli", "-t", "/map",
                   "-f", value.at("target").get<std::string>(), "--fmt", "pgm", "--ros-args",
                   "-p", "map_subscribe_transient_local:=true", "-p", "save_map_timeout:=5.0"});
    } else if (stage_ == "select_map") {
      args_["map_path"] = value.at("path"); args_["tasks"] = value.at("tasks"); stage_ = "handover";
      if (mode_ == "IDLE" && odom_at_ == 0 && !goal_ && !goal_pending_) stop_old();
    } else if (stage_ == "save_process") {
      stage_ = "saved_map";
      Json saved_args = args_; saved_args["initialize_mapping_origin"] = mapping_origin_ready_;
      file_start("saved_map", saved_args);
    } else {
      if (op_name_ == "list_maps") known_maps_ = value.at("maps");
      if (op_name_ == "save_map") {
        map_id_ = value.at("map_id").get<std::string>(); points_ = value.at("tasks"); reset_display();
      }
      if (op_name_ == "save_points") {
        points_ = value.at("tasks");
        if (mode_ == "NAVIGATION") {
          configure_navigation(points_navigation_ready_, value); return;
        }
      }
      // A read-only refresh must not change the active manager snapshot.
      finish(std::move(value));
    }
  }
  bool node_exists(const std::string &name) {
    for (const auto &node : get_node_names()) if (node == name || node == "/" + name) return true;
    return false;
  }
  void stop_old() {
    stopping_nodes_.clear();
    if (capability_.pid > 0) {
      for (const auto &name : {"slam_toolbox", "amcl", "bt_navigator"})
        if (node_exists(name)) stopping_nodes_.push_back(name);
    }
    if ((target_mode_ == "MANUAL" || target_mode_ == "IDLE") && lidar_.pid > 0) {
      stopping_nodes_.push_back("ld06"); lidar_.signal(SIGINT);
    }
    capability_.signal(SIGINT); stage_ = "stop_old";
  }
  void prepare_mapping_origin() {
    mapping_started_ros_ = now();
    mapping_origin_ready_ = false;
    mapping_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose", rclcpp::QoS(1).reliable(), [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        if (!busy() || stage_ != "prepare" || mode_ != "MAPPING" || mapping_origin_ready_ ||
            m->header.frame_id != "map" || rclcpp::Time(m->header.stamp) < mapping_started_ros_) return;
        const auto & p = m->pose.pose.position; const auto & q = m->pose.pose.orientation;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z) || !std::isfinite(q.w) || std::abs(p.x) > 1e-4 || std::abs(p.y) > 1e-4 ||
            std::abs(q.x) > 1e-4 || std::abs(q.y) > 1e-4 || std::abs(q.z) > 1e-4 ||
            std::abs(q.w*q.w - 1) > 1e-4) {
          ready_ = false; finish({}, "新建图起点未归零，请核对 SLAM 版本后重新建图"); return;
        }
        mapping_origin_ready_ = true;
        points_ = {{"建图起点", {{"x", 0.0}, {"y", 0.0}, {"yaw", 0.0}}}};
        mapping_pose_sub_.reset();
      });
  }
  void start_mode() {
    if ((target_mode_ == "MAPPING" || target_mode_ == "NAVIGATION") &&
        (node_exists("slam_toolbox") || node_exists("amcl") || node_exists("bt_navigator")))
      throw std::runtime_error("外部 SLAM/AMCL/Nav2 冲突，先停止对应启动入口");
    for (const auto &endpoint : get_publishers_info_by_topic("/cmd_vel"))
      if (endpoint.node_name() != get_name()) throw std::runtime_error("外部 /cmd_vel 发布者冲突");
    if (target_mode_ != "IDLE") {
      bool serial_exists = node_exists("stm32_communication");
      bool odom_exists = node_exists("chassis_odometry");
      if (serial_exists != odom_exists) throw std::runtime_error("底盘节点不完整，请检查已有启动入口");
      if (!serial_exists && base_.pid <= 0) {
        std::vector<std::string> args{"ros2", "launch", "k1muse_bringup",
          "stm32_communication.launch.py", "serial_device:=" + serial_, "use_ekf:=" + std::string(use_ekf_ ? "true" : "false")};
        if (use_ekf_) { args.push_back("imu_xyz:=" + imu_xyz_); args.push_back("imu_rpy:=" + imu_rpy_); }
        base_.start(args);
      }
      if ((target_mode_ == "MAPPING" || target_mode_ == "NAVIGATION") &&
          !node_exists("ld06") && lidar_.pid <= 0)
        lidar_.start({"ros2", "launch", "k1muse_bringup", "ld06_bringup.launch.py", "port_name:=" + laser_});
    }
    mode_ = target_mode_; map_id_.clear(); map_seen_ = scan_seen_ = false;
    mapping_origin_ready_ = false; mapping_pose_sub_.reset();
    for (auto &[name, client] : lifecycle_) { (void)name; client->prune_pending_requests(); }
    manage_->prune_pending_requests();
    if (compute_pending_) reply_navigation_compute(false,"navigation mode replaced");
    navigation_compute_ = "unknown"; localization_paused_ = false;
    active_.clear(); active_at_.clear(); pending_.clear(); pending_at_.clear();
    nav_starting_ = false; initial_at_ = 0;
    points_ = mode_ == "NAVIGATION" ? args_.at("tasks") : Json::object();
    nav_state_ = "idle"; cancel_latched_ = false; navigation_prepared_ = false;
    reset_display(); ensure_task_subscriptions();
    if (mode_ == "MAPPING") {
      prepare_mapping_origin();
      capability_.start({"ros2", "launch", "k1muse_bringup", "slam_mapping.launch.py", "visualize:=false"});
    }
    if (mode_ == "NAVIGATION") {
      map_id_ = args_.at("map_id").get<std::string>();
      std::string path = args_.at("map_path").get<std::string>();
      capability_.start({"ros2", "launch", "k1muse_bringup", "navigation.launch.py", "map:=" + path,
                         "visualize:=false", "navigation_autostart:=false", "navigation_output:=/cmd_vel_nav_smoothed",
                         "use_composition:=" + std::string(navigation_use_composition_ ? "true" : "false")});
    }
    stage_ = "prepare"; deadline_ = steady() + prepare_timeout_; ensure_task_subscriptions(); publish_state();
  }
  void health() {
    try {
      if (busy() && (stage_ == "navigate" || stage_ == "fetch_drug" || stage_ == "survey_warehouse") &&
          result_.value("phase", "") == "pending" && steady() - action_sent_at_ >= 5) {
        result_["phase"] = "admission_unresolved";
        result_["admission_error"] = "Action acknowledgement not received; inspect before retrying";
        RCLCPP_WARN(get_logger(), "Action admission unresolved operation=%s request=%s elapsed_ms=%.1f",
          op_.c_str(), request_id_.c_str(), (steady()-action_sent_at_)*1000);
      }
      if (intent_pending_ && steady() >= intent_pending_->deadline) {
        auto pending = std::move(*intent_pending_); intent_pending_.reset();
        intent_client_->remove_pending_request(pending.sequence);
        Request::Response response; response.operation_id = pending.request_id;
        response.error = "intent_control_timeout";
        pending.service->send_response(*pending.header, response);
      }
      if (!token_.empty() && steady() - lease_ >= 3 &&
          (telemetry_ || map_view_ || paths_ || laser_view_ || intent_sub_)) {
        views(false, false, false, false); intent_view(false);
      }
      if (mode_ == "NAVIGATION" && busy() && (stage_ == "prepare" || stage_ == "localize"))
        poll_lifecycle();
      if (mode_ == "NAVIGATION" && navigation_ready_ &&
          (!nav_client_->action_server_is_ready() || !node_exists("bt_navigator")))
        navigation_unavailable("task_manager or bt_navigator disappeared");
      for (auto child : {&base_, &lidar_, &capability_})
        if (child->pid > 0 && child->poll() && stage_ != "stop_old") {
          gated_ = true; disable(); zero();
          ready_ = initial_available_ = navigation_ready_ = navigation_prepared_ = false;
          if (motion_busy()) nav_state_ = "blocked";
          if (busy()) finish({}, "owned process exited");
        }
      if (file_.pid > 0 && !busy()) file_.poll();
      publish_state(true); update_fast_timer();
    } catch (const std::exception &e) { RCLCPP_ERROR(get_logger(), "%s", e.what()); }
  }
  bool is_active(const std::string &name) {
    return active_[name] && steady() - active_at_[name] < 3;
  }
  bool all_navigation_active() {
    for (const auto &name : {"controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother"})
      if (!is_active(name)) return false;
    return true;
  }
  bool navigation_group_paused() {
    if (navigation_compute_ != "paused" || compute_pending_ || !navigation_lifecycle_known()) return false;
    for (const auto &name : {"controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother"})
      if (active_[name]) return false;
    return true;
  }
  bool navigation_lifecycle_known() const {
    for (const auto &name : {"controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother"}) {
      const auto at = active_at_.find(name);
      if (at == active_at_.end() || steady() - at->second >= 3) return false;
    }
    return true;
  }
  void poll_lifecycle() {
    for (auto &[name, client] : lifecycle_) {
      if (pending_[name] && steady() - pending_at_[name] > 2) {
        client->prune_pending_requests(); pending_[name] = false;
      }
      if (pending_[name] || !client->service_is_ready()) continue;
      pending_[name] = true; pending_at_[name] = steady(); const auto epoch = mode_epoch_;
      const auto sent = pending_at_[name];
      client->async_send_request(std::make_shared<GetState::Request>(), [this, name, epoch, sent](rclcpp::Client<GetState>::SharedFuture future) {
        if (epoch != mode_epoch_ || !pending_[name] || pending_at_[name] != sent) return;
        pending_[name] = false; active_[name] = future.get()->current_state.id == 3;
        active_at_[name] = steady();
      });
    }
  }
  bool localized() {
    if (!tf_ || initial_at_ == 0 || !localization_pose_received_) return false;
    try {
      auto t = tf_->lookupTransform("map", "base_link", tf2::TimePointZero);
      return rclcpp::Time(t.header.stamp) >= initial_ros_ &&
        std::abs((now() - rclcpp::Time(t.header.stamp)).seconds()) < 1;
    } catch (const tf2::TransformException &) { return false; }
  }
  void ensure_tf() {
    const bool needed = map_view_ || mode_ == "NAVIGATION";
    if (!needed) { listener_.reset(); tf_.reset(); return; }
    if (!tf_) {
      tf_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(3));
      listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_, this, false);
    }
  }
  void ensure_task_subscriptions() {
    const auto display = rclcpp::QoS(1).best_effort();
    const bool poll_odom = mode_ == "IDLE";
    // IDLE owns no motion; its odometry is display data, not handover feedback.
    if (poll_odom) odom_at_ = stopped_since_ = 0;
    if (odom_sub_ && odom_polled_ != poll_odom) odom_sub_.reset();
    if ((mode_ != "IDLE" || telemetry_) && !odom_sub_) {
      odom_polled_ = poll_odom;
      rclcpp::SubscriptionOptions options;
      if (poll_odom) options.callback_group = display_group_;
      odom_sub_ = create_subscription<Odom>("/odom", display,
        [this](Odom::ConstSharedPtr m) { receive_odom(*m); }, options);
    }
    if (mode_ == "IDLE" && !telemetry_) { odom_sub_.reset(); odom_at_ = stopped_since_ = 0; }
    const bool preparing = busy() && stage_ == "prepare" &&
      (mode_ == "MAPPING" || mode_ == "NAVIGATION");
    if ((preparing || map_view_) && !map_sub_)
      map_sub_ = create_subscription<Map>("/map", rclcpp::QoS(1).reliable().transient_local(), [this](Map::SharedPtr m) {
        map_seen_ = true; map_at_ = steady();
        if (map_view_ && (!map_cache_ ||
            m->header.frame_id != map_cache_->header.frame_id ||
            m->info.width != map_cache_->info.width || m->info.height != map_cache_->info.height ||
            m->info.resolution != map_cache_->info.resolution || m->info.origin != map_cache_->info.origin ||
            m->data != map_cache_->data)) {
          map_cache_ = m; map_dirty_ = true;
        }
      });
    if (!preparing && !map_view_) map_sub_.reset();
    if ((preparing || laser_view_) && !scan_sub_)
      scan_sub_ = create_subscription<Scan>("/scan", display, [this](Scan::SharedPtr m) {
        scan_seen_ = true; scan_at_ = steady();
        if (!laser_view_ || !tf_ || m->ranges.empty() || !rate("scan", .5)) return;
        try {
          // BufferCore's three-argument lookup never waits for another executor thread.
          auto t = tf_->lookupTransform("map", m->header.frame_id,
            tf2_ros::fromRclcpp(rclcpp::Time(m->header.stamp)));
          Pose p; p.header = m->header; p.header.frame_id = "map";
          p.pose.position.x = t.transform.translation.x; p.pose.position.y = t.transform.translation.y;
          p.pose.position.z = t.transform.translation.z; p.pose.orientation = t.transform.rotation;
          Scan reduced = *m; size_t stride = std::max<size_t>(1, (m->ranges.size() + 359) / 360);
          reduced.ranges.clear(); reduced.intensities.clear();
          for (size_t i = 0; i < m->ranges.size(); i += stride) reduced.ranges.push_back(m->ranges[i]);
          reduced.angle_increment *= static_cast<float>(stride);
          reduced.time_increment *= static_cast<float>(stride);
          reduced.angle_max = reduced.angle_min + static_cast<float>(reduced.ranges.size() - 1) * reduced.angle_increment;
          scan_pose_pub_->publish(p); scan_pub_->publish(reduced);
        } catch (const tf2::TransformException &) {}
      });
    if (!preparing && !laser_view_) scan_sub_.reset();
    if (mode_ == "NAVIGATION" && !nav_sub_) nav_sub_ = create_subscription<Twist>(
      "/cmd_vel_nav_smoothed", rclcpp::QoS(1).reliable(), [this](const Twist::SharedPtr m) {
        if (mode_ == "NAVIGATION" && navigation_prepared_ && !gated_ &&
            !cancel_latched_ && motion_source_ == "nav" && !motion_goal_.empty() && nav_state_ == "running") output_->publish(*m);
      });
    if (mode_ == "NAVIGATION" && !task_motion_sub_) task_motion_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/k1muse/task/cmd_vel", rclcpp::QoS(1).reliable(), [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        if (mode_ != "NAVIGATION" || !navigation_prepared_ || gated_ || cancel_latched_ ||
            business_.empty() || motion_goal_.empty() || motion_source_ != "local" || nav_state_ != "running") return;
        const auto &v = m->twist; const double stamp = rclcpp::Time(m->header.stamp).seconds();
        const double age = now().seconds() - stamp;
        if (m->header.frame_id != "base_link" || stamp < local_open_ros_ || age < 0 || age > .200 ||
            !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) || !std::isfinite(v.angular.z) ||
            std::hypot(v.linear.x, v.linear.y) > .20001 || std::abs(v.angular.z) > .60001 ||
            v.linear.z != 0 || v.angular.x != 0 || v.angular.y != 0) return;
        local_received_ = steady(); output_->publish(v);
      });
    if (mode_ != "NAVIGATION") { nav_sub_.reset(); task_motion_sub_.reset(); }
    ensure_tf();
  }
  void receive_odom(const Odom &m) {
    double t = steady(); const auto &v = m.twist.twist;
    if (odom_at_ == 0 || t - odom_at_ > .250 ||
        std::hypot(v.linear.x, v.linear.y) >= .02 || std::abs(v.angular.z) >= .05)
      stopped_since_ = 0;
    odom_at_ = t;
    if (std::hypot(v.linear.x, v.linear.y) < .02 && std::abs(v.angular.z) < .05 && stopped_since_ == 0)
      stopped_since_ = t;
    if (telemetry_ && rate("odom", .5)) odom_pub_->publish(m);
  }
  void reset_display() {
    map_cache_.reset(); map_dirty_ = false; last_sent_.clear();
    // Recreate the transient publisher so a stale previous map is never latched.
    map_pub_.reset(); map_pub_ = create_publisher<Map>("/k1muse/web/map", rclcpp::QoS(1).reliable().transient_local());
    map_sub_.reset(); scan_sub_.reset(); listener_.reset(); tf_.reset();
    ensure_task_subscriptions();
  }
  void views(bool telemetry, bool map, bool paths, bool laser) {
    if (telemetry_ == telemetry && map_view_ == map &&
        paths_ == (map && paths) && laser_view_ == (map && laser)) return;
    telemetry_ = telemetry; map_view_ = map; paths_ = map && paths; laser_view_ = map && laser;
    const auto q = rclcpp::QoS(1).best_effort();
    rclcpp::SubscriptionOptions options; options.callback_group = display_group_;
    if (telemetry_ && !imu_sub_) imu_sub_ = create_subscription<Imu>(
      "/imu/data", q, [](Imu::ConstSharedPtr) {}, options);
    if (telemetry_ && !chassis_sub_) chassis_sub_ = create_subscription<Chassis>(
      "/k1muse/chassis/status", q, [](Chassis::ConstSharedPtr) {}, options);
    if (telemetry_ && !telemetry_timer_) telemetry_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
      rclcpp::MessageInfo info;
      if (odom_polled_ && odom_sub_ && odom_sub_->take_serialized(telemetry_sample_, info))
        odom_pub_->publish(telemetry_sample_);
      // Display-only data needs no decode/encode round trip in this node.
      if (imu_sub_ && imu_sub_->take_serialized(telemetry_sample_, info)) imu_pub_->publish(telemetry_sample_);
      if (chassis_sub_ && chassis_sub_->take_serialized(telemetry_sample_, info)) chassis_pub_->publish(telemetry_sample_);
    });
    if (!telemetry_) { telemetry_timer_.reset(); imu_sub_.reset(); chassis_sub_.reset(); }
    if (paths_ && !global_sub_) global_sub_ = create_subscription<Path>(
      "/plan", q, [this](Path::SharedPtr m) { if (rate("global", .5)) global_pub_->publish(*m); });
    if (paths_ && !local_sub_) local_sub_ = create_subscription<Path>(
      "/local_plan", q, [this](Path::SharedPtr m) { if (rate("local", .5)) local_pub_->publish(*m); });
    if (!paths_) { global_sub_.reset(); local_sub_.reset(); }
    if (map_view_ && !view_timer_) view_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {
      if (map_dirty_ && map_cache_ && rate("map", 1)) {
        map_pub_->publish(*map_cache_); map_dirty_ = false;
      }
      if (!tf_ || !map_cache_) return;
      try {
        auto t = tf_->lookupTransform("map", "base_link", tf2::TimePointZero);
        if (std::abs((now() - rclcpp::Time(t.header.stamp)).seconds()) > 1) return;
        Pose p; p.header = t.header; p.header.frame_id = "map";
        p.pose.position.x = t.transform.translation.x; p.pose.position.y = t.transform.translation.y;
        p.pose.position.z = t.transform.translation.z; p.pose.orientation = t.transform.rotation; pose_pub_->publish(p);
      } catch (const tf2::TransformException &) {}
    });
    if (!map_view_) {
      view_timer_.reset(); map_cache_.reset(); map_dirty_ = false;
      map_pub_.reset(); map_pub_ = create_publisher<Map>("/k1muse/web/map", rclcpp::QoS(1).reliable().transient_local());
    }
    ensure_task_subscriptions();
  }
};

#ifndef K1MUSE_WEB_NO_MAIN
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebOperator>());
  rclcpp::shutdown(); return 0;
}
#endif
