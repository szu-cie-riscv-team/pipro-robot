#pragma once

#include "k1muse_manager/task_manager_node.hpp"
#include "k1muse_manager/inference_control.hpp"
#include "k1muse_manager/grasp_plan.hpp"
#include "k1muse_manager/warehouse_config.hpp"
#include "k1muse_manager/survey_counter.hpp"
#include "k1muse_manager/inventory_store.hpp"
#include "k1muse_manager/drug_catalog.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <k1muse_interfaces/action/navigate_to_task.hpp>
#include <k1muse_interfaces/action/fetch_drug.hpp>
#include <k1muse_interfaces/action/survey_warehouse.hpp>
#include <k1muse_interfaces/action/lift_command.hpp>
#include <k1muse_interfaces/msg/arm_status.hpp>
#include <k1muse_interfaces/msg/lift_status.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <deque>
#include <memory>
#include <set>
#include <string>

namespace k1muse_manager
{
using Json = nlohmann::json;
using Task = k1muse_interfaces::action::NavigateToTask;
using Fetch = k1muse_interfaces::action::FetchDrug;
using Survey = k1muse_interfaces::action::SurveyWarehouse;
using SurveyHandle = rclcpp_action::ServerGoalHandle<Survey>;
using Nav = nav2_msgs::action::NavigateToPose;
using Arm = k1muse_interfaces::action::ArmCommand;
using Lift = k1muse_interfaces::action::LiftCommand;
using Control = k1muse_interfaces::srv::WebRequest;
using Trigger = std_srvs::srv::Trigger;
using Toggle = std_srvs::srv::SetBool;
using TaskHandle = rclcpp_action::ServerGoalHandle<Task>;
using FetchHandle = rclcpp_action::ServerGoalHandle<Fetch>;
using NavHandle = rclcpp_action::ClientGoalHandle<Nav>;
using Code = rclcpp_action::ResultCode;
using Pose = geometry_msgs::msg::PoseStamped;
using Odom = nav_msgs::msg::Odometry;
using D2 = vision_msgs::msg::Detection2DArray;
using D3 = vision_msgs::msg::Detection3DArray;
using ArmStatus = k1muse_interfaces::msg::ArmStatus;
using LiftStatus = k1muse_interfaces::msg::LiftStatus;
using Clock = std::chrono::steady_clock;

struct FetchConfig
{
  struct Geometry
  {
    std::optional<double> lift_zero_height_m;
    std::optional<std::array<double, 2>> camera_xy;
    std::optional<std::array<double, 4>> camera_rotation;
  } geometry;
  double observation_distance{.30};
  double cv_frame_age{1.0};  // QR runs at a lower rate than YOLO; source age, not receipt age.
  bool no_qr{false};
  double qr_lateral_tolerance{.03}, qr_max_travel{.30}, qr_align_timeout{15};
  struct Approach
  {
    bool enabled{false};
    double target_distance{.25}, min_distance{.20}, max_distance{.28}, lateral_tolerance{.04};
    double max_step{.05}, max_travel{.30}, shelf_clearance{.12};
  } approach;
  struct Framing
  {
    double target_u{.60}, tolerance_u{.05}, timeout{10}, max_frame_age{1.0};
  } framing;
  double search_timeout{20};
  std::optional<double> front_observation_drop{0.0}, top_observation_drop{0.0};
  Json startup_fields;  // Normalized immutable fields, compared only on explicit reload.
  bool enabled{false}, calibrated{false};
  std::string return_point;
  double withdraw{0}, speed{0.04}, yaw_speed{0.15}, position_tolerance{0.005}, yaw_tolerance{0.03};
  double status_age{0.5}, observation_age{1.0}, observation_timeout{5}, step_timeout{60};
  double service_timeout{3}, navigation_timeout{120}, local_timeout{10}, confidence{0.2};
  unsigned max_align{8};
  std::string grasp_mode{"front"};
  GraspProfile front, top;
};
// Optional error output keeps common geometry usable for survey on fetch-only errors.
FetchConfig load_fetch_config(const std::string & path,std::string * profile_error=nullptr);
void reload_fetch_profiles(FetchConfig & current, const FetchConfig & candidate);

struct SurveyConfig
{
  bool enabled{false};
  std::string class_id{"red_box"};
  std::array<double,4> roi{0,0,1,1};
  unsigned confirmations{2}, max_rescans{1};
  double deadband{2}, frame_gap{1.0}, frame_gap_distance{.02}, edge_adjust{.03}, distance_step{.03};
  std::optional<double> sweep_speed;
  std::optional<std::array<double,2>> distance_range;
};
SurveyConfig load_survey_config(const std::string &);

class TaskManagerNode : public rclcpp::Node
{
public:
  explicit TaskManagerNode(const rclcpp::NodeOptions & options);
  ~TaskManagerNode() override;

private:
  friend class FetchBoundaryTest;
  friend class FetchStartupTest;
  enum class Phase {Prepare, ToShelf, Lift, Settle, Scan, Detect, Align, Grasp, Exit, Recover, Withdraw, Return, Finish};
  enum class Pending {None, Service, Navigation, Arm, Lift, ArmStatus, LiftStatus, Local};
  enum class Holding {Unknown, Empty, Confirmed};
  enum class TaskKind {Navigate, Fetch, Survey};
  enum class SurveyPhase {Prepare, NextLayer, ToShelf, Lift, LocateStart, ObserveStart, Sweep, SettleEnd, Finalize, Recovery, Return, Finish};
  enum class LocalPurpose {Pose, Search, Approach, Qr, SurveySweep};
  struct SurveyRun
  {
    SurveyPhase phase{SurveyPhase::Prepare}, next{SurveyPhase::NextLayer};
    size_t index{0};
    std::string drug_name;
    std::vector<size_t> layer_indices;
    unsigned attempt{1};
    bool left_to_right{true}, entry_left{true}, initialized{false}, observing{false};
    bool end_aligned{false}, edge_adjusted{false};
    bool view_clipped{false};
    std::string shelf, layer_error, count_error, recovery_reason, return_status{"not_started"};
    ShelfCounter counter;
    Json results=Json::array(), feedback;
    int feedback_phase{-1},feedback_count{-2};
    size_t feedback_index{0};
    unsigned feedback_attempt{0};
    int64_t barrier{0}, last_boxes{0}, stop_source{0}, last_qr{0};
    int64_t qr_request{0}; // One source frame in flight, cleared before moving.
    std::optional<double> entry_observed;
    struct PoseSample {int64_t ns;double x,y,yaw;};
    std::deque<PoseSample> poses;
    std::array<double,9> projection{}; // K^-T R^T, row-major, fixed per camera profile.
    double fx{0},fy{0},cx{0},cy{0}, work_distance{0}, sweep_speed{0};
    double gap_x{0},gap_y{0}, sweep_origin{0};
    Clock::time_point phase_started{Clock::now()}, qr_started{Clock::now()}, sweep_started{};
  };
  struct Candidate
  {
    int64_t stamp{0};
    std::string id, frame, class_id;
    double q{0}, score{0};
    bool valid{false};
  };
  struct Marker
  {
    int64_t stamp{0};
    double min_q{0}, max_q{0}, q{0};
  };
  struct DepthAlignment
  {
    double distance{0}, lateral{0}, dx{0}, dy{0};
  };
  struct VisualFrame
  {
    int64_t source{0}, received{0};
    size_t detections{0};
    uint64_t age_samples{0};
    double age_sum_ms{0}, age_max_ms{0};
    const char * status{"not_received"};
  };
  struct CandidateSummary
  {
    size_t eligible{0}, identity{0}, wrong_class{0}, low_score{0};
    size_t invalid_box{0}, outside_roi{0}, boundary{0};
    const char * locked_reason{"not_seen"};
  };
  struct NavProgress
  {
    Clock::time_point started{}, near_started{}, finished{};
    bool received{false}, near{false}, ack_received{false}, accepted{false};
    double ack_ms{0};
    double xy_error{0}, yaw_error{0}, remaining{0}, x{0}, y{0}, yaw{0};
    double feedback_travel_m{0}, feedback_rotation_rad{0};
    int64_t source_ns{0};
    int recoveries{0}, terminal_code{-1};
  };
  struct Active
  {
    TaskKind kind{TaskKind::Navigate};
    bool business() const {return is_fetch || kind==TaskKind::Survey;}
    bool is_survey() const {return kind==TaskKind::Survey;}
    std::shared_ptr<TaskHandle> handle;
    std::shared_ptr<FetchHandle> fetch;
    std::shared_ptr<SurveyHandle> survey;
    std::unique_ptr<SurveyRun> survey_run;
    bool inventory_handled{false};
    uint8_t requested_quantity{1}, planned_quantity{1}, fetched_quantity{0};
    Json inventory_update;
    std::shared_ptr<NavHandle> nav;
    NavProgress navigation;
    Pose pose, return_pose;
    std::string id, leg_id, context, error, message;
    Code outcome{Code::UNKNOWN};
    bool started{false}, sent{false}, running{false}, cancel{false}, cancel_sent{false};
    bool stopped{false}, terminal{false}, blocked{false}, upper_cancel_pending{false}, close_failed{false};
    bool is_fetch{false}, nav_active{false}, business_reserved{false};
    bool navigation_compute_ready{false};
    bool cv_maybe{false}, yolo_maybe{false}, pause_requested{false}, one_step{false};
    bool had_frame{false}, waiting_permit{false};
    bool hardware_result{false}, grip_succeeded{false};
    bool search_ready{false}, left_confirmed{false}, local_open{false}, waiting_point{false}, tracking_lost{false};
    bool scan_started{false}, search_started{false}, framing_started{false};
    LocalPurpose local_purpose{LocalPurpose::Pose};
    WarehouseTarget target;
    Candidate candidate;
    Candidate last_candidate;  // Diagnostics only; never a motion input.
    VisualFrame frame_2d, frame_3d;
    CandidateSummary candidate_summary;
    const char * point_pair_status{"not_attempted"};
    Json last_point, hardware_diag, failure_snapshot;
    std::string locked_id;
    Clock::time_point lost_since{};
    std::deque<Candidate> candidates;
    D3::ConstSharedPtr targets;
    std::string boundary_skip, paused_boundary, qr_context_error;
    Marker left_marker, right_marker;
    int64_t cv_frame_stamp{0}, qr_point_stamp{0};
    unsigned qr_moves{0};
    double qr_travel{0};
    bool right_reached{false}, boundary_wait{false}, locating_right{false};
    double right_limit_m{0};
    double right_x{0}, right_y{0}, inward_x{0}, inward_y{0}, camera_right_sign{1};
    double scan_elapsed{0}, search_elapsed{0}, framing_elapsed{0};
    unsigned reacquisitions{0};
    Clock::time_point budget_tick{Clock::now()};
    Phase phase{Phase::Prepare}, next_phase{Phase::Scan};
    Pending pending{Pending::None};
    uint64_t operation{0}, leg{0};
    std::string drug, expected_class, image_frame, step_id, skip_before;
    std::string depth_mode{"off"};
    bool resources_parked{false};
    int width{0}, height{0};
    size_t step_index{0};
    unsigned align_attempts{0}, grip_attempts{0};
    double search_offset{0}, search_start_offset{0}, shelf_odom_x{0}, shelf_odom_y{0}, shelf_yaw{0};
    double inward_offset{0};  // Preserve the new observation line after approaching.
    double initial_shelf_clearance{0}, approach_limit{0};
    unsigned approach_steps{0};
    std::optional<double> camera_distance, camera_lateral;
    std::optional<geometry_msgs::msg::Transform> camera_from_arm;
    double move_x{0}, move_y{0}, move_yaw{0}, previous_x{0}, previous_y{0}, previous_yaw{0};
    double lift_goal{0};
    int64_t barrier{0}, target_stamp{0}, sent_stamp{0};
    Point3 surface{};
    GraspPlan plan;
    ArmGoal arm_goal;
    std::vector<ArmStep> recovery;
    Clock::time_point step_started{Clock::now()};
  };

  Json context_state() const;
  Json inspect() const;
  std::string fetch_admission_error(const WarehouseTarget *) const;
  Json navigation_diagnostics(const Active &) const;
  void record_visual_frame(VisualFrame &, const std_msgs::msg::Header &, size_t count);
  Json vision_diagnostics(const Active &) const;
  void capture_failure();
  void load_warehouse();
  Json warehouse_state() const;
  std::string warehouse_context_error() const;
  void configure(const Control::Request & req);
  Pose resolve(const Task::Goal & goal) const;
  rclcpp_action::GoalResponse admit(const rclcpp_action::GoalUUID &, const Task::Goal &);
  rclcpp_action::GoalResponse admit_fetch(const rclcpp_action::GoalUUID &, const Fetch::Goal &);
  void request_cancel();
  bool cancel_current();
  void cancel_nav(const std::shared_ptr<Active> & task);
  void advance();
  void send_nav(const std::shared_ptr<Active> & task);
  void navigation_compute(const std::shared_ptr<Active> & task, bool enabled);
  void set_gate(const std::shared_ptr<Active> & task, const std::string & state);
  void gate_done(const std::shared_ptr<Active> &, const std::string &, bool, const std::string &);
  void begin_navigation(const Pose & pose, Phase next);
  void navigation_done(const std::shared_ptr<Active> & task);

  void setup_fetch();
  void setup_survey();
  void advance_business();
  bool ready_pose_ok() const;
  std::string survey_admission_error(const std::string & drug = {},
    std::vector<size_t> * selected = nullptr) const;
  const SurveyLayer & survey_layer(const SurveyRun & run) const;
  rclcpp_action::GoalResponse admit_survey(const rclcpp_action::GoalUUID &,const Survey::Goal &);
  void advance_survey();
  void enter_survey(SurveyPhase);
  void settle_survey(SurveyPhase);
  void survey_local_tick();
  void survey_local_finished();
  void survey_feedback();
  Json survey_state() const;
  void survey_layer_failed(const std::string &);
  void finalize_survey_layer();
  void finish_survey();
  void start_survey_vision();
  bool initialize_survey_frame();
  bool survey_pose(int64_t,SurveyRun::PoseSample &) const;
  double survey_camera_position(const SurveyRun::PoseSample &) const;
  void clear_survey_depth();
  bool survey_line(int64_t,std::array<double,3> &,SurveyRun::PoseSample * = nullptr) const;
  void on_survey_camera(sensor_msgs::msg::CameraInfo::ConstSharedPtr);
  void on_survey_boxes(D2::ConstSharedPtr);
  void on_survey_cv(std_msgs::msg::String::ConstSharedPtr);
  void on_survey_qr(D3::ConstSharedPtr);
  void apply_fetch_inventory_delta();
  void task_control(const Control::Request &, Control::Response &);
  void advance_fetch();
  void enter(Phase phase, const std::string & step);
  void event(const std::string & name, const std::string & detail = "");
  void deadline(double seconds, const std::string & error);
  void cancel_deadline();
  void fail(const std::string & error, bool uncertain = false);
  void finish_fetch();
  void web_call(const std::string & op, Json args, std::function<void(const Json &)> done);
  void toggle_yolo(bool enabled, std::function<void()> done);
  void depth_mode(const std::string & mode, std::function<void()> done);
  void stop_vision(std::function<void()> done);
  void settle(Phase next);
  void scan();
  bool initialize_search_frame();
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr message);
  void detect();
  void search_next();
  void on_qr_point(D3::ConstSharedPtr message);
  void on_cv(std_msgs::msg::String::ConstSharedPtr message);
  void on_detections(D2::ConstSharedPtr message);
  void on_targets(D3::ConstSharedPtr message);
  void align_or_plan();
  bool depth_alignment(const Point3 & surface, DepthAlignment & alignment, std::string & error) const;
  void try_target_pair();
  void update_budgets();
  bool stage_boundary(const std::string & name);
  void vision_timer();
  void open_local(LocalPurpose purpose);
  void local_finished();
  void zero_local();
  void send_lift(double target, Phase next);
  void send_arm(const ArmStep & step);
  void check_hardware();
  void hardware_done();
  void complete_fetch_item();
  void begin_fetch_return();
  const GraspProfile & profile() const;
  const std::vector<ArmStep> & steps() const;
  bool hardware_ok(bool idle = true) const;
  Json feedback_state() const;
  bool stopped() const;
  bool fresh(int64_t stamp, double max_age) const;
  bool matches(const std::shared_ptr<Active> &, uint64_t operation) const;
  bool pause_boundary(const std::string & id);
  void start_local(double dx, double dy, Phase next, double turn = 0, LocalPurpose purpose = LocalPurpose::Pose);
  void local_tick();
  void stop_local();
  void on_odom(Odom::ConstSharedPtr message);
  static const char * phase_name(Phase phase);

  std::unique_ptr<InferenceControl> inference_;
  std::string instance_, context_, web_, map_, gate_state_;
  bool ready_{false}, configuring_{false};
  Json points_ = Json::object();
  Json last_navigation_;
  std::shared_ptr<Active> active_;
  // Default mutually-exclusive group: accepted runs inline after admission.
  // A response-send failure may leave this candidate, never an active task.
  std::shared_ptr<Active> prepared_goal_;
  std::chrono::milliseconds timeout_{1000};
  uint64_t gate_sequence_{0};
  int64_t gate_request_{0};
  rclcpp::TimerBase::SharedPtr deadline_;
  rclcpp_action::Client<Nav>::SharedPtr nav_;
  rclcpp_action::Server<Task>::SharedPtr server_;
  rclcpp::Client<Control>::SharedPtr gate_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr upper_cancel_, fetch_cancel_;
  rclcpp::Service<Control>::SharedPtr control_, debug_;
  rclcpp::Service<Trigger>::SharedPtr cancel_;

  FetchConfig config_;
  SurveyConfig survey_config_;
  std::string survey_file_,survey_error_,motion_config_error_;
  std::vector<SurveyLayer> survey_layers_;
  InventoryStore inventory_;
  Json last_survey_result_,last_survey_rejection_;
  rclcpp_action::Server<Survey>::SharedPtr survey_server_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr survey_cancel_;
  std::string config_file_, config_error_, warehouse_file_, warehouse_error_;
  WarehouseTable warehouse_;
  std::optional<WarehouseAnchor> warehouse_anchor_;
  bool warehouse_loaded_{false};
  Json warehouse_capabilities_ = Json::object();
  std::set<std::string> drugs_, breakpoints_;
  Json drug_catalog_{{"drugs",Json::array()},{"error",""}};
  Holding holding_{Holding::Unknown};
  Holding left_holding_{Holding::Unknown}, right_holding_{Holding::Unknown};
  bool motion_inhibited_{false}, recovery_pending_{false};
  uint64_t recovery_sequence_{0};
  Json last_result_ = Json::object();
  Json last_fetch_rejection_ = Json::object();
  rclcpp_action::Server<Fetch>::SharedPtr fetch_server_;
  rclcpp_action::Client<Arm>::SharedPtr arm_client_;
  rclcpp_action::Client<Lift>::SharedPtr lift_client_;
  rclcpp::Client<Toggle>::SharedPtr yolo_;
  rclcpp::Client<Control>::SharedPtr depth_control_;
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_sub_;
  rclcpp::Subscription<LiftStatus>::SharedPtr lift_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cv_sub_;
  rclcpp::Publisher<D2>::SharedPtr qr_request_pub_;
  rclcpp::Subscription<D3>::SharedPtr qr_point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<D2>::SharedPtr detections_sub_;
  rclcpp::Subscription<D3>::SharedPtr targets_sub_;
  ArmStatus::ConstSharedPtr arm_status_;
  LiftStatus::ConstSharedPtr lift_status_;
  Odom::ConstSharedPtr odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr local_pub_;
  rclcpp::TimerBase::SharedPtr task_deadline_, local_timer_;
};
}  // namespace k1muse_manager
