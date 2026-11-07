#include "k1muse_manager/task_manager_node_impl.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace k1muse_manager
{
namespace
{
int64_t stamp(const std_msgs::msg::Header & h)
{return int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;}
double yaw(const Odom & o) {return 2*std::atan2(o.pose.pose.orientation.z,o.pose.pose.orientation.w);}
Pose pose(const WarehousePose & p)
{
  Pose result;result.header.frame_id="map";result.pose.position.x=p.x_m;result.pose.position.y=p.y_m;
  result.pose.orientation.z=std::sin(p.yaw_rad/2);result.pose.orientation.w=std::cos(p.yaw_rad/2);return result;
}
double elapsed(Clock::time_point p) {return std::chrono::duration<double>(Clock::now()-p).count();}
}
SurveyConfig load_survey_config(const std::string & file)
{
  const auto n=YAML::LoadFile(file);SurveyConfig c;
  c.enabled=n["enabled"].as<bool>(false);c.class_id=n["class_id"].as<std::string>("red_box");
  if(n["roi"]) {if(n["roi"].size()!=4) {throw std::runtime_error("survey_roi_invalid");}
    for(size_t i=0;i<4;++i) {c.roi[i]=n["roi"][i].as<double>();}}
  const auto integer=[&](const char * key,unsigned fallback) {
    if(!n[key]) {return fallback;}const auto s=n[key].Scalar();
    if(s.empty() || s.find_first_not_of("0123456789")!=std::string::npos) {throw std::runtime_error("survey_integer_invalid");}
    return n[key].as<unsigned>();
  };
  c.confirmations=integer("confirm_observations",2);c.max_rescans=integer("max_rescans",1);
  const auto number=[&](const char * key,double fallback) {const double v=n[key].as<double>(fallback);
      if(!std::isfinite(v)) {throw std::runtime_error("survey_nonfinite");}return v;};
  c.deadband=number("side_deadband_px",2);c.frame_gap=number("max_frame_gap_sec",1.0);
  c.frame_gap_distance=number("max_frame_gap_distance_m",.02);
  c.edge_adjust=number("edge_adjust_max_m",.03);c.distance_step=number("distance_adjust_step_m",.03);
  if(n["sweep_speed_mps"] && !n["sweep_speed_mps"].IsNull()) {
    c.sweep_speed=number("sweep_speed_mps",0);
    if(*c.sweep_speed<=0 || *c.sweep_speed>.20) {throw std::runtime_error("survey_sweep_speed_invalid");}
  }
  for(auto v:c.roi) {if(!std::isfinite(v) || v<0 || v>1) {throw std::runtime_error("survey_roi_invalid");}}
  if(c.class_id.empty() || c.roi[0]>=c.roi[2] || c.roi[1]>=c.roi[3] || !c.confirmations ||
    c.max_rescans>1 || c.deadband<0 || c.frame_gap<=0 || c.frame_gap_distance<=0 || c.edge_adjust<0 || c.distance_step<0)
  {throw std::runtime_error("survey_config_invalid");}
  if(n["distance_range_m"] && !n["distance_range_m"].IsNull()) {
    if(n["distance_range_m"].size()!=2) {throw std::runtime_error("survey_distance_range_invalid");}
    std::array<double,2> range{n["distance_range_m"][0].as<double>(),n["distance_range_m"][1].as<double>()};
    if(!std::isfinite(range[0]) || !std::isfinite(range[1]) || range[0]<=0 || range[0]>=range[1]) {throw std::runtime_error("survey_distance_range_invalid");}
    c.distance_range=range;
  }
  return c;
}
void TaskManagerNode::setup_survey()
{
  survey_cancel_=create_client<action_msgs::srv::CancelGoal>("/k1muse/task/survey/_action/cancel_goal");
  survey_server_=rclcpp_action::create_server<Survey>(this,"/k1muse/task/survey",
    [this](const auto & id,std::shared_ptr<const Survey::Goal> g){return admit_survey(id,*g);},
    [this](std::shared_ptr<SurveyHandle> h) {
      if(!active_ || active_->survey!=h) {return rclcpp_action::CancelResponse::REJECT;}
      request_cancel();return rclcpp_action::CancelResponse::ACCEPT;
    },[this](std::shared_ptr<SurveyHandle> h) {
      active_=std::move(prepared_goal_);active_->survey=h;inference_->business(true);advance_survey();
    });
}
void TaskManagerNode::advance_business()
{if(active_ && active_->is_survey()) {advance_survey();} else {advance_fetch();}}
bool TaskManagerNode::ready_pose_ok() const
{
  if(!arm_status_) {return false;}
  for(const auto * p:{&config_.front,&config_.top}) {
    const auto it=p->poses.find("ready");
    if((p->calibrated || survey_config_.enabled) && it!=p->poses.end()) {
      const auto & v=it->second;
      if(std::hypot(std::hypot(arm_status_->current_x-v[0],arm_status_->current_y-v[1]),arm_status_->current_z-v[2])<=.010) {return true;}
    }
  }
  return false;
}
const SurveyLayer & TaskManagerNode::survey_layer(const SurveyRun & run) const
{return survey_layers_[run.layer_indices[run.index]];}
std::string TaskManagerNode::survey_admission_error(
  const std::string & drug, std::vector<size_t> * selected) const
{
  if(!survey_error_.empty()) {return survey_error_;}
  if(!survey_config_.enabled) {return "survey_disabled";}
  const auto error=warehouse_context_error();if(!error.empty()) {return error;}
  if(active_ || recovery_pending_) {return "task_busy";}
  if(motion_inhibited_) {return "motion_inhibited";}
  if(holding_!=Holding::Empty) {return "holding_not_empty";}
  if(!hardware_ok() || !stopped() || !ready_pose_ok()) {return "motion_preparation_required";}
  if(!nav_->action_server_is_ready() || !gate_->service_is_ready() || !lift_client_->action_server_is_ready()) {return "survey_backend_unavailable";}
  if(survey_config_.distance_range && ((*survey_config_.distance_range)[0]>config_.observation_distance ||
    (*survey_config_.distance_range)[1]<config_.observation_distance)) {return "survey_distance_range_excludes_observation_distance";}
  bool matched=false,scannable=false;
  for(size_t i=0;i<survey_layers_.size();++i) {
    const auto & layer=survey_layers_[i];
    if(!drug.empty() && layer.drug!=drug) {continue;}
    matched=true;scannable=scannable || layer.scannable();
    if(selected) {selected->push_back(i);}
  }
  if(!drug.empty() && !matched) {return "survey_drug_not_found";}
  if(!scannable) {return "no_scannable_layers";}
  return "";
}
rclcpp_action::GoalResponse TaskManagerNode::admit_survey(const rclcpp_action::GoalUUID & id,const Survey::Goal & goal)
{
  prepared_goal_.reset();
  try {
    auto t=std::make_shared<Active>();t->kind=TaskKind::Survey;t->survey_run=std::make_unique<SurveyRun>();
    auto & run=*t->survey_run;run.drug_name=goal.drug_name;
    const auto error=survey_admission_error(goal.drug_name,&run.layer_indices);
    if(!error.empty()) {throw std::runtime_error(error);}
    if(!goal.map_id.empty() && goal.map_id!=map_) {throw std::runtime_error("map_changed");}
    t->id=rclcpp_action::to_string(id);t->context=context_;
    Task::Goal back;back.task_name=config_.return_point;t->return_pose=resolve(back);
    for(const auto i:run.layer_indices) {
      const auto & l=survey_layers_[i];
      run.results.push_back({{"shelf_id",l.shelf_id},{"layer",l.layer},{"drug",l.drug},
        {"status",l.drug.empty()?"skipped":"not_visited"},{"attempts",0},{"quantity",nullptr},{"reason",""}});
    }
    prepared_goal_=std::move(t);return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } catch(const std::exception & e) {
    last_survey_rejection_={{"goal_id",rclcpp_action::to_string(id)},{"reason",e.what()}};
    RCLCPP_WARN(get_logger(),"Survey rejected: %s",e.what());return rclcpp_action::GoalResponse::REJECT;
  }
}
void TaskManagerNode::enter_survey(SurveyPhase phase)
{
  auto & s=*active_->survey_run;
  if(phase!=s.phase && (phase==SurveyPhase::LocateStart || phase==SurveyPhase::SettleEnd)) {s.qr_started=Clock::now();}
  s.phase=phase;s.phase_started=Clock::now();
  active_->waiting_permit=false;survey_feedback();
}
Json TaskManagerNode::survey_state() const
{
  Json r={{"enabled",survey_config_.enabled},{"config_error",survey_error_},{"admission_error",survey_admission_error()},
    {"last_survey_rejection",last_survey_rejection_},{"last_result",last_survey_result_}};
  if(active_ && active_->is_survey()) {
    const auto & s=*active_->survey_run;r["drug_name"]=s.drug_name;r["current"]=s.feedback;r["error"]=active_->error;r["blocked"]=active_->blocked;
    r["source_ns"]=s.last_boxes;r["qr_source_ns"]=s.last_qr;r["layer_error"]=s.layer_error;r["count_error"]=s.count_error;
  }
  return r;
}
void TaskManagerNode::survey_feedback()
{
  if(!active_ || !active_->is_survey()) {return;}
  auto t=active_;auto & s=*t->survey_run;
  const char * stages[]={"prepare","next_layer","to_shelf","lift","locate_start","observe_start","sweep","settle_end","finalize_layer","recovery","return","finish"};
  const bool current=s.index<s.layer_indices.size() && s.phase!=SurveyPhase::Return && s.phase!=SurveyPhase::Finish;
  const auto * l=current?&survey_layer(s):nullptr;
  const int count=s.observing?s.counter.provisional_count():-1;
  if(s.feedback_phase==int(s.phase) && s.feedback_count==count && s.feedback_index==s.index && s.feedback_attempt==s.attempt) {return;}
  s.feedback_phase=int(s.phase);s.feedback_count=count;s.feedback_index=s.index;s.feedback_attempt=s.attempt;
  const Json feedback={{"stage",stages[static_cast<size_t>(s.phase)]},{"shelf_id",l?l->shelf_id:""},
    {"layer",l?l->layer:0},{"direction",l?(s.left_to_right?"left_to_right":"right_to_left"):"none"},
    {"attempt",l?s.attempt:0},{"provisional_count",count}};
  if(feedback==s.feedback) {return;}s.feedback=feedback;
  if(!t->survey || !t->survey->is_active()) {return;}
  auto f=std::make_shared<Survey::Feedback>();f->stage=feedback.at("stage");f->shelf_id=feedback.at("shelf_id");
  f->layer=feedback.at("layer");f->direction=feedback.at("direction");f->attempt=feedback.at("attempt");f->provisional_count=feedback.at("provisional_count");
  t->survey->publish_feedback(f);
}
void TaskManagerNode::settle_survey(SurveyPhase next)
{
  auto t=active_;t->survey_run->next=next;
  web_call("wait_stopped",Json::object(),[this,t](const Json &) {
    if(!hardware_ok() || !stopped()) {fail("survey positioning did not stop");return;}
    enter_survey(t->survey_run->next);advance_survey();
  });
}
void TaskManagerNode::advance_survey()
{
  auto t=active_;if(!t || !t->is_survey() || t->pending!=Pending::None || t->nav_active || t->blocked || t->upper_cancel_pending) {return;}
  auto & s=*t->survey_run;
  if(t->cancel || !t->error.empty()) {finish_survey();return;}
  if(!s.layer_error.empty() && s.phase!=SurveyPhase::Recovery) {enter_survey(SurveyPhase::Recovery);}
  switch(s.phase) {
    case SurveyPhase::Prepare:
      web_call("set_business",{{"state","starting"}},[this,t](const Json &) {
        t->business_reserved=true;enter_survey(SurveyPhase::NextLayer);advance_survey();
      });break;
    case SurveyPhase::NextLayer: {
      while(s.index<s.layer_indices.size() && !survey_layer(s).scannable()) {
        const auto & l=survey_layer(s);auto & result=s.results[s.index];
        if(!l.drug.empty()) {
          result["status"]="incomplete";result["reason"]=l.unavailable_reasons.empty()?"unavailable":l.unavailable_reasons.front();
          inventory_.record_incomplete_layer(l.shelf_id,l.layer,l.drug,result.at("reason"));
        }
        ++s.index;
      }
      if(s.index==s.layer_indices.size()) {
        stop_vision([this,t] {auto & run=*t->survey_run;run.observing=false;
          enter_survey(SurveyPhase::Return);run.next=SurveyPhase::Finish;begin_navigation(t->return_pose,Phase::Finish);});return;
      }
      const auto & l=survey_layer(s);const bool new_shelf=l.shelf_id!=s.shelf;
      const bool navigate=new_shelf || !s.end_aligned;
      if(new_shelf) {s.entry_left=true;s.shelf=l.shelf_id;}
      s.left_to_right=s.entry_left;s.attempt=1;s.initialized=false;s.observing=false;
      s.end_aligned=s.edge_adjusted=false;s.layer_error.clear();s.count_error.clear();s.sweep_started={};
      s.work_distance=config_.observation_distance;s.sweep_speed=survey_config_.sweep_speed.value_or(config_.speed);s.results[s.index]["attempts"]=1;
      t->qr_moves=0;t->qr_travel=0;t->qr_point_stamp=0;
      if(navigate) {
        stop_vision([this,t] {auto & run=*t->survey_run;const auto & layer=survey_layer(run);
          enter_survey(SurveyPhase::ToShelf);run.next=SurveyPhase::Lift;
          begin_navigation(pose(run.entry_left?*layer.left_pose:*layer.right_pose),Phase::Lift);});
      } else {enter_survey(SurveyPhase::Lift);advance_survey();}
      break;
    }
    case SurveyPhase::Lift: {
      const auto z=*survey_layer(s).lift_z_m;s.next=SurveyPhase::LocateStart;
      if(std::abs(lift_status_->current_z-z)>.002) {send_lift(z,Phase::Scan);}
      else {settle_survey(SurveyPhase::LocateStart);}break;
    }
    case SurveyPhase::LocateStart:
      start_survey_vision();break;
    case SurveyPhase::ObserveStart: {
      // The entry coordinate, not a marker crossing, determines whether an
      // outside observation step is needed for boxes immediately at the edge.
      const double direction=s.left_to_right?1:-1;
      if(!s.edge_adjusted && survey_config_.edge_adjust>config_.position_tolerance &&
        (!s.entry_observed || *s.entry_observed>direction*s.sweep_origin)) {
        s.edge_adjusted=true;s.next=SurveyPhase::ObserveStart;
        const SurveyRun::PoseSample current{stamp(odom_->header),odom_->pose.pose.position.x,
          odom_->pose.pose.position.y,yaw(*odom_)};
        const double d=s.sweep_origin-direction*survey_config_.edge_adjust-survey_camera_position(current);
        start_local(d*t->right_x,d*t->right_y,Phase::Scan,0,LocalPurpose::Qr);break;
      }
      enter_survey(SurveyPhase::Sweep);s.sweep_started=Clock::now();s.next=SurveyPhase::SettleEnd;
      t->qr_moves=0;t->qr_travel=0;open_local(LocalPurpose::SurveySweep);break;
    }
    case SurveyPhase::SettleEnd:
      // The same QR depth callback now aligns the terminal side, without clearing count evidence.
      break;
    case SurveyPhase::Finalize:
      finalize_survey_layer();break;
    case SurveyPhase::Recovery: {
      if(t->local_open) {stop_local();break;}
      // A lost coverage interval cannot be repaired by waiting at the end.
      clear_survey_depth();
      if(s.attempt<=survey_config_.max_rescans) {
        ++s.attempt;s.results[s.index]["attempts"]=s.attempt;
        s.recovery_reason=s.layer_error;s.layer_error.clear();s.count_error.clear();s.observing=false;
        s.initialized=s.end_aligned=s.edge_adjusted=false;
        s.counter.reset(survey_config_.confirmations,survey_config_.deadband,s.left_to_right);
        s.sweep_speed=survey_config_.sweep_speed.value_or(config_.speed);
        if(s.recovery_reason=="field_of_view" && survey_config_.distance_range && survey_config_.distance_step>0 &&
          (*survey_config_.distance_range)[1]>config_.observation_distance+config_.position_tolerance) {
          s.work_distance=std::min((*survey_config_.distance_range)[1],config_.observation_distance+survey_config_.distance_step);
        }
        stop_vision([this,t] {
          auto & run=*t->survey_run;const auto & l=survey_layer(run);
          auto p=run.entry_left?*l.left_pose:*l.right_pose;const double d=run.work_distance-config_.observation_distance;
          p.x_m-=std::cos(p.yaw_rad)*d;p.y_m-=std::sin(p.yaw_rad)*d;
          t->qr_moves=0;t->qr_travel=0;t->qr_point_stamp=0;
          enter_survey(SurveyPhase::ToShelf);run.next=SurveyPhase::Lift;begin_navigation(pose(p),Phase::Lift);
        });break;
      }
      const auto & l=survey_layer(s);auto & result=s.results[s.index];
      const auto reason=s.sweep_started!=Clock::time_point{} && !s.count_error.empty()?s.count_error:s.layer_error;
      result["status"]="incomplete";result["reason"]=reason;
      inventory_.record_incomplete_layer(l.shelf_id,l.layer,l.drug,reason);
      if(s.end_aligned) {s.entry_left=!s.left_to_right;}
      s.observing=false;s.layer_error.clear();++s.index;
      enter_survey(SurveyPhase::NextLayer);advance_survey();break;
    }
    case SurveyPhase::Finish:finish_survey();break;
    default:break;
  }
}
void TaskManagerNode::start_survey_vision()
{
  auto t=active_;auto & s=*t->survey_run;
  if(t->depth_mode != "marker") {depth_mode("marker", [this] {start_survey_vision();});return;}
  if(!s.observing) {
    s.observing=true;s.barrier=now().nanoseconds();s.last_boxes=s.last_qr=0;
    s.poses.clear();s.qr_request=0;s.entry_observed.reset();s.count_error.clear();s.view_clipped=false;
    s.counter.reset(survey_config_.confirmations,survey_config_.deadband,s.left_to_right);
    s.phase_started=Clock::now();s.fx=0;t->width=t->height=0;t->image_frame.clear();
  }
  if(!tf_buffer_) {tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock(),tf2::durationFromSec(2));
    tf_listener_=std::make_unique<tf2_ros::TransformListener>(*tf_buffer_,this,false);}
  if(!camera_info_sub_) {camera_info_sub_=create_subscription<sensor_msgs::msg::CameraInfo>(
    "/k1muse/front_camera/color/camera_info",rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){on_camera_info(m);});}
  if(!cv_sub_) {cv_sub_=create_subscription<std_msgs::msg::String>("/k1muse/perception/cv/observations",rclcpp::QoS(1),[this](std_msgs::msg::String::ConstSharedPtr m){on_cv(m);});}
  if(!qr_request_pub_) {qr_request_pub_=create_publisher<D2>("/k1muse/perception/qr_depth/requests",rclcpp::QoS(1));}
  if(!qr_point_sub_) {qr_point_sub_=create_subscription<D3>("/k1muse/perception/qr_depth/points",rclcpp::QoS(1),[this](D3::ConstSharedPtr m){on_qr_point(m);});}
  vision_timer();
  if(!t->cv_maybe) {
    if(!inference_->idle()) {t->waiting_permit=true;return;}
    t->waiting_permit=false;t->cv_maybe=true;t->pending=Pending::Service;const auto op=++t->operation;
    deadline(config_.service_timeout,"survey CV start acknowledgement timeout");
    inference_->scan(true,t->id+":cv",[this,t,op](bool ok) {
      if(!matches(t,op)) {return;}cancel_deadline();
      if(!ok) {
        if(inference_->owns_scan(t->id+":cv")) {fail("survey CV start unresolved",true);}
        else {t->pending=Pending::None;t->cv_maybe=false;fail("survey CV unavailable");}return;
      }
      t->pending=Pending::None;advance_business();
    });return;
  }
  if(!t->yolo_maybe) {
    if(!inference_->idle()) {t->waiting_permit=true;return;}
    if(!detections_sub_) {detections_sub_=create_subscription<D2>("/k1muse/perception/detections",rclcpp::QoS(1),[this](D2::ConstSharedPtr m){on_detections(m);});}
    t->waiting_permit=false;toggle_yolo(true,[this]{advance_business();});return;
  }
  if(!s.initialized) {initialize_survey_frame();}
}
void TaskManagerNode::on_survey_camera(sensor_msgs::msg::CameraInfo::ConstSharedPtr m)
{
  auto t=active_;auto & s=*t->survey_run;if(!s.observing || t->cancel) {return;}
  if(!fresh(stamp(m->header),config_.observation_age) || m->width==0 || m->height==0 || m->header.frame_id.empty() ||
    !std::isfinite(m->k[0]) || !std::isfinite(m->k[4]) || !std::isfinite(m->k[2]) || !std::isfinite(m->k[5]) || m->k[0]<=0 || m->k[4]<=0) {return;}
  if(s.fx && (s.fx!=m->k[0] || s.fy!=m->k[4] || s.cx!=m->k[2] || s.cy!=m->k[5] ||
    t->width!=int(m->width) || t->height!=int(m->height) || t->image_frame!=m->header.frame_id)) {survey_layer_failed("camera_profile_changed");return;}
  if(!s.fx) {
    s.fx=m->k[0];s.fy=m->k[4];s.cx=m->k[2];s.cy=m->k[5];t->width=m->width;t->height=m->height;t->image_frame=m->header.frame_id;
    if(config_.geometry.camera_rotation) {
      const auto & q=*config_.geometry.camera_rotation;const tf2::Matrix3x3 r(tf2::Quaternion(q[0],q[1],q[2],q[3]));
      for(size_t j=0;j<3;++j) {
        s.projection[j]=r[j][0]/s.fx;s.projection[3+j]=r[j][1]/s.fy;
        s.projection[6+j]=r[j][2]-s.cx*r[j][0]/s.fx-s.cy*r[j][1]/s.fy;
      }
    }
  }
  if(s.phase==SurveyPhase::LocateStart && t->pending==Pending::None) {advance_survey();}
}
bool TaskManagerNode::initialize_survey_frame()
{
  auto t=active_;auto & s=*t->survey_run;
  if(!s.fx || !tf_buffer_ || !odom_ || !stopped() || !config_.geometry.camera_rotation || !config_.geometry.camera_xy) {return false;}
  try {
    const auto tf=tf_buffer_->lookupTransform("odom","map",tf2_ros::fromRclcpp(rclcpp::Time(odom_->header.stamp)));
    if(!fresh(stamp(tf.header),config_.status_age)) {return false;}
    const auto & l=survey_layer(s);const auto & p=s.entry_left?*l.left_pose:*l.right_pose;
    const double a=p.yaw_rad+2*std::atan2(tf.transform.rotation.z,tf.transform.rotation.w);
    t->inward_x=std::cos(a);t->inward_y=std::sin(a);t->right_x=t->inward_y;t->right_y=-t->inward_x;t->shelf_yaw=a;
    t->shelf_odom_x=odom_->pose.pose.position.x;t->shelf_odom_y=odom_->pose.pose.position.y;t->inward_offset=0;
    s.initialized=true;return true;
  } catch(const tf2::TransformException &) {return false;}
}
bool TaskManagerNode::survey_pose(int64_t ns,SurveyRun::PoseSample & p) const
{
  const auto & s=*active_->survey_run;if(!s.initialized || s.poses.size()<2) {return false;}
  const auto upper=std::lower_bound(s.poses.begin(),s.poses.end(),ns,[](const auto & p,int64_t stamp){return p.ns<stamp;});
  if(upper==s.poses.end()) {return false;}
  p=*upper;
  if(p.ns!=ns) {
    if(upper==s.poses.begin()) {return false;}const auto & a=*std::prev(upper);
    if((upper->ns-a.ns)*1e-9>config_.status_age) {return false;}
    const double f=double(ns-a.ns)/double(upper->ns-a.ns);p={ns,a.x+f*(p.x-a.x),a.y+f*(p.y-a.y),a.yaw+f*warehouse_wrap(p.yaw-a.yaw)};
  }
  return true;
}
double TaskManagerNode::survey_camera_position(const SurveyRun::PoseSample & p) const
{
  const auto & t=*active_;const auto & c=*config_.geometry.camera_xy;
  const double x=p.x+std::cos(p.yaw)*c[0]-std::sin(p.yaw)*c[1];
  const double y=p.y+std::sin(p.yaw)*c[0]+std::cos(p.yaw)*c[1];
  return (x-t.shelf_odom_x)*t.right_x+(y-t.shelf_odom_y)*t.right_y;
}
bool TaskManagerNode::survey_line(int64_t ns,std::array<double,3> & line,SurveyRun::PoseSample * result) const
{
  const auto t=active_;const auto & s=*t->survey_run;
  SurveyRun::PoseSample p;if(!survey_pose(ns,p)) {return false;}
  const std::array<double,3> e{std::cos(p.yaw)*t->right_x+std::sin(p.yaw)*t->right_y,
    -std::sin(p.yaw)*t->right_x+std::cos(p.yaw)*t->right_y,0};
  for(size_t i=0;i<3;++i) {line[i]=s.projection[3*i]*e[0]+s.projection[3*i+1]*e[1];}
  const double norm=std::hypot(line[0],line[1]);if(norm<1e-9) {return false;}
  for(auto & v:line) {v/=norm;v*=s.left_to_right?1:-1;}
  if(result) {*result=p;}return true;
}
void TaskManagerNode::on_survey_boxes(D2::ConstSharedPtr message)
{
  auto t=active_;auto & s=*t->survey_run;if(!s.observing || !t->yolo_maybe || t->cancel || !t->error.empty()) {return;}
  const auto ns=stamp(message->header);record_visual_frame(t->frame_2d,message->header,message->detections.size());
  if(ns<=s.barrier || ns<=s.last_boxes || !fresh(ns,config_.observation_age) || message->header.frame_id!=t->image_frame) {return;}
  std::array<double,3> line{};SurveyRun::PoseSample p;
  if(!survey_line(ns,line,&p)) {return;}
  if(s.last_boxes && (ns-s.last_boxes)*1e-9>survey_config_.frame_gap &&
    std::hypot(p.x-s.gap_x,p.y-s.gap_y)>survey_config_.frame_gap_distance && s.count_error.empty()) {
    s.count_error="coverage_gap";
    event("coverage_gap", "source_gap_sec="+std::to_string((ns-s.last_boxes)*1e-9)+
      " distance_m="+std::to_string(std::hypot(p.x-s.gap_x,p.y-s.gap_y)));
  }
  s.last_boxes=ns;s.gap_x=p.x;s.gap_y=p.y;
  std::vector<ShelfCounter::Box> boxes;boxes.reserve(message->detections.size());
  const auto & roi=survey_config_.roi;
  for(const auto & d:message->detections) {
    if(d.header!=message->header || d.id.empty() || d.results.empty()) {continue;}
    if(d.id.rfind("frame/",0)==0) {fail("survey_requires_tracker: enable use_tracker in YOLO configuration");return;}
    const auto & h=d.results.front().hypothesis;const auto & b=d.bbox;const auto & c=b.center.position;
    if(h.class_id!=survey_config_.class_id || !std::isfinite(h.score) || h.score<config_.confidence ||
      !std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(b.size_x) || !std::isfinite(b.size_y) ||
      b.size_x<=0 || b.size_y<=0 || b.center.theta!=0 || c.x<roi[0]*t->width || c.x>roi[2]*t->width || c.y<roi[1]*t->height || c.y>roi[3]*t->height) {continue;}
    boxes.push_back({d.id,line[0]*c.x+line[1]*c.y+line[2]});
  }
  const double camera=survey_camera_position(p);
  if(s.phase==SurveyPhase::LocateStart) {
    const double entry=(s.left_to_right?1:-1)*camera;
    s.entry_observed=s.entry_observed?std::min(*s.entry_observed,entry):entry;
  }
  if(s.count_error.empty()) {s.counter.observe_boxes(ns,camera,boxes);}
  survey_feedback();
  if(s.phase==SurveyPhase::ObserveStart || s.phase==SurveyPhase::Finalize || s.phase==SurveyPhase::Recovery) {advance_survey();}
}
void TaskManagerNode::clear_survey_depth()
{
  auto & s=*active_->survey_run;
  if(!s.qr_request) {return;}
  s.qr_request=0;
  if(qr_request_pub_) {qr_request_pub_->publish(D2{});}
}
void TaskManagerNode::on_survey_cv(std_msgs::msg::String::ConstSharedPtr message)
{
  auto t=active_;auto & s=*t->survey_run;
  if(!s.observing || !s.initialized || !t->cv_maybe || t->cancel || !t->error.empty() ||
    !s.layer_error.empty() || s.index>=s.layer_indices.size()) {return;}
  try {
    const auto doc=Json::parse(message->data);const auto & h=doc.at("header");const auto & st=h.at("stamp");
    const int64_t sec=st.at("sec"),nano=st.at("nanosec");if(sec<0 || sec>INT32_MAX || nano<0 || nano>=1000000000) {return;}
    const int64_t ns=sec*1000000000LL+nano;
    if(ns<=s.barrier || ns<=s.last_qr || !fresh(ns,config_.cv_frame_age) || !doc.value("error",std::string()).empty()) {return;}
    if(doc.at("image_width")!=t->width || doc.at("image_height")!=t->height || h.at("frame_id")!=t->image_frame) {survey_layer_failed("camera_profile_changed");return;}
    s.last_qr=ns;
    if(s.qr_request && !fresh(s.qr_request,config_.cv_frame_age)) {clear_survey_depth();}
    const bool start=s.phase==SurveyPhase::LocateStart;
    const bool sweep=s.phase==SurveyPhase::Sweep && t->pending==Pending::Local;
    const bool measuring=(start || s.phase==SurveyPhase::SettleEnd) && t->pending==Pending::None && stopped();
    if(s.qr_request || (!sweep && !measuring) || !hardware_ok() || ns<=s.stop_source) {return;}
    const auto & layer=survey_layer(s);const bool left=start?s.entry_left:!s.entry_left;
    const auto & marker=left?layer.left_qr:layer.right_qr;
    if(!marker || !marker->marker_id) {return;}
    D2 request;request.header.frame_id=t->image_frame;request.header.stamp.sec=sec;request.header.stamp.nanosec=nano;
    unsigned matches=0;
    for(const auto & code:doc.at("markers")) {
      if(!code.at("id").is_number_integer() || code.at("id")!=*marker->marker_id) {continue;}
      ++matches;const auto & corners=code.at("corners");if(corners.size()!=4) {continue;}
      double x=0,y=0,xmin=t->width,xmax=0,ymin=t->height,ymax=0;bool valid=true;
      for(const auto & point:corners) {
        if(!point.is_array() || point.size()!=2) {valid=false;break;}const double u=point[0],v=point[1];
        if(!std::isfinite(u) || !std::isfinite(v)) {valid=false;break;}
        if(u<0 || u>=t->width || v<0 || v>=t->height) {s.view_clipped=true;valid=false;break;}
        x+=u/4;y+=v/4;xmin=std::min(xmin,u);xmax=std::max(xmax,u);ymin=std::min(ymin,v);ymax=std::max(ymax,v);
      }
      if(!valid || xmax<=xmin || ymax<=ymin) {continue;}
      vision_msgs::msg::Detection2D d;d.header=request.header;const std::string side=left?"left":"right";
      d.id=layer.shelf_id+"/"+std::to_string(layer.layer)+"/"+side;
      d.bbox.center.position.x=x;d.bbox.center.position.y=y;d.bbox.size_x=xmax-xmin;d.bbox.size_y=ymax-ymin;
      vision_msgs::msg::ObjectHypothesisWithPose result;result.hypothesis.class_id="qr_"+side;result.hypothesis.score=1;
      d.results={result};request.detections={d};
    }
    if(matches==1 && request.detections.size()==1) {
      s.qr_request=ns;qr_request_pub_->publish(request);
    }
  } catch(const Json::exception & e) {RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Invalid survey marker: %s",e.what());}
}
void TaskManagerNode::on_survey_qr(D3::ConstSharedPtr message)
{
  auto t=active_;auto & s=*t->survey_run;
  const auto ns=stamp(message->header);
  if(!s.observing || !s.initialized || !s.qr_request || ns!=s.qr_request ||
    message->header.frame_id!=t->image_frame) {return;}
  clear_survey_depth(); // Empty/invalid results also release this source-frame request.
  if(ns<=s.barrier || ns<=t->qr_point_stamp || !fresh(ns,config_.cv_frame_age) ||
    t->cancel || !t->error.empty() || !s.layer_error.empty() || !hardware_ok()) {return;}
  t->qr_point_stamp=ns;
  const bool start=s.phase==SurveyPhase::LocateStart;
  const bool sweep=s.phase==SurveyPhase::Sweep && t->pending==Pending::Local;
  if(!sweep && (!(start || s.phase==SurveyPhase::SettleEnd) || t->pending!=Pending::None || !stopped())) {return;}
  if(message->detections.size()!=1) {return;}
  const auto & l=survey_layer(s);const bool left=start?s.entry_left:!s.entry_left;
  const std::string side=left?"left":"right";const auto & d=message->detections.front();const auto & p=d.bbox.center.position;
  if(d.header!=message->header || d.id!=l.shelf_id+"/"+std::to_string(l.layer)+"/"+side ||
    d.results.empty() || d.results.front().hypothesis.class_id!="qr_"+side ||
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z<=0) {return;}
  SurveyRun::PoseSample source;if(!survey_pose(ns,source)) {return;}
  const auto & q=*config_.geometry.camera_rotation;
  const auto v=tf2::Matrix3x3(tf2::Quaternion(q[0],q[1],q[2],q[3]))*tf2::Vector3(p.x,p.y,p.z);
  const double offset=(std::cos(source.yaw)*v.x()-std::sin(source.yaw)*v.y())*t->right_x+
    (std::sin(source.yaw)*v.x()+std::cos(source.yaw)*v.y())*t->right_y;
  const double boundary=survey_camera_position(source)+offset;
  const SurveyRun::PoseSample current{stamp(odom_->header),odom_->pose.pose.position.x,
    odom_->pose.pose.position.y,yaw(*odom_)};
  const double delta=boundary-survey_camera_position(current);
  if(!sweep && std::abs(offset)<=config_.qr_lateral_tolerance) {
    s.counter.set_boundary(left,boundary);
    event("endpoint_aligned",side+" position_m="+std::to_string(boundary)+" residual_m="+std::to_string(offset));
    if(start) {
      s.sweep_origin=boundary;
      if(!s.count_error.empty()) {
        s.counter.reset(survey_config_.confirmations,survey_config_.deadband,s.left_to_right);
        s.counter.set_boundary(left,boundary);s.count_error.clear();s.entry_observed.reset();
        s.barrier=now().nanoseconds();s.last_boxes=0;
      }
      enter_survey(SurveyPhase::ObserveStart);advance_survey();
    } else {
      s.end_aligned=true;enter_survey(SurveyPhase::Finalize);advance_survey();
    }
    return;
  }
  if(t->qr_moves>=2 || t->qr_travel+std::abs(delta)>config_.qr_max_travel) {survey_layer_failed("qr_alignment_limit");return;}
  ++t->qr_moves;t->qr_travel+=std::abs(delta);
  event("endpoint_move",side+" source_ns="+std::to_string(ns)+" delta_m="+std::to_string(delta));
  if(sweep) {
    // Keep the open local leg: no stop/handshake just because the marker appeared.
    enter_survey(SurveyPhase::SettleEnd);s.next=SurveyPhase::SettleEnd;
    t->local_purpose=LocalPurpose::Qr;
    t->move_x=current.x+delta*t->right_x;t->move_y=current.y+delta*t->right_y;t->move_yaw=t->shelf_yaw;
    local_tick();
  } else {
    s.next=s.phase;
    start_local(delta*t->right_x,delta*t->right_y,Phase::Scan,warehouse_wrap(t->shelf_yaw-current.yaw),LocalPurpose::Qr);
  }
}
void TaskManagerNode::survey_local_tick()
{
  auto t=active_;auto & s=*t->survey_run;
  if(t->pending==Pending::Service) {if(t->local_open) {zero_local();}return;}
  if(t->cancel || !t->error.empty()) {if(t->local_open) {stop_local();}else {advance_survey();}return;}
  if(t->pending!=Pending::None && t->pending!=Pending::Local) {return;}
  if(!hardware_ok() || !odom_ || !fresh(stamp(odom_->header),config_.status_age)) {fail("survey feedback stale/faulted");return;}
  if(!s.layer_error.empty() && s.phase!=SurveyPhase::Recovery) {if(t->local_open) {stop_local();}else {advance_survey();}return;}
  if(s.qr_request && !fresh(s.qr_request,config_.cv_frame_age)) {clear_survey_depth();}
  if(s.phase==SurveyPhase::Recovery) {advance_survey();return;}
  if(s.phase==SurveyPhase::LocateStart && !s.initialized) {initialize_survey_frame();}
  if((s.phase==SurveyPhase::LocateStart || s.phase==SurveyPhase::SettleEnd) && elapsed(s.qr_started)>config_.qr_align_timeout) {survey_layer_failed("qr_observation_timeout");return;}
  if(!t->local_open) {if(s.phase==SurveyPhase::Finalize) {advance_survey();}return;}
  const auto & p=odom_->pose.pose.position;const double a=yaw(*odom_);
  if(std::hypot(p.x-t->previous_x,p.y-t->previous_y)>.30 || std::abs(warehouse_wrap(a-t->previous_yaw))>.6) {fail("odometry jumped during survey");return;}
  t->previous_x=p.x;t->previous_y=p.y;t->previous_yaw=a;
  double x=0,y=0,turn=0;
  if(t->local_purpose!=LocalPurpose::SurveySweep) {
    x=t->move_x-p.x;y=t->move_y-p.y;turn=warehouse_wrap(t->move_yaw-a);
    if(std::hypot(x,y)<=config_.position_tolerance && std::abs(turn)<=config_.yaw_tolerance) {stop_local();return;}
  } else {
    const auto & l=survey_layer(s);const double width=l.right_qr->x_m-l.left_qr->x_m;
    const double budget=(width+2*survey_config_.edge_adjust)/s.sweep_speed+2*config_.observation_timeout;
    if(elapsed(s.sweep_started)>budget) {survey_layer_failed("sweep_timeout");return;}
    const SurveyRun::PoseSample current{stamp(odom_->header),p.x,p.y,a};
    const double progress=(survey_camera_position(current)-s.sweep_origin)*(s.left_to_right?1:-1);
    if(progress>width+survey_config_.edge_adjust+config_.position_tolerance) {survey_layer_failed("end_boundary_missing");return;}
    const double drift=(p.x-t->shelf_odom_x)*t->inward_x+(p.y-t->shelf_odom_y)*t->inward_y;
    const double remaining=std::max(0.0,width+survey_config_.edge_adjust-progress);
    const double inward=std::clamp(-drift,-s.sweep_speed,s.sweep_speed);
    if(remaining<=config_.position_tolerance) {
      if(s.qr_request) {zero_local();return;}
      survey_layer_failed("end_boundary_missing");return;
    }
    const double along=std::min(s.sweep_speed,remaining/.05)*(s.left_to_right?1:-1);
    x=along*t->right_x+inward*t->inward_x;y=along*t->right_y+inward*t->inward_y;turn=warehouse_wrap(t->shelf_yaw-a);
  }
  const double n=std::hypot(x,y),speed=(t->local_purpose==LocalPurpose::SurveySweep || s.phase==SurveyPhase::SettleEnd)?s.sweep_speed:config_.speed;
  if(t->local_purpose==LocalPurpose::Qr && n>config_.position_tolerance) {
    const double limited=std::min({speed,std::max(n,.02),n/.05});x*=limited/n;y*=limited/n;
  } else if(n>speed) {x*=speed/n;y*=speed/n;}
  geometry_msgs::msg::TwistStamped cmd;cmd.header.stamp=now();cmd.header.frame_id="base_link";
  cmd.twist.linear.x=std::cos(a)*x+std::sin(a)*y;cmd.twist.linear.y=-std::sin(a)*x+std::cos(a)*y;
  cmd.twist.angular.z=std::clamp(turn,-config_.yaw_speed,config_.yaw_speed);local_pub_->publish(cmd);
}
void TaskManagerNode::survey_local_finished()
{
  auto t=active_;auto & s=*t->survey_run;s.stop_source=stamp(odom_->header);
  if(t->cancel || !t->error.empty()) {advance_survey();return;}
  clear_survey_depth();
  if(s.next==SurveyPhase::ObserveStart && s.edge_adjusted) {
    s.counter.reset(survey_config_.confirmations,survey_config_.deadband,s.left_to_right);
    s.counter.set_boundary(s.entry_left,s.sweep_origin);s.count_error.clear();
    s.barrier=now().nanoseconds();s.last_boxes=s.last_qr=0;
  }
  enter_survey(s.next);vision_timer();advance_survey();
}
void TaskManagerNode::survey_layer_failed(const std::string & reason)
{
  if(!active_ || !active_->is_survey()) {return;}auto t=active_;auto & s=*t->survey_run;
  if(s.layer_error.empty()) {
    s.layer_error=s.view_clipped && (reason=="qr_observation_timeout" || reason=="end_boundary_missing")?"field_of_view":reason;
    event("layer_failed",s.layer_error);
  }
  if(t->pending==Pending::Local) {stop_local();}
  else if(t->pending==Pending::None) {advance_survey();}
}
void TaskManagerNode::finalize_survey_layer()
{
  auto t=active_;auto & s=*t->survey_run;
  if(!s.end_aligned || !stopped()) {return;}
  if(!s.count_error.empty()) {survey_layer_failed(s.count_error);return;}
  if(s.last_boxes<=s.stop_source) {
    if(elapsed(s.phase_started)>config_.observation_timeout) {survey_layer_failed("boundary_observation_timeout");}
    return;
  }
  const auto result=s.counter.finalize();
  if(!result.complete) {
    if(result.reason=="crossing_unconfirmed" && elapsed(s.phase_started)<config_.observation_timeout) {return;}
    survey_layer_failed(result.reason);return;
  }
  const auto & l=survey_layer(s);auto & row=s.results[s.index];row["status"]="complete";row["quantity"]=result.quantity;
  inventory_.commit_scan_layer(l.shelf_id,l.layer,l.drug,result.quantity);
  s.entry_left=!s.left_to_right;s.observing=false;++s.index;
  enter_survey(SurveyPhase::NextLayer);advance_survey();
}
void TaskManagerNode::finish_survey()
{
  auto t=active_;auto & s=*t->survey_run;
  if(t->pending!=Pending::None || t->nav_active || t->blocked) {return;}
  if(t->local_open) {stop_local();return;}
  if(local_timer_) {local_timer_->cancel();local_timer_.reset();}
  cancel_deadline();
  if(t->cv_maybe || t->yolo_maybe || camera_info_sub_ || t->depth_mode != "off") {stop_vision([this]{finish_survey();});return;}
  if(t->business_reserved && !t->resources_parked && ready_) {
    web_call("set_navigation_compute",{{"state","parked"}},[this,t](const Json &) {
      t->resources_parked=true;finish_survey();
    });return;
  }
  if(t->business_reserved) {web_call("set_business",{{"state","finished"},{"allow_motion",!motion_inhibited_}},[this,t](const Json &) {
    t->business_reserved=false;finish_survey();});return;}
  if(s.observing && s.index<s.layer_indices.size() && s.results[s.index].at("status")=="not_visited") {
    const auto & l=survey_layer(s);s.results[s.index]["status"]="incomplete";
    s.results[s.index]["reason"]=t->cancel?"canceled":t->error;
    inventory_.record_incomplete_layer(l.shelf_id,l.layer,l.drug,s.results[s.index].at("reason"));
  }
  s.observing=false;
  const bool success=t->error.empty() && !t->cancel && s.return_status=="succeeded";
  bool complete=success && inventory_.persisted();
  for(const auto & row:s.results) {if(row.at("status")!="complete" && row.at("status")!="skipped") {complete=false;}}
  last_survey_result_={{"map_id",warehouse_.map_id},{"warehouse_revision",warehouse_.revision},{"drug_name",s.drug_name},
    {"completion",complete?"complete":"partial"},{"persisted",inventory_.persisted()},
    {"return_status",s.return_status},{"layers",s.results}};
  auto result=std::make_shared<Survey::Result>();result->message=success?(complete?"Inventory complete; returned to origin":"Inventory partial; returned to origin"):
    !t->error.empty()?t->error:"Inventory canceled";result->summary_json=last_survey_result_.dump();
  if(t->cancel && t->error.empty() && !t->survey->is_canceling()) {
    task_deadline_=create_wall_timer(std::chrono::milliseconds(1),[this,t]{cancel_deadline();if(active_==t) {finish_survey();}});return;
  }
  enter_survey(SurveyPhase::Finish);active_.reset();inference_->business(false);
  if(success) {t->survey->succeed(result);}else if(t->error.empty() && t->survey->is_canceling()) {t->survey->canceled(result);}else {t->survey->abort(result);}
}
void TaskManagerNode::apply_fetch_inventory_delta()
{
  auto t=active_;if(!t || t->inventory_handled) {return;}t->inventory_handled=true;
  try {
    const auto r=inventory_.apply_fetch_inventory_delta(t->target);
    t->inventory_update={{"handled",true},{"quantity",r.at("quantity")},{"persisted",r.at("persisted")},{"error",r.at("error")}};
  }
  catch(const std::exception & e) {t->inventory_update={{"handled",true},{"quantity",nullptr},{"persisted",false},{"error",e.what()}};}
}
}  // namespace k1muse_manager
