#include "k1muse_manager/task_manager_node_impl.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <limits>

namespace k1muse_manager
{
namespace
{
int64_t stamp(const std_msgs::msg::Header & h)
{
  return h.stamp.sec < 0 || h.stamp.nanosec >= 1000000000u ? 0 :
    int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;
}
int64_t json_stamp(const Json & h)
{
  const auto sec=h.at("sec").get<int64_t>(), ns=h.at("nanosec").get<int64_t>();
  return sec<0 || sec>std::numeric_limits<int32_t>::max() || ns<0 || ns>=1000000000 ? 0 : sec*1000000000LL+ns;
}
}
void TaskManagerNode::record_visual_frame(VisualFrame & frame,const std_msgs::msg::Header & header,size_t count)
{
  frame.source=stamp(header);frame.received=now().nanoseconds();frame.detections=count;
  if(frame.source>0 && frame.received>=frame.source) {
    const double age=(frame.received-frame.source)*1e-6;
    ++frame.age_samples;frame.age_sum_ms+=age;frame.age_max_ms=std::max(frame.age_max_ms,age);
  }
}
Json TaskManagerNode::vision_diagnostics(const Active & t) const
{
  const auto current=now().nanoseconds();
  const auto frame_json=[&](const VisualFrame & f) -> Json {
      return {{"source_ns",f.source},{"received_ns",f.received},{"detections",f.detections},{"status",f.status},
        {"age_at_receive_ms",f.source>0?Json((f.received-f.source)*1e-6):Json(nullptr)},
        {"age_now_ms",f.source>0?Json((current-f.source)*1e-6):Json(nullptr)},
        {"age_samples",f.age_samples},{"age_mean_ms",f.age_samples?Json(f.age_sum_ms/f.age_samples):Json(nullptr)},
        {"age_max_ms",f.age_samples?Json(f.age_max_ms):Json(nullptr)}};
    };
  const auto candidate_json=[&](const Candidate & c) -> Json {
      if(!c.valid) {return nullptr;}
      return {{"source_ns",c.stamp},{"id",c.id},{"frame",c.frame},{"class_id",c.class_id},{"q",c.q},{"score",c.score},
        {"age_ms",(current-c.stamp)*1e-6}};
    };
  const auto & s=t.candidate_summary;
  return {{"frame_2d",frame_json(t.frame_2d)},{"frame_3d",frame_json(t.frame_3d)},
    {"eligible",s.eligible},{"rejected",{{"identity",s.identity},{"class",s.wrong_class},{"score",s.low_score},
      {"box",s.invalid_box},{"roi",s.outside_roi},{"boundary",s.boundary}}},
    {"locked_id",t.locked_id},{"locked_reason",s.locked_reason},{"tracking_lost",t.tracking_lost},
    {"candidate",candidate_json(t.candidate)},{"last_valid_candidate",candidate_json(t.last_candidate)},
    {"point_pair_status",t.point_pair_status},{"last_point",t.last_point}};
}
void TaskManagerNode::update_budgets()
{
  auto t=active_; if(!t || !t->is_fetch) {return;}
  const auto now=Clock::now(); const double dt=std::chrono::duration<double>(now-t->budget_tick).count();
  t->budget_tick=now;
  if(t->pause_requested || t->blocked) {return;}
  if(t->scan_started && (!t->left_confirmed || t->locating_right) && (t->phase==Phase::Scan || (t->phase==Phase::Settle && t->next_phase==Phase::Scan))) {t->scan_elapsed+=dt;}
  if(t->search_started && t->phase==Phase::Detect) {t->search_elapsed+=dt;}
  if(t->framing_started && (t->phase==Phase::Align ||
    ((t->phase==Phase::Settle || t->pending==Pending::Lift || t->pending==Pending::LiftStatus) && t->next_phase==Phase::Align))) {
    t->framing_elapsed+=dt;
  }
}
bool TaskManagerNode::stage_boundary(const std::string & name)
{
  auto t=active_;
  if(t->boundary_skip==name) {t->boundary_skip.clear();return false;}
  if(!breakpoints_.count(name)) {return false;}
  update_budgets(); t->pause_requested=true;t->paused_boundary=name;
  event("breakpoint",name);return true;
}
void TaskManagerNode::scan()
{
  if(active_->depth_mode != "both") {depth_mode("both", [this] {scan();});return;}

  auto t=active_;
  if(config_.no_qr) {
    if(t->search_ready) {enter(Phase::Detect,"yolo_search");advance_fetch();return;}
    if(!tf_buffer_) {
      tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock(),tf2::durationFromSec(2.0));
      tf_listener_=std::make_unique<tf2_ros::TransformListener>(*tf_buffer_,this,false);
    }
    if(!camera_info_sub_) {
      update_budgets();
      const double remaining=config_.observation_timeout-t->scan_elapsed;
      if(remaining<=0) {fail("no-QR camera/TF preparation timeout");return;}
      t->scan_started=true;t->budget_tick=Clock::now();t->barrier=now().nanoseconds();
      deadline(remaining,"no-QR camera/TF preparation timeout");
      camera_info_sub_=create_subscription<sensor_msgs::msg::CameraInfo>(
        "/k1muse/front_camera/color/camera_info",rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){on_camera_info(m);});
    }
    return;
  }
  if((!t->left_confirmed || t->locating_right) && !qr_point_sub_) {
    if(!qr_request_pub_) {qr_request_pub_=create_publisher<D2>("/k1muse/perception/qr_depth/requests",rclcpp::QoS(1));}
    qr_point_sub_=create_subscription<D3>("/k1muse/perception/qr_depth/points",rclcpp::QoS(1),
      [this](D3::ConstSharedPtr m){on_qr_point(m);});
  }
  auto wait_left = [this,t] {
      update_budgets();
      const double remaining=(t->search_ready?config_.qr_align_timeout:config_.observation_timeout)-t->scan_elapsed;
      if(remaining<=0) {fail("QR endpoint acquisition/alignment timeout");return;}
      deadline(remaining,"QR endpoint acquisition/alignment timeout");
    };
  if(t->cv_maybe) {
    if(t->left_confirmed && !t->locating_right) {if(t->phase==Phase::Scan) {enter(Phase::Detect,"yolo_search");}advance_fetch();}
    else {wait_left();}
    return;
  }
  if(!inference_->idle()) {
    if(!t->waiting_permit) {t->waiting_permit=true;deadline(config_.observation_timeout,"waiting for QR startup");}
    return;
  }
  if(!t->left_confirmed && !tf_buffer_) {
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock(),tf2::durationFromSec(2.0));
    tf_listener_=std::make_unique<tf2_ros::TransformListener>(*tf_buffer_,this,false);
  }
  if(!cv_sub_) {cv_sub_=create_subscription<std_msgs::msg::String>("/k1muse/perception/cv/observations",rclcpp::QoS(1).reliable(),[this](std_msgs::msg::String::ConstSharedPtr m){on_cv(m);});}
  t->waiting_permit=false;t->pending=Pending::Service;const auto op=++t->operation;t->cv_maybe=true;
  deadline(config_.service_timeout,"CV start timeout");
  inference_->scan(true,t->id+":cv",[this,t,op,wait_left](bool ok) {
    if(!matches(t,op)) {return;}cancel_deadline();
    if(!ok) {
      if(!inference_->owns_scan(t->id+":cv")) {t->cv_maybe=false;t->pending=Pending::None;fail("CV unavailable before acquisition");}
      else {t->blocked=true;t->error="CV start not acknowledged";event("blocked");}
      return;
    }
    t->pending=Pending::None;
    t->cv_frame_stamp=0;t->left_marker={};t->right_marker={};
    if(t->cancel || !t->error.empty() || t->pause_requested) {advance_fetch();return;}
    t->barrier=now().nanoseconds();t->scan_started=true;t->budget_tick=Clock::now();
    if(t->left_confirmed && !t->locating_right) {if(t->phase==Phase::Scan) {enter(Phase::Detect,"yolo_search");}advance_fetch();}
    else {wait_left();event("observing");}
  });
}
bool TaskManagerNode::initialize_search_frame()
{
  auto t=active_;
  if(!stopped() || !hardware_ok() || !config_.geometry.camera_xy) {return false;}
  // Both modes use configured shelf orientation; QR supplies identity, not pose.
  try {
    // Query the populated cache without the ROS timeout/polling overload.
    const auto transform=tf_buffer_->lookupTransform("odom","map",tf2_ros::fromRclcpp(rclcpp::Time(odom_->header.stamp)));
    if(!fresh(stamp(transform.header),config_.status_age)) {return false;}
    if((config_.approach.enabled || t->target.fetch->grasp=="top") && !t->camera_from_arm) {
      t->camera_from_arm=tf_buffer_->lookupTransform(t->image_frame,"arm_base",tf2::TimePointZero).transform;
    }
    const auto & rq=transform.transform.rotation;
    const double inward=t->target.observation_pose->yaw_rad+2*std::atan2(rq.z,rq.w);
    t->inward_x=std::cos(inward);t->inward_y=std::sin(inward);
    t->right_x=t->inward_y;t->right_y=-t->inward_x;
    const auto & xy=*config_.geometry.camera_xy;const auto & goal=*t->target.observation_pose;
    const double gx=goal.x_m+std::cos(goal.yaw_rad)*xy[0]-std::sin(goal.yaw_rad)*xy[1];
    const double gy=goal.y_m+std::sin(goal.yaw_rad)*xy[0]+std::cos(goal.yaw_rad)*xy[1];
    const double a=2*std::atan2(rq.z,rq.w);
    const double expected_x=std::cos(a)*gx-std::sin(a)*gy+transform.transform.translation.x;
    const double expected_y=std::sin(a)*gx+std::cos(a)*gy+transform.transform.translation.y;
    const double yaw=2*std::atan2(odom_->pose.pose.orientation.z,odom_->pose.pose.orientation.w);
    const double actual_x=odom_->pose.pose.position.x+std::cos(yaw)*xy[0]-std::sin(yaw)*xy[1];
    const double actual_y=odom_->pose.pose.position.y+std::sin(yaw)*xy[0]+std::cos(yaw)*xy[1];
    const double initial_right=(actual_x-expected_x)*t->right_x+(actual_y-expected_y)*t->right_y;
    if(config_.approach.enabled || t->target.fetch->grasp=="top") {
      t->initial_shelf_clearance=config_.observation_distance-
        ((actual_x-expected_x)*t->inward_x+(actual_y-expected_y)*t->inward_y);
      t->approach_limit=std::min(config_.approach.max_travel,
        std::max(0.0,t->initial_shelf_clearance-config_.approach.shelf_clearance));
    }
    const auto & q=*config_.geometry.camera_rotation;
    const tf2::Matrix3x3 rotation(tf2::Quaternion(q[0],q[1],q[2],q[3]));
    const auto camera_right=rotation*tf2::Vector3(1,0,0);
    const double rx=std::cos(yaw)*camera_right.x()-std::sin(yaw)*camera_right.y();
    const double ry=std::sin(yaw)*camera_right.x()+std::cos(yaw)*camera_right.y();
    const double projection=rx*t->right_x+ry*t->right_y;
    if(std::abs(projection)<.5) {fail("camera horizontal axis incompatible with shelf framing");return false;}
    t->camera_right_sign=projection>0?1:-1;t->shelf_yaw=yaw;
    // Shift only the lateral origin: inward travel still starts at actual arrival.
    t->shelf_odom_x=odom_->pose.pose.position.x-initial_right*t->right_x;
    t->shelf_odom_y=odom_->pose.pose.position.y-initial_right*t->right_y;
    t->search_offset=initial_right;t->search_start_offset=std::min(0.0,initial_right);t->right_limit_m=*t->target.fetch->search_max_m;t->search_ready=true;
    t->qr_context_error.clear();
    event("shelf_reference","initial_camera_right_m="+std::to_string(initial_right)+
      " search_max_m="+std::to_string(*t->target.fetch->search_max_m));
  } catch(const tf2::TransformException & error) {t->qr_context_error=error.what();return false;}
  tf_listener_.reset();tf_buffer_.reset();
  return true;
}
void TaskManagerNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
{
  if(active_ && active_->is_survey()) {on_survey_camera(message);return;}
  auto t=active_;
  if(!t || !config_.no_qr || !camera_info_sub_ || t->phase!=Phase::Scan || t->search_ready ||
    t->cancel || t->pause_requested || !t->error.empty() || t->pending!=Pending::None ||
    stamp(message->header)<=t->barrier || !fresh(stamp(message->header),config_.observation_age) ||
    !message->width || !message->height || message->header.frame_id.empty()) {return;}
  t->width=message->width;t->height=message->height;t->image_frame=message->header.frame_id;
  if(!initialize_search_frame()) {return;}
  camera_info_sub_.reset();cancel_deadline();
  event("qr_skipped","using configured shelf/search bounds; identity not scanned");
  enter(Phase::Detect,"yolo_search");advance_fetch();
}
void TaskManagerNode::on_cv(std_msgs::msg::String::ConstSharedPtr message)
{
  if(active_ && active_->is_survey()) {on_survey_cv(message);return;}
  auto t=active_;
  if(!t || !t->is_fetch || !t->cv_maybe || t->cancel || t->pause_requested || !t->error.empty()) {return;}
  try {
    const auto v=Json::parse(message->data);const auto & header=v.at("header");
    const int64_t ns=json_stamp(header.at("stamp"));
    if(ns<=t->cv_frame_stamp || !fresh(ns,config_.cv_frame_age) || !v.value("error",std::string()).empty()) {return;}
    const int width=v.at("image_width"),height=v.at("image_height");const std::string frame=header.at("frame_id");
    if(width<=0 || height<=0 || frame.empty()) {return;}
    if(t->left_confirmed && (width!=t->width || height!=t->height || frame!=t->image_frame)) {
      fail("camera profile/frame changed during fetch");return;
    }
    t->width=width;t->height=height;t->image_frame=frame;t->cv_frame_stamp=ns;
    Marker left,right;unsigned left_count=0,right_count=0;
    vision_msgs::msg::Detection2D endpoint_request;
    for(const auto & code:v.at("markers")) {
      const auto & id=code.at("id");
      if(!id.is_number_integer()) {continue;}
      std::string side;
      if(t->target.left_marker_id && id==*t->target.left_marker_id) {side="left";}
      else if(t->target.right_marker_id && id==*t->target.right_marker_id) {side="right";}
      else {continue;}
      const auto & points=code.at("corners");if(points.size()!=4) {continue;}
      Marker marker;marker.stamp=ns;marker.min_q=1;marker.max_q=0;
      bool valid=true;double min_v=height,max_v=0,center_v=0;
      for(const auto & point:points) {
        if(!point.is_array() || point.size()!=2) {valid=false;break;}
        const double x=point[0],y=point[1];
        if(!std::isfinite(x) || !std::isfinite(y) || x<0 || x>=width || y<0 || y>=height) {valid=false;break;}
        marker.min_q=std::min(marker.min_q,x/width);marker.max_q=std::max(marker.max_q,x/width);marker.q+=x/width/4;
        min_v=std::min(min_v,y);max_v=std::max(max_v,y);center_v+=y/4;
      }
      if(!valid || marker.max_q<=marker.min_q || max_v<=min_v) {continue;}
      if(side=="left") {left=marker;++left_count;} else {right=marker;++right_count;}
      if((side=="right")==t->locating_right) {
        endpoint_request.id=t->target.shelf_id+"/"+std::to_string(t->target.layer)+"/"+side;
        endpoint_request.bbox.center.position.x=marker.q*width;endpoint_request.bbox.center.position.y=center_v;
        endpoint_request.bbox.size_x=(marker.max_q-marker.min_q)*width;endpoint_request.bbox.size_y=max_v-min_v;
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id="qr_"+side;hypothesis.hypothesis.score=1;
        endpoint_request.results={hypothesis};
      }
    }
    if(left_count==1) {t->left_marker=left;}
    if(right_count==1) {t->right_marker=right;}
    if(t->phase==Phase::Scan && (!t->left_confirmed || t->locating_right) &&
      (t->locating_right?right_count:left_count)==1 &&
      ns>t->barrier && t->pending==Pending::None && stopped() && hardware_ok()) {
      if(!t->search_ready) {
        if(!initialize_search_frame()) {return;}
        t->scan_elapsed=0;t->budget_tick=Clock::now();
        deadline(config_.qr_align_timeout,"left QR depth alignment timeout");
      }
      endpoint_request.header.frame_id=frame;endpoint_request.header.stamp.sec=ns/1000000000LL;
      endpoint_request.header.stamp.nanosec=ns%1000000000LL;
      D2 request;request.header=endpoint_request.header;request.detections.push_back(std::move(endpoint_request));
      if(qr_request_pub_) {qr_request_pub_->publish(request);}
    }
    if(t->left_confirmed && !t->right_reached && !t->locating_right && right_count==1 &&
      t->phase==Phase::Detect && (t->pending==Pending::Local || t->pending==Pending::None)) {
      const double error=t->camera_right_sign*(right.q-config_.framing.target_u);
      if(error<=config_.framing.tolerance_u) {
        // Seeing the marker starts measured positioning; it is not an end condition.
        t->locating_right=true;t->qr_moves=0;t->qr_travel=0;t->qr_point_stamp=0;
        t->scan_elapsed=0;t->budget_tick=Clock::now();
        t->next_phase=Phase::Scan;
        if(t->local_open) {stop_local();} else {settle(Phase::Scan);}
      }
    }
  } catch(const Json::exception & error) {RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Invalid marker observation: %s",error.what());}
}
void TaskManagerNode::on_qr_point(D3::ConstSharedPtr message)
{
  if(active_ && active_->is_survey()) {on_survey_qr(message);return;}
  auto t=active_;
  if(!t || !t->is_fetch || config_.no_qr || t->phase!=Phase::Scan || (t->left_confirmed && !t->locating_right) ||
    !t->search_ready || t->pending!=Pending::None || t->cancel || t->pause_requested ||
    !t->error.empty() || !stopped() || !hardware_ok()) {return;}
  const auto ns=stamp(message->header);
  if(ns<=t->barrier || ns<=t->qr_point_stamp || !fresh(ns,config_.cv_frame_age) ||
    message->header.frame_id!=t->image_frame || message->detections.size()!=1) {return;}
  const auto & detection=message->detections.front();
  if(detection.id!=t->target.shelf_id+"/"+std::to_string(t->target.layer)+(t->locating_right?"/right":"/left") ||
    detection.header!=message->header) {return;}
  const auto & p=detection.bbox.center.position;
  if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z<=0) {return;}
  t->qr_point_stamp=ns;
  const auto & q=*config_.geometry.camera_rotation;
  const tf2::Matrix3x3 rotation(tf2::Quaternion(q[0],q[1],q[2],q[3]));
  const double lateral=(rotation*tf2::Vector3(p.x,p.y,p.z)).y();
  event("qr_depth","source_ns="+std::to_string(ns)+" lateral_m="+std::to_string(lateral));
  if(std::abs(lateral)<=config_.qr_lateral_tolerance) {
    if(t->locating_right) {
      t->locating_right=false;t->right_reached=true;t->boundary_wait=true;
      const auto & arrived=odom_->pose.pose.position;
      t->right_limit_m=(arrived.x-t->shelf_odom_x)*t->right_x+(arrived.y-t->shelf_odom_y)*t->right_y;
      if(qr_request_pub_) {qr_request_pub_->publish(D2{});}qr_point_sub_.reset();
      cancel_deadline();event("right_boundary_confirmed","depth aligned");
      t->barrier=now().nanoseconds();t->candidate={};t->locked_id.clear();
      enter(Phase::Detect,"right_end_observation");advance_fetch();return;
    }
    t->left_confirmed=true;
    // A measured left boundary replaces navigation's lateral error estimate.
    const auto & arrived=odom_->pose.pose.position;
    const double offset=(arrived.x-t->shelf_odom_x)*t->right_x+(arrived.y-t->shelf_odom_y)*t->right_y;
    t->shelf_odom_x+=offset*t->right_x;t->shelf_odom_y+=offset*t->right_y;
    t->inward_offset=(arrived.x-t->shelf_odom_x)*t->inward_x+(arrived.y-t->shelf_odom_y)*t->inward_y;
    t->search_offset=0;t->search_start_offset=0;
    if(qr_request_pub_) {qr_request_pub_->publish(D2{});}qr_point_sub_.reset();
    cancel_deadline();event("left_boundary_confirmed","depth aligned");
    enter(Phase::Detect,"yolo_search");advance_fetch();return;
  }
  if(t->qr_moves>=2 || t->qr_travel+std::abs(lateral)>config_.qr_max_travel) {
    fail("left QR lateral correction/travel limit reached");return;
  }
  ++t->qr_moves;t->qr_travel+=std::abs(lateral);
  const auto & orientation=odom_->pose.pose.orientation;
  const double yaw=2*std::atan2(orientation.z,orientation.w);
  start_local(-std::sin(yaw)*lateral,std::cos(yaw)*lateral,Phase::Scan,0,LocalPurpose::Qr);
}
void TaskManagerNode::vision_timer()
{
  if(!local_timer_) {local_timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{local_tick();});}
}
void TaskManagerNode::detect()
{
  if(active_->depth_mode!="both") {depth_mode("both",[this] {detect();});return;}
  auto t=active_;if(!t->search_ready || (!config_.no_qr && !t->cv_maybe)) {scan();return;}
  if(t->yolo_maybe) {if(t->phase==Phase::Detect || t->phase==Phase::Align) {vision_timer();}return;}
  if(!inference_->idle()) {
    if(!t->waiting_permit) {t->waiting_permit=true;deadline(config_.observation_timeout,"waiting for YOLO permit");}
    return;
  }
  if(stage_boundary("before:yolo_search")) {advance_fetch();return;}
  if(!detections_sub_) {detections_sub_=create_subscription<D2>("/k1muse/perception/detections",rclcpp::QoS(1).reliable(),[this](D2::ConstSharedPtr m){on_detections(m);});}
  if(!targets_sub_) {targets_sub_=create_subscription<D3>("/k1muse/perception/arm_targets",rclcpp::QoS(1).reliable(),[this](D3::ConstSharedPtr m){on_targets(m);});}
  t->waiting_permit=false;
  toggle_yolo(true,[this,t] {
    if(t->cancel || !t->error.empty() || t->pause_requested) {advance_fetch();return;}
    t->barrier=now().nanoseconds();t->candidate={};t->candidates.clear();t->targets.reset();
    t->search_started=true;t->budget_tick=Clock::now();cancel_deadline();
    if(t->phase==Phase::Detect || t->phase==Phase::Align) {vision_timer();}event("observing");
    if(t->phase==Phase::Grasp || t->phase==Phase::Recover) {advance_fetch();}
  });
}
void TaskManagerNode::search_next()
{
  // Continuous search is driven only by the existing 20 Hz local timer.
  auto t=active_;if(!t || t->pending!=Pending::None) {return;}
  open_local(LocalPurpose::Search);
}
void TaskManagerNode::on_detections(D2::ConstSharedPtr message)
{
  if(active_ && active_->is_survey()) {on_survey_boxes(message);return;}
  auto t=active_;if(!t || !t->is_fetch || (t->phase!=Phase::Detect && t->phase!=Phase::Align) ||
    !t->search_ready || !t->yolo_maybe || t->cancel || t->pause_requested || !t->error.empty()) {return;}
  const auto ns=stamp(message->header);
  record_visual_frame(t->frame_2d,message->header,message->detections.size());
  auto & summary=t->candidate_summary;summary={};summary.locked_reason="not_evaluated";
  auto & status=t->frame_2d.status;
  if(ns<=t->barrier) {status="before_barrier";return;}
  if(ns<=t->candidate.stamp) {status="duplicate_or_late";return;}
  if(!fresh(ns,config_.observation_age) || !fresh(ns,config_.framing.max_frame_age)) {status="stale_or_future";return;}
  if(message->header.frame_id!=t->image_frame) {status="wrong_frame";return;}
  summary.locked_reason=t->locked_id.empty()?"not_locked":"not_in_frame";
  const auto & roi=*t->target.fetch->roi;Candidate best;best.stamp=ns;best.frame=message->header.frame_id;
  Candidate locked=best;
  const bool replacing=t->phase==Phase::Align && t->waiting_point && t->tracking_lost &&
    t->pending==Pending::None && !t->local_open && stopped() &&
    Clock::now()-t->lost_since>std::chrono::seconds(2);
  double rank=std::numeric_limits<double>::infinity();
  for(const auto & detection:message->detections) {
    const auto reject=[&](size_t & count,const char * reason) {
        ++count;if(!t->locked_id.empty() && detection.id==t->locked_id) {summary.locked_reason=reason;}
      };
    if(detection.id.empty() || detection.results.empty() || detection.header!=message->header) {reject(summary.identity,"identity");continue;}
    const auto & h=detection.results.front().hypothesis;const auto & b=detection.bbox;
    if(h.class_id!=t->expected_class) {reject(summary.wrong_class,"class");continue;}
    if(!std::isfinite(h.score) || h.score<config_.confidence) {reject(summary.low_score,"score");continue;}
    if(!std::isfinite(b.center.position.x) || !std::isfinite(b.center.position.y) ||
      !std::isfinite(b.size_x) || !std::isfinite(b.size_y) || b.size_x<=0 || b.size_y<=0 ||
      b.center.theta!=0) {reject(summary.invalid_box,"box");continue;}
    if(b.center.position.x<roi[0]*t->width ||
      b.center.position.x>roi[2]*t->width || b.center.position.y<roi[1]*t->height ||
      b.center.position.y>roi[3]*t->height) {reject(summary.outside_roi,"roi");continue;}
    const double xmin=(b.center.position.x-b.size_x/2)/t->width, xmax=(b.center.position.x+b.size_x/2)/t->width;
    const auto close_marker=[&](const Marker & marker) {
        return marker.stamp>0 && std::abs(double(ns-marker.stamp))*1e-9<=config_.framing.max_frame_age;
      };
    if(close_marker(t->left_marker) && (t->camera_right_sign>0 ? xmin<t->left_marker.max_q : xmax>t->left_marker.min_q)) {reject(summary.boundary,"boundary");continue;}
    if(close_marker(t->right_marker) && (t->camera_right_sign>0 ? xmax>t->right_marker.min_q : xmin<t->right_marker.max_q)) {reject(summary.boundary,"boundary");continue;}
    if(replacing && detection.id!=t->locked_id) {
      if(t->candidates.size()==16) {t->candidates.pop_front();}
      t->candidates.push_back({ns,detection.id,message->header.frame_id,h.class_id,
        b.center.position.x/t->width,h.score,true});
    }
    ++summary.eligible;if(detection.id==t->locked_id) {summary.locked_reason="eligible";}
    const double q=b.center.position.x/t->width, order=q*t->camera_right_sign;
    if(detection.id==t->locked_id) {locked={ns,detection.id,message->header.frame_id,h.class_id,q,h.score,true};}
    if(order<rank || (order==rank && (h.score>best.score || (h.score==best.score && detection.id<best.id)))) {
      rank=order;best={ns,detection.id,message->header.frame_id,h.class_id,q,h.score,true};
    }
  }
  if(t->locked_id.empty()) {
    // Do not reserve a box while traversing the lead-in before the search window.
    if(best.valid && (t->phase==Phase::Align || t->search_offset>=*t->target.fetch->search_min_m-config_.position_tolerance)) {
      t->locked_id=best.id;
      event("target_locked","detection="+best.id+" source_ns="+std::to_string(ns));
    } else {best=locked;}
  } else if(locked.valid) {
    best=locked;
    if(t->tracking_lost) {event("target_reacquired","detection="+t->locked_id+" source_ns="+std::to_string(ns));}
    t->tracking_lost=false;
  } else {
    if(!t->tracking_lost) {
      t->tracking_lost=true;t->lost_since=Clock::now();++t->reacquisitions;
      t->candidates.clear();t->targets.reset();t->target_stamp=0;
      event("target_lost","detection="+t->locked_id+" source_ns="+std::to_string(ns));
    }
    // Execute an already measured approach unchanged; choose a replacement only
    // after stopping and receiving a new frame beyond the existing stop barrier.
    // Align replacements require a matched, feasible post-stop depth below.
    const bool can_switch=t->phase==Phase::Detect && stopped() && (t->pending==Pending::None ||
      (t->pending==Pending::Local && t->local_purpose!=LocalPurpose::Approach));
    if(best.valid && can_switch && Clock::now()-t->lost_since>std::chrono::seconds(2)) {
      event("target_switched","from="+t->locked_id+" to="+best.id+" source_ns="+std::to_string(ns));
      t->locked_id=best.id;t->tracking_lost=false;
      t->candidates.clear();t->targets.reset();t->target_stamp=0;t->camera_distance.reset();
    } else {best=locked;}
  }
  t->candidate=best;t->had_frame=true;
  status=best.valid?"selected":t->tracking_lost?"reacquiring":summary.eligible?"search_lead_in":"no_eligible_candidate";
  if(best.valid) {t->last_candidate=best;summary.locked_reason="eligible";}
  if(t->waiting_point && t->phase==Phase::Align && best.valid && stopped()) {
    while(!t->candidates.empty() && !fresh(t->candidates.front().stamp,config_.observation_age)) {t->candidates.pop_front();}
    if(t->candidates.size()==16) {t->candidates.pop_front();}
    t->candidates.push_back(std::move(best));try_target_pair();
  } else if(replacing) {try_target_pair();}
}
void TaskManagerNode::on_targets(D3::ConstSharedPtr message)
{
  auto t=active_;if(!t || !t->is_fetch || t->phase!=Phase::Align) {return;}
  record_visual_frame(t->frame_3d,message->header,message->detections.size());
  auto & status=t->frame_3d.status;status="not_ready";
  if(!t->waiting_point ||
    t->pending!=Pending::None || t->cancel || t->pause_requested || !t->error.empty()) {return;}
  if(message->header.frame_id!="arm_base") {status="wrong_frame";return;}
  if(stamp(message->header)<=t->barrier) {status="before_barrier";return;}
  if(!fresh(stamp(message->header),config_.observation_age)) {status="stale_or_future";return;}
  status=message->detections.empty()?"empty_depth":"accepted";
  if(!t->targets || stamp(message->header)>=stamp(t->targets->header)) {t->targets=std::move(message);}
  try_target_pair();
}
void TaskManagerNode::try_target_pair()
{
  auto t=active_;if(!t || t->phase!=Phase::Align || !t->waiting_point) {return;}
  const bool replacing=t->tracking_lost && !t->local_open &&
    Clock::now()-t->lost_since>std::chrono::seconds(2);
  t->point_pair_status=!t->targets?"no_3d":t->tracking_lost?"target_missing":
    !t->candidate.valid?"no_2d":"not_ready";
  if(!t->targets || t->pending!=Pending::None || t->pause_requested || t->cancel || !t->error.empty() ||
    (!replacing && (t->tracking_lost || !t->candidate.valid ||
      !fresh(t->candidate.stamp,config_.framing.max_frame_age))) || !stopped() || !hardware_ok()) {return;}
  const auto ns=stamp(t->targets->header);if(ns<=t->barrier || !fresh(ns,config_.observation_age)) {
    t->point_pair_status="expired_3d";return;
  }
  bool matched_stamp=false,blocked_replacement=false;Candidate chosen;Point3 surface{};
  double rank=std::numeric_limits<double>::infinity();
  for(const auto & candidate:t->candidates) {
    if((!replacing && candidate.id!=t->locked_id) || candidate.stamp!=ns || candidate.frame!=t->image_frame ||
      !fresh(candidate.stamp,config_.observation_age) || !fresh(candidate.stamp,config_.framing.max_frame_age)) {continue;}
    if(replacing && std::none_of(t->candidates.begin(),t->candidates.end(),[&](const Candidate & latest) {
        return latest.id==candidate.id && latest.stamp==t->candidate.stamp;
      })) {continue;}
    matched_stamp=true;
    for(const auto & detection:t->targets->detections) {
      if(detection.id!=candidate.id || detection.header!=t->targets->header || detection.results.empty() ||
        detection.results.front().hypothesis.class_id!=candidate.class_id) {continue;}
      const auto & p=detection.results.front().pose.pose.position;const Point3 point{p.x,p.y,p.z};
      if(!finite_point(point)) {continue;}
      if(replacing && (config_.approach.enabled || t->target.fetch->grasp=="top")) {
        DepthAlignment alignment;std::string error;
        if(!depth_alignment(point,alignment,error)) {blocked_replacement=true;continue;}
      }
      const double order=candidate.q*t->camera_right_sign;
      if(!chosen.valid || order<rank || (order==rank &&
        (candidate.score>chosen.score || (candidate.score==chosen.score && candidate.id<chosen.id)))) {
        chosen=candidate;surface=point;rank=order;
      }
    }
  }
  if(chosen.valid) {
    if(replacing && chosen.id!=t->locked_id) {
      event("target_switched","from="+t->locked_id+" to="+chosen.id+" source_ns="+std::to_string(ns));
      const auto latest=std::find_if(t->candidates.begin(),t->candidates.end(),[&](const Candidate & visible) {
          return visible.id==chosen.id && visible.stamp==t->candidate.stamp;
        });  // Guaranteed by the current-frame visibility check above.
      t->locked_id=chosen.id;t->tracking_lost=false;t->candidate=*latest;t->last_candidate=t->candidate;
      t->candidate_summary.locked_reason="eligible";t->frame_2d.status="selected";
      t->candidates.clear();t->camera_distance.reset();
    }
    t->surface=surface;t->target_stamp=ns;t->point_pair_status="matched";
    t->last_point={{"source_ns",ns},{"id",chosen.id},{"arm_xyz",t->surface},
      {"age_at_select_ms",(now().nanoseconds()-ns)*1e-6}};
    event("target_selected","source_ns="+std::to_string(ns)+" detection="+chosen.id+" q="+std::to_string(chosen.q)+
      " arm_xyz="+std::to_string(surface[0])+","+std::to_string(surface[1])+","+std::to_string(surface[2]));
    t->waiting_point=false;align_or_plan();return;
  }
  t->point_pair_status=blocked_replacement?"replacement_outside_alignment_range":
    t->targets->detections.empty()?"empty_depth":matched_stamp?"no_matching_id_class":"no_matching_2d_stamp";
}
bool TaskManagerNode::depth_alignment(
  const Point3 & surface, DepthAlignment & result, std::string & error) const
{
  const auto & t=*active_;const auto & approach=config_.approach;Point3 tcp;
  if(!t.camera_from_arm || !surface_to_tcp(surface,profile(),tcp)) {
    error="invalid target/camera transform for depth alignment";return false;
  }
  const auto & transform=*t.camera_from_arm;const auto & q=transform.rotation;const auto & p=transform.translation;
  const auto optical=tf2::Matrix3x3(tf2::Quaternion(q.x,q.y,q.z,q.w))*
    tf2::Vector3(surface[0],surface[1],surface[2])+tf2::Vector3(p.x,p.y,p.z);
  result.distance=optical.z();result.lateral=optical.x();
  if(!std::isfinite(result.distance) || !std::isfinite(result.lateral) || result.distance<=0) {
    error="invalid camera forward depth";return false;
  }
  if(t.target.fetch->grasp=="top") {
    const auto & preferred=profile().preferred_tcp;
    const auto & cr=*config_.geometry.camera_rotation;
    const tf2::Matrix3x3 camera_to_base(tf2::Quaternion(cr[0],cr[1],cr[2],cr[3]));
    auto arm_error=tf2::Vector3(tcp[0]-preferred[0],tcp[1]-preferred[1],0);
    if(std::abs(arm_error.x())<=config_.position_tolerance) {arm_error.setX(0);}
    if(std::abs(arm_error.y())<=config_.position_tolerance) {arm_error.setY(0);}
    const auto base_error=camera_to_base*tf2::Matrix3x3(tf2::Quaternion(q.x,q.y,q.z,q.w))*arm_error;
    const double yaw=2*std::atan2(odom_->pose.pose.orientation.z,odom_->pose.pose.orientation.w);
    double dx=std::cos(yaw)*base_error.x()-std::sin(yaw)*base_error.y();
    double dy=std::sin(yaw)*base_error.x()+std::cos(yaw)*base_error.y();
    const auto & pos=odom_->pose.pose.position;
    const double inward=(pos.x-t.shelf_odom_x+dx)*t.inward_x+(pos.y-t.shelf_odom_y+dy)*t.inward_y;
    const double right=(pos.x-t.shelf_odom_x+dx)*t.right_x+(pos.y-t.shelf_odom_y+dy)*t.right_y;
    const double upper=t.right_reached?t.right_limit_m:*t.target.fetch->search_max_m;
    if(inward < -config_.position_tolerance || inward>t.approach_limit+config_.position_tolerance ||
      right < -*t.target.fetch->search_max_m-config_.position_tolerance || right>upper+config_.position_tolerance) {
      error="top TCP alignment outside existing shelf corridor";return false;
    }
    const double length=std::hypot(dx,dy),cap=t.approach_steps==0?.50:approach.max_step;
    if(length>cap) {dx*=cap/length;dy*=cap/length;}
    result.dx=dx;result.dy=dy;return true;
  }
  const double allowance=profile().align_lift?kGraspObservationToleranceM:0;
  const bool beyond_reach=tcp[0]>profile().maximum[0]+allowance;
  const bool depth_outside=result.distance<approach.min_distance-1e-6 || result.distance>approach.max_distance+1e-6;
  const bool lateral_outside=std::abs(result.lateral)>approach.lateral_tolerance+1e-6;
  if(!depth_outside && !lateral_outside && !beyond_reach) {return true;}
  const double minimum_step=std::max(.01,2*config_.position_tolerance);
  double forward=depth_outside?result.distance-approach.target_distance:0;
  // Move into the reference window, rather than requesting a sub-tolerance
  // correction to exactly 25cm when X is still beyond arm reach.
  if(beyond_reach && forward<tcp[0]-profile().maximum[0]) {
    forward=tcp[0]-profile().maximum[0]+minimum_step;
  }
  const auto & rotation=*config_.geometry.camera_rotation;
  const tf2::Matrix3x3 camera_rotation(tf2::Quaternion(rotation[0],rotation[1],rotation[2],rotation[3]));
  const auto base_error=camera_rotation*tf2::Vector3(lateral_outside?result.lateral:0,0,forward);
  const double yaw=2*std::atan2(odom_->pose.pose.orientation.z,odom_->pose.pose.orientation.w);
  double dx=std::cos(yaw)*base_error.x()-std::sin(yaw)*base_error.y();
  double dy=std::sin(yaw)*base_error.x()+std::cos(yaw)*base_error.y();
  double requested=std::hypot(dx,dy);
  if(requested>0 && requested<minimum_step) {dx*=minimum_step/requested;dy*=minimum_step/requested;}
  const auto & position=odom_->pose.pose.position;
  const double traveled=(position.x-t.shelf_odom_x)*t.inward_x+(position.y-t.shelf_odom_y)*t.inward_y;
  const double offset=(position.x-t.shelf_odom_x)*t.right_x+(position.y-t.shelf_odom_y)*t.right_y;
  const double upper=t.right_reached?t.right_limit_m:*t.target.fetch->search_max_m;
  const double requested_inward=dx*t.inward_x+dy*t.inward_y;
  const double requested_right=dx*t.right_x+dy*t.right_y;
  const double inward=std::clamp(requested_inward,-std::max(0.0,traveled),std::max(0.0,t.approach_limit-traveled));
  const double right=std::clamp(requested_right,
    std::min(0.0,-*t.target.fetch->search_max_m-offset),std::max(0.0,upper-offset));
  dx=inward*t.inward_x+right*t.right_x;dy=inward*t.inward_y+right*t.right_y;
  const auto applied=camera_rotation.transpose()*tf2::Vector3(
    std::cos(yaw)*dx+std::sin(yaw)*dy,-std::sin(yaw)*dx+std::cos(yaw)*dy,0);
  const double remaining_depth=result.distance-applied.z(), remaining_lateral=result.lateral-applied.x();
  // Test the full remaining corridor before capping this individual leg. In
  // particular, never chase a replacement whose lateral error cannot be removed.
  const bool blocked_lateral=std::abs(remaining_lateral)>approach.lateral_tolerance+1e-6;
  const bool blocked_depth=remaining_depth<approach.min_distance-1e-6 || remaining_depth>approach.max_distance+1e-6;
  const double length=std::hypot(dx,dy),cap=t.approach_steps==0?.50:approach.max_step;
  if(blocked_lateral || blocked_depth || std::min(length,cap)<minimum_step-1e-6) {
    error=std::string("depth alignment blocked: ")+(blocked_lateral?"lateral_boundary":blocked_depth?"forward_clearance/travel":"minimum_executable_step")+
      " camera_forward_m="+std::to_string(result.distance)+" camera_right_m="+std::to_string(result.lateral)+
      " requested_inward_m="+std::to_string(requested_inward)+" available_inward_m="+std::to_string(t.approach_limit-traveled)+
      " requested_right_m="+std::to_string(requested_right)+" offset_m="+std::to_string(offset)+
      " lower_m="+std::to_string(-*t.target.fetch->search_max_m)+" upper_m="+std::to_string(upper)+
      " bounded_step_m="+std::to_string(std::min(length,cap));return false;
  }
  if(length>cap) {dx*=cap/length;dy*=cap/length;}
  result.dx=dx;result.dy=dy;return true;
}
void TaskManagerNode::align_or_plan()
{
  auto t=active_;Point3 tcp;
  if(!hardware_ok() || !stopped() || !fresh(t->target_stamp,config_.observation_age) || !surface_to_tcp(t->surface,profile(),tcp)) {
    fail("target/hardware expired before grasp planning");return;
  }
  update_budgets();if(t->framing_elapsed>=config_.framing.timeout) {fail("framing budget exhausted");return;}
  if(t->target.fetch->grasp=="top") {
    DepthAlignment alignment;std::string error;
    if(!depth_alignment(t->surface,alignment,error)) {fail(error);return;}
    t->camera_distance=alignment.distance;t->camera_lateral=alignment.lateral;
    // Correct height before closing the distance so a high target stays in view.
    // Lift completion establishes a new observation barrier before XY alignment.
    const double dz=tcp[2]-profile().preferred_tcp[2];
    if(std::abs(dz)>config_.position_tolerance) {
      const double goal=lift_status_->current_z+dz;
      if(goal<kLiftMinM || goal>kLiftMaxM) {fail("top TCP lift unreachable");return;}
      if(++t->align_attempts>config_.max_align) {fail("lift alignment attempt limit");return;}
      send_lift(goal,Phase::Align);return;
    }
    if(alignment.dx!=0 || alignment.dy!=0) {
      ++t->approach_steps;
      start_local(alignment.dx,alignment.dy,Phase::Align,0,LocalPurpose::Approach);return;
    }
    ArmGoal goal;goal.command=t->fetched_quantity+1==t->planned_quantity?ArmGoal::TOP_PICK_HOLD:
      t->fetched_quantity==0?ArmGoal::TOP_PICK_PLACE_LEFT:ArmGoal::TOP_PICK_PLACE_RIGHT;
    goal.x=tcp[0];goal.y=tcp[1];goal.z=tcp[2];goal.expected_lift_z_m=lift_status_->current_z;
    t->plan={};t->plan.tcp=tcp;t->plan.pick={{"top_pick",goal}};
    if(local_timer_) {local_timer_->cancel();local_timer_.reset();}
    t->candidates.clear();t->targets.reset();t->step_index=0;
    enter(Phase::Grasp,"top_pick");advance_fetch();return;
  }
  if(config_.approach.enabled) {
    DepthAlignment alignment;std::string error;
    const bool feasible=depth_alignment(t->surface,alignment,error);
    t->camera_distance=alignment.distance;t->camera_lateral=alignment.lateral;
    if(!feasible) {fail(error);return;}
    if(alignment.dx!=0 || alignment.dy!=0) {
      // Manual-height debugging retains its exact configured region.
      const auto & p=profile();
      if(!p.align_lift && (tcp[2]<p.minimum[2] || tcp[2]>p.maximum[2])) {
        make_front_grasp_plan(tcp,p,t->plan,error);
        fail(error);return;
      }
      ++t->approach_steps;enter(Phase::Align,"depth_alignment");
      event("depth_alignment","camera_forward_m="+std::to_string(alignment.distance)+
        " camera_right_m="+std::to_string(alignment.lateral)+" odom_delta="+
        std::to_string(alignment.dx)+","+std::to_string(alignment.dy));
      start_local(alignment.dx,alignment.dy,Phase::Align,0,LocalPurpose::Approach);return;
    }
    event("depth_alignment_ready","camera_forward_m="+std::to_string(alignment.distance)+
      " camera_right_m="+std::to_string(alignment.lateral));
  }
  std::string error;
  if(profile().align_lift) {
    const auto started=Clock::now();double goal=lift_status_->current_z;
    const bool feasible=choose_grasp_lift(tcp,profile(),lift_status_->current_z,goal,t->plan,error);
    event("ik_lift_selection","elapsed_ms="+std::to_string(
      std::chrono::duration<double,std::milli>(Clock::now()-started).count())+
      " current_m="+std::to_string(lift_status_->current_z)+" goal_m="+std::to_string(goal)+
      " feasible="+(feasible?"true":"false"));
    if(!feasible) {fail(error);return;}
    if(goal!=double(lift_status_->current_z)) {
      if(++t->align_attempts>config_.max_align) {fail("lift alignment attempt limit");return;}
      if(local_timer_) {local_timer_->cancel();local_timer_.reset();}
      send_lift(goal,Phase::Align);return;
    }
  } else if(!make_front_grasp_plan(tcp,profile(),t->plan,error)) {
    fail(error);return;
  }
  if(local_timer_) {local_timer_->cancel();local_timer_.reset();}
  t->candidates.clear();t->targets.reset();t->step_index=0;
  enter(Phase::Grasp,"grasp");advance_fetch();
}
}  // namespace k1muse_manager
