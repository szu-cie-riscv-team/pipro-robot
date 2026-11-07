#include "k1muse_manager/task_manager_node_impl.hpp"
#include <algorithm>

namespace k1muse_manager
{
namespace
{
double angle(double a) {return std::atan2(std::sin(a),std::cos(a));}
int64_t source(const Odom & m) {return int64_t(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec;}
bool planar(const Odom & m)
{
  const auto & p=m.pose.pose.position;const auto & q=m.pose.pose.orientation;
  return m.header.frame_id=="odom" && m.child_frame_id=="base_link" &&
    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(q.x) && std::isfinite(q.y) &&
    std::isfinite(q.z) && std::isfinite(q.w) && std::abs(q.x)<1e-3 && std::abs(q.y)<1e-3 &&
    std::abs(q.z*q.z+q.w*q.w-1)<1e-3;
}
}
void TaskManagerNode::on_odom(Odom::ConstSharedPtr m)
{
  if(active_ && active_->business() && active_->local_open && odom_ &&
    (source(*m)<source(*odom_) || !planar(*m))) {
    // Keep the last valid sample for the stop boundary; never command from reset odometry.
    fail("odometry reset/invalid during local motion",active_->pending==Pending::Service);return;
  }
  odom_=m;
  if(active_ && active_->is_survey() && active_->survey_run->observing && planar(*m)) {
    auto & samples=active_->survey_run->poses;
    const int64_t ns=source(*m);
    if(!samples.empty() && ns<samples.back().ns) {survey_layer_failed("odometry_reset");return;}
    if(samples.empty() || ns>samples.back().ns) {
      samples.push_back({ns,m->pose.pose.position.x,m->pose.pose.position.y,
        2*std::atan2(m->pose.pose.orientation.z,m->pose.pose.orientation.w)});
      const double horizon=std::max(config_.observation_age,config_.cv_frame_age)+config_.observation_timeout;
      while(samples.size()>2 && (ns-samples[1].ns)*1e-9>horizon) {samples.pop_front();}
    }
  }
  check_hardware();
}
void TaskManagerNode::zero_local()
{
  geometry_msgs::msg::TwistStamped command;command.header.stamp=now();command.header.frame_id="base_link";
  local_pub_->publish(command);
}
void TaskManagerNode::start_local(double dx,double dy,Phase next,double turn,LocalPurpose purpose)
{
  auto t=active_;
  if(!odom_ || !planar(*odom_) ||
    !std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(turn) || std::hypot(dx,dy)>.5+1e-9) {
    fail("local motion precondition/displacement invalid");return;
  }
  t->next_phase=next;t->move_x=odom_->pose.pose.position.x+dx;t->move_y=odom_->pose.pose.position.y+dy;
  t->move_yaw=angle(2*std::atan2(odom_->pose.pose.orientation.z,odom_->pose.pose.orientation.w)+turn);
  t->target_stamp=0;t->candidates.clear();t->targets.reset();open_local(purpose);
}
void TaskManagerNode::open_local(LocalPurpose purpose)
{
  auto t=active_;if(t->pending!=Pending::None || t->local_open) {return;}
  if(!hardware_ok() || !stopped() || motion_inhibited_ || !odom_ || !planar(*odom_)) {
    fail("local motion requires fresh idle hardware");return;
  }
  t->local_purpose=purpose;
  if(purpose==LocalPurpose::Search) {
    t->next_phase=Phase::Align;t->waiting_point=false;t->candidates.clear();t->targets.reset();
  }
  t->previous_x=odom_->pose.pose.position.x;t->previous_y=odom_->pose.pose.position.y;
  t->previous_yaw=2*std::atan2(odom_->pose.pose.orientation.z,odom_->pose.pose.orientation.w);
  t->leg_id=t->id+":local:"+std::to_string(++t->leg);
  web_call("set_motion",{{"goal_id",t->leg_id},{"source","local"},{"state","starting"}},[this,t](const Json &) {
    t->local_open=true;
    if(t->cancel || !t->error.empty() || t->pause_requested) {stop_local();return;}
    web_call("set_motion",{{"goal_id",t->leg_id},{"source","local"},{"state","running"}},[this,t](const Json &) {
      t->pending=Pending::Local;++t->operation;
      if(t->local_purpose==LocalPurpose::Pose || t->local_purpose==LocalPurpose::Approach || t->local_purpose==LocalPurpose::Qr) {
        const double travel=std::hypot(t->move_x-odom_->pose.pose.position.x,t->move_y-odom_->pose.pose.position.y);
        deadline(config_.local_timeout+(t->local_purpose!=LocalPurpose::Pose?2*travel/config_.speed:0),"local motion timeout");
      }
      vision_timer();event("local_begin","purpose="+std::to_string(int(t->local_purpose)));local_tick();
    });
  });
}
void TaskManagerNode::local_tick()
{
  auto t=active_;if(!t || !t->business() || t->blocked) {return;}
  if(t->is_survey()) {survey_local_tick();return;}
  update_budgets();
  if(t->pending==Pending::Service) {if(t->local_open) {zero_local();}return;}
  if(t->pending!=Pending::None && t->pending!=Pending::Local) {return;}
  if(t->cancel || t->pause_requested || !t->error.empty()) {
    if(t->local_open) {stop_local();} else {advance_fetch();}return;
  }
  if(t->phase==Phase::Scan && t->search_ready && (!t->left_confirmed || t->locating_right) &&
    t->scan_elapsed>=config_.qr_align_timeout) {fail("left QR depth alignment timeout");return;}
  const bool visual=t->phase==Phase::Detect || t->phase==Phase::Align;
  if(!t->local_open && !visual) {return;}
  if((t->phase==Phase::Detect && t->search_elapsed>=config_.search_timeout) ||
    (t->phase==Phase::Align && t->framing_elapsed>=config_.framing.timeout)) {fail("search/framing budget exhausted");return;}
  if(!odom_ || !planar(*odom_) || !fresh(source(*odom_),config_.status_age) || !hardware_ok()) {
    fail("local feedback stale/faulted");return;
  }
  const auto & p=odom_->pose.pose.position;const auto & q=odom_->pose.pose.orientation;
  const double yaw=2*std::atan2(q.z,q.w);
  if(t->local_open && (std::hypot(p.x-t->previous_x,p.y-t->previous_y)>.30 ||
    std::abs(angle(yaw-t->previous_yaw))>.6)) {fail("odometry jumped during local motion");return;}
  t->previous_x=p.x;t->previous_y=p.y;t->previous_yaw=yaw;
  double x=0,y=0,dyaw=0;
  if(t->local_open && (t->local_purpose==LocalPurpose::Pose || t->local_purpose==LocalPurpose::Approach || t->local_purpose==LocalPurpose::Qr)) {
    if(t->local_purpose==LocalPurpose::Approach && t->right_reached) {
      const double upper=t->right_limit_m;
      const double goal_right=(t->move_x-t->shelf_odom_x)*t->right_x+(t->move_y-t->shelf_odom_y)*t->right_y;
      if(goal_right>upper) {
        t->move_x-=(goal_right-upper)*t->right_x;t->move_y-=(goal_right-upper)*t->right_y;
      }
    }
    x=t->move_x-p.x;y=t->move_y-p.y;dyaw=angle(t->move_yaw-yaw);
    if(std::hypot(x,y)<=config_.position_tolerance && std::abs(dyaw)<=config_.yaw_tolerance) {stop_local();return;}
    if(t->local_purpose==LocalPurpose::Approach) {
      const double traveled=(p.x-t->shelf_odom_x)*t->inward_x+(p.y-t->shelf_odom_y)*t->inward_y;
      if(traveled < -config_.position_tolerance || traveled>t->approach_limit+config_.position_tolerance) {fail("depth approach shelf clearance/travel limit exceeded");return;}
      t->search_offset=(p.x-t->shelf_odom_x)*t->right_x+(p.y-t->shelf_odom_y)*t->right_y;
      const double upper=t->right_reached?t->right_limit_m:*t->target.fetch->search_max_m;
      if(t->search_offset < -*t->target.fetch->search_max_m-config_.position_tolerance ||
        t->search_offset>upper+config_.position_tolerance) {fail("depth alignment outside lateral travel boundary");return;}
      // Execute the measured displacement with odometry; reobserve depth after stopping.
    }
  } else if(visual) {
    t->search_offset=(p.x-t->shelf_odom_x)*t->right_x+(p.y-t->shelf_odom_y)*t->right_y;
    const double lower=*t->target.fetch->search_min_m,upper=*t->target.fetch->search_max_m;
    // Both phases use the frozen shelf reference. A leftward navigation error
    // permits a rightward lead-in; Align may also correct left of shelf zero.
    const double actual_lower=t->phase==Phase::Align?-upper:t->search_start_offset;
    const double actual_upper=t->right_reached?t->right_limit_m:upper;
    if(t->search_offset < actual_lower-config_.position_tolerance ||
      t->search_offset>actual_upper+config_.position_tolerance) {
      fail(t->phase==Phase::Align?"actual position outside alignment lateral boundary":
        "actual position outside shelf search boundary");return;
    }
    if(!config_.no_qr && !fresh(t->cv_frame_stamp,config_.cv_frame_age)) {zero_local();return;}
    const bool fresh_frame=fresh(t->candidate.stamp,config_.framing.max_frame_age);
    const bool found=fresh_frame && t->candidate.valid &&
      (t->phase==Phase::Align || t->search_offset>=lower-config_.position_tolerance);
    // A lock may be acquired/lost between two control ticks while still in Detect.
    if(!t->locked_id.empty() && !found) {zero_local();return;}
    if(t->phase==Phase::Detect && (t->right_reached || (config_.no_qr && t->boundary_wait))) {
      zero_local();
      if(!t->boundary_wait) {
        t->boundary_wait=true;t->next_phase=Phase::Detect;
        if(t->local_open) {stop_local();} else {settle(Phase::Detect);}
        return;
      }
      // Ignore pre-stop YOLO frames even if they arrived after the QR result.
      if(!fresh_frame || t->candidate.stamp<=t->barrier) {return;}
      if(!found) {begin_fetch_return();return;}
    }
    if(t->phase==Phase::Detect && found) {
      enter(Phase::Align,"depth_alignment");t->waiting_point=false;t->framing_started=true;t->budget_tick=Clock::now();
      event("depth_alignment_begin","detection="+t->locked_id);
      if(t->local_open) {t->next_phase=Phase::Align;stop_local();}
      else {settle(Phase::Align);}return;
    }
    double right_speed=0;
    if(t->phase==Phase::Align) {
      zero_local();
      // Stopped 2D/3D pairs drive measured alignment. No pixel servo remains.
      if(t->waiting_point && found) {try_target_pair();}
      return;
    } else {
      // An empty but fresh 2D frame permits search. Missing/stale frames never do.
      if(!fresh_frame) {zero_local();return;}
      if(t->search_offset>=upper-config_.position_tolerance) {
        if(!config_.no_qr) {fail("search limit reached before right marker depth alignment");return;}
        t->boundary_wait=true;t->next_phase=Phase::Detect;
        if(t->local_open) {stop_local();} else {settle(Phase::Detect);}return;
      }
      right_speed=config_.speed;
      if(t->pending==Pending::None) {search_next();return;}
    }
    // Traverse a possible left-of-zero arrival only to enter the search window;
    // Detect still commands rightward search only.
    const double min_offset=t->search_offset<lower ? t->search_start_offset : lower;
    const double right_limit=t->right_reached ? t->right_limit_m : upper;
    right_speed=std::clamp(right_speed,(min_offset-t->search_offset)/.05,
      std::max(0.0,(right_limit-t->search_offset)/.05));
    const double drift=(p.x-t->shelf_odom_x)*t->inward_x+(p.y-t->shelf_odom_y)*t->inward_y-t->inward_offset;
    const double inward_speed=std::clamp(-drift,-config_.speed,config_.speed);
    x=t->right_x*right_speed+t->inward_x*inward_speed;
    y=t->right_y*right_speed+t->inward_y*inward_speed;dyaw=angle(t->shelf_yaw-yaw);
  } else {return;}
  double vx=std::cos(yaw)*x+std::sin(yaw)*y,vy=-std::sin(yaw)*x+std::cos(yaw)*y;
  const double norm=std::hypot(vx,vy);
  if((t->local_purpose==LocalPurpose::Approach || t->local_purpose==LocalPurpose::Qr) && t->local_open && norm>config_.position_tolerance) {
    // Avoid crawling below the chassis dead band while still bounding the last control step.
    const double speed=std::min({config_.speed,std::max(norm,.02),norm/.05});
    vx*=speed/norm;vy*=speed/norm;
  } else if(norm>config_.speed) {vx*=config_.speed/norm;vy*=config_.speed/norm;}
  geometry_msgs::msg::TwistStamped command;command.header.stamp=now();command.header.frame_id="base_link";
  command.twist.linear.x=vx;command.twist.linear.y=vy;
  command.twist.angular.z=std::clamp(dyaw,-config_.yaw_speed,config_.yaw_speed);local_pub_->publish(command);
}
void TaskManagerNode::stop_local()
{
  auto t=active_;if(!t || t->pending==Pending::Service) {return;}
  if(local_timer_) {local_timer_->cancel();local_timer_.reset();}cancel_deadline();zero_local();
  if(!t->local_open) {advance_business();return;}
  t->pending=Pending::None;
  web_call("set_motion",{{"goal_id",t->leg_id},{"source","local"},{"state","succeeded"}},[this,t](const Json &) {
    t->local_open=false;
    web_call("wait_stopped",Json::object(),[this,t](const Json &) {
      if(!stopped()) {fail("local stop acknowledgement without fresh stopped odometry",true);return;}
      event("local_end","actual="+std::to_string(odom_->pose.pose.position.x)+","+std::to_string(odom_->pose.pose.position.y));
      local_finished();
    });
  });
}
void TaskManagerNode::local_finished()
{
  auto t=active_;
  if(t->is_survey()) {survey_local_finished();return;}
  const bool approached=t->local_purpose==LocalPurpose::Approach;
  if(approached) {
    const auto & p=odom_->pose.pose.position;
    t->inward_offset=(p.x-t->shelf_odom_x)*t->inward_x+(p.y-t->shelf_odom_y)*t->inward_y;
    t->search_offset=(p.x-t->shelf_odom_x)*t->right_x+(p.y-t->shelf_odom_y)*t->right_y;
    event("distance_stop","inward_m="+std::to_string(t->inward_offset));
  }
  if(t->cancel || !t->error.empty()) {advance_fetch();return;}
  if(t->pause_requested) {advance_fetch();return;}
  const auto next=t->next_phase;enter(next,phase_name(next));t->barrier=now().nanoseconds();
  if(next==Phase::Detect && t->boundary_wait) {t->candidate={};}
  if(next==Phase::Align) {
    if(t->tracking_lost) {t->lost_since=Clock::now();}
    t->waiting_point=true;t->candidate={};t->candidates.clear();t->targets.reset();
    stage_boundary(approached?"after:distance_stop":"after:framing_stop");
  }
  advance_fetch();
}
}  // namespace k1muse_manager
