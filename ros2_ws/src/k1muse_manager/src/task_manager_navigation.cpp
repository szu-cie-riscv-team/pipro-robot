#include "k1muse_manager/task_manager_node_impl.hpp"

namespace k1muse_manager
{
Json TaskManagerNode::navigation_diagnostics(const Active & t) const
{
  const auto & n=t.navigation;if(n.started==Clock::time_point{}) {return nullptr;}
  const auto current=n.finished==Clock::time_point{}?Clock::now():n.finished;
  return {{"task_id",t.id},{"leg_id",t.leg_id},{"elapsed_sec",std::chrono::duration<double>(current-n.started).count()},
    {"ack_received",n.ack_received},{"accepted",n.ack_received?Json(n.accepted):Json(nullptr)},
    {"ack_ms",n.ack_received?Json(n.ack_ms):Json(nullptr)},
    {"has_feedback",n.received},{"xy_error_m",n.received?Json(n.xy_error):Json(nullptr)},
    {"yaw_error_rad",n.received?Json(n.yaw_error):Json(nullptr)},
    {"distance_remaining_m",n.received?Json(n.remaining):Json(nullptr)},{"recoveries",n.recoveries},
    {"near_goal_radius_m",.30},{"near_goal",n.near},
    {"near_elapsed_sec",n.near?Json(std::chrono::duration<double>(current-n.near_started).count()):Json(nullptr)},
    {"feedback_travel_m",n.feedback_travel_m},{"feedback_rotation_rad",n.feedback_rotation_rad},
    {"pose_source_ns",n.source_ns},{"pose_age_ms",n.source_ns>0?Json((now().nanoseconds()-n.source_ns)*1e-6):Json(nullptr)},
    {"terminal_code",n.terminal_code<0?Json(nullptr):Json(n.terminal_code)}};
}
void TaskManagerNode::cancel_nav(const std::shared_ptr<Active> & task)
  {
    if (!task->nav || task->cancel_sent || task->terminal) {return;}
    task->cancel_sent = true;
    try {nav_->async_cancel_goal(task->nav,
      [this, task](action_msgs::srv::CancelGoal::Response::SharedPtr reply) {
        if (active_ != task || task->terminal) {return;}
        if (reply->return_code != action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
          task->message = "Nav2 cancellation rejected; awaiting actual terminal result";
          RCLCPP_WARN(get_logger(), "%s", task->message.c_str());
        }
      });} catch (const std::exception & e) {
      ready_ = false; task->error = e.what();
      RCLCPP_ERROR(get_logger(), "Nav2 cancel send failed: %s", e.what());
    }
  }

void TaskManagerNode::advance()
  {
    const auto task = active_;
    if (!task || (!task->handle && !task->fetch && !task->survey) || task->upper_cancel_pending || !gate_state_.empty()) {return;}
    if (task->business() && !task->nav_active) {advance_business(); return;}
    if (task->terminal) {
      if (task->close_failed) {return;}
      set_gate(task, task->outcome == Code::SUCCEEDED ? "succeeded" :
        task->outcome == Code::CANCELED ? "canceled" : "failed");
    } else if (task->blocked) {
      return;
    } else if (task->cancel) {
      if (!task->sent) {
        task->terminal = true; task->outcome = task->error.empty() ? Code::CANCELED : Code::ABORTED;
        task->message = "Canceled before sending a Nav2 goal";
        if (!task->started) {
          // No motion source was opened. In particular, a context change during
          // RESUME must not try to close a nonexistent old-context motion leg.
          if (ready_ && task->context == context_) {navigation_compute(task, false);}
          else {navigation_done(task);}
        } else {advance();}
      } else {
        cancel_nav(task);
        if (!task->stopped) {set_gate(task, "stopping");}
      }
    } else if (!task->started) {
      if (!inference_->navigation_available()) {task->waiting_permit=true;return;}
      task->waiting_permit=false;
      if (!task->navigation_compute_ready) {navigation_compute(task, true);}
      else {set_gate(task, "starting");}
    } else if (!task->sent) {
      send_nav(task);
    } else if (task->nav && !task->running) {
      set_gate(task, "running");
    }
  }

void TaskManagerNode::navigation_compute(const std::shared_ptr<Active> & task, bool enabled)
{
  const std::string state = enabled ? "active" : task->business() ? "paused" : "parked";
  gate_state_ = "compute_" + state;
  const auto serial = ++gate_sequence_;
  const auto started = Clock::now();
  // Keep the pending response after timeout: a lifecycle transition cannot be canceled.
  auto timed_out = std::make_shared<bool>(false);
  auto complete = [this, task, serial, state, enabled, started, timed_out](
      bool success, const std::string & error) {
      if (active_ != task || serial != gate_sequence_) {return;}
      if (deadline_) {deadline_->cancel(); deadline_.reset();}
      gate_state_.clear();
      RCLCPP_INFO(get_logger(), "NAV compute=%s success=%d elapsed_ms=%.1f task=%s",
        state.c_str(), success, std::chrono::duration<double,std::milli>(Clock::now()-started).count(),
        task->id.c_str());
      if (!success || *timed_out) {
        if (!*timed_out && task->cancel && !ready_ && task->error.empty()) {
          if (!task->terminal) {
            task->outcome = Code::CANCELED; task->message = "Canceled before sending a Nav2 goal";
          }
          navigation_done(task); return;
        }
        ready_ = false; task->blocked = false;
        task->error = *timed_out ? "navigation compute result arrived after timeout" : error;
        task->message = task->error; task->outcome = Code::ABORTED;
        navigation_done(task); return;
      }
      if (enabled) {task->navigation_compute_ready = true; advance();}
      else {navigation_done(task);}
    };
  if (!gate_->service_is_ready()) {complete(false, "navigation compute owner unavailable"); return;}
  auto request = std::make_shared<Control::Request>();
  request->request_id = std::to_string(serial); request->client_id = instance_;
  request->operation = "set_navigation_compute";
  request->arguments = Json{{"manager_instance", instance_}, {"context_id", task->context},
    {"business_id", task->business() ? task->id : ""}, {"state", state}}.dump();
  deadline_ = create_wall_timer(std::chrono::seconds(15), [this, task, serial, timed_out] {
    if (active_ != task || serial != gate_sequence_) {return;}
    if (deadline_) {deadline_->cancel(); deadline_.reset();}
    *timed_out = true; ready_ = false; task->blocked = true;
    task->error = "navigation compute timeout: awaiting actual transition result";
    RCLCPP_ERROR(get_logger(), "%s", task->error.c_str());
  });
  try {gate_->async_send_request(request,
    [complete](rclcpp::Client<Control>::SharedFuture future) {
      try {const auto reply = future.get(); complete(reply->accepted, reply->error);}
      catch (const std::exception & e) {complete(false, e.what());}
    });
  } catch (const std::exception & e) {complete(false, e.what());}
}

void TaskManagerNode::send_nav(const std::shared_ptr<Active> & task)
  {
    task->navigation={};task->navigation.started=Clock::now();
    if (task->business()) {deadline(config_.navigation_timeout, "navigation terminal timeout");}
    Nav::Goal goal; goal.pose = task->pose; goal.pose.header.stamp = now();
    RCLCPP_INFO(get_logger(), "NAV task=%s leg=%s context=%s map=%s goal=(%.4f,%.4f,%.4f)",
      task->id.c_str(), task->leg_id.c_str(), task->context.c_str(), map_.c_str(),
      goal.pose.pose.position.x, goal.pose.pose.position.y,
      2*std::atan2(goal.pose.pose.orientation.z,goal.pose.pose.orientation.w));
    task->sent = true;
    auto options = rclcpp_action::Client<Nav>::SendGoalOptions();
    options.goal_response_callback = [this, task, leg=task->leg](std::shared_ptr<NavHandle> handle) {
        if (active_ != task || task->leg!=leg) {return;}
        auto & n=task->navigation;n.ack_received=true;n.accepted=bool(handle);
        n.ack_ms=std::chrono::duration<double,std::milli>(Clock::now()-n.started).count();
        task->nav = handle;
        if (!handle) {
          task->navigation.finished=Clock::now();last_navigation_=navigation_diagnostics(*task);
          last_navigation_["goal_rejected"]=true;
          task->terminal = true;
          task->outcome = task->cancel && task->error.empty() ? Code::CANCELED : Code::ABORTED;
          task->message = "Nav2 rejected the goal";
        } else {
          RCLCPP_INFO(get_logger(), "NAV task=%s leg=%s accepted_goal=%s ack_ms=%.1f", task->id.c_str(),
            task->leg_id.c_str(), rclcpp_action::to_string(handle->get_goal_id()).c_str(),n.ack_ms);
          if (task->cancel) {cancel_nav(task);}
        }
        advance();
      };
    options.feedback_callback = [this, task, leg=task->leg](std::shared_ptr<NavHandle>,
        std::shared_ptr<const Nav::Feedback> feedback) {
        if (active_ != task || task->leg!=leg || task->terminal) {return;}
        // Diagnose actual map-frame errors instead of treating path length as arrival.
        const auto & pose=feedback->current_pose;
        const auto & p=pose.pose.position;const auto & q=pose.pose.orientation;
        const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        if(pose.header.frame_id==task->pose.header.frame_id &&
          std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(yaw)) {
          auto & n=task->navigation;
          const auto angle=[](double a){return std::atan2(std::sin(a),std::cos(a));};
          if(n.received) {
            n.feedback_travel_m+=std::hypot(p.x-n.x,p.y-n.y);
            n.feedback_rotation_rad+=std::abs(angle(yaw-n.yaw));
          }
          n.received=true;n.x=p.x;n.y=p.y;n.yaw=yaw;
          n.xy_error=std::hypot(p.x-task->pose.pose.position.x,p.y-task->pose.pose.position.y);
          n.yaw_error=angle(2*std::atan2(task->pose.pose.orientation.z,task->pose.pose.orientation.w)-yaw);
          n.remaining=feedback->distance_remaining;n.recoveries=feedback->number_of_recoveries;
          n.source_ns=int64_t(pose.header.stamp.sec)*1000000000LL+pose.header.stamp.nanosec;
          if(!n.near && n.xy_error<=.30) {
            n.near=true;n.near_started=Clock::now();
            RCLCPP_INFO(get_logger(),"NAV task=%s leg=%s near_goal xy=%.4f yaw=%.4f recoveries=%d",
              task->id.c_str(),task->leg_id.c_str(),n.xy_error,n.yaw_error,n.recoveries);
          }
        }
        auto result = std::make_shared<Task::Feedback>();
        result->distance_remaining = feedback->distance_remaining;
        if (task->handle) {task->handle->publish_feedback(result);}
      };
    options.result_callback = [this, task, leg=task->leg](const NavHandle::WrappedResult & result) {
        if (active_ != task || task->leg!=leg) {return;}
        task->navigation.finished=Clock::now();task->navigation.terminal_code=int(result.code);
        last_navigation_=navigation_diagnostics(*task);
        RCLCPP_INFO(get_logger(), "NAV task=%s leg=%s terminal_code=%d goal=%s summary=%s", task->id.c_str(),
          task->leg_id.c_str(), int(result.code), rclcpp_action::to_string(result.goal_id).c_str(),last_navigation_.dump().c_str());
        task->terminal = true;
        task->outcome = task->error.empty() ? result.code : Code::ABORTED;
        if (result.code == Code::CANCELED && !task->cancel) {task->outcome = Code::ABORTED;}
        task->message = result.code == Code::SUCCEEDED ? "Nav2 reached the target" :
          result.code == Code::CANCELED ? "Nav2 canceled the target" : "Nav2 failed the target";
        advance();
      };
    try {nav_->async_send_goal(goal, options);}
    catch (const std::exception & e) {
      // A send exception does not prove that the remote endpoint saw no request.
      task->blocked = true; ready_ = false; task->error = e.what();
      set_gate(task, "stopping");
    }
  }

void TaskManagerNode::set_gate(const std::shared_ptr<Active> & task, const std::string & state)
  {
    gate_state_ = state;
    const auto serial = ++gate_sequence_;
    auto request = std::make_shared<Control::Request>();
    request->request_id = std::to_string(serial); request->client_id = instance_;
    request->operation = "set_navigation";
    request->arguments = Json{{"manager_instance", instance_}, {"context_id", task->context},
      {"goal_id", task->leg_id}, {"sequence", serial}, {"state", state}, {"ready", ready_},
      {"business_id", task->business() ? task->id : ""}, {"source", "nav"}}.dump();
    deadline_ = create_wall_timer(timeout_, [this, task, serial, state] {
        if (serial != gate_sequence_ || active_ != task || gate_state_.empty()) {return;}
        gate_->remove_pending_request(gate_request_);
        gate_done(task, state, false, "navigation control timeout");
      });
    try {
      if (!gate_->service_is_ready()) {throw std::runtime_error("navigation output unavailable");}
      auto future = gate_->async_send_request(request,
        [this, task, serial, state](rclcpp::Client<Control>::SharedFuture future) {
          if (serial != gate_sequence_ || active_ != task || gate_state_.empty()) {return;}
          const auto reply = future.get();
          gate_done(task, state, reply->accepted, reply->error);
        });
      gate_request_ = future.request_id;
    } catch (const std::exception & e) {gate_done(task, state, false, e.what());}
  }

void TaskManagerNode::gate_done(const std::shared_ptr<Active> & task, const std::string & state,
    bool success, const std::string & error)
  {
    if (deadline_) {deadline_->cancel(); deadline_.reset();}
    gate_state_.clear();
    if (!success) {
      const bool canceled_before_send = state == "starting" && (task->cancel || task->upper_cancel_pending) &&
        !task->sent && task->error.empty();
      ready_ = false; task->error = canceled_before_send ? "" : error; task->cancel = true;
      RCLCPP_ERROR(get_logger(), "Navigation %s: %s", state.c_str(), error.c_str());
      if (state != "starting" && state != "running") {
        task->close_failed = state != "stopping";
        task->blocked = true; cancel_nav(task); return;
      }
      if (!task->sent) {
        task->terminal = true; task->outcome = canceled_before_send ? Code::CANCELED : Code::ABORTED;
        task->message = canceled_before_send ? "Canceled before sending a Nav2 goal" : error;
      }
      cancel_nav(task); advance(); return;
    }
    if (state == "starting") {task->started = true;}
    else if (state == "running") {task->running = true;}
    else if (state == "stopping") {task->stopped = true;}
    else {
      if (task->business()) {cancel_deadline();} // The Nav2 leg ended; PAUSE has its own deadline.
      if (ready_ && task->context == context_) {navigation_compute(task, false);}
      else {navigation_done(task);} // Context/mode teardown already owns the old group.
      return;
    }
    advance();
  }


}  // namespace k1muse_manager
