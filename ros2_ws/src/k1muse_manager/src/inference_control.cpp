#include "k1muse_manager/inference_control.hpp"

#include <chrono>
#include <utility>

namespace k1muse_manager
{
using namespace std::chrono_literals;

InferenceControl::InferenceControl(rclcpp::Node & node) : node_(node)
{
  const auto timeout = node_.declare_parameter<int>("inference_control_timeout_ms", 500);
  if (timeout < 1 || timeout > 1000) {
    throw std::invalid_argument("inference_control_timeout_ms must be in [1,1000]");
  }
  timeout_ = std::chrono::milliseconds(timeout);
  instance_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  input_ = node_.create_client<Control>("/k1muse/voice/input/control");
  intent_ = node_.create_client<Control>("/k1muse/voice/intent/control");
  vision_ = node_.create_client<Control>("/k1muse/perception/yolo/control");
  listening_ = node_.create_client<Listening>("/k1muse/voice/input/set_listening");
  cv_ = node_.create_client<Listening>("/k1muse/perception/cv/set_enabled");
  status_ = node_.create_publisher<std_msgs::msg::String>(
    "/k1muse/task/inference/status", rclcpp::QoS(1).reliable().transient_local());
  control_ = node_.create_service<Control>("/k1muse/task/inference/control",
    [this](Header header, Control::Request::SharedPtr request) {control(header, request);});
  publish();
  const bool automatic = node_.declare_parameter<bool>("inference_auto_initialize", false);
  const bool voice = node_.declare_parameter<bool>("inference_enable_voice", true);
  const bool yolo = node_.declare_parameter<bool>("inference_enable_yolo", true);
  const auto startup_ms = node_.declare_parameter<int>("inference_startup_timeout_ms", 30000);
  if (startup_ms < 1000) {throw std::invalid_argument("inference_startup_timeout_ms must be >=1000");}
  if (automatic) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(startup_ms);
    startup_wait_ = node_.create_wall_timer(100ms, [this, voice, yolo, end] {
      if (phase_ != "starting") {startup_wait_->cancel(); startup_wait_.reset(); return;}
      if ((!voice || (input_->service_is_ready() && intent_->service_is_ready() &&
        listening_->service_is_ready())) && (!yolo || vision_->service_is_ready())) {
        startup_wait_->cancel(); startup_wait_.reset(); initialize(voice, yolo);
      } else if (std::chrono::steady_clock::now() >= end) {
        startup_wait_->cancel(); startup_wait_.reset(); block("startup_services_unavailable");
      }
    });
  }
}

void InferenceControl::reply(Header header, const Control::Request & request,
  const std::string & error, Json result)
{
  Control::Response response;
  response.operation_id = request.request_id;
  response.accepted = error.empty(); response.error = error;
  if (response.accepted) {response.result = result.dump();}
  control_->send_response(*header, response);
}

void InferenceControl::publish()
{
  const auto value = Json{{"phase", phase_}, {"owner", owner_}, {"work_id", work_id_},
    {"input_enabled", input_enabled_}, {"cv_work_id", cv_work_}, {"cv_pending", cv_pending_}, {"last_error", error_}}.dump();
  if (value == published_) {return;}
  std_msgs::msg::String message; message.data = value; status_->publish(message); published_ = value;
  if (idle() && idle_callback_) {idle_callback_();}
}

void InferenceControl::cancel_deadline()
{
  if (deadline_) {deadline_->cancel(); deadline_.reset();}
}

void InferenceControl::block(const std::string & error)
{
  RCLCPP_ERROR(node_.get_logger(), "INFERENCE event=blocked phase=%s owner=%s work=%s error=%s",
    phase_.c_str(), owner_.c_str(), work_id_.c_str(), error.c_str());
  if (prepare_deadline_) {prepare_deadline_->cancel(); prepare_deadline_.reset();}
  if (phase_ != "blocked") {blocked_phase_ = phase_;}
  phase_ = "blocked"; error_ = error; input_enabled_ = false; publish();
}

void InferenceControl::initialize(bool voice, bool yolo)
{
  enable_voice_ = voice; enable_yolo_ = yolo; error_.clear();
  if (voice) {prepare("voice_input");}
  else if (yolo) {prepare("yolo_detector");}
  else {resume();}
}

void InferenceControl::after_prepare(const std::string & owner)
{
  if (owner == "voice_input") {
    voice_ready_ = true;
    prepare(enable_yolo_ ? "yolo_detector" : "voice_command");
  } else if (owner == "yolo_detector") {
    yolo_ready_ = true;
    if (enable_voice_) {prepare("voice_command");} else {resume();}
  } else {resume();}
}

void InferenceControl::call(rclcpp::Client<Control>::SharedPtr client,
  const std::string & operation, Json args,
  std::function<void(Control::Response::SharedPtr)> done)
{
  const auto rpc = ++rpc_;
  if (!client->service_is_ready()) {done(nullptr); return;}
  auto request = std::make_shared<Control::Request>();
  request->client_id = "task_manager"; request->operation = operation;
  request->request_id = instance_ + "_rpc_" + std::to_string(rpc);
  request->arguments = args.dump();
  const auto pending = client->async_send_request(request,
    [this, rpc, done](rclcpp::Client<Control>::SharedFuture future) {
      if (rpc != rpc_) {return;}
      ++rpc_; cancel_deadline(); done(future.get());
    });
  deadline_ = node_.create_wall_timer(timeout_,
    [this, client, id = pending.request_id, rpc, done] {
      if (rpc != rpc_) {return;}
      ++rpc_; cancel_deadline(); client->remove_pending_request(id); done(nullptr);
    });
}

void InferenceControl::listening(bool enabled, std::function<void(bool)> done)
{
  set_bool(listening_, enabled, std::move(done));
}

void InferenceControl::set_bool(
  rclcpp::Client<Listening>::SharedPtr client, bool enabled, std::function<void(bool)> done)
{
  const auto rpc = ++rpc_;
  if (!client->service_is_ready()) {done(false); return;}
  auto request = std::make_shared<Listening::Request>(); request->data = enabled;
  const auto pending = client->async_send_request(request,
    [this, rpc, done](rclcpp::Client<Listening>::SharedFuture future) {
      if (rpc != rpc_) {return;}
      ++rpc_; cancel_deadline(); done(future.get()->success);
    });
  deadline_ = node_.create_wall_timer(client == cv_ ? std::chrono::milliseconds(2000) : timeout_, [this, client, rpc, id = pending.request_id, done] {
      if (rpc != rpc_) {return;}
    ++rpc_; cancel_deadline(); client->remove_pending_request(id); done(false);
    });
}

void InferenceControl::scan(bool enabled, const std::string & work, std::function<void(bool)> done)
{
  if (cv_pending_ || work.empty() || (enabled ? (!idle() || !cv_work_.empty()) : cv_work_ != work)) {
    done(false); return;
  }
  if (!cv_->service_is_ready()) {done(false); return;}
  if (enabled) {cv_pending_ = true; cv_work_ = work; phase_ = "pausing"; input_enabled_ = false; publish();}
  auto toggle = [this, enabled, work, done](bool paused) {
      if (cv_work_ != work) {return;}
      if (!paused) {cv_pending_ = false; cv_failed_ = true; block("cv_pause_not_acknowledged"); done(false); return;}
      cv_pending_ = true;
      const auto rpc = ++cv_rpc_;
      auto finish_toggle = [this, enabled, work, rpc, done](bool success) {
          if (cv_rpc_ != rpc || cv_work_ != work) {return;}
          ++cv_rpc_; cv_pending_ = false;
          if (cv_deadline_) {cv_deadline_->cancel(); cv_deadline_.reset();}
          if (!success) {
            cv_failed_ = true; block(enabled ? "cv_enable_not_acknowledged" : "cv_disable_not_acknowledged");
          } else {
            cv_failed_ = false;
            if (enabled) {phase_ = "idle";}
            else {
              cv_work_.clear();
              if (error_.rfind("cv_", 0) == 0) {
                error_.clear(); phase_ = work_id_.empty() ? "idle" : blocked_phase_;
              }
              if (work_id_.empty()) {resume();}
            }
            publish();
          }
          done(success);
        };
      auto request = std::make_shared<Listening::Request>(); request->data = enabled;
      try {
        const auto pending = cv_->async_send_request(request,
          [finish_toggle](rclcpp::Client<Listening>::SharedFuture response) {finish_toggle(response.get()->success);});
        cv_deadline_ = node_.create_wall_timer(2s, [this, id = pending.request_id, finish_toggle] {
          cv_->remove_pending_request(id); finish_toggle(false);
        });
      } catch (const std::exception &) {finish_toggle(false);}
    };
  // QR is task-scoped RGB work on CPU4/5, independent of YOLO's original permit.
  // Pause speech once; do not touch its RPC sequence while toggling CV alongside vision.
  if (enabled && enable_voice_) {listening(false, std::move(toggle));} else {toggle(true);}
}

void InferenceControl::prepare(const std::string & owner)
{
  phase_ = "preparing"; owner_ = owner;
  work_id_ = instance_ + "_prepare_" + std::to_string(++sequence_); publish();
  const auto id = work_id_;
  RCLCPP_INFO(node_.get_logger(), "PREPARE event=begin owner=%s work=%s",
    owner.c_str(), id.c_str());
  arm_prepare_deadline();
  Json args{{"work_id",id}};
  if(owner=="voice_command") {args["drug_catalog"]=drug_catalog_;}
  call(owner == "voice_input" ? input_ : owner == "yolo_detector" ? vision_ : intent_,
    owner == "voice_command" ? "prepare_runtime" : "prepare", std::move(args),
    [this, id](Control::Response::SharedPtr response) {
      if (work_id_ != id || phase_ != "preparing") {return;}
      if (!response || !response->accepted) {block("prepare_not_acknowledged");}
    });
}

void InferenceControl::arm_prepare_deadline()
{
  if (prepare_deadline_) {prepare_deadline_->cancel();}
  const auto id = work_id_, stage = phase_;
  prepare_deadline_ = node_.create_wall_timer(60s, [this, id, stage] {
    if (work_id_ == id && phase_ == stage) {block("prepare_timeout");}
  });
}

void InferenceControl::business(bool active)
{
  business_ = active;
  if (active && phase_ == "wake_wait") {
    phase_ = "pausing"; input_enabled_ = false; publish();
    listening(false, [this](bool success) {
      if (!success) {block("business_pause_not_acknowledged"); return;}
      phase_ = "idle"; publish();
    });
  } else if (!active && idle()) {resume();}
}

void InferenceControl::resume()
{
  if (cv_failed_) {block("cv_state_unresolved"); return;}
  if (!cv_work_.empty()) {
    phase_ = "idle"; owner_.clear(); work_id_.clear(); input_enabled_ = false; publish(); return;
  }
  if (business_ || !enable_voice_ || !voice_ready_) {
    phase_ = "idle"; owner_.clear(); work_id_.clear(); input_enabled_ = false; publish(); return;
  }
  phase_ = "resuming"; owner_.clear(); work_id_.clear(); publish();
  listening(true, [this](bool success) {
    if (!success) {block("resume_not_acknowledged"); return;}
    phase_ = "wake_wait"; input_enabled_ = true; publish();
  });
}

void InferenceControl::finish()
{
  completed_ = work_id_; work_id_.clear(); owner_.clear(); resume();
}

void InferenceControl::control(Header header, Control::Request::SharedPtr request)
{
  try {
    const auto args = Json::parse(request->arguments.empty() ? "{}" : request->arguments);
    const auto id = args.value("work_id", std::string());
    const auto & role = request->client_id;
    if (request->operation == "initialize") {
      if (phase_ != "starting") {reply(header, *request, "inference_busy"); return;}
      reply(header, *request, {}, {{"status", "running"}});
      initialize(args.value("enable_voice", true), args.value("enable_yolo", false)); return;
    }
    if (id.empty()) {reply(header, *request, "invalid_work_id"); return;}
    if (request->operation == "acquire") {
      const auto stage = args.at("stage").get<std::string>();
      const bool speech = stage == "speech" && role == "voice_input" && voice_ready_ && !business_;
      const bool vision = stage == "vision" && role == "yolo_detector" && yolo_ready_;
      const bool restart = stage == "restart" && role == "voice_command" && voice_ready_ && !business_;
      if (!cv_work_.empty() && !vision) {reply(header, *request, "task_vision_active"); return;}
      if (!speech && !vision && !restart) {
        reply(header, *request, "unsupported_stage"); return;
      }
      if (restart && phase_ == "intent" && owner_ == role &&
        args.value("replaces", std::string()) == work_id_ && id != work_id_) {
        ++rpc_; cancel_deadline();
        completed_ = work_id_; work_id_ = id; phase_ = "restarting"; publish();
        arm_prepare_deadline();
        reply(header, *request, {}, {{"granted", true}}); return;
      }
      if ((phase_ != "wake_wait" && phase_ != "idle") || !work_id_.empty()) {
        reply(header, *request, "inference_busy"); return;
      }
      if (id == completed_) {reply(header, *request, "stale_work_id"); return;}
      phase_ = "pausing"; input_enabled_ = false; owner_ = role; work_id_ = id; publish();
      const auto target = speech ? "speech" : vision ? "vision" : "restarting";
      auto grant = [this, header, request, id, target](bool success) {
        if (work_id_ != id) {return;}
        if (!success) {
          block("pause_not_acknowledged"); reply(header, *request, "pause_not_acknowledged"); return;
        }
        phase_ = target; input_enabled_ = phase_ == "speech"; publish(); reply(header, *request, {}, {{"granted", true}});
        if (phase_ == "restarting") {arm_prepare_deadline();}
      };
      if (enable_voice_ && cv_work_.empty()) {listening(false, std::move(grant));} else {grant(true);}
      return;
    }
    if (request->operation == "speech_done") {
      if (id != work_id_ || role != "voice_input" || role != owner_ ||
        (phase_ != "speech" && phase_ != "asr"))
      {reply(header, *request, "stale_work_id"); return;}
      phase_ = "asr"; input_enabled_ = false; publish();
      reply(header, *request, {}, {{"granted", true}}); return;
    }
    if (request->operation == "asr_done") {
      if (role == "voice_input" && (id == completed_ ||
        (id == work_id_ && phase_ == "intent"))) {
        reply(header, *request); return;
      }
      if (id != work_id_ || phase_ != "asr" || role != owner_) {
        reply(header, *request, "stale_work_id"); return;
      }
      const auto text = args.at("text").get<std::string>();
      reply(header, *request);
      if (text.empty()) {finish(); return;}
      phase_ = "intent"; owner_ = "voice_command"; publish();
      call(intent_, "accept_transcript", {{"work_id", id}, {"text", text}},
        [this, id](Control::Response::SharedPtr response) {
          if (work_id_ != id || phase_ != "intent") {return;}
          if (!response) {block("intent_not_acknowledged");}
          else if (!response->accepted) {error_ = response->error; finish();}
        });
      return;
    }
    if (request->operation == "release") {
      if (id == completed_) {reply(header, *request); return;}
      const auto stage = phase_ == "blocked" ? blocked_phase_ : phase_;
      if (id != work_id_ || role != owner_ ||
        (stage != "preparing" && stage != "intent" && stage != "asr" && stage != "speech" &&
        stage != "vision" && stage != "restarting" && stage != "pausing"))
      {reply(header, *request, "stale_work_id"); return;}
      // Release may arrive before the short prepare/accept ACK. Retire that RPC.
      ++rpc_; cancel_deadline();
      if (stage == "restarting" && prepare_deadline_) {
        prepare_deadline_->cancel(); prepare_deadline_.reset();
      }
      reply(header, *request);
      if (stage == "preparing") {
        RCLCPP_INFO(node_.get_logger(), "PREPARE event=complete owner=%s work=%s outcome=%s",
          role.c_str(), id.c_str(), args.value("outcome", "failed").c_str());
        if (prepare_deadline_) {prepare_deadline_->cancel(); prepare_deadline_.reset();}
        completed_ = id;
        if (args.value("outcome", "failed") != "succeeded") {
          block(args.value("error", "prepare_failed")); return;
        }
        error_.clear(); after_prepare(owner_);
      } else {
        const bool failed = stage == "vision" && args.value("outcome", "succeeded") == "failed";
        const auto error = args.value("error", "vision_failed");
        if (failed) {yolo_ready_ = false; error_ = "vision_failed: " + error;}
        finish();
        if (failed && vision_failure_) {vision_failure_(error);}
      }
      return;
    }
    reply(header, *request, "unsupported_operation");
  } catch (const Json::exception &) {reply(header, *request, "invalid_arguments");}
}
}  // namespace k1muse_manager
