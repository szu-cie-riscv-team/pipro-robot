#include "k1muse_manager/voice_command_node.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <array>
#include <cctype>
#include <memory>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <unistd.h>
#include <sys/syscall.h>

#include <k1muse_interfaces/action/navigate_to_task.hpp>
#include <k1muse_interfaces/action/fetch_drug.hpp>
#include <k1muse_interfaces/action/survey_warehouse.hpp>
#include <k1muse_interfaces/srv/web_request.hpp>

namespace k1muse_manager
{
namespace
{

using NavigateToTask = k1muse_interfaces::action::NavigateToTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToTask>;
using FetchDrug = k1muse_interfaces::action::FetchDrug;
using FetchGoalHandle = rclcpp_action::ClientGoalHandle<FetchDrug>;
using SurveyWarehouse = k1muse_interfaces::action::SurveyWarehouse;
using SurveyGoalHandle = rclcpp_action::ClientGoalHandle<SurveyWarehouse>;
using Trigger = std_srvs::srv::Trigger;
using IntentControl = k1muse_interfaces::srv::WebRequest;
using Json = IntentJson;

// Humble generates the UUID inside async_send_goal. Observe that standard hook
// so a lost admission ACK still has an inspectable identity; execution stays in Client.
template<class Action>
class VoiceActionClient : public rclcpp_action::Client<Action>
{
public:
  VoiceActionClient(rclcpp::Node * node, const std::string & name)
  : rclcpp_action::Client<Action>(node->get_node_base_interface(),
      node->get_node_graph_interface(), node->get_node_logging_interface(), name) {}
  rclcpp_action::GoalUUID last_id{};
protected:
  rclcpp_action::GoalUUID generate_goal_id() override
  {last_id = rclcpp_action::Client<Action>::generate_goal_id(); return last_id;}
};

std::string trim_ascii_space(const std::string & text)
{
  std::size_t first = 0;
  while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first])) != 0) {
    ++first;
  }
  std::size_t last = text.size();
  while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1])) != 0) {
    --last;
  }
  return text.substr(first, last - first);
}

bool starts_with(const std::string & text, const std::string & prefix)
{
  return text.compare(0, prefix.size(), prefix) == 0;
}

const std::string * canonical_drug(const std::string & name, const std::vector<VoiceDrug> & drugs)
{
  const std::string * matched=nullptr;
  for(const auto & drug:drugs) {
    const bool matches=name==drug.name ||
      std::find(drug.aliases.begin(),drug.aliases.end(),name)!=drug.aliases.end();
    if(matches) {
      if(matched && *matched!=drug.name) {return nullptr;}
      matched=&drug.name;
    }
  }
  return matched;
}

class VoiceCommandNode : public rclcpp::Node
{
public:
  explicit VoiceCommandNode(const rclcpp::NodeOptions & options, CloudTransport transport,
    std::function<void(const std::string &, const Json &)> restart,
    std::string prepare_work, Json catalog)
  : Node("voice_command", options),
    address_word_(declare_parameter<std::string>("address_word", "")),
    navigate_word_(declare_parameter<std::string>("navigate_word", "")),
    cancel_navigation_word_(declare_parameter<std::string>("cancel_navigation_word", "")),
    startup_config_(declare_parameter<std::string>("intent_config_file", "")),
    cloud_config_(declare_parameter<std::string>("cloud_config_file", ""),
      declare_parameter<std::string>("cloud_prompt_file", "")),
    local_config_(declare_parameter<std::string>("local_config_file", ""),
      declare_parameter<std::string>("local_prompt_file", "")),
    cloud_worker_(std::move(transport)), restart_(std::move(restart))
  {
    if (trim_ascii_space(address_word_).empty() || trim_ascii_space(navigate_word_).empty() ||
      trim_ascii_space(cancel_navigation_word_).empty())
    {
      throw std::invalid_argument("C15 command words must be set in the parameter YAML");
    }

    startup_mode_override_ = declare_parameter<std::string>("startup_mode_override", "");
    if (!startup_mode_override_.empty() && startup_mode_override_ != "rules" &&
      startup_mode_override_ != "cloud" && startup_mode_override_ != "local")
    {throw std::invalid_argument("startup_mode_override must be empty, rules, cloud or local");}
    active_startup_mode_ = startup_mode_override_.empty() ? startup_config_.mode : startup_mode_override_;
    active_startup_error_ = startup_mode_override_.empty() ? startup_config_.error : "";
    active_config_ = cloud_config_.api;
    active_prompt_ = cloud_config_.prompt;
    active_api_error_ = cloud_config_.api_error;
    active_prompt_error_ = cloud_config_.prompt_error;
    install_drug_catalog(catalog.is_null() ?
      Json{{"drugs",Json::array()},{"error","catalog_not_prepared"}} : catalog);
    instance_id_ = std::to_string(::getpid()) + "_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());

    navigation_ = std::make_shared<VoiceActionClient<NavigateToTask>>(
      this, "/k1muse/task/navigate");
    get_node_waitables_interface()->add_waitable(navigation_, nullptr);
    fetch_ = std::make_shared<VoiceActionClient<FetchDrug>>(this, "/k1muse/task/fetch_drug");
    get_node_waitables_interface()->add_waitable(fetch_, nullptr);
    survey_ = std::make_shared<VoiceActionClient<SurveyWarehouse>>(this, "/k1muse/task/survey");
    get_node_waitables_interface()->add_waitable(survey_, nullptr);
    cancel_task_ = create_client<Trigger>("/k1muse/task/cancel");
    inference_control_ = create_client<IntentControl>("/k1muse/task/inference/control");
    inference_status_ = create_subscription<std_msgs::msg::String>(
      "/k1muse/task/inference/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        try {inference_ = Json::parse(message->data); publish_intent_state();}
        catch (const Json::exception &) {}
      });
    intent_status_ = create_publisher<std_msgs::msg::String>(
      "/k1muse/voice/intent/status", rclcpp::QoS(1).reliable().transient_local());
    intent_control_ = create_service<IntentControl>("/k1muse/voice/intent/control",
      [this](std::shared_ptr<rmw_request_id_t> header, IntentControl::Request::SharedPtr request) {
        if (request->operation == "restart_node") {request_restart(header, request); return;}
        IntentControl::Response response; control(*request, response);
        intent_control_->send_response(*header, response);
      });
    if (!active_startup_error_.empty()) {set_error(active_startup_error_);}
    else if (!cloud_config_.api_error.empty()) {set_error(cloud_config_.api_error);}
    else if (!cloud_config_.prompt_error.empty()) {set_error(cloud_config_.prompt_error);}
    if (active_startup_mode_ == "local" && !local_config_.error.empty()) {
      local_status_ = "unavailable"; set_error(local_config_.error);
    }
    if (!prepare_work.empty()) {
      inference_work_ = prepare_work; last_inference_work_ = prepare_work; activating_ = true;
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      startup_wait_ = create_wall_timer(std::chrono::milliseconds(50), [this, end] {
        if (inference_control_->service_is_ready()) {
          startup_wait_->cancel(); startup_wait_.reset(); prepare_runtime();
        } else if (std::chrono::steady_clock::now() >= end) {
          startup_wait_->cancel(); startup_wait_.reset(); activating_ = false;
          set_error("manager_unavailable"); publish_intent_state();
        }
      });
    }
    publish_intent_state();
    RCLCPP_INFO(get_logger(), "C15 startup=%s; override=%s; mode=%s; drug names=%zu",
      active_startup_mode_.c_str(), startup_mode_override_.c_str(), mode_.c_str(), drugs_.size());
  }

  ~VoiceCommandNode() override
  {
    get_node_waitables_interface()->remove_waitable(navigation_, nullptr);
    if (fetch_) {get_node_waitables_interface()->remove_waitable(fetch_, nullptr);}
    cloud_worker_.stop();
    const auto result = cloud_worker_.take_result();
    if (result && !result->job.probe) {
      log_voice_result(result->job.backend.c_str(), "canceled", result->duration_ms);
    }
  }

private:
  void watch_admission(const std::string & kind, const rclcpp_action::GoalUUID & id)
  {
    constexpr char hex[]="0123456789abcdef";
    std::string uuid;uuid.reserve(32);
    for (auto byte : id) {uuid+=hex[byte>>4];uuid+=hex[byte&15];}
    goal_admission_ = {{"kind", kind}, {"goal_id", uuid}, {"status", "pending"}};
    admission_deadline_ = create_wall_timer(std::chrono::seconds(5), [this] {
      admission_deadline_->cancel(); admission_deadline_.reset();
      if (!goal_pending_) {return;}
      goal_admission_["status"] = "unresolved";
      set_error("goal_admission_unresolved", goal_admission_.dump());
      RCLCPP_ERROR(get_logger(), "Goal admission unresolved; inspect original goal: %s",
        goal_admission_.dump().c_str());
      publish_intent_state(); // Keep busy and the original request; no replay.
    });
    publish_intent_state();
  }

  void admission_received(bool accepted)
  {
    if (admission_deadline_) {admission_deadline_->cancel(); admission_deadline_.reset();}
    goal_pending_ = false;
    goal_admission_["status"] = accepted ? "accepted" : "rejected";
    if (last_error_.value("code", "") == "goal_admission_unresolved") {set_error("");}
    finish_inference(); publish_intent_state();
  }

  void finish_inference()
  {
    if (inference_work_.empty() || completion_pending_ || goal_pending_ || cloud_worker_.busy() ||
      restarting_ || restart_requesting_) {return;}
    auto request = std::make_shared<IntentControl::Request>();
    request->request_id = instance_id_ + "_release_" + std::to_string(++sequence_);
    request->client_id = "voice_command"; request->operation = "release";
    const auto work = inference_work_;
    request->arguments = Json{{"work_id", work}, {"outcome", "succeeded"}}.dump();
    completion_pending_ = true;
    completion_ = {{"work_id", work}, {"request_id", request->request_id}, {"status", "pending"}};
    auto unresolved = [this] {
      completion_["status"] = "unresolved";
      set_error("inference_completion_unresolved", completion_.dump()); publish_intent_state();
    };
    if (!inference_control_->service_is_ready()) {unresolved(); return;}
    completion_deadline_ = create_wall_timer(std::chrono::milliseconds(1500), [this, unresolved] {
      completion_deadline_->cancel(); completion_deadline_.reset(); unresolved();
    });
    try {
      const auto id = request->request_id;
      inference_control_->async_send_request(request, [this, work, id](rclcpp::Client<IntentControl>::SharedFuture f) {
        if (completion_deadline_) {completion_deadline_->cancel(); completion_deadline_.reset();}
        try {
          const auto response = f.get();
          if (!response->accepted || response->operation_id != id) {
            throw std::runtime_error(response->error.empty() ? "completion_ack_mismatch" : response->error);
          }
          completion_["status"] = "acknowledged"; completion_pending_ = false;
          if (inference_work_ == work) {inference_work_.clear();}
          if (last_error_.value("code", "") == "inference_completion_unresolved") {set_error("");}
        } catch (const std::exception & error) {
          completion_["status"] = "unresolved"; set_error("inference_completion_unresolved", error.what());
        }
        publish_intent_state();
      });
    } catch (const std::exception &) {
      completion_deadline_->cancel(); completion_deadline_.reset(); unresolved();
    }
  }

  void on_transcript(const std::string & text)
  {
    if (restarting_ || restart_requesting_ || activating_) {return;}
    const auto command = parse_voice_command(
      text, address_word_, navigate_word_, cancel_navigation_word_, drugs_);
    if (command.type == VoiceCommandType::CancelTask || command.type == VoiceCommandType::Cancel) {
      invalidate(false); last_intent_ = nullptr;
      execute_command(command, text);
      log_voice_result(mode_.c_str(), command.type == VoiceCommandType::CancelTask ?
        "cancel_task" : "cancel_navigation");
      publish_intent_state(); finish_inference(); return;
    }
    last_intent_ = nullptr;
    if (busy_) {
      set_error("business_rejected");
      log_voice_result(mode_.c_str(), "null", -1, "business_rejected");
      publish_intent_state(); finish_inference(); return;
    }
    if (mode_ == "cloud" || mode_ == "local") {
      if (text.size() > 512) {set_error("unsupported_input");}
      else if (cloud_worker_.busy()) {set_error("api_busy");}
      else {
        try {
          const auto id = "intent_" + std::to_string(++sequence_);
          if (mode_ == "local") {start_local(text, id, false);}
          else {start_cloud(text, id, false, false);}
        }
        catch (...) {
          const auto failed_mode = mode_;
          cloud_enabled_ = false; mode_ = "rules";
          if (failed_mode == "cloud" && active_config_ == cloud_config_.api &&
            active_system_prompt_ == saved_system_prompt_) {
            api_status_ = "unavailable";
          }
          if (failed_mode == "local") {local_status_ = "unavailable";}
          const auto failure = failed_mode == "local" ? "local_backend_unavailable" :
            "cloud_backend_unavailable";
          cloud_worker_.cancel(true); run_rules(text); set_error(failure);
          log_voice_result((failed_mode + "->rules").c_str(), last_intent_.dump(), -1,
            failure);
        }
        publish_intent_state(); finish_inference(); return;
      }
    } else {
      last_error_ = {{"code", ""}, {"message", ""}};
      last_request_ = {{"id", "intent_" + std::to_string(++sequence_)},
        {"kind", "intent"}, {"status", "succeeded"}};
      apply_rules(command, text);
    }
    const auto output = mode_ == "rules" && last_intent_.is_null() &&
      command.type == VoiceCommandType::Navigate ?
      "navigate:" + Json(command.task_name).dump() : last_intent_.dump();
    log_voice_result(mode_.c_str(), output, -1, last_error_.value("code", ""));
    publish_intent_state();
    finish_inference();
  }

  void run_rules(const std::string & text)
  {
    apply_rules(parse_voice_command(
      text, address_word_, navigate_word_, cancel_navigation_word_, drugs_), text);
  }

  void apply_rules(const VoiceCommand & command, const std::string & text)
  {
    if (command.type == VoiceCommandType::Fetch) {
      last_intent_ = VoiceIntent{"fetch", command.drug_name, command.quantity}.json();
    } else if (command.type == VoiceCommandType::Survey) {
      last_intent_ = VoiceIntent{"survey", command.drug_name, 0}.json();
    } else if (command.type == VoiceCommandType::None) {
      last_intent_ = VoiceIntent{"unknown", "", 0}.json(); set_error("unknown");
    } else {last_intent_ = nullptr;}
    execute_command(command, text);
  }

  void execute_command(const VoiceCommand & command, const std::string & text = {})
  {
    if (command.type == VoiceCommandType::Survey) {if(!busy_) {request_survey(command.drug_name);}else {set_error("business_busy");}return;}
    if (command.type == VoiceCommandType::None) {
      if (starts_with(trim_ascii_space(text), "帮我拿")) {
        RCLCPP_WARN(get_logger(), "Unsupported or ambiguous fetch command: %s", text.c_str());
      }
      return;
    }
    if (command.type == VoiceCommandType::CancelTask) {
      request_task_cancel();
      return;
    }
    if (command.type == VoiceCommandType::Fetch) {
      if (busy_) {
        RCLCPP_WARN(get_logger(), "Rejecting fetch while a voice task is active");
      } else {
        request_fetch(command.drug_name,command.quantity);
      }
      return;
    }
    if (command.type == VoiceCommandType::Cancel) {
      if (fetch_active_ || survey_active_) {
        RCLCPP_WARN(get_logger(), "Use '取消任务' to cancel the complete fetch task");
        return;
      }
      if (busy_ && !cancel_requested_) {
        cancel_requested_ = true;
        if (goal_handle_) {
          navigation_->async_cancel_goal(goal_handle_);
        }
        RCLCPP_INFO(get_logger(), "Navigation cancellation requested");
      }
      return;
    }
    if (busy_) {
      RCLCPP_WARN(get_logger(), "Ignoring voice navigation while a voice task is active");
      return;
    }
    if (!navigation_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "C07 navigation Action is unavailable");
      return;
    }

    NavigateToTask::Goal goal;
    goal.task_name = command.task_name;
    goal.use_pose = false;  // Bind the name to the manager's prepared map at admission.
    busy_ = true;
    goal_pending_ = true;
    publish_intent_state();
    cancel_requested_ = false;
    goal_handle_.reset();

    auto options = rclcpp_action::Client<NavigateToTask>::SendGoalOptions();
    options.goal_response_callback = [this](const std::shared_ptr<GoalHandle> handle) {
        admission_received(bool(handle));
        if (!handle) {
          RCLCPP_WARN(get_logger(), "C07 rejected the voice navigation goal");
          clear_goal();
          return;
        }
        goal_handle_ = handle;
        if (cancel_requested_) {
          navigation_->async_cancel_goal(handle);
        }
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        const char * message = result.result ? result.result->message.c_str() : "";
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Voice navigation succeeded: %s", message);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Voice navigation canceled: %s", message);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Voice navigation failed: %s", message);
            break;
          default:
            RCLCPP_WARN(get_logger(), "Voice navigation ended with unknown status: %s", message);
            break;
        }
        clear_goal();
      };

    navigation_->async_send_goal(goal, options);
    watch_admission("navigate", navigation_->last_id);
    RCLCPP_INFO(get_logger(), "Requested navigation to '%s'", command.task_name.c_str());
  }

  void request_survey(const std::string & drug)
  {
    if(!survey_ || !survey_->action_server_is_ready()) {set_error("business_unavailable");return;}
    busy_=true;goal_pending_=true;survey_active_=true;cancel_requested_=false;survey_goal_.reset();
    rclcpp_action::Client<SurveyWarehouse>::SendGoalOptions options;
    options.goal_response_callback=[this](SurveyGoalHandle::SharedPtr handle) {
      admission_received(bool(handle));
      if(!handle) {set_error("business_rejected");clear_goal();return;}
      survey_goal_=handle;if(cancel_requested_) {survey_->async_cancel_goal(handle);}
    };
    options.feedback_callback=[this](SurveyGoalHandle::SharedPtr,const std::shared_ptr<const SurveyWarehouse::Feedback> f) {
      RCLCPP_INFO(get_logger(),"Survey %s %s/%d attempt=%u count=%d",f->stage.c_str(),f->shelf_id.c_str(),f->layer,f->attempt,f->provisional_count);
    };
    options.result_callback=[this](const SurveyGoalHandle::WrappedResult & r) {
      RCLCPP_INFO(get_logger(),"Survey terminal=%d: %s",int(r.code),r.result?r.result->message.c_str():"");clear_goal();
    };
    SurveyWarehouse::Goal goal;goal.drug_name=drug;
    survey_->async_send_goal(goal,options);watch_admission("survey",survey_->last_id);
  }

  void request_fetch(const std::string & drug_name, int quantity)
  {
    if(quantity<1 || quantity>3) {set_error("unsupported_quantity");return;}
    if (!fetch_ || !fetch_->action_server_is_ready()) {
      set_error("business_unavailable");
      RCLCPP_WARN(get_logger(), "FetchDrug business Action is unavailable");
      return;
    }
    FetchDrug::Goal goal;
    goal.drug_name = drug_name;goal.quantity=quantity;
    busy_ = true;
    goal_pending_ = true;
    fetch_active_ = true;
    cancel_requested_ = false;
    fetch_goal_.reset();
    rclcpp_action::Client<FetchDrug>::SendGoalOptions options;
    options.goal_response_callback = [this, drug_name](FetchGoalHandle::SharedPtr handle) {
        admission_received(bool(handle));
        if (!handle) {
          set_error("business_rejected"); publish_intent_state();
        RCLCPP_WARN(get_logger(), "Business rejected fetch '%s'", drug_name.c_str());
          clear_goal();
          return;
        }
        fetch_goal_ = handle;
        RCLCPP_INFO(get_logger(), "Business accepted fetch '%s'", drug_name.c_str());
        if (cancel_requested_) {
          fetch_->async_cancel_goal(handle);
        }
      };
    options.feedback_callback = [this](
      FetchGoalHandle::SharedPtr, const std::shared_ptr<const FetchDrug::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Fetch stage: %s", feedback->stage.c_str());
      };
    options.result_callback = [this](const FetchGoalHandle::WrappedResult & result) {
        const char * message = result.result ? result.result->message.c_str() : "";
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Fetch succeeded: %s", message);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Fetch canceled: %s", message);
            break;
          default:
            RCLCPP_WARN(get_logger(), "Fetch failed: %s", message);
            break;
        }
        clear_goal();
      };
    fetch_->async_send_goal(goal, options);
    watch_admission("fetch", fetch_->last_id);
    RCLCPP_INFO(get_logger(), "Requested fetch drug='%s' quantity=%d", drug_name.c_str(),quantity);
  }

  void request_task_cancel()
  {
    if (fetch_active_ || survey_active_) {
      // Keep cancellation scoped to this request while its goal response is pending.
      cancel_requested_ = true;
    }
    if (!cancel_task_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Complete task cancellation service is unavailable");
      if (fetch_goal_) {
        fetch_->async_cancel_goal(fetch_goal_);
        RCLCPP_INFO(get_logger(), "Requested cancellation of the voice-owned fetch only");
      }
      if(survey_goal_) {survey_->async_cancel_goal(survey_goal_);}
      return;
    }
    cancel_task_->async_send_request(std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture response) {
        const auto result = response.get();
        if (result->success) {
          RCLCPP_INFO(get_logger(), "Task cancellation request accepted: %s", result->message.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Task cancellation request rejected: %s", result->message.c_str());
        }
      });
  }

  void clear_goal()
  {
    if (admission_deadline_) {admission_deadline_->cancel(); admission_deadline_.reset();}
    goal_pending_ = false;
    if (goal_admission_.value("status", "") == "accepted") {goal_admission_["status"] = "terminal";}
    goal_handle_.reset();
    fetch_goal_.reset();
    survey_goal_.reset();survey_active_=false;
    fetch_active_ = false;
    cancel_requested_ = false;
    busy_ = false;
    publish_intent_state();
  }

  void set_error(const std::string & code, const std::string & message = {})
  {
    last_error_ = {{"code", code}, {"message", message.empty() ? code : message}};
  }

  void log_voice_result(const char * mode, const std::string & output,
    double model_ms = -1, const std::string & error = {})
  {
    last_intent_backend_ = mode;
    last_intent_duration_ = model_ms >= 0 ? Json(model_ms) : Json(nullptr);
    const auto suffix = error.empty() ? std::string() : " | error=" + error;
    if (model_ms >= 0) {
      RCLCPP_INFO(get_logger(), "VOICE mode=%s | model=%.2f ms | result=%s%s",
        mode, model_ms, output.c_str(), suffix.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "VOICE mode=%s | result=%s%s",
        mode, output.c_str(), suffix.c_str());
    }
  }

  Json intent_state() const
  {
    auto inference = inference_.empty() ? Json{{"phase", "unavailable"}, {"owner", ""},
      {"work_id", ""}, {"input_enabled", false}, {"last_error", ""}} : inference_;
    const auto phase = inference.value("phase", "unavailable");
    inference["restart_allowed"] = !busy_ && !completion_pending_ && !restarting_ && !restart_requesting_ &&
      (phase == "wake_wait" || phase == "idle" ||
      (phase == "intent" && inference.value("owner", "") == "voice_command"));
    return {{"instance_id", instance_id_}, {"business_busy", busy_}, {"inference", inference},
      {"goal_admission", goal_admission_}, {"completion", completion_},
      {"startup_mode", startup_config_.mode}, {"restart_required", restart_required_},
      {"startup_mode_override", startup_mode_override_}, {"active_startup_mode", active_startup_mode_},
      {"preparing", activating_}, {"restarting", restarting_ || restart_requesting_},
      {"mode", mode_}, {"cloud_enabled", cloud_enabled_}, {"api_status", api_status_},
      {"api_key_set", !cloud_config_.api.api_key.empty()}, {"config_revision", config_revision_},
      {"model", mode_ == "local" ?
        std::filesystem::path(local_config_.config.model_path).filename().string() :
        active_config_.model}, {"http_busy", cloud_worker_.busy()},
      {"local", {{"status", local_status_}, {"model", local_config_.config.model_path},
        {"busy", (mode_ == "local" || local_status_ == "loading" ||
          local_status_ == "stopping") && cloud_worker_.busy()}}},
      {"last_error", last_error_}, {"last_check_at", last_check_},
      {"last_duration_ms", last_duration_}, {"last_duration_backend", last_duration_backend_},
      {"last_request", last_request_},
      {"last_intent", last_intent_}, {"last_intent_backend", last_intent_backend_},
      {"last_intent_duration_ms", last_intent_duration_}};
  }

  void publish_intent_state()
  {
    if (!intent_status_) {return;}
    const auto state = intent_state().dump();
    if (state == published_state_) {return;}
    std_msgs::msg::String message; message.data = state;
    intent_status_->publish(message); published_state_ = state;
  }

  void invalidate(bool release)
  {
    ++generation_; activating_ = false;
    if (cloud_worker_.busy()) {
      last_request_["status"] = "canceled";
      if (api_status_ == "checking") {api_status_ = "unchecked";}
    }
    cloud_worker_.cancel(release);
  }

  void install_drug_catalog(const Json & catalog)
  {
    drugs_=parse_drug_catalog({{"drugs",catalog.at("drugs")}});
    catalog_error_=catalog.at("error").get<std::string>();
    active_system_prompt_=active_prompt_;
    saved_system_prompt_=cloud_config_.prompt;
    if(local_config_.error.empty()) {local_system_prompt_=local_config_.prompt;}
  }

  Json drug_catalog() const
  {return {{"drugs",drug_catalog_json(drugs_)},{"error",catalog_error_}};}

  void start_cloud(const std::string & text, const std::string & id, bool probe, bool activate)
  {
    if (cloud_worker_.busy()) {throw std::runtime_error("api_busy");}
    const bool preview = probe && !activate;
    if (preview ? !cloud_config_.ready() :
      (!active_api_error_.empty() || !active_prompt_error_.empty() || active_config_.api_key.empty()))
    {throw std::runtime_error("unconfigured");}
    CloudJob job{generation_, id, text, preview ? saved_system_prompt_ : active_system_prompt_,
      preview ? cloud_config_.api : active_config_, probe, activate, "cloud", {}};
    const bool describes_saved = saved_matches(job);
    if (!cloud_worker_.start(std::move(job))) {
      throw std::runtime_error("cloud_backend_unavailable");
    }
    activating_ = activate;
    last_request_ = {{"id", id}, {"kind", probe ? "probe" : "intent"}, {"status", "running"}};
    last_request_["backend"] = "cloud";
    last_error_ = {{"code", ""}, {"message", ""}};
    if (probe && describes_saved) {api_status_ = "checking";}
    if (!probe) {last_intent_ = nullptr;}
    result_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {consume_cloud();});
  }

  void start_local(const std::string & text, const std::string & id, bool activate)
  {
    if (!local_config_.error.empty()) {throw std::runtime_error("local_unavailable");}
    CloudJob job;
    job.generation = generation_; job.id = id; job.text = text;
    job.system_prompt = local_system_prompt_;
    job.probe = activate; job.activate = activate;
    job.backend = "local"; job.local = local_config_.config;
    if (!cloud_worker_.start(std::move(job))) {throw std::runtime_error("local_unavailable");}
    activating_ = activate;
    if (activate) {local_status_ = "loading";}
    last_request_ = {{"id", id}, {"kind", activate ? "warmup" : "intent"},
      {"backend", "local"}, {"status", "running"}};
    last_error_ = {{"code", ""}, {"message", ""}};
    if (!activate) {last_intent_ = nullptr;}
    result_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {consume_cloud();});
    publish_intent_state();
  }

  void consume_cloud()
  {
    auto result = cloud_worker_.take_result();
    if (!result) {return;}
    result_timer_->cancel(); result_timer_.reset();
    if (result->job.generation != generation_) {
      if (result->job.backend == "local" && !result->reply.error.empty()) {
        local_status_ = "unavailable";
        mode_ = "rules";
      }
      if (!result->job.probe) {
        log_voice_result(result->job.backend.c_str(), "canceled", result->duration_ms);
      }
      publish_intent_state(); finish_inference(); return;
    }
    const bool local = result->job.backend == "local";
    const bool current_saved = !local && saved_matches(result->job);
    const bool preview = result->job.probe && !result->job.activate;
    if (result->job.activate) {activating_ = false;}
    last_duration_ = result->duration_ms;
    last_duration_backend_ = result->job.backend;
    if (current_saved) {
      const auto now = std::time(nullptr); std::tm utc{}; ::gmtime_r(&now, &utc);
      char stamp[32]; std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%SZ", &utc);
      last_check_ = stamp;
    }
    std::string error = result->reply.error;
    VoiceIntent intent;
    if (!local && error.empty() &&
      (result->reply.http_status < 200 || result->reply.http_status >= 300)) {
      error = "http_error";
    }
    if (error.empty() && local) {
      if (result->intent) {intent = *result->intent;}
      else {error = "invalid_intent_format";}
    } else if (error.empty()) {
      try {
        const auto body = Json::parse(result->reply.body);
        const auto & choice = body.at("choices").at(0);
        if (choice.at("finish_reason") != "stop") {error = "truncated_response";}
        else {
          const auto text = choice.at("message").at("content").get<std::string>();
          if (text.empty()) {error = "empty_response";}
          else {intent = parse_voice_intent(text);}
        }
      } catch (...) {error = "invalid_intent_format";}
    }
    if (error.empty() && result->job.probe && intent.action != "unknown") {
      error = "test_output_mismatch";
    }
    if (result->job.probe && result->job.activate) {
      RCLCPP_INFO(get_logger(), "PREPARE stage=INTENT event=warmup_end backend=%s elapsed_ms=%.2f error=%s",
        result->job.backend.c_str(), result->duration_ms, error.c_str());
    }
    if (!error.empty()) {
      if (!preview) {
        cloud_enabled_ = false; mode_ = "rules"; cloud_worker_.cancel(true);
      }
      if (local) {local_status_ = "unavailable";}
      if (current_saved) {api_status_ = "unavailable";}
      last_request_["status"] = "failed";
      if (!result->job.probe) {run_rules(result->job.text);}
      set_error(error, error == "http_error" ?
        "HTTP " + std::to_string(result->reply.http_status) : error);
    } else {
      if (current_saved) {api_status_ = "available";}
      last_request_["status"] = "succeeded";
      if (result->job.probe) {
        if (result->job.activate) {
          mode_ = local ? "local" : "cloud";
          if (local) {local_status_ = "ready";}
          else {cloud_enabled_ = true;}
        }
      } else {
        if (!intent.drug.empty()) {
          const auto * canonical=canonical_drug(intent.drug,drugs_);
          if(canonical) {intent.drug=*canonical;}
          else if(intent.action=="survey") {intent=VoiceIntent{"unknown","",0};}
        }
        if (local || intent.action=="survey") {
          const VoiceDrug * mentioned=nullptr;int count=0;
          for(const auto & drug:drugs_) {
            bool found=result->job.text.find(drug.name)!=std::string::npos;
            for(const auto & alias:drug.aliases) {
              found=found || result->job.text.find(alias)!=std::string::npos;
            }
            if(found) {mentioned=&drug;++count;}
          }
          const bool unnamed_survey=intent.action=="survey" && intent.drug.empty();
          const bool named=intent.action=="fetch" || (intent.action=="survey" && !intent.drug.empty());
          if(count>1 || (unnamed_survey && count!=0) ||
            (local && named && (count!=1 || !mentioned || intent.drug!=mentioned->name))) {
            intent=VoiceIntent{"unknown","",0};
          }
        }
        last_intent_ = intent.json();
        if (intent.action == "unknown") {set_error("unknown");}
        else if (intent.action == "survey") {request_survey(intent.drug);}
        else if (std::none_of(drugs_.begin(), drugs_.end(), [&](const VoiceDrug & drug) {
          return drug.name == intent.drug;
        })) {set_error("unsupported_drug");}
        else {
          const int quantity=resolve_fetch_quantity(result->job.text);
          last_intent_["quantity"]=quantity;request_fetch(intent.drug,quantity);
        }
      }
    }
    if (!result->job.probe) {
      log_voice_result(error.empty() ? result->job.backend.c_str() :
        (result->job.backend + "->rules").c_str(), last_intent_.dump(),
        result->duration_ms, last_error_.value("code", ""));
    }
    publish_intent_state();
    finish_inference();
  }

  bool saved_matches(const CloudJob & job) const
  {
    return cloud_config_.ready() && job.config == cloud_config_.api &&
           job.system_prompt == saved_system_prompt_;
  }

  void update_restart_required()
  {
    restart_required_ = (startup_mode_override_.empty() &&
      (startup_config_.mode != active_startup_mode_ || startup_config_.error != active_startup_error_)) ||
      !(cloud_config_.api == active_config_) || cloud_config_.prompt != active_prompt_ ||
      cloud_config_.api_error != active_api_error_ || cloud_config_.prompt_error != active_prompt_error_;
  }

  void prepare_runtime()
  {
    activating_ = false;
    if (!runtime_prepared_) {
      RCLCPP_INFO(get_logger(), "PREPARE stage=INTENT event=begin backend=%s pid=%ld tid=%ld work=%s",
        active_startup_mode_.c_str(), static_cast<long>(::getpid()),
        static_cast<long>(::syscall(SYS_gettid)), inference_work_.c_str());
      runtime_prepared_ = true;
      try {
        if (!active_startup_error_.empty()) {throw std::runtime_error(active_startup_error_);}
        if (active_startup_mode_ == "local") {
          start_local(probe_text_, "startup_" + instance_id_, true); return;
        }
        if (active_startup_mode_ == "cloud") {
          start_cloud(probe_text_, "startup_" + instance_id_, true, true); return;
        }
      } catch (const std::exception & error) {
        if (active_startup_mode_ == "local") {local_status_ = "unavailable";}
        set_error(error.what());
      }
    }
    finish_inference(); publish_intent_state();
  }

  void request_restart(std::shared_ptr<rmw_request_id_t> header,
    IntentControl::Request::SharedPtr request)
  {
    if (!request->request_id.empty() && request->request_id == previous_id_) {
      intent_control_->send_response(*header, previous_response_); return;
    }
    auto replied = std::make_shared<bool>(false);
    auto respond = [this, header, request, replied](bool accepted, const std::string & error) {
      if (*replied) {return;}
      *replied = true;
      IntentControl::Response response; response.operation_id = request->request_id;
      response.accepted = accepted; response.error = error;
      if (accepted) {response.result = Json{{"status", "running"}, {"state", intent_state()}}.dump();}
      else {set_error(error);}
      previous_id_ = request->request_id; previous_response_ = response;
      intent_control_->send_response(*header, response); publish_intent_state();
    };
    if (request->request_id.empty()) {respond(false, "invalid_config"); return;}
    if (busy_) {respond(false, "business_rejected"); return;}
    if (completion_pending_) {respond(false, "inference_completion_unresolved"); return;}
    if (restarting_ || restart_requesting_) {respond(false, "restart_in_progress"); return;}
    if (!restart_) {respond(false, "restart_unavailable"); return;}
    if (!inference_control_->service_is_ready()) {respond(false, "manager_unavailable"); return;}
    const auto work = instance_id_ + "_restart_" + std::to_string(++sequence_);
    auto acquire = std::make_shared<IntentControl::Request>();
    acquire->request_id = work; acquire->client_id = "voice_command"; acquire->operation = "acquire";
    Json args{{"work_id", work}, {"stage", "restart"}};
    if (!inference_work_.empty()) {args["replaces"] = inference_work_;}
    acquire->arguments = args.dump(); restart_requesting_ = true;
    restart_deadline_ = create_wall_timer(std::chrono::milliseconds(1500), [this, respond] {
      restart_deadline_->cancel(); restart_deadline_.reset(); respond(false, "permit_timeout");
    });
    inference_control_->async_send_request(acquire,
      [this, work, replied, respond](rclcpp::Client<IntentControl>::SharedFuture future) {
        if (restart_deadline_) {restart_deadline_->cancel(); restart_deadline_.reset();}
        const auto response = future.get();
        const auto value = Json::parse(response->result, nullptr, false);
        const bool granted = response->accepted && value.is_object() && value.value("granted", false);
        restart_requesting_ = false;
        if (!granted) {
          respond(false, response->error); finish_inference(); publish_intent_state(); return;
        }
        inference_work_ = work;
        if (*replied) {
          // A late grant only owns the old computation's remaining cleanup.
          finish_inference(); publish_intent_state(); return;
        }
        restarting_ = true;
        if (local_status_ == "ready" || local_status_ == "loading") {local_status_ = "stopping";}
        invalidate(true); respond(true, "");
        // The executor sends the held ACK before its spin returns to main.
        restart_(work,drug_catalog());
      });
    publish_intent_state();
  }

  void control(const IntentControl::Request & request, IntentControl::Response & response)
  {
    response.operation_id = request.request_id;
    if (request.operation == "accept_transcript" || request.operation == "prepare_runtime") {
      try {
        if (request.client_id != "task_manager") {throw std::runtime_error("invalid_caller");}
        const auto args = Json::parse(request.arguments);
        const auto work = args.at("work_id").get<std::string>();
        if (work.empty()) {throw std::runtime_error("invalid_work_id");}
        if (work == last_inference_work_) {
          response.accepted = true; response.result = "{\"status\":\"running\"}"; return;
        }
        if (!inference_work_.empty() || restarting_ ||
          (cloud_worker_.busy() && (request.operation != "prepare_runtime" || !activating_)))
        {throw std::runtime_error("inference_busy");}
        if (request.operation == "accept_transcript" && activating_) {
          throw std::runtime_error("inference_busy");
        }
        const auto text = request.operation == "accept_transcript" ?
          args.at("text").get<std::string>() : std::string();
        if(request.operation=="prepare_runtime") {install_drug_catalog(args.at("drug_catalog"));}
        inference_work_ = work; last_inference_work_ = work;
        response.accepted = true; response.result = "{\"status\":\"running\"}";
        if (request.operation == "accept_transcript") {on_transcript(text);}
        else {prepare_runtime();}
      } catch (const std::exception & error) {response.error = error.what();}
      return;
    }
    const bool modifying = request.operation != "get_config";
    if (modifying && !request.request_id.empty() && request.request_id == previous_id_) {
      response = previous_response_; return;
    }
    try {
      if (request.request_id.empty()) {throw std::runtime_error("invalid_config");}
      const auto args = Json::parse(request.arguments.empty() ? "{}" : request.arguments);
      if (modifying && restarting_) {throw std::runtime_error("restart_in_progress");}
      Json result{{"status", "succeeded"}};
      if (request.operation == "get_config") {
        result["drug_catalog"]=drug_catalog();
        result["cloud"] = cloud_config_.api.public_json(); result["system_prompt"] = cloud_config_.prompt;
        result["modes"] = {{"rules_available", true}, {"cloud_available", true},
          {"local_available", local_config_.error.empty() && LocalLlm::available()}};
      } else if (request.operation == "check_cloud") {
        if (!runtime_prepared_) {throw std::runtime_error("inference_busy");}
        if (cloud_worker_.busy()) {throw std::runtime_error("api_busy");}
        VoiceCloudConfig saved(get_parameter("cloud_config_file").as_string(),
          get_parameter("cloud_prompt_file").as_string());
        if (!(saved.api == cloud_config_.api) || saved.prompt != cloud_config_.prompt ||
          saved.api_error != cloud_config_.api_error || saved.prompt_error != cloud_config_.prompt_error) {
          ++config_revision_; api_status_ = "unchecked"; last_check_ = nullptr;
        }
        cloud_config_ = std::move(saved);
        saved_system_prompt_ = cloud_config_.prompt;
        update_restart_required();
        start_cloud(probe_text_, request.request_id, true, false); result["status"] = "running";
      } else {throw std::runtime_error("invalid_config");}
      result["state"] = intent_state();
      response.accepted = true; response.result = result.dump();
    } catch (const std::exception & e) {
      const std::string code = e.what();
      constexpr std::array<const char *, 10> known{{"unconfigured", "invalid_config",
        "config_write_failed", "local_unavailable", "api_busy", "cloud_backend_unavailable",
        "business_rejected", "restart_unavailable", "restart_in_progress", "inference_busy"}};
      response.error = std::find(known.begin(), known.end(), code) != known.end() ? code : "invalid_config";
      response.accepted = false; set_error(response.error);
    }
    if (modifying) {previous_id_ = request.request_id; previous_response_ = response;}
    publish_intent_state();
  }

  std::string address_word_;
  std::string navigate_word_;
  std::string cancel_navigation_word_;
  std::vector<VoiceDrug> drugs_;
  VoiceStartupConfig startup_config_;
  VoiceCloudConfig cloud_config_;
  VoiceLocalConfig local_config_;
  VoiceCloudWorker cloud_worker_;
  std::function<void(const std::string &, const Json &)> restart_;
  std::string catalog_error_{"catalog_not_prepared"};
  CloudConfig active_config_;
  std::string startup_mode_override_, active_startup_mode_, active_startup_error_, active_prompt_,
    active_api_error_, active_prompt_error_, active_system_prompt_, saved_system_prompt_,
    local_system_prompt_, instance_id_;
  bool restart_required_{false}, restarting_{false}, restart_requesting_{false}, runtime_prepared_{false};
  std::string mode_{"rules"}, api_status_{"unchecked"}, published_state_, previous_id_;
  std::string local_status_{"unloaded"}, last_duration_backend_, last_intent_backend_;
  bool cloud_enabled_{false}, activating_{false};
  uint64_t generation_{0}, config_revision_{0}, sequence_{0};
  Json last_error_{{"code", ""}, {"message", ""}};
  Json last_request_{{"id", ""}, {"kind", ""}, {"status", "idle"}};
  Json last_check_ = nullptr, last_duration_ = nullptr, last_intent_ = nullptr;
  Json last_intent_duration_ = nullptr;
  const std::string probe_text_{"接口检测，没有取药或盘点请求，请输出 unknown JSON。"};
  IntentControl::Response previous_response_;
  rclcpp::Service<IntentControl>::SharedPtr intent_control_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr intent_status_;
  rclcpp::TimerBase::SharedPtr result_timer_;
  rclcpp::TimerBase::SharedPtr startup_wait_, restart_deadline_;
  std::shared_ptr<VoiceActionClient<NavigateToTask>> navigation_;
  std::shared_ptr<VoiceActionClient<FetchDrug>> fetch_;
  std::shared_ptr<VoiceActionClient<SurveyWarehouse>> survey_;
  rclcpp::Client<Trigger>::SharedPtr cancel_task_;
  std::shared_ptr<GoalHandle> goal_handle_;
  FetchGoalHandle::SharedPtr fetch_goal_;
  SurveyGoalHandle::SharedPtr survey_goal_;
  bool survey_active_{false};
  bool fetch_active_{false};
  bool busy_{false};
  bool cancel_requested_{false};
  bool goal_pending_{false}, completion_pending_{false};
  Json goal_admission_ = Json::object(), completion_ = Json::object();
  rclcpp::TimerBase::SharedPtr admission_deadline_, completion_deadline_;
  std::string inference_work_, last_inference_work_;
  Json inference_ = Json::object();
  rclcpp::Client<IntentControl>::SharedPtr inference_control_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inference_status_;
};

}  // namespace

namespace
{
int extract_fetch_quantity(std::string & text)
{
  for(const auto word:{"几盒","几个","多少盒","多少个","若干","一些"}) {
    if(text.find(word)!=std::string::npos) {return 0;}
  }
  int quantity=1;bool seen=false;
  for(size_t i=0;i<text.size();) {
    size_t end=i;int value=0;bool number=false;
    if((text[i]>='0' && text[i]<='9') || text[i]=='-') {
      if(text[end]=='-') {++end;}
      while(end<text.size() && text[end]>='0' && text[end]<='9') {
        value=std::min(100,value*10+text[end]-'0');++end;number=true;
      }
      if(text[i]=='-') {value=0;}
    } else {
      for(const auto & entry:std::initializer_list<std::pair<std::string_view,int>>{
        {"零",0},{"一",1},{"二",2},{"两",2},{"三",3},{"四",4},{"五",5},{"六",6},{"七",7},{"八",8},{"九",9},{"十",10},{"百",100}}) {
        if(text.compare(i,entry.first.size(),entry.first)==0) {
          end=i+entry.first.size();value=entry.second;number=true;break;
        }
      }
    }
    if(!number) {++i;continue;}
    size_t unit=end;while(unit<text.size() && text[unit]==' ') {++unit;}
    size_t unit_size=0;
    for(const auto word:{"盒","个"}) {
      if(text.compare(unit,std::string_view(word).size(),word)==0) {unit_size=std::string_view(word).size();break;}
    }
    if(unit_size) {
      if(seen || value<1 || value>3) {return 0;}
      // Reject composite Chinese numbers, decimals and ranges instead of reading only their last digit.
      if(i>0 && text[i-1]!=' ' && (text[i-1]=='.' || text[i-1]=='~' ||
        static_cast<unsigned char>(text[i-1])>=128)) {
        for(const auto prefix:{"十","百","零","一","二","两","三","四","五","六","七","八","九","点","到"}) {
          const size_t n=std::string_view(prefix).size();
          if(i>=n && text.compare(i-n,n,prefix)==0) {return 0;}
        }
        if(text[i-1]=='.' || text[i-1]=='~') {return 0;}
      }
      seen=true;quantity=value;text.erase(i,unit+unit_size-i);
    } else {i=end;}
  }
  return quantity;
}
}
int resolve_fetch_quantity(const std::string & text)
{
  auto copy=text;return extract_fetch_quantity(copy);
}

VoiceCommand parse_voice_command(
  const std::string & text, const std::string & address_word,
  const std::string & navigate_word, const std::string & cancel_navigation_word,
  const std::vector<VoiceDrug> & drugs)
{
  std::string command = trim_ascii_space(text);
  constexpr std::array<std::string_view, 6> punctuation{{"。", ".", "！", "!", "？", "?"}};
  for (const auto mark : punctuation) {
    if (command.size() >= mark.size() &&
      command.compare(command.size() - mark.size(), mark.size(), mark) == 0)
    {
      command.erase(command.size() - mark.size());
      command = trim_ascii_space(command);
      break;
    }
  }
  std::string survey=command;
  for(const auto prefix:{"帮我","请"}) {
    if(starts_with(survey,prefix)) {survey=trim_ascii_space(survey.substr(std::string_view(prefix).size()));break;}
  }
  // Match complete, finite templates; never dispatch on a keyword substring.
  for(const auto verb:{"记录","统计","盘点"}) {
    if(!starts_with(survey,verb)) {continue;}
    std::string name=trim_ascii_space(survey.substr(std::string_view(verb).size()));
    if(starts_with(name,"一下")) {name=trim_ascii_space(name.substr(std::string_view("一下").size()));}
    bool quantity_suffix=false;
    for(const auto suffix:{"的数量","的数目","数量","数目"}) {
      const auto n=std::string_view(suffix).size();
      if(name.size()>=n && name.compare(name.size()-n,n,suffix)==0) {
        name=trim_ascii_space(name.substr(0,name.size()-n));quantity_suffix=true;break;
      }
    }
    if(quantity_suffix || std::string_view(verb)=="盘点") {
      if(const auto * canonical=canonical_drug(name,drugs)) {
        return {VoiceCommandType::Survey,"",*canonical};
      }
    }
  }
  for(const auto whole:{"熟悉仓库","熟悉一下仓库","盘点仓库","盘点一下仓库",
      "统计药品数量","统计仓库药品数量","统计仓库药品数目","记录仓库药品数目",
      "记录仓库药品数量","统计一下仓库里有多少药"}) {
    if(survey==whole) {return {VoiceCommandType::Survey,""};}
  }
  if (command == "取消任务") {
    return {VoiceCommandType::CancelTask, ""};
  }
  if (starts_with(command, "帮我拿")) {
    std::string name = trim_ascii_space(command.substr(std::string_view("帮我拿").size()));
    const int quantity=extract_fetch_quantity(name);
    if(quantity==0) {return {};}
    name=trim_ascii_space(name);
    const auto * matched_name=canonical_drug(name,drugs);
    if (matched_name && !name.empty()) {
      return {VoiceCommandType::Fetch, "", *matched_name,quantity};
    }
    return {};
  }
  if (!starts_with(command, address_word)) {
    return {};
  }
  command.erase(0, address_word.size());
  if (starts_with(command, "，")) {
    command.erase(0, std::string_view("，").size());
  } else if (starts_with(command, ",")) {
    command.erase(0, 1);
  }
  if (command == cancel_navigation_word) {
    return {VoiceCommandType::Cancel, ""};
  }
  if (starts_with(command, navigate_word)) {
    const auto task_name = trim_ascii_space(command.substr(navigate_word.size()));
    if (!task_name.empty()) {
      return {VoiceCommandType::Navigate, task_name};
    }
  }
  return {};
}

std::shared_ptr<rclcpp::Node> make_voice_command_node(
  const rclcpp::NodeOptions & options, CloudTransport transport,
  std::function<void(const std::string &, const Json &)> restart,
  std::string prepare_work, Json catalog)
{
  return std::make_shared<VoiceCommandNode>(options, std::move(transport), std::move(restart),
    std::move(prepare_work),std::move(catalog));
}

}  // namespace k1muse_manager
