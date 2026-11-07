#pragma once

#include <k1muse_interfaces/srv/web_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nlohmann/json.hpp>

namespace k1muse_manager
{
// Owned by task_manager and used only from its default callback group.
class InferenceControl
{
public:
  explicit InferenceControl(rclcpp::Node & node);
  void set_drug_catalog(nlohmann::json catalog) {drug_catalog_=std::move(catalog);}
  bool idle() const {return (phase_ == "idle" || phase_ == "wake_wait") && work_id_.empty();}
  bool navigation_available() const {return phase_ == "starting" || idle();}
  bool owns_scan(const std::string & work) const {return cv_work_ == work;}
  void on_idle(std::function<void()> callback) {idle_callback_ = std::move(callback);}
  void on_vision_failure(std::function<void(const std::string &)> callback)
  {vision_failure_ = std::move(callback);}
  void business(bool active);
  void scan(bool enabled, const std::string & work, std::function<void(bool)> done);


private:
  using Control = k1muse_interfaces::srv::WebRequest;
  using Listening = std_srvs::srv::SetBool;
  using Json = nlohmann::json;
  using Header = std::shared_ptr<rmw_request_id_t>;
  void control(Header header, Control::Request::SharedPtr request);
  void reply(Header header, const Control::Request & request,
    const std::string & error = {}, Json result = {{"status", "succeeded"}});
  void publish();
  void block(const std::string & error);
  void prepare(const std::string & owner);
  void initialize(bool voice, bool yolo);
  void after_prepare(const std::string & owner);
  void arm_prepare_deadline();
  void resume();
  void finish();
  void call(rclcpp::Client<Control>::SharedPtr client, const std::string & operation,
    Json args, std::function<void(Control::Response::SharedPtr)> done);
  void listening(bool enabled, std::function<void(bool)> done);
  void set_bool(rclcpp::Client<Listening>::SharedPtr client, bool enabled, std::function<void(bool)> done);
  void cancel_deadline();

  rclcpp::Node & node_;
  rclcpp::Service<Control>::SharedPtr control_;
  rclcpp::Client<Control>::SharedPtr input_, intent_, vision_;
  rclcpp::Client<Listening>::SharedPtr listening_, cv_;
  std::function<void()> idle_callback_;
  std::function<void(const std::string &)> vision_failure_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr deadline_, prepare_deadline_, startup_wait_, cv_deadline_;
  std::string phase_{"starting"}, owner_, work_id_, completed_, error_, published_;
  std::string instance_;
  uint64_t sequence_{0}, rpc_{0}, cv_rpc_{0};
  std::string cv_work_;
  bool cv_pending_{false}, cv_failed_{false};
  bool input_enabled_{false}, business_{false};
  bool enable_voice_{false}, enable_yolo_{false}, voice_ready_{false}, yolo_ready_{false};
  Json drug_catalog_{{"drugs",Json::array()},{"error",""}};
  std::string blocked_phase_;
  std::chrono::milliseconds timeout_{500};
};
}  // namespace k1muse_manager
