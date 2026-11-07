#include "k1muse_manager/voice_intent.hpp"

#include <curl/curl.h>

#include <chrono>
#include <cstdint>
#include <condition_variable>
#include <csignal>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>


namespace k1muse_manager
{
namespace
{
struct Receive
{
  const std::atomic_bool & canceled;
  std::string body;
  bool too_large{false};
};
std::size_t receive(char * data, std::size_t size, std::size_t count, void * context)
{
  auto & reply = *static_cast<Receive *>(context);
  if (size && count > (65536 - reply.body.size()) / size) {
    reply.too_large = true; return 0;
  }
  reply.body.append(data, size * count);
  return size * count;
}
int progress(void * context, curl_off_t, curl_off_t, curl_off_t, curl_off_t)
{
  return static_cast<Receive *>(context)->canceled.load(std::memory_order_relaxed) ? 1 : 0;
}
}  // namespace

bool CloudHttpClient::initialize()
{
  static const bool initialized = [] {
      std::signal(SIGPIPE, SIG_IGN);
      if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {return false;}
      return (curl_version_info(CURLVERSION_NOW)->features & CURL_VERSION_ASYNCHDNS) != 0;
    }();
  return initialized;
}

CloudHttpClient::CloudHttpClient(std::string ca_file)
: ca_file_(std::move(ca_file))
{
  handle_ = curl_easy_init();
  if (!handle_) {throw std::runtime_error("cloud_backend_unavailable");}
}

CloudHttpClient::~CloudHttpClient()
{
  if (handle_) {curl_easy_cleanup(static_cast<CURL *>(handle_));}
}

CloudReply CloudHttpClient::perform(const CloudJob & job, const std::atomic_bool & canceled)
{
  auto * curl = static_cast<CURL *>(handle_);
  curl_easy_reset(curl);  // Keeps curl's existing connection cache.
  std::string url = job.config.base_url;
  while (!url.empty() && url.back() == '/') {url.pop_back();}
  url += "/chat/completions";
  const auto body = cloud_request_json(job).dump();
  const std::string authorization = "Authorization: Bearer " + job.config.api_key;
  curl_slist * headers = curl_slist_append(nullptr, "Content-Type: application/json");
  {
    auto * with_auth = curl_slist_append(headers, authorization.c_str());
    if (!with_auth) {curl_slist_free_all(headers); throw std::runtime_error("network_error");}
    headers = with_auth;
  }
  Receive received{canceled, {}, false};
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS,
    static_cast<long>(job.config.connect_timeout_ms));
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS,
    static_cast<long>(job.config.request_timeout_ms));
  curl_easy_setopt(curl, CURLOPT_MAXCONNECTS, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
  if (!ca_file_.empty()) {curl_easy_setopt(curl, CURLOPT_CAINFO, ca_file_.c_str());}
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, receive);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &received);
  curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, progress);
  curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &received);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
  const auto code = curl_easy_perform(curl);
  CloudReply reply;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &reply.http_status);
  // Unhook stack-owned pointers before returning or processing another job.
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, nullptr);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, nullptr);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
  curl_easy_setopt(curl, CURLOPT_XFERINFODATA, nullptr);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, nullptr);
  curl_slist_free_all(headers);
  reply.body = std::move(received.body);
  if (canceled.load()) {reply.error = "canceled";}
  else if (received.too_large) {reply.error = "response_too_large";}
  else if (code == CURLE_OPERATION_TIMEDOUT) {reply.error = "timeout";}
  else if (code != CURLE_OK) {reply.error = "network_error";}
  return reply;
}

struct VoiceCloudWorker::Impl
{
  explicit Impl(CloudTransport value) : transport(std::move(value)) {}
  CloudTransport transport;
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::thread thread;
  std::atomic_bool canceled{false};
  bool stopping{false}, occupied{false}, release{false};
  std::optional<CloudJob> job;
  std::optional<CloudResult> result;

  void run()
  {
    std::unique_ptr<CloudHttpClient> client;
    LocalLlm local;
    for (;;) {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [this] {return stopping || job.has_value() || release;});
      if (stopping) {break;}
      if (release) {release = false; lock.unlock(); local.close(); client.reset(); lock.lock();}
      if (!job) {continue;}
      auto current = std::move(*job); job.reset();
      lock.unlock();
      CloudResult completed; completed.job = std::move(current);
      const auto begin = std::chrono::steady_clock::now();
      try {
        if (canceled.load()) {completed.reply.error = "canceled";}
        else if (transport) {completed.reply = transport(completed.job, canceled);}
        else if (completed.job.backend == "local") {
          auto reply = local.infer(completed.job.local, completed.job.system_prompt,
            completed.job.text, voice_intent_schema(), canceled, completed.job.probe);
          completed.reply.body = std::move(reply.body);
          completed.reply.error = std::move(reply.error);
        } else {
          if (!client) {client = std::make_unique<CloudHttpClient>();}
          completed.reply = client->perform(completed.job, canceled);
        }
      } catch (...) {completed.reply.error = completed.job.backend == "local" ?
          "local_backend_unavailable" : "network_error";}
      completed.duration_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
      if (completed.job.backend == "local") {
        if (canceled.load()) {completed.reply.error = "canceled";}
        if (completed.reply.error.empty()) {
          try {
            completed.intent = parse_voice_intent(completed.reply.body);
            if (completed.job.probe && completed.intent->action != "unknown") {
              completed.reply.error = "test_output_mismatch";
            }
          } catch (...) {completed.reply.error = "invalid_intent_format";}
        }
        // Shape/probe failures must also release model objects before publishing
        // a result that may hand the permit to another model or rule fallback.
        if (!completed.reply.error.empty()) {local.close();}
      }
      lock.lock();
      // Keep occupied until the ROS executor consumes this result.
      result = std::move(completed);
    }
  }
};

VoiceCloudWorker::VoiceCloudWorker(CloudTransport transport)
: impl_(std::make_unique<Impl>(std::move(transport))) {}
VoiceCloudWorker::~VoiceCloudWorker()
{
  stop();
}
void VoiceCloudWorker::stop()
{
  {std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stopping = true; impl_->canceled.store(true);}
  impl_->cv.notify_one();
  if (impl_->thread.joinable()) {impl_->thread.join();}
}
bool VoiceCloudWorker::start(CloudJob job)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->occupied || impl_->stopping) {return false;}
  if (!impl_->transport) {
    if (job.backend == "local") {
      if (!LocalLlm::available()) {return false;}
    } else if (!CloudHttpClient::initialize()) {return false;}
  }
  if (!impl_->thread.joinable()) {
    impl_->thread = std::thread([this] {impl_->run();});
  }
  impl_->canceled.store(false);
  impl_->occupied = true;
  impl_->job = std::move(job);
  impl_->cv.notify_one();
  return true;
}
void VoiceCloudWorker::cancel(bool release_transport)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->canceled.store(true);
  impl_->release = impl_->release || release_transport;
  impl_->cv.notify_one();
}
std::optional<CloudResult> VoiceCloudWorker::take_result()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->result) {return std::nullopt;}
  auto result = std::move(impl_->result);
  impl_->result.reset(); impl_->occupied = false;
  return result;
}
bool VoiceCloudWorker::busy() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->occupied;
}
}  // namespace k1muse_manager
