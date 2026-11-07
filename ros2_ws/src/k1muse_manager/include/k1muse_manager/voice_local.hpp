#ifndef K1MUSE_MANAGER__VOICE_LOCAL_HPP_
#define K1MUSE_MANAGER__VOICE_LOCAL_HPP_

#include <atomic>
#include <memory>
#include <string>

namespace k1muse_manager
{
// Keep vendor/JSON types inside the implementation: common uses JSON 3.12 ABI.
struct LocalConfig
{
  std::string model_path;
  int threads{4}, context_size{1024}, batch_size{128}, ubatch_size{128};
  bool mmap{false};
  std::string flash_attention{"auto"};
  int poll_level{0}, max_tokens{96}, startup_timeout_ms{60000}, request_timeout_ms{10000};
};

struct LocalReply
{
  std::string body;
  std::string error;
};

// Owned and called only by the existing intent worker, including destruction.
class LocalLlm
{
public:
  LocalLlm();
  ~LocalLlm();
  LocalLlm(const LocalLlm &) = delete;
  LocalLlm & operator=(const LocalLlm &) = delete;
  LocalReply infer(const LocalConfig & config, const std::string & system,
    const std::string & text, const std::string & schema,
    const std::atomic_bool & canceled, bool warmup);
  void close();
  static bool available();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace k1muse_manager
#endif  // K1MUSE_MANAGER__VOICE_LOCAL_HPP_
