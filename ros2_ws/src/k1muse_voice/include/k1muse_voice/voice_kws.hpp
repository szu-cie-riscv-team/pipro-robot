#ifndef K1MUSE_VOICE__VOICE_KWS_HPP_
#define K1MUSE_VOICE__VOICE_KWS_HPP_

#include <cstddef>
#include <memory>
#include <string>

namespace sherpa_onnx {class KeywordSpotter; class OnlineStream;}

namespace k1muse_voice
{
struct KwsConfig
{
  std::string encoder, decoder, joiner, tokens, keywords;
  double threshold{0.25};
};

// Called synchronously by the existing capture worker. No audio ownership/thread.
class VoiceKws
{
public:
  explicit VoiceKws(const KwsConfig & config);
  ~VoiceKws();
  void begin();
  void stop();
  std::string detect(const float * samples, std::size_t count);
private:
  std::unique_ptr<sherpa_onnx::KeywordSpotter> spotter_;
  std::unique_ptr<sherpa_onnx::OnlineStream> stream_;
};
}  // namespace k1muse_voice
#endif  // K1MUSE_VOICE__VOICE_KWS_HPP_
