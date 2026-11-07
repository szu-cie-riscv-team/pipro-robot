#include "k1muse_voice/voice_kws.hpp"

#include <stdexcept>
#include "sherpa-onnx/csrc/keyword-spotter.h"
#include "sherpa-onnx/csrc/online-stream.h"

namespace k1muse_voice
{
VoiceKws::VoiceKws(const KwsConfig & config)
{
  sherpa_onnx::KeywordSpotterConfig options;
  options.model_config.transducer.encoder = config.encoder;
  options.model_config.transducer.decoder = config.decoder;
  options.model_config.transducer.joiner = config.joiner;
  options.model_config.tokens = config.tokens;
  options.model_config.provider_config.provider = "cpu";
  options.model_config.num_threads = 1;
  options.keywords_file = config.keywords;
  options.keywords_threshold = static_cast<float>(config.threshold);
  options.keywords_score = 1.0F;
  options.max_active_paths = 4;
  options.num_trailing_blanks = 1;
  if (!options.Validate()) {throw std::runtime_error("invalid KWS configuration/model files");}
  spotter_ = std::make_unique<sherpa_onnx::KeywordSpotter>(options);
}
VoiceKws::~VoiceKws() = default;
void VoiceKws::begin() {stream_ = spotter_->CreateStream();}
void VoiceKws::stop() {stream_.reset();}
std::string VoiceKws::detect(const float * samples, std::size_t count)
{
  stream_->AcceptWaveform(16000, samples, count);
  while (spotter_->IsReady(stream_.get())) {
    spotter_->DecodeStream(stream_.get());
    auto result = spotter_->GetResult(stream_.get());
    if (!result.keyword.empty()) {return result.keyword;}
  }
  return {};
}
}  // namespace k1muse_voice
