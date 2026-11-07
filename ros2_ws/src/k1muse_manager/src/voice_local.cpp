#include "k1muse_manager/voice_local.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <vector>

#ifdef K1MUSE_HAS_LOCAL_LLM
#include <llama.h>
#include <ggml-cpu.h>
#include "chat.h"
#include "json-schema-to-grammar.h"
#include <nlohmann/json.hpp>
static_assert(NLOHMANN_JSON_VERSION_MAJOR == 3 && NLOHMANN_JSON_VERSION_MINOR == 12 &&
  NLOHMANN_JSON_VERSION_PATCH == 0, "SpaceMIT common requires the matching JSON 3.12 ABI");
#endif

namespace k1muse_manager
{
#ifdef K1MUSE_HAS_LOCAL_LLM
namespace
{
using Clock = std::chrono::steady_clock;

void check(bool value, const char * error)
{
  if (!value) {throw std::runtime_error(error);}
}

llama_token sample_greedy_with_grammar(llama_sampler * sampler, llama_context * context)
{
  // The chain is grammar -> greedy. Check the best candidate first, as in
  // upstream common_sampler_sample(grammar_first=false), then accept once.
  const auto * logits = llama_get_logits_ith(context, -1);
  const auto count = llama_vocab_n_tokens(llama_model_get_vocab(llama_get_model(context)));
  llama_token best = 0;
  for (llama_token token = 1; token < count; ++token) {
    if (logits[token] > logits[best]) {best = token;}
  }
  llama_token_data candidate{best, logits[best], 0.0f};
  llama_token_data_array single{&candidate, 1, -1, false};
  llama_sampler_apply(llama_sampler_chain_get(sampler, 0), &single);
  if (candidate.logit != -std::numeric_limits<float>::infinity()) {
    llama_sampler_accept(sampler, best);
    return best;
  }
  // The original full-vocabulary path filters and accepts the fallback token.
  return llama_sampler_sample(sampler, context, -1);
}

void backend_init()
{
  static std::once_flag initialized;
  std::call_once(initialized, [] {
    llama_log_set([](ggml_log_level level, const char * text, void *) {
      if (level >= GGML_LOG_LEVEL_WARN) {std::fputs(text, stderr);}
    }, nullptr);
    llama_backend_init();
  });
}
}  // namespace

struct LocalLlm::Impl
{
  llama_model * model{nullptr};
  llama_context * context{nullptr};
  llama_sampler * sampler{nullptr};
  ggml_threadpool_t pool{nullptr};
  llama_batch batch{};
  common_chat_templates_ptr templates;
  std::vector<llama_token> cached;
  const std::atomic_bool * canceled{nullptr};
  Clock::time_point deadline;

  ~Impl()
  {
    if (context) {
      llama_set_abort_callback(context, nullptr, nullptr);
      llama_synchronize(context);
    }
    if (sampler) {llama_sampler_free(sampler);}
    templates.reset();
    if (context) {
      llama_detach_threadpool(context);
      llama_free(context);
    }
    if (pool) {ggml_threadpool_free(pool);}
    if (batch.token) {llama_batch_free(batch);}
    if (model) {llama_model_free(model);}
    // The process-wide official runtime/backend is retained for ASR and YOLO.
  }

  bool aborted() const
  {
    return canceled->load(std::memory_order_relaxed) || Clock::now() >= deadline;
  }

  void check_deadline() const
  {
    check(!canceled->load(std::memory_order_relaxed), "canceled");
    check(Clock::now() < deadline, "timeout");
  }

  void prepare(const LocalConfig & config, const std::string & schema)
  {
    backend_init();
    check_deadline();
    auto params = llama_model_default_params();
    params.n_gpu_layers = 0;
    params.load_mode = config.mmap ? LLAMA_LOAD_MODE_MMAP : LLAMA_LOAD_MODE_NONE;
    params.progress_callback = [](float, void * data) {
      return !static_cast<Impl *>(data)->aborted();
    };
    params.progress_callback_user_data = this;
    model = llama_model_load_from_file(config.model_path.c_str(), params);
    check_deadline();
    check(model != nullptr, "local_model_load_failed");

    auto ctx = llama_context_default_params();
    ctx.n_ctx = config.context_size;
    ctx.n_batch = config.batch_size;
    ctx.n_ubatch = config.ubatch_size;
    ctx.n_seq_max = 1;
    ctx.n_threads = ctx.n_threads_batch = config.threads;
    ctx.flash_attn_type = config.flash_attention == "on" ? LLAMA_FLASH_ATTN_TYPE_ENABLED :
      config.flash_attention == "off" ? LLAMA_FLASH_ATTN_TYPE_DISABLED : LLAMA_FLASH_ATTN_TYPE_AUTO;
    ctx.offload_kqv = false;
    ctx.op_offload = false;
    ctx.no_perf = true;
    ctx.abort_callback = [](void * data) {return static_cast<Impl *>(data)->aborted();};
    ctx.abort_callback_data = this;
    context = llama_init_from_model(model, ctx);
    check_deadline();
    check(context != nullptr, "local_context_failed");

    auto threads = ggml_threadpool_params_default(config.threads);
    threads.poll = config.poll_level;
    for (int cpu = 0; cpu < 4; ++cpu) {threads.cpumask[cpu] = true;}
    pool = ggml_threadpool_new(&threads);
    check(pool != nullptr, "local_threadpool_failed");
    llama_attach_threadpool(context, pool, pool);
    batch = llama_batch_init(config.batch_size, 0, 1);
    check(batch.token != nullptr, "local_batch_failed");

    templates = common_chat_templates_init(model, "");
    check(templates != nullptr, "local_template_failed");
    const auto grammar = json_schema_to_grammar(nlohmann::ordered_json::parse(schema), true);
    auto * grammar_sampler = llama_sampler_init_grammar(
      llama_model_get_vocab(model), grammar.c_str(), "root");
    check(grammar_sampler != nullptr, "local_grammar_failed");
    auto sampling = llama_sampler_chain_default_params();
    sampling.no_perf = true;
    sampler = llama_sampler_chain_init(sampling);
    llama_sampler_chain_add(sampler, grammar_sampler);
    llama_sampler_chain_add(sampler, llama_sampler_init_greedy());
    check_deadline();
  }

  void decode(const llama_token * tokens, std::size_t count, int batch_size)
  {
    for (std::size_t offset = 0; offset < count;) {
      check_deadline();
      const auto n = std::min(count - offset, static_cast<std::size_t>(batch_size));
      batch.n_tokens = static_cast<int32_t>(n);
      for (std::size_t i = 0; i < n; ++i) {
        batch.token[i] = tokens[offset + i];
        batch.pos[i] = static_cast<llama_pos>(cached.size() + i);
        batch.n_seq_id[i] = 1;
        batch.seq_id[i][0] = 0;
        batch.logits[i] = offset + i + 1 == count;
      }
      const auto code = llama_decode(context, batch);
      if (code != 0) {
        check_deadline();
        throw std::runtime_error("local_decode_failed");
      }
      cached.insert(cached.end(), tokens + offset, tokens + offset + n);
      offset += n;
    }
  }

  std::string generate(const LocalConfig & config, const std::string & system,
    const std::string & text)
  {
    common_chat_templates_inputs chat;
    common_chat_msg system_msg;
    system_msg.role = "system"; system_msg.content = system;
    common_chat_msg user_msg;
    user_msg.role = "user"; user_msg.content = text;
    chat.messages = {std::move(system_msg), std::move(user_msg)};
    chat.use_jinja = true;
    chat.enable_thinking = false;
    chat.add_generation_prompt = true;
    const auto prompt = common_chat_templates_apply(templates.get(), chat).prompt;
    const auto * vocab = llama_model_get_vocab(model);
    auto size = llama_tokenize(vocab, prompt.data(), static_cast<int32_t>(prompt.size()),
      nullptr, 0, true, true);
    check(size < 0, "local_template_failed");
    std::vector<llama_token> tokens(static_cast<std::size_t>(-size));
    size = llama_tokenize(vocab, prompt.data(), static_cast<int32_t>(prompt.size()),
      tokens.data(), static_cast<int32_t>(tokens.size()), true, true);
    check(size > 0, "local_template_failed");
    tokens.resize(static_cast<std::size_t>(size));
    check(tokens.size() + config.max_tokens <= llama_n_ctx(context), "context_exceeded");

    std::size_t prefix = 0;
    // Re-decode at least the final input token, even for an identical command,
    // so logits never come from the previous answer's last generated token.
    while (prefix < cached.size() && prefix + 1 < tokens.size() &&
      cached[prefix] == tokens[prefix]) {++prefix;}
    check(llama_memory_seq_rm(llama_get_memory(context), 0,
      static_cast<llama_pos>(prefix), -1), "local_cache_failed");
    cached.resize(prefix);
    llama_sampler_reset(sampler);
    decode(tokens.data() + prefix, tokens.size() - prefix, config.batch_size);

    std::string result;
    std::vector<char> piece(256);
    for (int i = 0; i < config.max_tokens; ++i) {
      check_deadline();
      const auto token = sample_greedy_with_grammar(sampler, context);
      if (llama_vocab_is_eog(vocab, token)) {
        llama_synchronize(context);
        check_deadline();
        check(!result.empty(), "empty_response");
        return result;
      }
      auto n = llama_token_to_piece(vocab, token, piece.data(),
        static_cast<int32_t>(piece.size()), 0, false);
      if (n < 0) {
        piece.resize(static_cast<std::size_t>(-n));
        n = llama_token_to_piece(vocab, token, piece.data(),
          static_cast<int32_t>(piece.size()), 0, false);
      }
      check(n >= 0, "local_decode_failed");
      result.append(piece.data(), static_cast<std::size_t>(n));
      decode(&token, 1, config.batch_size);
    }
    throw std::runtime_error("truncated_response");
  }
};
#else
struct LocalLlm::Impl {};
#endif

LocalLlm::LocalLlm() = default;
LocalLlm::~LocalLlm() = default;
void LocalLlm::close() {impl_.reset();}
bool LocalLlm::available()
{
#ifdef K1MUSE_HAS_LOCAL_LLM
  return true;
#else
  return false;
#endif
}

LocalReply LocalLlm::infer(const LocalConfig & config, const std::string & system,
  const std::string & text, const std::string & schema,
  const std::atomic_bool & canceled, bool warmup)
{
#ifdef K1MUSE_HAS_LOCAL_LLM
  if (!impl_) {impl_ = std::make_unique<Impl>();}
  impl_->canceled = &canceled;
  impl_->deadline = Clock::now() + std::chrono::milliseconds(
    warmup ? config.startup_timeout_ms : config.request_timeout_ms);
  try {
    impl_->check_deadline();
    if (!impl_->model) {impl_->prepare(config, schema);}
    return {impl_->generate(config, system, text), ""};
  } catch (const std::exception & error) {
    const std::string reason = error.what();
    close();  // Synchronizes before the result can release the manager permit.
    return {{}, reason};
  }
#else
  (void)config; (void)system; (void)text; (void)schema; (void)canceled; (void)warmup;
  return {{}, "local_backend_unavailable"};
#endif
}
}  // namespace k1muse_manager
