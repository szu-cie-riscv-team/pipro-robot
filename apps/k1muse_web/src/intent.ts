import { WarehouseOperation, WarehouseRequest } from './warehouse_operation';
// This page owns only browser drafts and the small, on-demand intent status stream.
export const intentHTML = `
  <div class="intent-layout"><aside class="panel intent-sidebar"><h2>意图识别模式</h2>
    <div class="intent-tabs"><button data-intent="cloud">云端推理</button>
      <button data-intent="local">本地推理</button><button data-intent="rules">正则匹配</button></div>
    <label class="intent-mobile">选择方案<select id="intent-select"><option value="cloud">云端推理</option>
      <option value="local">本地推理</option><option value="rules">正则匹配</option></select></label>
    <p class="hint">面板用于查看和设置；实际模式以板端状态为准。首次未配置时使用正则匹配；此后按板端已保存的模式启动。</p></aside>
  <div class="intent-main"><div class="panel"><div class="intent-heading"><h2>板端运行状态</h2>
    <div class="intent-restart-actions"><button id="intent-reload">重新读取配置</button><button id="intent-restart">重启节点</button></div></div><pre id="intent-status">连接机器人后读取</pre>
    <p class="hint">重启仅重新加载意图识别及其配置；保留语音识别和视觉模型。准备期间暂停接收语音。</p>
    <p id="intent-message" role="status" aria-live="polite"></p></div>
  <div id="intent-cloud" hidden><div class="panel"><h2>云端 API</h2>
    <div class="intent-heading"><label class="check"><input id="intent-enabled" type="checkbox">使用云端 API（重启生效）</label>
      <button id="intent-check">检测已保存配置</button></div>
    <p class="hint">由板端直接调用。保存只影响下次启动，重启后检测；运行失败退回正则匹配，显式重启恢复。</p>
    <fieldset id="intent-api-fields"><legend>连接配置 <span id="intent-api-dirty"></span></legend>
      <label>API 类型<select id="intent-api_type"><option value="deepseek">DeepSeek</option>
        <option value="openai_compatible">OpenAI-compatible</option></select></label>
      <label>Base URL<input id="intent-base_url" type="url" placeholder="https://api.deepseek.com" autocomplete="off"></label>
      <label>模型 ID<input id="intent-model" placeholder="deepseek-flash" autocomplete="off"></label>
      <p class="hint">DeepSeek V4.1 Flash 预设模型 ID：deepseek-flash。板端追加 /chat/completions。</p>
      <label>API Key <span id="intent-key-state"></span><input id="intent-api_key" type="password"
        autocomplete="new-password" placeholder="留空保留板端已有 key"></label>
      <div class="buttons"><button id="intent-clear-key" type="button">清除已有 Key</button></div>
      <p id="intent-key-hint" class="hint">已有 Key 不回显；新输入只保留在当前页面内存。</p>
      <details><summary>请求参数</summary><div class="intent-numbers">
        <label>连接超时（ms）<input id="intent-connect_timeout_ms" type="number" min="1000" max="5000" step="1"></label>
        <label>总超时（ms）<input id="intent-request_timeout_ms" type="number" min="1000" max="30000" step="1"></label>
        <label>最大输出 token<input id="intent-max_tokens" type="number" min="64" max="1024" step="1"></label>
      </div></details><button id="intent-save-api" class="primary wide">保存 API 配置</button></fieldset>
    <fieldset id="intent-prompt-fields"><legend>系统提示词 <span id="intent-prompt-dirty"></span></legend>
      <label for="intent-prompt">基础 system_prompt</label><textarea id="intent-prompt" rows="9" spellcheck="false"></textarea>
      <p class="hint">独立保存到 cloud_prompt.yaml。程序附加统一 JSON 契约和药名清单；保存后重启生效，不自动检测 API。</p>
      <button id="intent-save-prompt" class="primary wide">保存系统提示词</button></fieldset>
  </div></div>
  <div id="intent-local" class="panel" hidden><h2>本地推理</h2><label class="check"><input id="intent-local-enabled" type="checkbox">使用本地模式（重启生效）</label><p id="intent-local-state">连接机器人后读取本地配置与准备状态。</p>
    <p class="hint">保存后点击“重启节点”加载模型，准备完成后才使用本地推理；空闲时模型常驻。模型处理期间不接收新语音，结束后恢复。</p>
    <p class="hint">模型参数和系统提示词首版在板端 local_llm.yaml、local_prompt.yaml 中修改，重启后加载。</p></div>
  <div id="intent-rules" class="panel"><h2>正则匹配</h2><button id="intent-use-rules">设为下次启动模式</button>
    <p>先说“小赛”，停顿后说一条口令；每次口令需重新唤醒。</p>
    <p>支持“帮我拿〈药名〉”“帮我拿一盒〈药名〉”，每次取一件。</p>
    <p id="intent-drugs">运行药品目录尚未读取。</p><button data-page="warehouse">到仓库管理编辑药品目录</button>
    <p>“取消任务”请求取消完整任务；“小赛取消导航”仅取消语音发起的导航。</p>
    <p class="hint">“熟悉仓库”“记录仓库药品数目”执行整仓盘点；“记录〈药名〉的数量”盘点该药全部配置位置。
      模糊、否定、多药和多件请求不取药。导航和取药完成状态由业务端确认。</p></div>
  <div class="panel"><h2>最近意图</h2><p id="intent-result-meta">暂无真实口令结果</p><pre id="intent-result">暂无结果</pre>
    <p class="hint">仅显示结构化识别结果，不代表任务已执行或完成。API 检测不覆盖此项。</p></div>
  </div></div>`;

type Request = WarehouseRequest;
const names: Record<string, string> = { rules: "正则匹配", cloud: "云端推理", local: "本地推理" };
const phases: Record<string, string> = {
  starting: "等待启动", preparing: "模型准备", wake_wait: "等待唤醒", speech: "已唤醒，等待本句", pausing: "暂停语音中",
  asr: "语音识别", intent: "意图识别", vision: "视觉识别", restarting: "重建意图节点",
  resuming: "恢复语音中", idle: "推理空闲", blocked: "等待处理", unavailable: "不可用"
};
const availability: Record<string, string> = {
  unchecked: "尚未检测", checking: "检测中", available: "最近一次通过", unavailable: "最近调用失败"
};
const localStates: Record<string, string> = {
  unloaded: "未加载", loading: "加载与预热中", ready: "已就绪", stopping: "释放模型中", unavailable: "不可用"
};
const fields = ["api_type", "base_url", "model", "connect_timeout_ms", "request_timeout_ms", "max_tokens"];

export class IntentPanel {
  private state: any;
  private connected = false;
  private loaded = false;
  private stateVersion = 0;
  private active = false;
  private epoch = 0;
  private pending = false;
  private selected = "rules";
  private apiDirty = false;
  private promptDirty = false;
  private apiRevision = 0;
  private promptRevision = 0;
  private clearKey = false;
  private loadId = 0;
  private saved: any;
  private files: WarehouseOperation;
  private checkId?: string;
  private loading = false;
  private instanceReloadNeeded = false;
  private restartFrom?: string;
  private restartAccepted = false;
  private retiredInstances = new Set<string>();

  constructor(private root: HTMLElement, private request: Request) {
    this.files = new WarehouseOperation(request);
    for (const b of root.querySelectorAll<HTMLButtonElement>("[data-intent]"))
      b.onclick = () => void this.select(b.dataset.intent!);
    this.field("select").onchange = () => void this.select(this.field("select").value);
    for (const f of [...fields, "api_key"]) this.field(f).addEventListener("input", () => {
      this.apiDirty = true; ++this.apiRevision;
      if (f === "api_key") this.clearKey = false;
      this.render();
    });
    this.field("prompt").addEventListener("input", () => {
      this.promptDirty = true; ++this.promptRevision; this.render();
    });
    this.button("clear-key").onclick = () => {
      this.clearKey = true; this.field("api_key").value = "";
      this.apiDirty = true; ++this.apiRevision; this.render();
    };
    this.button("reload").onclick = () => void this.load();
    this.button("restart").onclick = () => void this.restart();
    this.button("use-rules").onclick = () => void this.saveMode("rules");
    this.field("local-enabled").onchange = () => {
      const enabled = (this.field("local-enabled") as HTMLInputElement).checked;
      this.render(); void this.saveMode(enabled ? "local" : this.saved?.startup_mode === "local" ? "rules" : this.saved?.startup_mode || "rules");
    };
    this.button("save-api").onclick = () => void this.saveAPI();
    this.button("save-prompt").onclick = () => void this.savePrompt();
    this.field("enabled").onchange = () => {
      const enabled = (this.field("enabled") as HTMLInputElement).checked;
      this.render(); void this.saveMode(enabled ? "cloud" : this.saved?.startup_mode === "cloud" ? "rules" : this.saved?.startup_mode || "rules");
    };
    this.button("check").onclick = () => void this.run("voice_intent_check_cloud");
    this.render();
  }
  private element(name: string) { return this.root.querySelector<HTMLElement>("#intent-" + name)!; }
  private field(name: string) { return this.element(name) as HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement; }
  private button(name: string) { return this.element(name) as HTMLButtonElement; }
  private message(text: string, error = false) {
    this.element("message").textContent = text;
    this.element("message").classList.toggle("error", error);
  }
  setConnected(value: boolean) {
    ++this.epoch; ++this.loadId; this.connected = value; this.loaded = false;
    this.state = undefined; this.saved = undefined; this.checkId = undefined; ++this.stateVersion; this.pending = false;
    this.restartFrom = undefined; this.restartAccepted = false; this.retiredInstances.clear();
    this.loading = false; this.instanceReloadNeeded = false;
    this.message(value ? "正在读取板端配置…" : "连接机器人后可配置；未确认的修改需重新读回核对。");
    this.render();
  }
  enter() {
    this.active = true;
    if (this.pending || this.loading) this.instanceReloadNeeded = true;
    else { this.instanceReloadNeeded = false; void this.load(true); }
  }
  leave() {
    this.active = false;
    if ((this.apiDirty || this.promptDirty) && !confirm("有未保存的意图配置。确定：保留草稿；取消：丢弃草稿。")) {
      this.apiDirty = this.promptDirty = this.clearKey = false;
      this.field("api_key").value = "";
    }
  }
  status(message: { data: string }) {
    if (!this.connected || !this.active) return;
    try {
      const next = JSON.parse(message.data);
      if (this.retiredInstances.has(next.instance_id)) return;
      ++this.stateVersion;
      this.applyState(next);
      this.render();
    } catch { this.message("板端意图状态格式错误", true); }
  }
  private applyState(next: any) {
    const old = this.state?.instance_id;
    const wasRestarting = this.state?.restarting;
    const changed = !!(old && next.instance_id && old !== next.instance_id);
    if (changed) this.retiredInstances.add(old);
    this.state = next;
    if (this.restartFrom && next.instance_id && next.instance_id !== this.restartFrom) {
      if (next.preparing) this.message("意图节点已重建，正在加载配置、准备模型…");
      else if (["wake_wait", "idle"].includes(next.inference?.phase) && !next.restarting) {
        this.restartFrom = undefined;
        const failed = (next.active_startup_mode ?? next.startup_mode) !== next.mode;
        this.message(failed ? `准备失败，当前使用${names[next.mode] || next.mode}；请查看板端原因。` :
          `重启完成，当前使用${names[next.mode] || next.mode}。`, failed);
      } else this.message("意图节点已重建，等待板端恢复语音入口。");
    } else if (this.restartFrom && !this.restartAccepted && wasRestarting && !next.restarting) {
      this.restartFrom = undefined; this.message("重启未执行，原实例继续运行；可查看板端原因。", true);
    }
    return changed;
  }
  private reloadNewInstance() {
    if (!this.active || !this.connected || !this.instanceReloadNeeded || this.pending || this.loading) return;
    this.instanceReloadNeeded = false; void this.load();
  }
  async load(selectActual = false) {
    if (!this.active || !this.connected || this.loading) return;
    this.loading = true; this.render();
    const epoch = this.epoch, loadId = ++this.loadId;
    try {
      if (this.files.pending) await this.files.resume();
      const result = await this.files.start("voice_intent_get", {});
      if (epoch !== this.epoch || loadId !== this.loadId) return;
      this.applySaved(result);
      this.loaded = true;
      if (this.element("message").textContent === "正在读取板端配置…") this.message("已读取板端配置。");
      if (selectActual) this.selected = this.saved.startup_mode || "rules";
      this.render();
      return true;
    } catch (e) { if (epoch === this.epoch && loadId === this.loadId) this.message(String(e), true); }
    finally { if (epoch === this.epoch) { this.loading = false; this.render(); this.reloadNewInstance(); } }
  }
  private applySaved(result: any) {
    this.saved = result;
    if (!this.apiDirty) for (const f of fields) this.field(f).value = String(result.cloud[f]);
    if (!this.promptDirty) this.field("prompt").value = result.system_prompt;
    const errors = Object.values(result.config_errors || {}).filter(Boolean);
    if (errors.length) this.message("部分保存配置无效，可编辑后重新保存：" + errors.join("、"), true);
    this.element('drugs').textContent = '药品目录与运行内容在仓库管理中查看；保存目录后需重启相关运行组加载。';
  }
  private async run(operation: string, args: unknown = {}, saved?: () => void) {
    if (!this.active || !this.connected || this.pending || this.loading || !this.loaded) return;
    if (this.files.pending) { this.message("上次配置操作结果未知，请点重新读取查询原结果。", true); return; }
    const epoch = this.epoch, instance = this.state?.instance_id, version = this.stateVersion;
    this.pending = true; this.render();
    try {
      if (saved) {
        const result = await this.files.start(operation, args);
        if (epoch !== this.epoch) return;
        if (result.saved !== true) throw new Error("保存结果未知，请重新读取配置核对");
        saved(); this.checkId = undefined; this.applySaved(result);
        this.message("已保存，下次启动生效。");
      } else {
        const { result } = await this.request(operation, args);
        if (epoch !== this.epoch || instance !== this.state?.instance_id) return;
        if (result.state && version === this.stateVersion) this.applyState(result.state);
        this.checkId = result.state?.last_request?.id;
        this.message(result.status === "running" ? "板端检测已受理，结果仅对应本次检查。" : "板端已确认操作。");
      }
    } catch (e) {
      if (epoch !== this.epoch) return;
      this.message(this.files.pending ? "操作结果未知，请重新读取查询原结果；不要重复保存。" : String(e), true);
    } finally { if (epoch === this.epoch) { this.pending = false; this.render(); this.reloadNewInstance(); } }
  }
  private select(mode: string) {
    this.selected = mode; this.render();
  }
  private async saveMode(mode: string) {
    await this.run("voice_intent_set_mode", { mode }, () => {});
  }
  private async restart() {
    if (!this.active || !this.connected || !this.loaded || this.pending || this.loading || this.files.pending || this.restartFrom ||
      this.state?.business_busy || this.state?.inference?.restart_allowed !== true) return;
    if (this.apiDirty || this.promptDirty) {
      this.message("请先保存或丢弃未保存的配置，再重启节点。", true); return;
    }
    const epoch = this.epoch, old = this.state?.instance_id, version = this.stateVersion;
    this.pending = true; this.restartFrom = old; this.restartAccepted = false; this.render();
    try {
      const { result } = await this.request("voice_intent_restart");
      if (epoch !== this.epoch) return;
      if (result.status !== "running") throw new Error("重启未受理");
      if (this.state?.instance_id === old) {
        this.restartFrom = old; this.restartAccepted = true;
        if (result.state && version === this.stateVersion) this.applyState(result.state);
        this.message("重启已受理，正在收尾旧请求、释放意图模型…");
      }
    } catch (e) {
      if (epoch !== this.epoch) return;
      if (this.state?.instance_id !== old) return;
      const text = String(e);
      const unknown = text.includes("timeout") || text.includes("结果未知") || text.includes("断开");
      if (unknown) {
        this.message("重启结果未知，正在读回状态；请勿重复提交。", true);
        // File readback cannot prove a runtime restart did not happen. Wait for status.
      } else { this.restartFrom = undefined; this.message(text, true); }
    } finally { if (epoch === this.epoch) { this.pending = false; this.render(); this.reloadNewInstance(); } }
  }
  private async saveAPI() {
    const data: Record<string, string | number> = {};
    for (const f of fields) {
      const field = this.field(f) as HTMLInputElement;
      if (!field.reportValidity()) return;
      data[f] = f.endsWith("_ms") || f === "max_tokens" ? Number(field.value) : field.value;
    }
    if (Number(data.connect_timeout_ms) > Number(data.request_timeout_ms)) {
      this.message("总超时不能小于连接超时。", true); return;
    }
    const key = this.field("api_key").value;
    if (this.clearKey) data.api_key = ""; else if (key) data.api_key = key;
    const revision = this.apiRevision;
    await this.run("voice_intent_save_cloud", data, () => {
      if (revision !== this.apiRevision) return;
      this.apiDirty = false; this.clearKey = false; this.field("api_key").value = "";
    });
  }
  private async savePrompt() {
    const system_prompt = this.field("prompt").value, revision = this.promptRevision;
    await this.run("voice_intent_save_prompt", { system_prompt }, () => {
      if (revision === this.promptRevision) this.promptDirty = false;
    });
  }
  private render() {
    for (const mode of Object.keys(names)) {
      this.element(mode).hidden = this.selected !== mode;
      this.root.querySelector(`[data-intent="${mode}"]`)!.classList.toggle("active", this.selected === mode);
    }
    this.field("select").value = this.selected;
    const s = this.state;
    const modelName = s?.mode === "local" || ["loading", "stopping"].includes(s?.local?.status) ?
      s.local?.model?.split("/").pop() : s?.mode === "cloud" ? s.model : "无（正则匹配）";
    const callKind = ({ probe: "API 检测", warmup: "模型预热", intent: "口令识别" } as Record<string, string>)[s?.last_request?.kind] || s?.last_request?.kind;
    const backend = (value: string) => value?.split("->").map(v => names[v] || v).join(" → ");
    const callStatus = ({ idle: "空闲", running: "进行中", succeeded: "已完成", failed: "失败", canceled: "已取消" } as Record<string, string>)[s?.last_request?.status] || s?.last_request?.status;
    this.element("status").textContent = s ?
      `当前实际模式：${names[s.mode] || s.mode}\n保存模式：${names[this.saved?.startup_mode] || this.saved?.startup_mode || "尚未读取"}（下次启动生效）\n${s.preparing ? "启动准备中，暂不接收新口令\n" : this.restartFrom || s.restarting ? "重启中\n" : ""}云端 API：${s.cloud_enabled ? "已开启" : "已关闭"} · ${this.checkId && this.checkId === s.last_request?.id ? availability[s.api_status] || s.api_status : "运行节点历史状态，未检测本页保存内容"}\n` +
      `本次启动选择：${names[s.active_startup_mode ?? s.startup_mode] || s.active_startup_mode || s.startup_mode}${s.startup_mode_override ? "（启动参数覆盖保存模式；节点重建后仍保持）" : ""}\n` +
      `模型：${modelName || "尚未准备"} · 模型请求：${s.http_busy ? "运行中" : "空闲"}\n` +
      `处理阶段：${phases[s.inference?.phase] || s.inference?.phase || "不可用"}\n语音输入：${!this.restartFrom && !s.restarting && !s.preparing && s.inference?.input_enabled ? (s.inference?.phase === "wake_wait" ? "等待“小赛”唤醒" : "仅接收唤醒后的本句（先等待静音）") : "暂停；当前不接收语音，取消请使用现有网页业务入口"}${s.inference?.last_error ? " · " + s.inference.last_error : ""}\n` +
      `最近检测：${s.last_check_at || "暂无"}\n最近模型耗时：${s.last_duration_ms == null ? "暂无" : `${backend(s.last_duration_backend)} / ${Number(s.last_duration_ms).toFixed(1)} ms`}\n` +
      `最近请求：${callKind ? `${backend(s.last_request.backend || "rules")} / ${callKind} / ${callStatus || "已完成"}` : "暂无"}\n` +
      `配置版本：${s.config_revision}${s.last_error?.code ? "\n原因：" + s.last_error.code + (s.last_error.message !== s.last_error.code ? " · " + s.last_error.message : "") : ""}` :
      (this.connected ? `运行状态不可查询。保存模式：${names[this.saved?.startup_mode] || "尚未读取"}；配置仍可编辑。` : "连接机器人后读取");
    this.element("result").textContent = s?.last_intent ? JSON.stringify(s.last_intent, null, 2) : "暂无结果";
    this.element("result-meta").textContent = s?.last_intent ?
      `${backend(s.last_intent_backend) || "后端未知"}${s.last_intent_duration_ms == null ? "" : ` · 模型耗时 ${Number(s.last_intent_duration_ms).toFixed(1)} ms`}` : "暂无真实口令结果";
    this.element("key-state").textContent = this.saved?.cloud?.api_key_set ? "（板端已配置）" : "（板端未配置）";
    this.element("key-hint").textContent = this.clearKey ? "下次保存将清除板端 Key；当前实例在重启前继续使用原配置。" : "留空保留已有 Key；读取不返回原文。";
    this.element("api-dirty").textContent = this.apiDirty ? "· 草稿未保存" : "";
    this.element("prompt-dirty").textContent = this.promptDirty ? "· 草稿未保存" : "";
    const ready = this.active && this.connected && this.loaded;
    for (const name of ["api-fields", "prompt-fields"]) (this.element(name) as HTMLFieldSetElement).disabled = !ready;
    for (const name of ["save-api", "save-prompt", "reload"]) this.button(name).disabled = !this.connected || this.pending || this.loading || (name !== "reload" && !this.loaded);
    const toggle = this.field("enabled") as HTMLInputElement;
    toggle.checked = this.saved?.startup_mode === "cloud";
    toggle.disabled = !ready || this.pending || this.loading || !!this.files.pending;
    const local = this.field("local-enabled") as HTMLInputElement;
    local.checked = this.saved?.startup_mode === "local";
    local.disabled = toggle.disabled;
    this.element("local-state").textContent = s?.local ?
      `运行准备状态：${localStates[s.local.status] || s.local.status} · 模型：${s.local.model?.split("/").pop() || "未配置"}` :
      "运行状态不可查询；仍可保存下次使用本地模式。模型参数在板端文件中配置。";
    this.button("use-rules").disabled = toggle.disabled;
    this.button("restart").disabled = !ready || !!this.files.pending || this.pending || this.loading || !!this.restartFrom || !!s?.restarting ||
      !!s?.business_busy || s?.inference?.restart_allowed !== true;
    this.button("restart").title = s?.business_busy ? "本节点业务正在执行，终态后可重启" :
      s?.inference?.restart_allowed !== true ? `当前阶段：${phases[s?.inference?.phase] || "不可用"}，稍后重试` : "重新加载板端已保存配置";
    this.button("reload").textContent = this.files.pending ? "查询原配置操作结果" : "重新读取配置";
    this.button("check").disabled = !ready || !s || !!this.files.pending || this.pending || this.loading || !this.saved?.cloud?.api_key_set ||
      !!s?.http_busy || !!s?.preparing || ["starting", "preparing"].includes(s?.inference?.phase);
  }
}
