import "./style.css";
import { whenVisible } from "./visibility";
import { Connection } from "./connection";
import { boundedVelocity, escapeHTML as esc, id, keyVelocity, Target, yaw, webOperation, robotMotionBusy, initialPoseAvailable } from "./core";
import { MapView } from "./map";
import { IntentPanel, intentHTML } from "./intent";
import type { WarehousePanel } from "./warehouse";

document.querySelector("#app")!.innerHTML = `
  <header><a class="brand" href="#home">K1<span>MUSE</span><small>机器人控制台</small></a><a href="./pharmacy.html" style="color:inherit">药房操作台 ↗</a>
    <div class="connection"><label for="host">机器人 IP</label><input id="host" placeholder="输入机器人 IP 或主机名">
    <button id="connect">连接</button><span id="connected" class="badge">未连接</span></div></header>
  <nav><button data-page="home" class="active">总览</button><button data-page="teleop">遥控底盘</button>
    <button data-page="map">建图与导航</button><button data-page="warehouse">仓库管理</button>
    <button data-page="intent">意图识别模式</button>
    <button disabled>性能监控 <small>后续</small></button><button disabled>相机 <small>后续</small></button></nav>
  <main><div class="statusbar"><span id="mode">IDLE</span><span id="readiness">等待连接</span>
    <span id="navstate"></span><button id="query">查询操作结果</button></div>
    <div id="notice" role="status" aria-live="polite">连接同一局域网中的机器人，选择需要的功能。</div>
    <section id="home"><div class="intro"><div class="eyebrow">K1MUSE / WORKSPACE</div>
      <h1>让机器人工作，<br>在浏览器中掌握现场。</h1><p>按需查看与控制。地图绘制在手机或电脑完成。</p></div>
      <div class="cards"><button data-page="teleop" class="card"><span>01 / CONTROL</span><h2>遥控底盘 ↗</h2>
        <p>WASD 平移 · Q/E 转向<br>键盘与回弹滑杆控制</p></button>
      <button data-page="map" class="card"><span>02 / SPATIAL</span><h2>建图与导航 ↗</h2>
        <p>二维地图 · 标点 · 路径<br>手动建图与单目标导航</p></button>
      <button data-page="warehouse" class="card"><span>03 / WAREHOUSE</span><h2>仓库管理 ↗</h2><p>货架建模 · 药品与二维码标记<br>手动同步仓库配置</p></button>
      <button data-page="intent" class="card"><span>04 / VOICE</span><h2>意图识别模式 ↗</h2>
        <p>云端推理 · 本地推理 · 正则匹配<br>板端配置与运行状态</p></button>
      <div class="card muted"><span>04 / DEBUG</span><h2>性能监控</h2><p>按需采样与运行诊断<br>后续开放</p></div></div>
      <p class="hint">未打开的功能不申请显示数据。首版不传图像，不执行性能采样。</p></section>
    <section id="warehouse" hidden></section>
    <section id="work" hidden>
      <div class="workspace"><aside>
        <div class="panel"><h2>运行模式</h2><div class="buttons"><button id="manual">进入手动模式</button>
          <button id="idle">停止当前模式</button></div>
          <div class="map-only"><button id="mapping" class="wide">开始手动建图</button>
            <label>保存地图名称<input id="mapname" placeholder="warehouse_01" maxlength="64"></label>
            <label class="check"><input id="overwrite" type="checkbox">允许覆盖同名地图</label><button id="savemap" class="wide">保存当前地图</button>
            <label>已保存地图<select id="maps"><option value="">先刷新地图列表</option></select></label>
            <div class="buttons"><button id="refresh">刷新地图</button><button id="navigation">加载导航</button></div>
          </div></div>
        <div class="panel"><h2>底盘遥控 <span id="armed">未启用</span></h2>
          <p class="hint">W/S 前后 · A/D 左右 · Q/E 旋转。松手归零，离开页面后需重新启用。</p>
          <div class="buttons"><button id="enable" class="primary">启用遥控</button><button id="stop" class="danger">停止</button></div>
          <label>速度比例 <output id="ratio-value">50%</output><input id="ratio" type="range" min="0" max="100" value="50"></label>
          <label>前后 <input id="axis-x" class="axis" type="range" min="-100" max="100" value="0"></label>
          <label>左右 <input id="axis-y" class="axis" type="range" min="-100" max="100" value="0"></label>
          <label>旋转 <input id="axis-yaw" class="axis" type="range" min="-100" max="100" value="0"></label>
          <output id="velocity">X 0.00 · Y 0.00 m/s · 转向 0.00 rad/s</output>
        </div>
        <div class="panel"><h2>实时状态</h2><pre id="telemetry">等待数据</pre></div>
      </aside>
      <div id="map-panel" class="map-only"><div class="map-toolbar"><h2>地图工作区</h2>
        <label class="check"><input id="laser" type="checkbox">激光</label>
        <label class="check"><input id="paths" type="checkbox" checked>路径</label></div>
        <canvas id="canvas" aria-label="二维地图，按下并拖动指定位置和朝向"></canvas>
        <div class="map-controls"><label>地图操作<select id="tool"><option value="point">创建标点</option>
          <option value="initial">设置 / 重设定位</option><option value="goal">设置导航目标</option></select></label>
          <label>名称<input id="pointname" maxlength="64" placeholder="货架 A"></label>
          <label>X（米）<input id="pointx" type="number" step="any" placeholder="0.00"></label>
          <label>Y（米）<input id="pointy" type="number" step="any" placeholder="0.00"></label>
          <label>朝向（度）<input id="pointyaw" type="number" value="0" step="any"></label>
          <button id="usepose">使用当前位置</button><button id="target">确认选中位置</button></div>
        <div id="targetinfo" class="hint">拖动地图选点，或输入地图坐标 X、Y 和朝向，再点击确认。</div>
        <div class="points"><div class="map-toolbar"><h2>地图标点 <span id="dirty"></span></h2>
          <button id="loadpoints">读取标点</button><button id="savepoints">保存标点</button></div>
          <div id="pointlist" class="pointlist">当前没有标点</div></div>
      </div>
      <div id="teleop-info" class="panel teleop-only"><div class="eyebrow">MANUAL CONTROL</div><h1>每一次输入，<br>都由你掌控。</h1>
        <div class="keys"><span>Q</span><span>W</span><span>E</span><span>A</span><span>S</span><span>D</span></div>
        <p>手动模式与建图模式支持遥控。导航运行时，先取消导航并等待停止，再启用遥控。</p></div>
      </div><div class="footer-actions"><span id="operation">无进行中的操作</span><button id="cancel" class="danger">取消导航并进入手动</button></div>
    </section><section id="intent" hidden>${intentHTML}</section></main><footer>LAN CONTROL / Foxglove Bridge · K1Muse</footer>`;
const el = <T extends HTMLElement = HTMLElement>(name: string) => document.getElementById(name) as T;
const input = (name: string) => el<HTMLInputElement>(name);
const button = (name: string) => el<HTMLButtonElement>(name);
input("host").value = localStorage.getItem("k1muse-host") || location.hostname;
const clientId = id();
let control: Connection | undefined, display: Connection | undefined, state: any;
let page = "home", armed = false, busy = false, connecting = false, enabling = false, ratio = .5, slider = false;
let inputEpoch = 0;
let keys = new Set<string>(), velocity: Target = { x: 0, y: 0, yaw: 0 }, blockedAt = 0;
let sendTimer: ReturnType<typeof setInterval> | undefined, renewTimer: ReturnType<typeof setInterval> | undefined;
let draft: Record<string, Target> = {}, dirty = false, mapUnsaved = false;
let currentOp = "", requestId = "", mapRevision = "";
let warehouse: WarehousePanel | undefined;
let warehouseLoading: Promise<WarehousePanel> | undefined;
async function warehousePanel() {
  return warehouseLoading ??= import("./warehouse").then(({ WarehousePanel }) => {
    warehouse = new WarehousePanel(el("warehouse"), async (operation, args, requestId) => {
      if (!control?.token) throw new Error("请先连接机器人");
      if (!operation.startsWith('drug_catalog_')) disarm();
      return control.request(operation, args, requestId);
    }, () => { updateDisplay(true); void renew(); }, value => { busy = value; renderState(); });
    warehouse.setConnected(!!control?.token); warehouse.setState(state); return warehouse;
  });
}
const telemetryValues: Record<string, string> = {};
const map = new MapView(el<HTMLCanvasElement>("canvas"));
const intent = new IntentPanel(el("intent"), async (operation, args, requestId) => {
  if (!control?.token) throw new Error("请先连接机器人");
  return await control.request(operation, args, requestId);
});
function notice(text: string, error = false) {
  el("notice").textContent = text; el("notice").classList.toggle("error", error);
}
function ratioVelocity() {
  return slider ? boundedVelocity(velocity, ratio) : keyVelocity(keys, ratio);
}
function clearInput() {
  keys.clear(); velocity = { x: 0, y: 0, yaw: 0 }; slider = false;
  for (const axis of document.querySelectorAll<HTMLInputElement>(".axis")) axis.value = "0";
  el("velocity").textContent = "X 0.00 · Y 0.00 m/s · 转向 0.00 rad/s";
}
function disarm(send = true) {
  ++inputEpoch;
  const wasArmed = armed; armed = false; clearInput();
  if (sendTimer) clearInterval(sendTimer); sendTimer = undefined; blockedAt = 0;
  if (send && control?.token && wasArmed) {
    control.send({ x: 0, y: 0, yaw: 0 });
    control.request("set_teleop_enabled", { enabled: false }).catch(e => notice(String(e), true));
  }
  renderState();
}
function controlsReady() {
  return !!control?.token && !!state?.ready && ["MANUAL", "MAPPING"].includes(state.mode) &&
    !busy && !webOperation(state).web_busy && !navigationBusy();
}
function navigationBusy() {
  return robotMotionBusy(state);
}
function renderState() {
  const operation = webOperation(state), blocked = busy || !!operation.web_busy;
  el("armed").textContent = armed ? "已启用" : "未启用";
  el("armed").className = armed ? "enabled" : "";
  button("enable").disabled = !controlsReady() || armed || enabling;
  for (const a of document.querySelectorAll<HTMLInputElement>(".axis")) a.disabled = !armed;
  button("target").disabled = !control?.token || blocked || !map.geometry || !map.target ||
    (map.tool !== "point" && navigationBusy()) ||
    (map.tool === "initial" && !initialPoseAvailable(state)) ||
    (map.tool === "goal" && !state?.navigation_ready);
  for (const name of ["manual", "idle", "mapping", "refresh", "navigation", "savemap", "loadpoints", "savepoints"])
    button(name).disabled = !control?.token || blocked;
  for (const name of ["manual", "idle", "mapping", "navigation", "savepoints"])
    button(name).disabled ||= navigationBusy();
  el("pointlist").querySelectorAll<HTMLButtonElement>("button[data-go]").forEach(b => {
    b.disabled = !control?.token || blocked || navigationBusy() || !state?.navigation_ready || dirty;
  });
  button("savemap").disabled ||= state?.mode !== "MAPPING" || !state?.ready;
  button("navigation").disabled ||= !el<HTMLSelectElement>("maps").value;
  button("savepoints").disabled ||= !state?.map_id || !dirty;
  button("loadpoints").disabled ||= !state?.map_id;
  button("usepose").disabled = !map.pose;
  button("cancel").disabled = !control?.token || state?.mode !== "NAVIGATION";
  el("mode").textContent = state?.mode || "IDLE";
  el("readiness").textContent = !control?.token ? "等待连接" : !state?.ready ? "准备中 / 未就绪" :
    state.mode === "NAVIGATION" ? (state.navigation_ready ? "导航已就绪" :
      operation.web_busy ? "导航准备 / 定位确认中" : operation.status === 'failed' ? "导航未就绪，请查看操作结果" : "等待设置定位") : "已就绪";
  el("navstate").textContent = state?.navigation_state ? "导航：" + state.navigation_state : "";
}
function updateDisplay(rebuild = false) {
  if (!control?.token || document.hidden) { display?.close(); display = undefined; return; }
  if (rebuild) { display?.close(); display = undefined; map.clear(); warehouse?.clearMap(); selectTarget(); }
  if (page === "home" || page === "warehouse" && !warehouse?.wantsMap()) { display?.close(); display = undefined; }
  else {
    if (!display) {
      display = new Connection(control.ws.url, clientId);
      display.onMessage = (topic, message) => {
        if (document.hidden) return;
        if (page === "warehouse" && topic === "map") { warehouse?.mapMessage(message); }
        else if (topic === "intent_status") { intent.status(message); }
        else if (["odom", "chassis", "imu"].includes(topic)) {
          if (topic === "odom") {
            const p = message.pose.pose.position, v = message.twist.twist;
            telemetryValues.odom = `里程计 X ${p.x.toFixed(2)}  Y ${p.y.toFixed(2)} m\n朝向 ${(yaw(message.pose.pose.orientation) * 180 / Math.PI).toFixed(1)}°\n速度 X ${v.linear.x.toFixed(2)}  Y ${v.linear.y.toFixed(2)}\n转向 ${v.angular.z.toFixed(2)} rad/s`;
          } else if (topic === "imu") {
            telemetryValues.imu = `IMU 角速度 Z ${message.angular_velocity.z.toFixed(3)} rad/s\n加速度 Z ${message.linear_acceleration.z.toFixed(2)} m/s²`;
          } else {
            telemetryValues.chassis = `底盘状态码 ${message.state}\n电机异常标志 ${message.motor_block_flags}`;
          }
          el("telemetry").textContent = Object.values(telemetryValues).join("\n");
        } else { map.message(topic, message); renderState(); }
      };
      display.onError = error => notice(error, true);
    }
    const topics = page === "warehouse" ? ["map"] : page === "intent" ? ["intent_status"] : ["odom", "chassis", "imu"];
    if (page === "map") {
      topics.push("map", "pose");
      if (input("paths").checked) topics.push("global_path", "local_path");
      if (input("laser").checked) topics.push("scan", "scan_pose");
    }
    display.subscribe(topics);
  }
}
let renewing = false, viewsChanged = false;
async function renew(force = false) {
  if (!control?.token || document.hidden && !force) return;
  if (renewing) { viewsChanged = true; return; }
  const connection = control; renewing = true;
  try {
    do {
      viewsChanged = false;
      const visible = !document.hidden;
      await connection.request("set_views", visible ? {
        telemetry: page === "teleop" || page === "map", intent: page === "intent",
        map: page === "map" || page === "warehouse" && !!warehouse?.wantsMap(),
        paths: page === "map" && input("paths").checked, laser: page === "map" && input("laser").checked
      } : {});
    } while (viewsChanged && connection === control && !!connection.token);
  } catch (error) {
    if (connection === control) {
      notice(String(error), true);
      if (String(error).includes("session_expired")) disconnect();
    }
  } finally { renewing = false; }
}
let draftMapId = "";
function presentState() {
  if (!state || document.hidden) return;
  warehouse?.setState(state);
  const revision = state.mode + "/" + state.map_id;
  if (revision !== mapRevision) {
    mapRevision = revision; updateDisplay(true);
    if (draftMapId !== state.map_id) { draft = {}; dirty = false; draftMapId = state.map_id; }
    renderPoints();
  }
  map.showOrigin = state.mode === "MAPPING" && state.ready;
  map.draw();
  if (map.pose && performance.now() - map.poseAt > 1000) map.pose = undefined;
  el("operation").textContent = state.operation_id ?
    `当前操作：${JSON.parse(state.operation_result || "{}").status || "未知"}` : "无进行中的操作";
  renderState();
}

function disconnect() {
  intent.setConnected(false); warehouse?.setConnected(false); warehouse?.clearMap();
  disarm(false); connecting = false; state = undefined;
  if (renewTimer) clearInterval(renewTimer); renewTimer = undefined;
  const old = control; control = undefined; old?.close(); display?.close(); display = undefined; map.clear();
  selectTarget();
  for (const name of Object.keys(telemetryValues)) delete telemetryValues[name];
  el("telemetry").textContent = "等待数据";
  el("connected").textContent = "未连接"; button("connect").textContent = "连接"; renderState();
}
async function connect() {
  if (control || connecting) { control?.request("release_control").catch(() => {}); disconnect(); return; }
  const host = input("host").value.trim();
  if (!host || /[\s/]/.test(host)) { notice("请输入机器人 IP 或主机名", true); return; }
  localStorage.setItem("k1muse-host", host); connecting = true;
  notice("正在连接 Foxglove Bridge…");
  const c = new Connection(`ws://${host}:8765`, clientId, true); control = c;
  const timeout = setTimeout(() => { if (!c.token && control === c) { disconnect(); notice("连接超时，请检查机器人地址及服务", true); } }, 6000);
  c.onReady = async () => {
    if (c.token || control !== c) return;
    try {
      const response = await c.request("claim_control");
      if (control !== c) return;
      c.token = response.result.token; clearTimeout(timeout); connecting = false;
      intent.setConnected(true);
      if (!document.hidden && page === "warehouse") void warehouse?.connectedEnter().catch(error => notice(String(error), true));
      else warehouse?.setConnected(true);
      el("connected").textContent = "已连接"; button("connect").textContent = "断开";
      c.subscribe(["state"]); if (page === "intent") intent.enter(); updateDisplay(); await renew(true);
      renewTimer = setInterval(renew, 1000); notice("连接成功。进入手动模式后可启用遥控。"); renderState();
    } catch (error) { clearTimeout(timeout); disconnect(); notice(String(error), true); }
  };
  c.onClose = () => { clearTimeout(timeout); if (control === c) { disconnect(); notice("连接已断开，遥控已解除；重连后需重新启用。", true); } };
  c.onError = error => notice(error, true);
  c.onMessage = (topic, message) => {
    if (control !== c || topic !== "state") return;
    const previous = state;
    state = message;
    if (armed && !message.teleop_enabled) disarm(false);
    if (previous && previous.boot_id !== message.boot_id) { disconnect(); notice("板端节点已重启，请重新连接", true); return; }
    if (message.client_id && message.client_id !== clientId) { disconnect(); notice("操作会话已被另一网页接管，请手动连接。", true); return; }
    presentState();
  };
}
async function perform(operation: string, args: any = {}, wait = true): Promise<any> {
  if (!control?.token) throw new Error("请先连接机器人");
  disarm(); busy = true; renderState(); requestId = id();
  try {
    const connection = control;
    const response = await connection.request(operation, args, requestId);
    currentOp = response.operation_id;
    if (!wait || !currentOp) return response.result;
    let result = response.result;
    const deadline = Date.now() + (["set_mode", "set_initial_pose"].includes(operation) ? 90000 : 26000);
    while (result.status === "running") {
      await new Promise(resolve => setTimeout(resolve, 500));
      await whenVisible();
      result = (await connection.request("get_operation", { operation_id: currentOp })).result;
      if (result.status === "running" && Date.now() > deadline) throw new Error("结果未知，请查询操作结果");
    }
    if (result.status === "failed") throw new Error(result.error);
    notice("操作完成。"); return result.result ?? result;
  } finally { busy = false; renderState(); }
}
function action(name: string, fn: () => Promise<any> | void) {
  button(name).addEventListener("click", () => { Promise.resolve().then(fn).catch(e => notice(String(e), true)); });
}
function checkLeave(): boolean {
  return (!dirty || confirm("标点草稿未保存，继续将丢弃草稿。")) &&
    (!mapUnsaved || confirm("当前建图尚未保存，确认结束建图？"));
}
async function setMode(mode: string, args: any = {}) {
  if (!checkLeave()) return;
  await perform("set_mode", { mode, ...args });
  mapUnsaved = mode === "MAPPING";
}
action("connect", connect);
action("manual", () => setMode("MANUAL"));
action("idle", () => setMode("IDLE"));
action("mapping", () => setMode("MAPPING"));
action("navigation", () => setMode("NAVIGATION", { map_id: el<HTMLSelectElement>("maps").value }));
action("refresh", async () => {
  const data = await perform("list_maps");
  el("maps").innerHTML = '<option value="">选择地图</option>' + data.maps.map((m: any) =>
    `<option value="${esc(m.map_id)}">${esc(m.name)} · ${m.width}×${m.height}</option>`).join("");
  renderState();
});
action("savemap", async () => {
  const saved = await perform("save_map", { name: input("mapname").value.trim(), overwrite: input("overwrite").checked });
  draftMapId = saved.map_id; draft = saved.tasks; dirty = false; renderPoints();
  mapUnsaved = false;
});
action("stop", () => { disarm(); notice("遥控已停止，需重新启用。"); });
action("enable", async () => {
  if (armed || enabling || !controlsReady()) return;
  const epoch = inputEpoch, connection = control!;
  enabling = true; clearInput(); renderState();
  try { await connection.request("set_teleop_enabled", { enabled: true }); }
  finally { enabling = false; renderState(); }
  if (epoch !== inputEpoch || connection !== control || document.hidden) {
    connection.request("set_teleop_enabled", { enabled: false }).catch(() => {}); return;
  }
  armed = true; blockedAt = 0; renderState();
  sendTimer = setInterval(() => {
    const v = ratioVelocity();
    el("velocity").textContent = `X ${v.x.toFixed(2)} · Y ${v.y.toFixed(2)} m/s · 转向 ${v.yaw.toFixed(2)} rad/s`;
    if (!control?.send(v)) {
      blockedAt ||= performance.now();
      if (performance.now() - blockedAt >= 200) { disconnect(); notice("控制连接拥堵，遥控已解除。", true); }
    } else blockedAt = 0;
  }, 50);
});
action("cancel", () => perform("cancel_navigation"));
action("query", async () => {
  const operationId = state?.operation_id || currentOp;
  const data = await control!.request("get_operation", operationId ? { operation_id: operationId } : requestId ? { request_id: requestId } : {});
  notice(JSON.stringify(data.result));
});
function renderPoints() {
  map.points = draft; map.draw(); el("dirty").textContent = dirty ? "· 草稿未保存" : "";
  el("pointlist").innerHTML = Object.entries(draft).map(([name, p], i) =>
    `<div class="point"><strong>${esc(name)}</strong><span>${map.description(p)}</span>
    <button data-edit="${i}">编辑</button><button data-go="${i}">前往</button><button data-delete="${i}">删除</button></div>`).join("") || "当前没有标点";
  el("pointlist").querySelectorAll<HTMLButtonElement>("button").forEach(b => b.onclick = () => {
    const names = Object.keys(draft);
    if (b.dataset.delete) {
      delete draft[names[Number(b.dataset.delete)]]; dirty = true; renderPoints();
    } else if (b.dataset.edit) {
      const name = names[Number(b.dataset.edit)]; selectTarget({ ...draft[name] });
      map.tool = "point"; el<HTMLSelectElement>("tool").value = "point";
      input("pointname").value = name;
      const renamed = prompt("标点名称", name);
      if (renamed !== null && renamed.trim() && (renamed === name || !draft[renamed.trim()])) {
        delete draft[name]; input("pointname").value = renamed.trim();
        draft[renamed.trim()] = { ...map.target! }; dirty = true; renderPoints();
      }
      renderState();
    } else if (b.dataset.go) {
      const name = names[Number(b.dataset.go)];
      perform("navigate", { map_id: state?.map_id, name }, false).catch(e => notice(String(e), true));
    }
  });
  renderState();
}
action("loadpoints", async () => {
  if (dirty && !confirm("读取将丢弃本地标点草稿，继续？")) return;
  const data = await perform("get_points", { map_id: state.map_id }); draftMapId = data.map_id; draft = data.tasks; dirty = false; renderPoints();
});
action("savepoints", async () => {
  const data = await perform("save_points", { map_id: state.map_id, tasks: draft }); draftMapId = data.map_id; draft = data.tasks; dirty = false; renderPoints();
});
function readTarget(): Target | undefined {
  const x = input("pointx").valueAsNumber, y = input("pointy").valueAsNumber;
  const yaw = input("pointyaw").valueAsNumber * Math.PI / 180;
  return [x, y, yaw].every(Number.isFinite) ? { x, y, yaw } : undefined;
}
function updateTarget() {
  map.target = readTarget();
  el("targetinfo").textContent = map.target ? map.description(map.target) :
    "请输入完整的 X、Y 和朝向，或拖动地图选点；点击确认后生效。";
  map.draw(); renderState();
}
function selectTarget(target?: Target) {
  input("pointx").value = target ? String(target.x) : "";
  input("pointy").value = target ? String(target.y) : "";
  input("pointyaw").value = target ? String(target.yaw * 180 / Math.PI) : "0";
  updateTarget();
}
action("usepose", () => { if (map.pose) selectTarget(map.pose); });
map.onTarget = selectTarget;
for (const name of ["pointx", "pointy", "pointyaw"]) input(name).addEventListener("input", updateTarget);
action("target", async () => {
  const target = readTarget();
  if (!target) throw new Error("请输入有效的 X、Y 和朝向");
  if (map.tool === "initial") await perform("set_initial_pose", target);
  else if (map.tool === "goal") await perform("navigate", { map_id: state.map_id, ...target }, false);
  else {
    const name = input("pointname").value.trim();
    if (!state?.map_id) throw new Error("请先保存当前地图，再编辑标点");
    if (!name || name.length > 64) throw new Error("名称需要 1～64 个字符");
    if (!draft[name] && Object.keys(draft).length >= 100) throw new Error("最多保存 100 个标点");
    draft[name] = target; dirty = true; renderPoints(); notice("标点已加入本地草稿，保存后才会写入板端。");
  }
});
el("tool").addEventListener("change", () => { map.tool = el<HTMLSelectElement>("tool").value; renderState(); });
el("maps").addEventListener("change", renderState);
for (const name of ["paths", "laser"]) input(name).addEventListener("change", () => { updateDisplay(); void renew(); });
input("ratio").addEventListener("input", () => { ratio = Number(input("ratio").value) / 100; el("ratio-value").textContent = Math.round(ratio * 100) + "%"; });
for (const name of ["x", "y", "yaw"]) {
  const axis = input("axis-" + name);
  axis.addEventListener("pointerdown", e => {
    keys.clear(); slider = true; velocity = { x: 0, y: 0, yaw: 0 }; axis.setPointerCapture(e.pointerId);
  });
  axis.addEventListener("input", () => { if (armed) velocity[name as keyof Target] = Number(axis.value) / 100 * (name === "yaw" ? .6 : .2); });
  axis.addEventListener("pointerup", clearInput);
  axis.addEventListener("pointercancel", () => disarm());
  axis.addEventListener("lostpointercapture", clearInput);
}
function editing(e: KeyboardEvent) { return (e.target as HTMLElement)?.matches("input,textarea,select,[contenteditable]"); }
window.addEventListener("keydown", e => {
  if (!armed || e.repeat || editing(e) || slider || !/^Key[WASDQE]$/.test(e.code)) return;
  e.preventDefault(); keys.add(e.code);
});
window.addEventListener("keyup", e => { keys.delete(e.code); });
window.addEventListener("blur", () => disarm());
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    disarm(); display?.close(); display = undefined;
    void renew(true);
  } else {
    presentState(); updateDisplay(); void renew();
    if (page === "warehouse" && control?.token) void warehouse?.connectedEnter().catch(error => notice(String(error), true));
    notice(control?.token ? "页面已恢复，遥控需手动重新启用。" : "页面已恢复，请连接机器人。");
  }
});
window.addEventListener("pagehide", () => {
  disarm(); control?.request("set_views", {}).catch(() => {}); control?.request("release_control").catch(() => {}); disconnect();
});
function changePage(next: string) {
  if (!["home", "teleop", "map", "intent", "warehouse"].includes(next) || next === page && next !== "home") return;
  if (dirty && page === "map" && next !== "map" && !confirm("标点草稿未保存，离开页面？")) return;
  if (page === "intent" && next !== "intent") intent.leave();
  if (page === "warehouse" && next !== "warehouse") warehouse?.leave();
  disarm(); page = next;
  el("home").hidden = next !== "home"; el("work").hidden = next !== "teleop" && next !== "map";
  el("intent").hidden = next !== "intent"; el("warehouse").hidden = next !== "warehouse";
  document.querySelectorAll<HTMLElement>(".map-only").forEach(e => e.hidden = next !== "map");
  document.querySelectorAll<HTMLElement>(".teleop-only").forEach(e => e.hidden = next !== "teleop");
  document.querySelectorAll("nav button").forEach(e => e.classList.toggle("active", (e as HTMLElement).dataset.page === next));
  if (next === "intent") intent.enter();
  if (next === "warehouse") void warehousePanel().then(panel => { if (page === "warehouse") return panel.enter(); }).catch(error => notice(String(error), true));
  updateDisplay(); void renew(); map.draw(); renderState();
}
document.querySelectorAll<HTMLButtonElement>("[data-page]").forEach(b => b.onclick = () => changePage(b.dataset.page!));
document.querySelector(".brand")!.addEventListener("click", e => { e.preventDefault(); changePage("home"); });
changePage("home");
