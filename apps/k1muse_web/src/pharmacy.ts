import './pharmacy.css';
import { Connection } from './connection';
import { escapeHTML, fetchAdmission, id, robotMotionBusy, webOperation } from './core';
import { layerInventory, type Inventory } from './warehouse_inventory';
import { validateWarehouseDocument, type WarehouseDocument } from './warehouse_model';
import { WarehouseOperation } from './warehouse_operation';
import type { WarehouseScene, WarehouseSelection } from './warehouse_scene';

const el = <T extends HTMLElement = HTMLElement>(name: string) => document.getElementById(name) as T;
const button = (name: string) => el<HTMLButtonElement>(name);
const demo = new URLSearchParams(location.search).get('demo') === '1';
const clientId = id();
let connection: Connection | undefined;
let renew: ReturnType<typeof setInterval> | undefined;
let state: any;
let manager: any;
let layout: WarehouseDocument | undefined;
let inventory: Inventory | undefined;
let selection: WarehouseSelection = { shelfId: '', layer: null, qr: false };
let scene: WarehouseScene | undefined;
let sceneFailed = false;
let quantity = 1;
let refreshing = false;
let readingInventory = false;
let inventoryError = '';
let task: { kind: 'fetch_drug' | 'survey_warehouse'; requestId: string; operationId?: string; drug?: string; quantity?: number; pending: boolean; uncertain?: boolean; result: any } | undefined;
let simulation: ReturnType<typeof setInterval> | undefined;
let cancelPending = false;

function rows() {
  return layout?.layout.shelves.flatMap(shelf => shelf.levels.filter(level => level.drug).map(level => ({
    shelf, level, stock: layerInventory(inventory, layout!.map_id, shelf.id, level.layer, level.drug)
  }))) ?? [];
}
const selectedRow = () => rows().find(r => r.shelf.id === selection.shelfId && r.level.layer === selection.layer);
const busy = () => !!task?.pending || robotMotionBusy(state) || !!webOperation(state).web_busy;
const prepared = () => !!connection?.token && state?.mode === 'NAVIGATION' && state.navigation_ready === true &&
  !!layout && layout.map_id === state.map_id;
const showNotice = (text: string) => { el('notice').textContent = text; };

function renderInventory() {
  const filter = el<HTMLInputElement>('search').value.trim().toLowerCase();
  const visible = rows().filter(({shelf, level}) => [level.drug, shelf.name, shelf.id].some(v => v.toLowerCase().includes(filter)));
  el('items').innerHTML = visible.map(({shelf, level, stock}) =>
    `<button class="item" data-shelf="${escapeHTML(shelf.id)}" data-layer="${level.layer}" aria-pressed="${shelf.id === selection.shelfId && level.layer === selection.layer}"><strong>${escapeHTML(level.drug)}</strong><span>${stock?.quantity == null ? '未盘点' : stock.quantity + ' 盒'}</span><small>${escapeHTML(shelf.name || shelf.id)} · ${level.layer} 层${stock?.stale ? ' · 待复盘' : ''}</small></button>`
  ).join('') || `<p>${filter ? '没有匹配的药品' : '暂无可显示的库存'}</p>`;
  el('inventory-note').textContent = demo ? '演示数据' : inventoryError ||
    (inventory?.save_error ? '板端保存失败' : inventory?.load_error ? '板端库存读取异常' :
      inventory?.persisted ? '已保存至 K1' : inventory ? '板端库存尚未保存' : '等待读取库存');
}

const phases: Record<string, string> = {
  pending: '正在提交', accepted: '任务已受理', to_shelf: '前往货架',
  settle: '等待停稳', lift: '调整升降', search: '寻找药盒', detect: '寻找药盒',
  align: '对齐药盒', grasp: '正在取药', carry: '转运药盒', place: '放置药盒',
  sweep: '正在盘点', return: '返回中', returning: '返回中', admission_unresolved: '受理结果待确认'
};

function render() {
  const row = selectedRow();
  const active = busy();
  const ready = demo || prepared();
  const admission = demo ? { ready: true, reason: '' } : fetchAdmission(manager, row?.level.drug);
  const count = row?.stock?.quantity;
  const detail = row ? `${row.shelf.name || row.shelf.id} · 第 ${row.level.layer} 层` : '';
  el('device').textContent = demo ? '演示模式 · 模拟数据' : !connection?.token ? '未连接' : active ? '执行中' : ready ? '已就绪' : '未就绪';
  button('connect').hidden = demo;
  button('connect').textContent = connection?.token ? '断开连接' : connection ? '正在连接…' : '连接机器人';
  button('connect').disabled = !!connection && !connection.token;
  el('drug').textContent = row?.level.drug ?? '请选择药品';
  el('location').textContent = detail || '请选择一个货架层';
  el('selection-detail').textContent = row ? `${detail} · ${count == null ? '数量未知' : '库存 ' + count + ' 盒'}${row.stock?.stale ? '（待复盘）' : ''}` : '从库存列表或货架中选择';
  el('quantity').textContent = String(quantity);
  button('minus').disabled = !row || active || quantity <= 1;
  button('plus').disabled = !row || active || quantity >= 3;
  button('fetch').disabled = !ready || !row || active || refreshing || !admission.ready || (demo && count === 0);
  button('survey').disabled = !ready || active || refreshing || !rows().length;
  button('refresh').disabled = refreshing || readingInventory || (!demo && (!connection?.token || !state?.map_id));
  button('cancel').hidden = !task?.pending;
  button('cancel').disabled = cancelPending || !(demo || (!!connection?.token && !!task?.operationId && state?.operation_id === task.operationId));
  button('query').hidden = !task?.uncertain;
  button('query').disabled = !connection?.token;
  const updated = row?.stock?.updated_at;
  el('hint').textContent = active ? '请等待当前任务结束。' : !ready ? '连接后需由操作员完成机器人准备。' :
    !admission.ready && row ? `该药当前不可取（${admission.reason}）` : demo && count === 0 ? '这层暂无库存。' :
    updated ? `库存更新于 ${new Date(updated).toLocaleString('zh-CN')}` : '每次可取 1–3 盒。';
  if (!task) {
    el('progress').textContent = active ? '机器人正在执行另一项操作' : '暂无任务';
    return;
  }
  const result = task.result;
  const summary = result.result ?? result;
  const name = task.kind === 'fetch_drug' ? `${task.drug} · 请求 ${task.quantity} 盒` : '整仓盘点';
  const fetched = Number.isInteger(summary.fetched_quantity) ? ` · 实际取到 ${summary.fetched_quantity} 盒` : '';
  let status = phases[result.phase] ?? (result.phase ? `执行中（${result.phase}）` : '正在提交');
  if (task.uncertain) status = '结果待确认，请查询原任务';
  else if (cancelPending) status = '已请求取消，等待机器人确认';
  else if (result.status === 'canceled') status = '已取消';
  else if (result.status === 'failed') status = `未完成${result.error ? ' · ' + result.error : ''}`;
  else if (result.status === 'succeeded') {
    if (task.kind === 'fetch_drug' && summary.fetched_quantity > 0) status = demo ? '模拟取药完成' : '已返回，请按现场指引取走药品';
    else status = summary.completion === 'partial' ? '盘点部分完成' : task.kind === 'fetch_drug' ? '任务结束，本次未取到药品' : '盘点完成';
    if (task.kind === 'survey_warehouse' && summary.persisted === false) status += ' · 库存未保存';
  }
  el('progress').textContent = `${name} · ${status}${fetched}`;
}

async function drawShelf() {
  if (!layout?.layout.shelves.length || sceneFailed) return;
  if (!scene) {
    const host = el('scene');
    host.textContent = '';
    try {
      const { WarehouseScene } = await import('./warehouse_scene');
      if (!scene) {
        scene = new WarehouseScene(host, picked => {
          const row = rows().find(r => r.shelf.id === picked.shelfId && r.level.layer === picked.layer);
          if (row) choose(row.shelf.id, row.level.layer);
        }, () => {});
      }
    } catch {
      sceneFailed = true;
      host.textContent = '3D 视图暂不可用，请在库存列表查看货架层。';
    }
  }
  if (scene && layout) {
    scene.update(layout, selection, {
      camera_xy_in_base_m: null, lift_zero_height_m: null, observation_distance_m: 0,
      front: { calibrated: false, observation_drop_m: null },
      top: { calibrated: false, observation_drop_m: null }, canonical_drugs: []
    });
    scene.setEditable(false);
  }
}

function choose(shelfId: string, layer: number) {
  selection = { shelfId, layer, qr: false };
  quantity = 1;
  renderInventory();
  render();
  void drawShelf();
}

async function refresh() {
  if (refreshing || readingInventory || busy()) return;
  if (demo) { renderInventory(); showNotice('已刷新演示库存'); return; }
  const c = connection;
  if (!c?.token) return;
  refreshing = true;
  render();
  try {
    if (!state?.map_id) throw new Error('请在开发控制台加载地图并完成准备。');
    const mapId = state.map_id;
    const read = new WarehouseOperation(c.request.bind(c));
    await read.start('list_maps', {});
    const data = await read.start('warehouse_get', { map_id: mapId });
    const stock = await c.request('inventory_get', { map_id: mapId });
    const status = await read.start('warehouse_status', {});
    if (c !== connection || state?.map_id !== mapId) return;
    layout = data.document ? validateWarehouseDocument(data.document) : undefined;
    inventory = stock.result;
    inventoryError = '';
    manager = status.manager ?? data.manager;
    const row = selectedRow() ?? rows()[0];
    selection = { shelfId: row?.shelf.id ?? '', layer: row?.level.layer ?? null, qr: false };
    if (layout?.layout.shelves.length) void drawShelf();
    else { scene?.dispose(); scene = undefined; el('scene').textContent = '当前地图没有已同步的货架。'; }
    showNotice('');
  } catch (error) {
    if (connection === c) {
      manager = undefined;
      showNotice(String(error));
    }
  } finally { refreshing = false; renderInventory(); render(); }
}

async function readInventory() {
  const c = connection;
  const mapId = state?.map_id;
  if (!c?.token || !mapId || refreshing || readingInventory || document.hidden) return;
  readingInventory = true;
  render();
  try {
    const reply = await c.request('inventory_get', { map_id: mapId });
    if (c !== connection || state?.map_id !== mapId) return;
    if (reply.result.map_id !== mapId) throw new Error('库存与当前地图不匹配');
    inventory = reply.result;
    inventoryError = '';
  } catch {
    if (c === connection && state?.map_id === mapId) {
      inventory = undefined;
      inventoryError = '库存读取失败，请刷新';
    }
  } finally {
    readingInventory = false;
    renderInventory();
    render();
  }
}

function applyTask(result: any) {
  if (!task?.pending) return;
  task.result = result;
  task.uncertain = result.phase === 'admission_unresolved';
  if (['succeeded', 'failed', 'canceled'].includes(result.status)) {
    task.pending = false;
    task.uncertain = false;
    cancelPending = false;
    manager = undefined;
    if (!demo) {
      if (busy()) void readInventory();
      else void refresh();
    }
  }
  render();
}

function disconnect(message: string) {
  clearInterval(renew);
  const c = connection;
  connection = undefined;
  state = manager = inventory = undefined;
  inventoryError = '';
  if (task?.pending) task.uncertain = true;
  c?.close();
  showNotice(message);
  renderInventory();
  render();
}

function connect() {
  if (connection?.token) {
    void connection.request('release_control').catch(() => {}).finally(() => disconnect('连接已断开'));
    return;
  }
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const c = new Connection(`${scheme}://${location.hostname}:8765`, clientId, true);
  connection = c;
  let claiming = false;
  const timeout = setTimeout(() => { if (connection === c && !c.token) disconnect('连接超时，请检查板端服务。'); }, 6500);
  c.onReady = async () => {
    if (claiming || connection !== c) return;
    claiming = true;
    try {
      const reply = await c.request('claim_control');
      if (connection !== c) return;
      c.token = reply.result.token;
      clearTimeout(timeout);
      c.subscribe(['state']);
      renew = setInterval(() => {
        if (connection === c) void c.request('set_views').catch(error => { if (connection === c) disconnect(String(error)); });
      }, 1000);
      render();
    } catch (error) { if (connection === c) disconnect(String(error)); }
  };
  c.onClose = () => { clearTimeout(timeout); if (connection === c) disconnect('连接已断开。'); };
  c.onError = error => { if (connection === c) showNotice(error); };
  c.onMessage = (topic, message) => {
    if (topic !== 'state' || connection !== c) return;
    if ((state?.boot_id && state.boot_id !== message.boot_id) || (message.client_id && message.client_id !== clientId)) {
      disconnect('控制会话已变化，请重新连接。');
      return;
    }
    const previousMap = state?.map_id;
    const previousReady = state?.navigation_ready;
    state = message;
    if (task?.operationId && task.operationId === state.operation_id) applyTask(webOperation(state));
    if (state.map_id && (previousMap !== state.map_id || (!previousReady && state.navigation_ready)) && !busy()) void refresh();
    render();
  };
  render();
}

async function submit(kind: 'fetch_drug' | 'survey_warehouse') {
  if (button(kind === 'fetch_drug' ? 'fetch' : 'survey').disabled) return;
  const row = selectedRow();
  if (kind === 'survey_warehouse' && !confirm('开始整仓盘点？机器人将前往货架。')) return;
  task = { kind, requestId: id(), drug: row?.level.drug, quantity, pending: true, result: {} };
  const current = task;
  render();
  if (demo) {
    const stages = kind === 'fetch_drug' ? ['to_shelf', 'grasp', 'return'] : ['to_shelf', 'sweep', 'return'];
    let index = 0;
    applyTask({ status: 'running', phase: stages[index++] });
    simulation = setInterval(() => {
      if (index < stages.length) { applyTask({ status: 'running', phase: stages[index++] }); return; }
      clearInterval(simulation);
      if (kind === 'fetch_drug' && row?.stock) {
        const fetched = Math.min(current.quantity!, row.stock.quantity ?? current.quantity!);
        if (row.stock.quantity !== null) row.stock.quantity -= fetched;
        row.stock.updated_at = new Date().toISOString();
        applyTask({ status: 'succeeded', result: { fetched_quantity: fetched } });
      } else {
        for (const item of inventory!.layers) {
          item.quantity ??= 3;
          item.stale = false;
          item.updated_at = new Date().toISOString();
        }
        applyTask({ status: 'succeeded' });
      }
      renderInventory();
    }, 850);
    return;
  }
  try {
    const reply = await connection!.request(kind, kind === 'fetch_drug'
      ? { drug_name: current.drug, quantity: current.quantity } : { map_id: layout!.map_id }, current.requestId);
    current.operationId = reply.operation_id;
    applyTask(state?.operation_id === current.operationId ? webOperation(state) : reply.result);
  } catch (error) {
    if ((error as { rejected?: boolean }).rejected) applyTask({ status: 'failed', error: String(error) });
    else { current.uncertain = true; showNotice(String(error)); render(); }
  }
}

button('connect').onclick = connect;
button('refresh').onclick = () => {
  if (busy() && !demo) void readInventory();
  else void refresh();
};
document.addEventListener('visibilitychange', () => {
  if (!document.hidden && !demo) void readInventory();
});
button('angle').onclick = () => scene?.view(false);
button('top').onclick = () => scene?.view(true);
button('minus').onclick = () => { quantity--; render(); };
button('plus').onclick = () => { quantity++; render(); };
button('fetch').onclick = () => void submit('fetch_drug');
button('survey').onclick = () => void submit('survey_warehouse');
button('query').onclick = async () => {
  if (!connection?.token || !task?.pending) return;
  try {
    const reply = await connection.request('get_operation', { request_id: task.requestId });
    task.operationId = reply.operation_id;
    applyTask(reply.result);
  } catch (error) { showNotice(String(error)); }
};
button('cancel').onclick = async () => {
  if (demo) { clearInterval(simulation); applyTask({ status: 'canceled' }); return; }
  if (!connection?.token || !task?.operationId || state?.operation_id !== task.operationId) return;
  cancelPending = true;
  render();
  try {
    const reply = await connection.request('cancel_task');
    if (reply.result.cancel_request === 'rejected') throw new Error(reply.result.cancel_message || '取消请求被拒绝');
  } catch (error) { showNotice(String(error)); cancelPending = false; render(); }
};
el('search').oninput = renderInventory;
el('items').onclick = event => {
  const target = (event.target as HTMLElement).closest<HTMLButtonElement>('button[data-shelf]');
  if (target) choose(target.dataset.shelf!, Number(target.dataset.layer));
};

if (demo) {
  const products = [['酒精', '感冒灵颗粒'], ['阿莫西林', '布洛芬胶囊']];
  layout = { schema_version: 2, map_id: 'demo', revision: 1, layout: { anchor: null, shelves: products.map((drugs, i) => ({
    id: i ? 'B' : 'A', name: i ? 'B 货架' : 'A 货架', width_m: .65, depth_m: .35,
    pose: { x_m: i * .87, y_m: i * .2, yaw_rad: 0 },
    levels: drugs.map((drug, j) => ({ drug, layer: j + 1, height_m: .31 + j * .35,
      qr: { left: null, right: null }, fetch: null }))
  })) } };
  inventory = { map_id: 'demo', persisted: false, load_error: '', save_error: '', layers: rows().map(({shelf, level}, i) => ({
    shelf_id: shelf.id, layer: level.layer, drug: level.drug, quantity: [4, 7, null, 0][i],
    source: null, updated_at: i === 2 ? null : new Date().toISOString(), observed_at: null, stale: false, last_scan: null
  })) };
  choose('A', 1);
} else {
  el('scene').textContent = '连接机器人后查看当前货架。';
  renderInventory();
  render();
}
