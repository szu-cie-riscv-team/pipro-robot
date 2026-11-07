import './warehouse.css';
import { escapeHTML as esc, fetchAdmission, robotMotionBusy, webOperation, id } from './core';
import { Inventory, layerInventory, inventoryTotals } from './warehouse_inventory';
import { MapView } from './map';
import { QrSide, WarehouseDocument, WarehouseShelf, WarehouseLevel, validateWarehouseDocument, buildWarehouseTargets, warehouseQrPayload, markerIdCount } from './warehouse_model';
import { duplicateShelf, nextIdentity, newLevel, warehouseInputs, reasonLabels } from './warehouse_edit';
import { DrugCatalogPanel } from './drug_catalog';
import { WarehouseOperation, WarehouseRequest } from './warehouse_operation';
import type { WarehouseScene, WarehouseSelection, WarehouseSceneEdit } from './warehouse_scene';

type SavedPoint = { x: number; y: number; yaw: number };
const field = (name: string, label: string, value: string | number | null | undefined, numeric = false, suffix = '') =>
  '<label>' + label + '<input data-field="' + name + '" aria-label="' + label + '" ' + (numeric ? 'type="number" step="any"' : 'type="text"') + ' value="' + esc(String(value ?? '')) + '" ' + suffix + '></label>';
const option = (value: string | number, label: string, selected: boolean) => '<option value="' + esc(String(value)) + '"' + (selected ? ' selected' : '') + '>' + esc(label) + '</option>';
const format = (n: number | null | undefined, digits = 3) => n === null || n === undefined || !Number.isFinite(n) ? '未标定' : n.toFixed(digits);

export class WarehousePanel {
  private doc?: WarehouseDocument;
  private baseline?: WarehouseDocument;
  private maps: any[] = [];
  private points: Record<string, SavedPoint> = {};
  private manager: any = { available: false };
  private inventory?: Inventory;
  private inventoryError = '';
  private inventoryLoading = false;
  private inventoryEvent = '';
  private surveyRequest?: string;
  private inventoryVersion = 0;
  private inventoryRenderKey = '';
  private selection: WarehouseSelection = { shelfId: '', layer: null, qr: false };
  private scene?: WarehouseScene;
  private map?: MapView;
  private sceneEpoch = 0;
  private active = false;
  private connected = false;
  private working = false;
  private dirty = false;
  private migrationRequired = false;
  private syncedRevision = 0;
  private state: any;
  private op: WarehouseOperation;
  private catalog: DrugCatalogPanel;
  private pendingMap = '';
  private pendingVersion = 0;
  private editVersion = 0;
  private sceneMode: 'move' | 'rotate' | 'qr' = 'move';
  private reprint = new Set<string>();
  private heightWarnings = new Set<string>();
  private message = '连接机器人后读取已保存的地图和仓库配置。';
  constructor(private root: HTMLElement, private request: WarehouseRequest, private viewsChanged: () => void, private onBusy: (busy: boolean) => void = () => {}) {
    this.op = new WarehouseOperation(request);
    root.innerHTML = [
      '<div class="warehouse-heading"><div><div class="eyebrow">WAREHOUSE / 仓库管理</div><h1>货架布局与药品标记</h1></div><div class="buttons"><button data-action="refresh">读取板端配置</button><button data-action="query" hidden>查询原操作结果</button><button data-action="sync" class="primary">同步到板端</button></div></div>',
      '<p class="hint">在本机浏览器建模；编辑不发送运动。同步后需人工重启任务进程，已运行的进程继续使用旧版本。取药使用任务进程启动时加载的版本。</p>',
      '<div class="warehouse-status" role="status" aria-live="polite"></div><div class="warehouse-message" role="status" aria-live="polite"></div>',
      '<div class="panel warehouse-catalog" id="drug-catalog"></div>',
      '<label class="warehouse-map-select">已保存地图<select data-control="map"><option value="">连接后读取地图</option></select></label>',
      '<div class="warehouse-layout"><aside class="warehouse-editor"></aside><div class="warehouse-main">',
      '<div class="warehouse-viewport panel"><div class="warehouse-tools"><strong>仓库建模工作区</strong><button data-action="3d">3D视角</button><button data-action="top">俯视</button><button data-action="fit">查看全部</button><select data-control="transform" aria-label="模型拖动方式"><option value="move">货架平移 XY</option><option value="rotate">货架旋转 Z</option><option value="qr">标记移动 XZ</option></select></div><div class="warehouse-scene"></div><div class="hint">蓝：当前层 / +Y朝向 · 绿：机器人观察位 · 黄：标记位置 · 红：货架左前脚原点。拖动箭头修改，拖动画面调整视角。</div></div>',
      '<div class="panel warehouse-anchor"></div><div class="panel warehouse-preview"></div>',
      '<div class="panel warehouse-inventory"><h2>药品数量与盘点</h2><div class="buttons"><button data-action="survey">开始整仓盘点</button><button data-action="inventory-refresh">读取数量</button><button data-action="survey-query" hidden>查询原盘点结果</button></div><p class="warehouse-inventory-status"></p><div class="warehouse-inventory-table"></div><p class="hint">单排可见药盒；未统计为未知。任务空闲时可设数量，数量不限制取药。</p></div>',
      '<div class="panel"><div class="warehouse-tools"><h2>地图坐标核对</h2><label class="check"><input data-control="map-view" type="checkbox">查看当前导航地图</label></div><p class="hint warehouse-map-hint"></p><canvas class="warehouse-map" hidden aria-label="地图标点和派生机器人观察位，仅供核对"></canvas></div>',
      '<div class="panel warehouse-objects"></div><div class="panel warehouse-task"><h2>取药操作</h2><p class="hint">单次取同一种药品1～3盒；自动选择首个可执行顶抓层。需先卸空三个运输位置并完成准备。</p><button data-action="check-fetch">检查取药状态</button> <label>数量 <input data-control="fetch-quantity" type="number" min="1" max="3" step="1" value="1"></label> <button data-action="fetch">取所选药品</button><p class="warehouse-admission"></p><p class="warehouse-task-progress"></p> <button data-action="cancel">取消整个任务</button></div></div></div>',
      '<datalist id="warehouse-drugs"></datalist>'
    ].join('');
    this.catalog = new DrugCatalogPanel(this.q('.warehouse-catalog'), this.op, value => {
      this.working = value; this.onBusy(value); this.renderStatus();
    }, () => this.render());
    root.addEventListener('click', e => {
      const b = (e.target as HTMLElement).closest<HTMLButtonElement>('button[data-action]');
      if (b) void (b.dataset.action === 'inventory-save' ? this.saveInventory(b) : this.action(b.dataset.action!)).catch(error => { this.message = String(error); this.render(); });
      const row = (e.target as HTMLElement).closest<HTMLElement>('[data-select]');
      if (row) { this.selection = { shelfId: row.dataset.select!, layer: row.dataset.layer ? Number(row.dataset.layer) : null, qr: false }; this.render(); }
    });
    root.addEventListener('change', e => this.change(e.target as HTMLInputElement));
  }
  private q<T extends HTMLElement = HTMLElement>(selector: string): T { return this.root.querySelector<T>(selector)!; }
  private shelf(): WarehouseShelf | undefined { return this.doc?.layout.shelves.find(s => s.id === this.selection.shelfId); }
  private level(): WarehouseLevel | undefined { return this.shelf()?.levels.find(l => l.layer === this.selection.layer); }
  setConnected(connected: boolean) { this.connected = connected; this.catalog.setConnected(connected); if (!connected) { this.manager = { available: false }; this.inventory = undefined; this.inventoryError = '未连接'; } this.render(); }
  setState(state: any) {
    this.state = state; this.renderStatus(); this.renderMap();
    const op = webOperation(state), f = op.feedback ?? {};
    if (op.operation === 'survey_warehouse') {
      const key = [state.operation_id, f.shelf_id, f.layer, op.status].join('/');
      if (key !== this.inventoryEvent) {
        this.inventoryEvent = key;
        if (this.active && !document.hidden) {
          void this.readInventory();
          if (this.surveyRequest && op.status !== 'running') void this.querySurvey();
        }
      }
    }
  }
  async enter() {
    this.active = true; this.map = new MapView(this.q<HTMLCanvasElement>('.warehouse-map'), false);
    this.render(); await this.ensureScene();
    await this.catalog.enter();
    if (this.connected && !this.dirty && !this.op.pending && !webOperation(this.state).web_busy && !robotMotionBusy(this.state)) await this.refresh();
    else if (this.connected) await this.readInventory();
  }
  leave() {
    this.active = false; this.sceneEpoch++; this.scene?.dispose(); this.scene = undefined; this.map?.dispose(); this.map = undefined;
    // Keep the local draft; no persistence or RPC on leave/reconnect.
  }
  async connectedEnter() { this.setConnected(true); if(this.active) await this.catalog.enter(); if (this.active && !this.doc && !this.op.pending) await this.refresh(); }
  wantsMap(): boolean {
    return this.active && this.connected && this.q<HTMLInputElement>('[data-control="map-view"]').checked &&
      !!this.doc && this.state?.mode === 'NAVIGATION' && this.state?.map_id === this.doc.map_id;
  }
  mapMessage(msg: any) { if (this.wantsMap()) this.map?.message('map', msg); }
  clearMap() { this.map?.clear(); }
  private async ensureScene() {
    if (this.scene || !this.active) return;
    const epoch = ++this.sceneEpoch;
    try {
      const { WarehouseScene } = await import('./warehouse_scene');
      if (!this.active || epoch !== this.sceneEpoch) return;
      this.scene = new WarehouseScene(this.q('.warehouse-scene'), selection => { this.selection = selection; if (selection.qr) this.sceneMode = 'qr'; else if (this.sceneMode === 'qr') this.sceneMode = 'move'; this.q<HTMLSelectElement>('[data-control="transform"]').value = this.sceneMode; this.render(); }, change => this.sceneEdit(change));
      this.updateScene();
    } catch (error) { this.q('.warehouse-scene').textContent = '三维预览不可用；仍可通过数值编辑。' + String(error); }
  }
  private inputs() {
    const inputs = warehouseInputs(this.manager.capabilities);
    if(this.catalog.saved) inputs.canonical_drugs = this.catalog.saved.map(drug => drug.name);
    return inputs;
  }
  private updateScene() { if (this.doc) { this.scene?.setMode(this.sceneMode); this.scene?.update(this.doc, this.selection, this.inputs()); } }
  private renderStatus() {
    this.catalog.setBlocked(!!webOperation(this.state).web_busy && !this.op.pending);
    const loaded = !this.manager.available ? '不可查询' : this.manager.loaded_map_id !== this.doc?.map_id ? '其他地图 / 未加载' : this.manager.loaded_revision ?? '未加载';
    const match = this.manager.available && this.manager.loaded_map_id === this.doc?.map_id && this.manager.loaded_revision === this.syncedRevision;
    const status = this.working ? '处理中' : this.op.pending ? '结果未知，待查询原操作' : this.dirty ? '本地未同步' : this.syncedRevision ? (match ? '与当前进程版本一致' : '已同步，待人工重启确认') : '尚未同步';
    this.q('.warehouse-status').textContent = status + ' ｜ 板端已同步：' + this.syncedRevision + ' ｜ 进程已加载：' + loaded + (!this.connected ? ' ｜ 未连接' : '');
    this.q('.warehouse-message').textContent = this.message;
    const disable = this.working || !!this.op.pending || !this.connected;
    this.q<HTMLButtonElement>('[data-action="refresh"]').disabled = disable;
    this.q<HTMLButtonElement>('[data-action="sync"]').disabled = disable || !this.doc || !this.dirty;
    const query = this.q<HTMLButtonElement>('[data-action="query"]'); query.hidden = !this.op.pending; query.disabled = !this.connected || this.working;
    this.q<HTMLSelectElement>('[data-control="map"]').disabled = disable;
    // Editing during an unknown mutation would make revision attribution ambiguous. Resume first.
    this.q('.warehouse-editor').querySelectorAll<HTMLInputElement | HTMLButtonElement | HTMLSelectElement>('input,button,select').forEach(e => e.disabled = this.working || !!this.op.pending || !this.doc);
    this.q('.warehouse-anchor').querySelectorAll<HTMLInputElement | HTMLButtonElement | HTMLSelectElement>('input,button,select').forEach(e => e.disabled = this.working || !!this.op.pending || !this.doc);
    this.q<HTMLButtonElement>('[data-action="cancel"]').disabled = !this.connected || this.working || !!this.op.pending;
    this.q<HTMLSelectElement>('[data-control="transform"]').disabled = this.working || !!this.op.pending;
    const operation = webOperation(this.state), admission = fetchAdmission(this.manager, this.level()?.drug);
    this.q<HTMLButtonElement>('[data-action="check-fetch"]').disabled = disable || operation.web_busy || robotMotionBusy(this.state);
    this.q<HTMLButtonElement>('[data-action="fetch"]').disabled = disable || this.dirty || !match ||
      !admission.ready || !this.level()?.drug || this.state?.mode !== 'NAVIGATION' ||
      !this.state?.navigation_ready || operation.web_busy || operation.status === 'running' || robotMotionBusy(this.state);
    this.q('.warehouse-admission').textContent = '最近准入检查：' + (admission.ready ? '可提交（提交时仍核对实际状态）' : admission.reason) + '；状态改变后可重新检查。';
    this.q('.warehouse-task-progress').textContent = operation.phase === 'admission_unresolved' ?
      'Action受理结果未确认，请查询状态，不要重复下发。' :
      (operation.phase ? '任务阶段：' + operation.phase : operation.error || operation.result?.message || '');
    const progress = operation.result ?? operation;
    if (Number.isInteger(progress.requested_quantity)) {
      this.q('.warehouse-task-progress').textContent += ' ｜ 请求 ' + progress.requested_quantity +
        ' / 计划 ' + (progress.planned_quantity ?? '—') + ' / 已取 ' + (progress.fetched_quantity ?? 0);
    }
    if (this.manager.transport) {
      const t = this.manager.transport;
      this.q('.warehouse-task-progress').textContent += ' ｜ 最近运输状态：左 ' + t.left + ' / 右 ' + t.right + ' / 手 ' + t.hand;
    }
    if (operation.operation === 'survey_warehouse' && operation.feedback) {
      const f = operation.feedback;
      this.q('.warehouse-task-progress').textContent += ' ｜ ' + f.shelf_id + '/' + f.layer +
        ' ｜ ' + (f.direction === 'left_to_right' ? '左→右' : f.direction === 'right_to_left' ? '右→左' : '') +
        ' ｜ 第 ' + f.attempt + ' 次 ｜ 暂计 ' + (f.provisional_count >= 0 ? f.provisional_count : '—');
    }
    for (const side of ['left', 'right'] as const) {
      const print = this.root.querySelector<HTMLButtonElement>('[data-action="print-' + side + '"]');
      if (print) print.disabled = this.working || !!this.op.pending || !this.level() || !warehouseQrPayload(this.selection.shelfId, this.level()!, side) ||
        (this.level()?.qr[side]?.marker_id != null && !!this.doc && markerIdCount(this.doc, this.level()!.qr[side]!.marker_id!) !== 1);
    }
    this.scene?.setEditable(!this.working && !this.op.pending);
    this.renderInventory();
  }
  private renderInventory() {
    const busy = this.working || !!this.op.pending || robotMotionBusy(this.state) || webOperation(this.state).web_busy;
    const key = [this.inventoryVersion, this.doc?.map_id, this.editVersion, busy, this.connected, this.inventoryError, this.inventoryLoading,
      this.surveyRequest, this.state?.mode, this.state?.map_id, this.state?.navigation_ready].join('/');
    if (key === this.inventoryRenderKey) return;
    this.inventoryRenderKey = key;
    this.q<HTMLButtonElement>('[data-action="survey"]').disabled = !this.connected || busy || !!this.surveyRequest ||
      !this.doc || this.doc.map_id !== this.state?.map_id || !this.state?.navigation_ready;
    const mapId = this.doc?.map_id ?? this.state?.map_id;
    this.q<HTMLButtonElement>('[data-action="inventory-refresh"]').disabled = !this.connected || this.inventoryLoading || !mapId;
    const query = this.q<HTMLButtonElement>('[data-action="survey-query"]'); query.hidden = !this.surveyRequest; query.disabled = !this.connected;
    const data = this.inventory?.map_id === mapId ? this.inventory : undefined;
    this.q('.warehouse-inventory-status').textContent = this.inventoryError || (this.inventoryLoading ? '读取数量中…' :
      !data ? '数量暂不可查询' : data.load_error || data.save_error || (!data.persisted ? '内存数量尚未保存' : '数量已读回'));
    const rows: { drug: string; quantity: number | null | undefined }[] = [];
    const time = (v: string | null | undefined) => v ? new Date(v).toLocaleString() : '—';
    const displayLayers = this.doc ? this.doc.layout.shelves.flatMap(s => s.levels.map(l => ({ shelf: s.id, layer: l.layer, drug: l.drug }))) :
      (data?.layers ?? []).map(r => ({ shelf: r.shelf_id, layer: r.layer, drug: r.drug }));
    const body = displayLayers.map(l => {
      const s = { id: l.shelf };
      const r = layerInventory(data, mapId, s.id, l.layer, l.drug); rows.push({ drug: l.drug, quantity: r?.quantity });
      const key = 'data-shelf="' + esc(s.id) + '" data-layer="' + l.layer + '"';
      const reason = r?.last_scan?.status === 'incomplete' ? '未完成：' + r.last_scan.reason : '';
      return '<tr><td>' + esc(s.id) + '/' + l.layer + '</td><td>' + esc(l.drug || '未分配') + '</td><td>' + (r?.quantity ?? '未知') +
        (r?.stale ? '（旧配置记录）' : '') + '</td><td>' + esc(r?.source ?? '—') + '</td><td>' + esc(time(r?.updated_at)) +
        '</td><td>' + esc(time(r?.observed_at)) + '</td><td>' + esc(reason) + '</td><td><input aria-label="' + esc(s.id) + '/' + l.layer + '数量" type="number" min="0" max="2147483647" step="1" data-quantity ' + key +
        ' value="' + (r?.quantity ?? '') + '" ' + (busy || !this.connected || !l.drug || !data || !this.doc ? 'disabled' : '') + '><button data-action="inventory-save" ' + key +
        (busy || !this.connected || !l.drug || !data || !this.doc ? ' disabled' : '') + '>设值</button></td></tr>';
    }).join('');
    const totals = [...inventoryTotals(rows)].map(([drug, t]) => esc(drug) + '：' + (t.unknown ? '已知层小计 ' + t.known + '，未知 ' + t.unknown + ' 层' : '共 ' + t.known)).join('；');
    this.q('.warehouse-inventory-table').innerHTML = '<p>' + totals + '</p><div style="overflow:auto"><table><thead><tr><th>架/层</th><th>药品</th><th>数量</th><th>来源</th><th>数量更新时间</th><th>实物盘点时间</th><th>最近扫描</th><th>手动数量</th></tr></thead><tbody>' + body + '</tbody></table></div>';
  }
  private async readInventory() {
    const map = this.doc?.map_id ?? this.state?.map_id;
    if (!map || !this.connected || this.inventoryLoading || !this.active || document.hidden) return;
    this.inventoryLoading = true;
    try {
      const response = await this.request('inventory_get', { map_id: map });
      if ((this.doc?.map_id ?? this.state?.map_id) === map) { this.inventory = response.result; this.inventoryError = ''; this.inventoryVersion++; }
    } catch (error) { if ((this.doc?.map_id ?? this.state?.map_id) === map) { this.inventory = undefined; this.inventoryError = '数量暂不可查询：' + String(error); } }
    finally { this.inventoryLoading = false; if (this.active) this.renderInventory(); }
  }
  private async saveInventory(button: HTMLButtonElement) {
    const shelf = button.dataset.shelf!, layer = Number(button.dataset.layer);
    const level = this.doc?.layout.shelves.find(s => s.id === shelf)?.levels.find(l => l.layer === layer);
    const input = button.previousElementSibling as HTMLInputElement;
    const quantity = input.value.trim() === '' ? NaN : input.valueAsNumber;
    if (!level?.drug || !Number.isInteger(quantity) || quantity < 0 || quantity > 2147483647) throw new Error('请输入非负整数数量');
    const result = await this.execute('inventory_set', { map_id: this.doc!.map_id, shelf_id: shelf, layer, drug: level.drug, quantity });
    this.message = result.persisted ? '数量已保存。' : '内存数量已修改、尚未保存：' + (result.save_error || result.error || '');
    await this.readInventory(); this.renderStatus();
  }
  private async querySurvey() {
    if (!this.surveyRequest || !this.connected) return;
    const requestId = this.surveyRequest;
    try {
      const response = await this.request('get_operation', { request_id: requestId });
      if (this.surveyRequest !== requestId) return;
      const result = response.result;
      if (['succeeded', 'failed', 'canceled'].includes(result.status)) {
        this.surveyRequest = undefined;
        const r = result.result ?? {};
        this.message = '盘点：' + (r.completion === 'complete' ? '全部完成' : '部分完成 / 已取消或失败') +
          '；返航：' + (r.return_status ?? '未知') + (result.error ? '；' + result.error : '');
        await this.readInventory();
      } else this.message = '盘点仍在进行；' + (result.phase ?? '等待反馈');
    } catch (error) { this.message = '盘点回执未确认，请查询原操作：' + String(error); }
    this.renderStatus();
  }
  private render() {
    const shelves = this.doc?.layout.shelves ?? [], s = this.shelf(), l = this.level();
    const oldDetails = this.root.querySelector<HTMLDetailsElement>('.warehouse-debug')?.open;
    const editor = [ '<div class="panel"><h2>货架参数</h2><label>选择货架<select data-control="shelf">', option('', '选择货架', !s),
      ...shelves.map(v => option(v.id, (v.name || v.id) + ' · ' + v.id, v.id === s?.id)),
      '</select></label><div class="buttons"><button data-action="add-shelf">新增货架</button>', s ? '<button data-action="copy-shelf">复制</button><button data-action="delete-shelf" class="danger">删除</button>' : '', '</div>',
      s ? field('shelf.id', '货架 ID', s.id) + field('shelf.name', '显示名称', s.name) + '<div class="warehouse-fields">' + field('shelf.width_m', '宽度（cm）', s.width_m * 100, true) + field('shelf.depth_m', '深度（cm）', s.depth_m * 100, true) + '</div><h2>货架布局 · 相对布局原点</h2><div class="warehouse-fields">' + field('pose.x_m', '布局 X（m）', s.pose.x_m, true) + field('pose.y_m', '布局 Y（m）', s.pose.y_m, true) + field('pose.yaw_rad', '布局朝向（度）', s.pose.yaw_rad * 180 / Math.PI, true) + '</div><p class="hint">原点为货架左前脚地面点；X向右、Y向架内、Z向上。朝向为绕Z旋转。</p>' : '<p class="hint">先选择地图，再添加货架。默认两层上表面距地30/60cm，宽50cm、深20cm。</p>', '</div>'
    ];
    if (s) editor.push('<div class="panel"><h2>层与药品</h2><label>选择层<select data-control="layer">' + option('', '选择层', !l) + s.levels.map(v => option(v.layer, v.layer + '层 · ' + (v.drug || '未标药品'), v.layer === l?.layer)).join('') + '</select></label><button data-action="add-layer">新增层</button> ' + (l ? '<button data-action="delete-layer" class="danger">删除此层</button><div class="warehouse-fields">' + field('level.layer', '层 ID（不自动重编号）', l.layer, true) + field('level.height_m', '层板上表面距地（cm）', l.height_m * 100, true) + '</div>' + field('level.drug', '规范药品名称', l.drug, false, 'list="warehouse-drugs"') + '<p class="hint">每层一种；不管理数量。未知名称可保存，但暂不可取。</p>' : '') + '</div>');
    if (l) {
      const changed = this.reprint.has(this.key()), heightChanged = this.heightWarnings.has(this.key());
      editor.push('<div class="panel"><h2>左右 ArUco 标记 · 40×40 mm纸面</h2><p class="hint">标记 ID 对应本层边界，修改货架或药名不改变 ID。修改 ID 后须更换纸码。立柱正面Y=0，位置为标记中心。</p>' +
        (['left', 'right'] as const).map(side => {
          const marker = l.qr[side], title = side === 'left' ? '左' : '右';
          return '<h3>' + title + '边界</h3>' + (marker ? '<div class="warehouse-fields">' +
            field('qr.' + side + '.x_m', title + '码 X（cm）', marker.x_m * 100, true) +
            field('qr.' + side + '.z_m', title + '码 Z（cm）', marker.z_m * 100, true) +
            field('qr.' + side + '.marker_id', title + '标记 ID（0～49，留空为未配置）', marker.marker_id, true) +
            '</div>' + (marker.marker_id == null ? '<p class="warehouse-warning">标记 ID 未配置。</p>' : this.doc && markerIdCount(this.doc, marker.marker_id) > 1 ? '<p class="warehouse-warning">ID ' + marker.marker_id + ' 与其他边界冲突，请修改。</p>' : '') + '<div class="buttons"><button data-action="assign-' + side + '">分配空闲 ID</button><button data-action="select-' + side + '-qr">拖动</button><button data-action="delete-' + side + '-qr">删除</button><button data-action="print-' + side + '">预览与打印</button></div>' :
            '<button data-action="add-' + side + '-qr">新增' + title + '标记</button>');
        }).join('') + (changed ? '<p class="hint">架层名称已变，请核对纸码旁的文字标签。</p>' : '') +
        (heightChanged ? '<p class="warehouse-warning">层高已变；左右码 Z 保持原实测值，请人工核对。</p>' : '') + '</div>');
      const f = l.fetch;
      editor.push('<div class="panel"><h2>取药配置</h2><p class="hint">药名和模型类别由本页管理；运行抓法由启动 YAML 选择。</p>' +
        field('fetch.class_id', '模型 class_id', f?.class_id) + '<p>运行抓法：' + esc(this.inputs().grasp_mode ?? '未加载') + '</p></div>' +
        '<details class="panel warehouse-debug"' + (oldDetails ? ' open' : '') + '><summary>抓取调试参数</summary>' +
        field('fetch.roi', '归一化 ROI：x1,y1,x2,y2', f?.roi?.join(', ')) + '<div class="warehouse-fields">' + field('fetch.search_min_m', '搜索起点（m）', f?.search_min_m, true) + field('fetch.search_max_m', '搜索距离上限（m）', f?.search_max_m, true) + '</div><p class="hint">右码到参考线结束搜索；距离上限仅作漏码兜底。</p><button data-action="clear-fetch">清空此层抓取参数</button></details>');
    }
    this.q('.warehouse-editor').innerHTML = editor.join('');
    this.q<HTMLButtonElement>('[data-action="add-shelf"]').disabled = !this.doc;
    const a = this.doc?.layout.anchor;
    const anchorShelf = shelves.find(v => v.id === a?.shelf_id) ?? s;
    this.q('.warehouse-anchor').innerHTML = '<h2>地图锚定 · 机器人在参考立柱前的停车点</h2><div class="warehouse-fields"><label>参考货架<select data-control="anchor-shelf">' + option('', '选择货架', !anchorShelf) + shelves.map(v => option(v.id, v.name || v.id, v.id === anchorShelf?.id)).join('') + '</select></label><label>参考层<select data-control="anchor-layer">' + (anchorShelf?.levels.filter(v => v.qr.left).map(v => option(v.layer, v.layer + '层 · 左码', v.layer === (a?.layer ?? l?.layer))).join('') || option('', '先设置边界标记', true)) + '</select></label><label>已保存机器人标点<select data-control="anchor-point">' + option('', '选择已实测标点', !a) + Object.keys(this.points).map(name => option(name, name, name === a?.point_name)).join('') + '</select></label><label>相机至立柱平面（cm）<input data-control="anchor-distance" type="number" step="any" value="' + (a?.camera_distance_m !== undefined ? a.camera_distance_m * 100 : 30) + '"></label></div><div class="buttons"><button data-action="bind-anchor">使用此标点与参考左标记</button><button data-action="clear-anchor">解除锚定</button></div><p class="hint">标点是 base_link 停车位，不是货架原点。箭头是机器人面对的方向，即货架 +Y。相机安装位置由进程只读提供，不在网页猜测。</p>' + (a ? '<p class="hint">当前绑定：' + esc(a.shelf_id) + ' / ' + a.layer + '层 → ' + esc(a.point_name) + '；map (' + format(a.point_pose.x_m) + ', ' + format(a.point_pose.y_m) + ')，朝向 ' + format(a.point_pose.yaw_rad * 180 / Math.PI, 1) + '°</p>' : '<p class="warehouse-warning">尚未锚定，不能生成导航观察位。</p>');
    const inputs = this.inputs(), targets = this.doc ? buildWarehouseTargets(this.doc, inputs) : [];
    const target = targets.find(v => v.shelf_id === s?.id && v.layer === l?.layer);
    this.q('.warehouse-preview').innerHTML = '<h2>本地草稿预览</h2><p class="hint">这里只预览本地设置；进程当前使用的版本见顶部“已加载”。不会在线应用配置。</p><p>' + (target ? '当前层：' + esc(target.drug || '未标药品') + '；距地 ' + format(target.height_m * 100, 1) + ' cm；初始升降 Z ' + format(target.initial_lift_z_m) + ' m<br>底盘观察位 map：' + (target.observation_pose ? '(' + format(target.observation_pose.x_m) + ', ' + format(target.observation_pose.y_m) + ') · ' + format(target.observation_pose.yaw_rad * 180 / Math.PI, 1) + '°' : '缺少锚定或标定，无法推算') + '<br>' + (target.unavailable_reasons.map(r => esc(reasonLabels[r] ?? r)).join('；') || '草稿数据齐备，实际执行以进程加载版本和现场标定为准') : '选择货架中的一层查看。') + '</p><details><summary>只读机械参数与进程状态</summary><pre>' + esc(JSON.stringify({ manager_available: this.manager.available, instance_id: this.manager.instance_id ?? null, loaded_map_id: this.manager.loaded_map_id ?? null, loaded_revision: this.manager.loaded_revision ?? null, load_error: this.manager.load_error ?? null, capabilities: this.manager.capabilities ?? null }, null, 2)) + '</pre></details>';
    this.q('.warehouse-objects').innerHTML = '<h2>对象清单</h2><div class="warehouse-table"><table><thead><tr><th>货架 / 层</th><th>药品 / 左右码</th><th>参数状态</th></tr></thead><tbody>' + shelves.map(v => '<tr data-select="' + esc(v.id) + '" class="' + (v.id === s?.id ? 'selected' : '') + '"><td><button data-select="' + esc(v.id) + '">' + esc(v.name || v.id) + '</button></td><td>' + esc(v.id) + '</td><td>布局 (' + format(v.pose.x_m) + ', ' + format(v.pose.y_m) + ') / ' + format(v.pose.yaw_rad * 180 / Math.PI, 1) + '°</td></tr>' + v.levels.map(level => { const t = targets.find(t => t.shelf_id === v.id && t.layer === level.layer)!; return '<tr class="' + (v.id === s?.id && level.layer === l?.layer ? 'selected' : '') + '"><td><button data-select="' + esc(v.id) + '" data-layer="' + level.layer + '">' + esc(v.id) + ' / ' + level.layer + '层</button></td><td>' + esc(level.drug || '未标药品') + '<br><small>' + (level.qr.left ? '左码已设' : '缺左码') + ' / ' + (level.qr.right ? '右码已设' : '缺右码') + '</small></td><td>' + (t.unavailable_reasons.map(r => esc(reasonLabels[r] ?? r)).join('；') || '草稿数据齐备') + '</td></tr>'; }).join('')).join('') + '</tbody></table></div>';
    this.q('#warehouse-drugs').innerHTML = inputs.canonical_drugs.map(v => '<option value="' + esc(v) + '"></option>').join('');
    this.renderMap(); this.updateScene(); this.renderStatus();
    // Preserve per-action disabled conditions after applying page busy state.
    if (!this.doc) this.q<HTMLButtonElement>('[data-action="add-shelf"]').disabled = true;
  }
  private key() { return this.selection.shelfId + ':' + this.selection.layer; }
  private edit(mutator: (doc: WarehouseDocument, shelf?: WarehouseShelf, level?: WarehouseLevel) => void) {
    if (!this.doc || this.working || this.op.pending) return;
    const oldSelection = { ...this.selection };
    const next = structuredClone(this.doc), shelf = next.layout.shelves.find(s => s.id === this.selection.shelfId), level = shelf?.levels.find(l => l.layer === this.selection.layer);
    const oldKey = this.key(), before = this.level() ? JSON.stringify({ shelf: this.selection.shelfId, layer: this.level()!.layer }) : '';
    try {
      mutator(next, shelf, level); validateWarehouseDocument(next);
      this.doc = next; this.editVersion++; this.dirty = this.migrationRequired || JSON.stringify(this.doc.layout) !== JSON.stringify(this.baseline?.layout);
      const after = this.level() ? JSON.stringify({ shelf: this.selection.shelfId, layer: this.level()!.layer }) : '';
      if (before !== after) { this.reprint.add(oldKey); this.reprint.add(this.key()); }
      if (shelf && shelf.id !== oldSelection.shelfId) {
        for (const l of shelf.levels) this.reprint.add(shelf.id + ':' + l.layer);
      }
      this.message = '已更新本地草稿，尚未发送到板端。';
    } catch (error) { this.selection = oldSelection; this.message = '未应用此项修改：' + String(error); }
    this.render();
  }
  private sceneEdit(change: WarehouseSceneEdit) {
    this.edit((_doc, shelf, level) => {
      if (change.kind === 'qr' && level && this.selection.qr && level.qr[this.selection.qr]) { const marker = level.qr[this.selection.qr]!; marker.x_m = change.x; marker.z_m = change.z; }
      else if (change.kind === 'shelf' && shelf) shelf.pose = { x_m: change.x, y_m: change.y, yaw_rad: change.yaw };
    });
  }
  private change(input: HTMLInputElement) {
    if (input.dataset.control === 'map') {
      if (this.dirty) { input.value = this.doc!.map_id; this.message = '本地有未同步修改；先同步，或点击“读取板端配置”明确丢弃，再切换地图。'; this.renderStatus(); return; }
      void this.loadMap(input.value).catch(error => { this.message = String(error); this.render(); }); return;
    }
    if (input.dataset.control === 'shelf') { this.selection = { shelfId: input.value, layer: null, qr: false }; this.render(); return; }
    if (input.dataset.control === 'layer') { this.selection.layer = input.value ? Number(input.value) : null; this.selection.qr = false; this.render(); return; }
    if (input.dataset.control === 'map-view') { this.renderMap(); this.viewsChanged(); return; }
    if (input.dataset.control === 'transform') { this.sceneMode = input.value as typeof this.sceneMode; this.updateScene(); return; }
    if (input.dataset.control === 'anchor-shelf') {
      const shelf = this.doc?.layout.shelves.find(s => s.id === input.value);
      this.q<HTMLSelectElement>('[data-control="anchor-layer"]').innerHTML = shelf?.levels.filter(l => l.qr.left).map(l => option(l.layer, l.layer + '层 · 左码', false)).join('') || option('', '先设置边界标记', true); return;
    }
    if (!input.dataset.field) return;
    const [kind, name, coordinate] = input.dataset.field.split('.');
    this.edit((doc, shelf, level) => {
      let value: any = input.type === 'number' ? input.valueAsNumber : input.value;
      if (kind === 'shelf' && shelf) {
        if (name === 'width_m' || name === 'depth_m') value /= 100;
        if (name === 'id') { if (doc.layout.anchor?.shelf_id === shelf.id) doc.layout.anchor.shelf_id = value; this.selection.shelfId = value; }
        (shelf as any)[name] = value;
      } else if (kind === 'pose' && shelf) (shelf.pose as any)[name] = name === 'yaw_rad' ? value * Math.PI / 180 : value;
      else if (kind === 'level' && level) {
        if (name === 'height_m') { value /= 100; if ((level.qr.left || level.qr.right) && value !== level.height_m) this.heightWarnings.add(this.key()); }
        if (name === 'layer') { if (doc.layout.anchor?.shelf_id === shelf!.id && doc.layout.anchor.layer === level.layer) doc.layout.anchor.layer = value; this.selection.layer = value; }
        (level as any)[name] = value;
      } else if (kind === 'qr' && level && level.qr[name as QrSide]) (level.qr[name as QrSide] as any)[coordinate] = coordinate === 'marker_id' ? (input.value.trim() === '' ? null : value) : value / 100;
      else if (kind === 'fetch' && level) {
        level.fetch ??= { class_id: null, grasp: null, roi: null, search_min_m: null, search_max_m: null };
        if (input.value.trim() === '') value = null;
        else if (name === 'roi') { const parts = input.value.split(','); if (parts.length !== 4 || parts.some(v => !v.trim())) throw new Error('ROI需填写四个明确的数字'); value = parts.map(v => Number(v.trim())); }
        (level.fetch as any)[name] = value;
      }
    });
  }
  private async action(action: string) {
    if (action === 'inventory-refresh') { await this.readInventory(); return; }
    if (action === 'survey-query') { await this.querySurvey(); return; }
    if (action === 'survey') {
      if (!this.doc || this.surveyRequest || this.q<HTMLButtonElement>('[data-action="survey"]').disabled) return;
      this.surveyRequest = id(); this.working = true; this.renderStatus();
      try {
        await this.request('survey_warehouse', { map_id: this.doc.map_id }, this.surveyRequest);
        this.message = '已提交整仓盘点；完成后返回起点，可取消。';
      } catch (error) {
        if ((error as any)?.rejected) this.surveyRequest = undefined;
        this.message = '盘点提交未确认：' + String(error) + '；请先查询原操作。';
      } finally { this.working = false; this.renderStatus(); }
      return;
    }
    if (action === 'refresh') {
      if (this.dirty && !confirm('读取板端配置会丢弃当前本地草稿，继续？')) return;
      await this.refresh(); return;
    }
    if (action === 'sync') { await this.sync(); return; }
    if (action === 'query') { await this.query(); return; }
    if (action === 'check-fetch') {
      const data = await this.execute('warehouse_status', {});
      this.manager = data.manager; this.render(); return;
    }
    if (action === 'fetch') {
      if (this.q<HTMLButtonElement>('[data-action="fetch"]').disabled) return;
      const drug = this.level()?.drug; if (!drug) return;
      const quantity = Number(this.q<HTMLInputElement>('[data-control="fetch-quantity"]').value);
      if (!Number.isInteger(quantity) || quantity < 1 || quantity > 3) { this.message = '数量请输入1～3'; this.renderStatus(); return; }
      this.working = true; this.renderStatus();
      try {
        await this.request('fetch_drug', { drug_name: drug, quantity });
        this.message = '已提交取药：' + drug + '，请求' + quantity + '盒。请观察任务阶段；受理不表示抓取完成。';
      } catch (error) {
        this.message = '提交未确认：' + String(error) + '。请先检查当前任务状态，不要重复下发。';
      } finally { this.manager.fetch_admission = {}; this.manager.fetch_blocked_reason = 'status_unknown'; this.working = false; this.renderStatus(); }
      return;
    }
    if (action === 'cancel') { await this.request('cancel_task', {}); this.message = '已请求取消整个任务；请求受理不表示硬件已停止，以实际反馈为准。'; this.render(); return; }
    if (action === '3d' || action === 'top') { this.scene?.view(action === 'top'); return; }
    if (action === 'fit') { this.scene?.fit(); return; }
    if (action === 'print-left' || action === 'print-right') {
      const side: QrSide = action === 'print-left' ? 'left' : 'right', shelfId = this.selection.shelfId;
      const level = this.level(); if (!level) return;
      const snapshot = structuredClone(level);
      const { warehouseQrSvg, printWarehouseQr } = await import('./warehouse_qr');
      if (!this.active || this.selection.shelfId !== shelfId || !this.level() || JSON.stringify(this.level()) !== JSON.stringify(snapshot)) return;
      const id = snapshot.qr[side]?.marker_id;
      if (id == null || !this.doc || markerIdCount(this.doc, id) !== 1) { this.message = '标记 ID 未配置或发生冲突，请修正后打印。'; this.renderStatus(); return; }
      const qr = warehouseQrSvg(shelfId, snapshot, side);
      this.message = 'ArUco ID ' + qr.id + '，纸面40 mm、黑色码区30 mm；打印选择100%，请用直尺核对并替换旧纸码。';
      this.renderStatus(); printWarehouseQr(shelfId, snapshot, side); return;
    }
    if (action === 'select-left-qr' || action === 'select-right-qr') {
      this.sceneMode = 'qr'; this.selection.qr = action === 'select-left-qr' ? 'left' : 'right'; this.q<HTMLSelectElement>('[data-control="transform"]').value = 'qr'; this.updateScene(); return;
    }
    if (action === 'bind-anchor') {
      const shelf_id = this.q<HTMLSelectElement>('[data-control="anchor-shelf"]').value, layer = Number(this.q<HTMLSelectElement>('[data-control="anchor-layer"]').value);
      const point_name = this.q<HTMLSelectElement>('[data-control="anchor-point"]').value, point = this.points[point_name];
      const camera_distance_m = this.q<HTMLInputElement>('[data-control="anchor-distance"]').valueAsNumber / 100;
      if (!point) throw new Error('请先选择已保存且实测的机器人标点');
      this.edit(doc => { doc.layout.anchor = { shelf_id, layer, point_name, point_pose: { x_m: point.x, y_m: point.y, yaw_rad: point.yaw }, camera_distance_m }; }); return;
    }
    this.edit((doc, shelf, level) => {
      if (action === 'add-shelf') {
        const first = doc.layout.shelves.length === 0;
        const id = first ? 'A' : nextIdentity('shelf_', doc.layout.shelves.map(s => s.id));
        const added: WarehouseShelf = { id, name: first ? '货架 A' : '新货架', width_m: .5, depth_m: .2, pose: { x_m: doc.layout.shelves.length * .7, y_m: 0, yaw_rad: 0 }, levels: [newLevel(1, .3), newLevel(2, .6)] };
        if (first) { added.levels[0].drug = '酒精'; added.levels[1].drug = '感冒灵颗粒'; }
        doc.layout.shelves.push(added); this.selection = { shelfId: id, layer: 1, qr: false };
      } else if (action === 'copy-shelf' && shelf) { const copy = duplicateShelf(doc, shelf); doc.layout.shelves.push(copy); this.selection = { shelfId: copy.id, layer: null, qr: false }; }
      else if (action === 'delete-shelf' && shelf) { if (doc.layout.anchor?.shelf_id === shelf.id) doc.layout.anchor = null; doc.layout.shelves = doc.layout.shelves.filter(s => s !== shelf); this.selection = { shelfId: doc.layout.shelves[0]?.id ?? '', layer: null, qr: false }; }
      else if (action === 'add-layer' && shelf) { const layer = Math.max(0, ...shelf.levels.map(l => l.layer)) + 1; shelf.levels.push(newLevel(layer, Math.max(0, ...shelf.levels.map(l => l.height_m)) + .3)); this.selection.layer = layer; }
      else if (action === 'delete-layer' && level) { if (doc.layout.anchor?.shelf_id === shelf!.id && doc.layout.anchor.layer === level.layer) doc.layout.anchor = null; shelf!.levels = shelf!.levels.filter(l => l !== level); this.selection.layer = null; }
      else if ((action === 'add-left-qr' || action === 'add-right-qr') && level) {
        const side: QrSide = action === 'add-left-qr' ? 'left' : 'right';
        level.qr[side] = { x_m: side === 'left' ? 0 : shelf!.width_m, z_m: level.height_m };
      } else if ((action === 'delete-left-qr' || action === 'delete-right-qr') && level) {
        const side: QrSide = action === 'delete-left-qr' ? 'left' : 'right';
        if (side === 'left' && doc.layout.anchor?.shelf_id === shelf!.id && doc.layout.anchor.layer === level.layer) doc.layout.anchor = null;
        level.qr[side] = null; if (this.selection.qr === side) this.selection.qr = false;
      }
      else if ((action === 'assign-left' || action === 'assign-right') && level) {
        const side: QrSide = action === 'assign-left' ? 'left' : 'right';
        const marker = level.qr[side]; if (!marker) return;
        if (marker.marker_id != null) throw new Error('该边界已有ID，请直接编辑以更换');
        const id = Array.from({ length: 50 }, (_, i) => i).find(i => markerIdCount(doc, i) === 0);
        if (id === undefined) throw new Error('标记ID已用完（0～49）');
        marker.marker_id = id;
      }
      else if (action === 'clear-fetch' && level) level.fetch = null;
      else if (action === 'clear-anchor') doc.layout.anchor = null;
    });
    if (action.startsWith('delete-')) { this.message = '已在本地删除；关联锚定如受影响已解除。实体旧纸码请人工移除。'; this.renderStatus(); }
  }
  private async execute(operation: string, args: any) {
    this.working = true; this.onBusy(true); this.renderStatus();
    try { return await this.op.start(operation, args); }
    finally { this.working = false; this.onBusy(!!this.op.pending); this.renderStatus(); if (this.active && !this.op.pending) await this.ensureScene(); }
  }
  private async refresh() {
    if (!this.connected || this.working || this.op.pending) return;
    if (robotMotionBusy(this.state) || webOperation(this.state).web_busy) {
      await this.readInventory(); this.message = '任务中仅读取数量；结束后可读取仓库几何配置。'; this.renderStatus(); return;
    }
    this.pendingMap = '';
    const data = await this.execute('list_maps', {});
    this.maps = data.maps;
    const selected = this.doc?.map_id ?? this.state?.map_id;
    const mapId = this.maps.find(m => m.map_id === selected)?.map_id ?? this.maps[0]?.map_id ?? '';
    this.q('[data-control="map"]').innerHTML = option('', '选择已保存地图', !mapId) + this.maps.map(m => option(m.map_id, m.name, m.map_id === mapId)).join('');
    if (mapId) await this.loadMap(mapId); else { this.message = '暂无已保存地图；先到“建图与导航”建图、保存地图并设置机器人标点。'; this.render(); }
  }
  private async loadMap(mapId: string) {
    if (!mapId) { this.q<HTMLSelectElement>('[data-control="map"]').value = this.doc?.map_id ?? ''; return; }
    this.pendingMap = mapId;
    try {
      const data = await this.execute('warehouse_get', { map_id: mapId });
      this.applyRead(data); this.pendingMap = ''; this.viewsChanged(); await this.readInventory();
    } catch (error) { this.q<HTMLSelectElement>('[data-control="map"]').value = this.doc?.map_id ?? mapId; throw error; }
  }
  private applyRead(data: any) {
    this.doc = data.document ? validateWarehouseDocument(data.document) : { schema_version: 2, revision: 0, map_id: data.map_id, layout: { anchor: null, shelves: [] } };
    this.baseline = structuredClone(this.doc); this.syncedRevision = data.synced_revision; this.migrationRequired = data.migration_required === true; this.dirty = this.migrationRequired;
    this.points = data.points ?? {}; this.manager = data.manager ?? { available: false };
    this.selection = { shelfId: this.doc.layout.shelves[0]?.id ?? '', layer: this.doc.layout.shelves[0]?.levels[0]?.layer ?? null, qr: false };
    this.reprint.clear(); this.heightWarnings.clear(); this.message = this.migrationRequired ? '旧单码配置已转换为本地草稿：旧码位置保留为左码，右码尚未设置。请补充右码、打印并贴好两码，再同步和重启；同步时板端保留旧配置备份。' : '已读取板端配置；运行摘要请点检查取药状态。编辑只保留在本页，手动同步后写入板端。'; this.editVersion++;
    this.q<HTMLSelectElement>('[data-control="map"]').value = this.doc.map_id; this.map?.clear(); this.render();
  }
  private async sync() {
    if (!this.doc || !this.dirty || !this.connected) return;
    validateWarehouseDocument(this.doc);
    const { revision: _revision, ...document } = structuredClone(this.doc);
    const args = { map_id: this.doc.map_id, base_revision: this.syncedRevision, document };
    if (new TextEncoder().encode(JSON.stringify(args)).length > 65536) throw new Error('仓库配置超过64 KiB，请减少内容');
    this.pendingMap = this.doc.map_id; this.pendingVersion = this.editVersion;
    const data = await this.execute('warehouse_sync', args); this.applySync(data); this.pendingMap = '';
  }
  private applySync(data: any) {
    if (!this.doc || data.map_id !== this.doc.map_id || this.pendingVersion !== this.editVersion) throw new Error('同步结果不属于当前草稿，请重新读取核对');
    this.doc.revision = data.synced_revision; this.syncedRevision = data.synced_revision; this.baseline = structuredClone(this.doc); this.migrationRequired = false; this.dirty = false;
    this.message = '配置已写入板端。请人工重启任务进程，之后点击“读取板端配置”确认已加载版本；当前进程继续使用旧版本。'; this.render();
  }
  private async query() {
    const operation = this.op.pending?.operation;
    this.working = true; this.onBusy(true); this.renderStatus();
    try {
      const data = await this.op.resume();
      if (operation === 'warehouse_sync') this.applySync(data);
      else if (operation === 'warehouse_get') this.applyRead(data);
      else if (operation === 'warehouse_status') { this.manager = data.manager; this.catalog.accept(operation,data); this.render(); }
      else if (operation === 'drug_catalog_get' || operation === 'drug_catalog_save') this.catalog.accept(operation,data);
      else if (operation === 'inventory_set') { this.message = data.persisted ? '原数量修改已保存。' : '原数量修改尚未保存。'; await this.readInventory(); }
      else if (operation === 'list_maps') { this.maps = data.maps; this.message = '原地图查询已完成，请点击读取板端配置继续。'; this.render(); }
      else { this.message = '原操作已完成。'; this.render(); }
    } finally { this.working = false; this.onBusy(!!this.op.pending); this.renderStatus(); if (this.active && !this.op.pending) await this.ensureScene(); }
  }
  private renderMap() {
    const show = this.wantsMap(); this.q<HTMLCanvasElement>('.warehouse-map').hidden = !show;
    this.q('.warehouse-map-hint').textContent = show ? '紫色箭头：机器人标点及带“观察位”前缀的本地推算点；箭头表示机器人朝向。此视图不下发导航。' : '仅能叠加当前已加载且与所选仓库一致的导航地图；其他已保存地图可先用上方 map 数值核对，无需为编辑切换机器人模式。';
    if (this.map && this.doc) {
      const targets = buildWarehouseTargets(this.doc, this.inputs());
      this.map.points = { ...this.points };
      for (const t of targets) if (t.observation_pose) this.map.points['观察位 ' + t.shelf_id + '/' + t.layer] = { x: t.observation_pose.x_m, y: t.observation_pose.y_m, yaw: t.observation_pose.yaw_rad };
      this.map.draw();
    }
  }
}
