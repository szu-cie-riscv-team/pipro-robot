import { escapeHTML as esc } from './core';
import { WarehouseOperation } from './warehouse_operation';

export type Drug = { name: string; aliases: string[] };
export function normalizeDrugs(rows: Drug[]): Drug[] {
  const owners = new Map<string, string>();
  const name = (s: string) => {
    if (typeof s !== 'string' || !s.replace(/^[ \t\r\n\f\v]+|[ \t\r\n\f\v]+$/g, '')) throw new Error('药名和别名不能为空');
    return s.replace(/^[ \t\r\n\f\v]+|[ \t\r\n\f\v]+$/g, '');
  };
  return rows.map(row => {
    const canonical = name(row.name);
    if (owners.has(canonical)) throw new Error('药名或别名冲突：' + canonical);
    owners.set(canonical, canonical);
    const aliases: string[] = [];
    for (const value of row.aliases) {
      const alias = name(value);
      if (owners.has(alias)) {
        if (owners.get(alias) !== canonical) throw new Error('药名或别名冲突：' + alias);
      } else { owners.set(alias, canonical); aliases.push(alias); }
    }
    return { name: canonical, aliases };
  });
}

export function catalogMatch(saved: Drug[] | undefined, runtime: any): string {
  if (!runtime || runtime.error || !Array.isArray(runtime.drugs)) return runtime?.error || '不可查询';
  if (!saved) return '尚未读取保存内容';
  return JSON.stringify(saved) === JSON.stringify(runtime.drugs) ? '与保存内容一致' : '待重启运行组加载';
}

export class DrugCatalogPanel {
  saved?: Drug[];
  private rows: Drug[] = [];
  private dirty = false;
  private connected = false;
  private blocked = false;
  private working = false;
  private editVersion = 0;
  private requestVersion = 0;
  private epoch = 0;
  private message = '连接后可编辑；无需选择地图或启动语音。';
  private manager: any;

  constructor(private root: HTMLElement, private op: WarehouseOperation,
    private busy: (value: boolean) => void, private changed: () => void) {
    root.innerHTML = '<h2>药品知识目录</h2><p class="hint">规则、本地和云端共用规范药名及同药别名。保存后重启运行组加载；不会移动机器人或改变库存。</p>' +
      '<div class="buttons"><button data-catalog="load">重新读取</button><button data-catalog="status">查看运行目录</button><button data-catalog="add">新增药品</button><button data-catalog="save">保存目录</button><button data-catalog="query" hidden>查询原保存/读取结果</button></div>' +
      '<p class="catalog-message" role="status"></p><p class="catalog-runtime"></p><div class="catalog-rows"></div>';
    root.addEventListener('click', event => {
      const b = (event.target as HTMLElement).closest<HTMLButtonElement>('button[data-catalog]');
      if (!b) return;
      const action = b.dataset.catalog!;
      if (action === 'add') { this.rows.push({ name: '', aliases: [] }); this.dirty = true; ++this.editVersion; this.renderRows(); this.render(); }
      else if (action === 'delete') { this.rows.splice(Number(b.dataset.index), 1); this.dirty = true; ++this.editVersion; this.renderRows(); this.render(); }
      else void this.run(action).catch(error => { this.message = String(error); this.render(); });
    });
    root.addEventListener('input', event => {
      const input = event.target as HTMLInputElement;
      const index = input.dataset.catalogIndex;
      if (index === undefined) return;
      const row = this.rows[Number(index)];
      if (input.dataset.catalogField === 'name') row.name = input.value;
      else row.aliases = input.value.split('\n').filter(v => v.trim() !== '');
      this.dirty = true; ++this.editVersion; this.render();
    });
    this.render();
  }
  setConnected(value: boolean) {
    if (this.connected !== value) { ++this.epoch; this.manager = undefined; }
    this.connected = value; this.render();
  }
  setBlocked(value: boolean) { this.blocked = value; this.render(); }
  async enter() { if (this.connected && !this.dirty && !this.op.pending) await this.run('load'); }
  accept(operation: string, data: any, version = this.requestVersion) {
    if (operation === 'warehouse_status') { this.manager = data.manager?.drug_catalog; }
    else if (operation === 'drug_catalog_get' || operation === 'drug_catalog_save') {
      if (data.drugs !== null) {
        this.saved = data.drugs;
        if (version === this.editVersion) { this.rows = structuredClone(data.drugs); this.dirty = false; this.renderRows(); }
        this.changed();
      }
      this.message = data.error || (operation === 'drug_catalog_save' ? '目录已保存；运行内容需重启运行组加载。' : '已读取目录。');
    }
    this.render();
  }
  private async run(action: string) {
    if (!this.connected || this.working || this.blocked) return;
    if(action === 'query' && !this.op.pending?.operation.startsWith('drug_catalog_')) return;
    if (action !== 'query' && this.op.pending) throw new Error('请先查询原操作结果');
    const epoch = this.epoch, version = action === 'query' ? this.requestVersion : this.editVersion;
    if(action !== 'query') this.requestVersion = version;
    this.working = true; this.busy(true); this.render();
    try {
      const operation = action === 'query' ? this.op.pending!.operation : action === 'save' ? 'drug_catalog_save' : action === 'status' ? 'warehouse_status' : 'drug_catalog_get';
      const data = action === 'query' ? await this.op.resume() : await this.op.start(operation,
        action === 'save' ? { drugs: normalizeDrugs(this.rows) } : {});
      if (epoch !== this.epoch) return;
      this.accept(operation, data, version);
    } finally { this.working = false; this.busy(false); this.render(); }
  }
  private renderRows() {
    this.root.querySelector('.catalog-rows')!.innerHTML = this.rows.map((row, index) =>
      '<div class="catalog-row"><label>规范药名<input data-catalog-index="' + index + '" data-catalog-field="name" value="' + esc(row.name) + '"></label>' +
      '<label>别名（每行一个）<textarea rows="2" data-catalog-index="' + index + '" data-catalog-field="aliases">' + esc(row.aliases.join('\n')) + '</textarea></label>' +
      '<button data-catalog="delete" data-index="' + index + '">删除</button></div>').join('');
  }
  private render() {
    this.root.querySelector('.catalog-message')!.textContent = this.message + (this.dirty ? '（有未保存修改）' : '');
    this.root.querySelector('.catalog-runtime')!.textContent = 'TaskManager：' + catalogMatch(this.saved, this.manager);
    for (const b of this.root.querySelectorAll<HTMLButtonElement>('button[data-catalog]')) {
      b.disabled = !this.connected || this.working || this.blocked || (!!this.op.pending && b.dataset.catalog !== 'query');
      if (b.dataset.catalog === 'query') b.hidden = !this.op.pending?.operation.startsWith('drug_catalog_');
    }
  }
}
