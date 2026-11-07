import { strict as assert } from 'node:assert';
import { test } from 'node:test';
import { inventoryTotals, layerInventory, Inventory } from './warehouse_inventory';
import { register } from 'node:module';
register('../test/ignore-css.mjs', import.meta.url);
const { WarehousePanel } = await import('./warehouse');

test('unknown inventory remains distinct from zero and gives a partial subtotal', () => {
  const totals = inventoryTotals([{ drug: 'A', quantity: 0 }, { drug: 'A', quantity: 3 },
    { drug: 'A', quantity: null }, { drug: '', quantity: null }]);
  assert.deepEqual([...totals], [['A', { known: 3, unknown: 1 }]]);
});
test('changed map or drug cannot inherit another quantity', () => {
  const row = { shelf_id: 'A', layer: 1, drug: 'old', quantity: 4 };
  const data = { map_id: 'map', layers: [row] } as Inventory;
  assert.equal(layerInventory(data, 'other', 'A', 1, 'old'), undefined);
  assert.equal(layerInventory(data, 'map', 'A', 1, 'new'), undefined);
  assert.equal(layerInventory(data, 'map', 'A', 1, 'old'), row);
});
test('opening or refreshing during a task only reads inventory even without geometry', async () => {
  Object.defineProperty(globalThis, 'document', { configurable: true, value: { hidden: false } });
  const page: any = Object.create(WarehousePanel.prototype);
  page.connected = page.active = true; page.working = page.inventoryLoading = false; page.op = {};
  page.state = { map_id: 'current', operation_result: JSON.stringify({ web_busy: true, motion_busy: true }) };
  const requests: any[] = [];
  page.request = async (operation: string, args: any) => { requests.push({ operation, args }); return { result: { map_id: 'current', layers: [] } }; };
  page.renderInventory = page.renderStatus = () => {};
  await page.refresh();
  assert.deepEqual(requests, [{ operation: 'inventory_get', args: { map_id: 'current' } }]);
  assert.equal(page.inventory.map_id, 'current');
});
test('inventory can display manager rows before loading warehouse geometry', () => {
  const page: any = Object.create(WarehousePanel.prototype);
  page.connected = true; page.op = {}; page.inventoryVersion = 1;
  page.state = { map_id: 'current', operation_result: JSON.stringify({ web_busy: true }) };
  page.inventory = { map_id: 'current', persisted: true, layers: [{ shelf_id: 'A', layer: 1, drug: 'D', quantity: 3, source: 'scan' }] };
  const elements = new Map<string, any>();
  page.q = (selector: string) => { if (!elements.has(selector)) elements.set(selector, {}); return elements.get(selector); };
  page.renderInventory();
  const html = elements.get('.warehouse-inventory-table').innerHTML;
  assert.match(html, /A\/1/); assert.match(html, /<td>3<\/td>/);
  assert.equal(elements.get('[data-action="inventory-refresh"]').disabled, false);
});
