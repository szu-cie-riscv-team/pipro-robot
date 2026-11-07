import assert from 'node:assert/strict';
import { test } from 'node:test';
import { readFileSync } from 'node:fs';
import { duplicateShelf, warehouseInputs } from './warehouse_edit';
import { validateWarehouseDocument, warehouseQrPayload } from './warehouse_model';
import { warehouseQrSvg } from './warehouse_qr';
import { WarehouseOperation } from './warehouse_operation';
const fixture = JSON.parse(readFileSync(new URL('../../../ros2_ws/src/k1muse_manager/test/data/warehouse_cases.json', import.meta.url), 'utf8'));

test('duplicate keeps nonsequential layer identities but never clones execution calibration or shelf identity', () => {
  const doc = structuredClone(fixture.document), copy = duplicateShelf(doc, doc.layout.shelves[0]);
  assert.deepEqual(copy.levels.map(l => l.layer), [1, 12]);
  assert.deepEqual(copy.levels.map(l => l.drug), ['酒精', '感冒灵颗粒']);
  assert.ok(copy.levels.every(l => l.fetch === null));
  assert.notEqual(copy.id, doc.layout.shelves[0].id);
  assert.equal(copy.levels[0].qr.left!.x_m, doc.layout.shelves[0].levels[0].qr.left.x_m);
  doc.layout.shelves.push(copy); validateWarehouseDocument(doc);
  assert.equal(fixture.document.layout.shelves[0].levels[0].fetch.class_id, 'red_box');
});
test('missing manager measurements remain unconfigured, never install browser camera or lift defaults', () => {
  const input = warehouseInputs(null);
  assert.equal(input.camera_xy_in_base_m, null); assert.equal(input.lift_zero_height_m, null);
  assert.ok(Number.isNaN(input.observation_distance_m)); assert.equal(input.front.observation_drop_m, null);
  const cap = warehouseInputs({ geometry: { color_optical_origin_xy_in_base_m: [.14, -.025], lift_zero_height_m: .222 }, navigation: { observation_distance_m: .3 }, grasp: { front: { calibration_confirmed: true, observation_drop_m: .05 } }, canonical_drugs: ['酒精'] });
  assert.deepEqual(cap.camera_xy_in_base_m, [.14, -.025]); assert.equal(cap.front.calibrated, true);
});
test('ArUco marker uses configured ID and 40mm sheet', () => {
  const level = structuredClone(fixture.document.layout.shelves[0].levels[0]);
  level.qr.left.marker_id = 0;
  const { svg, widthMm, id } = warehouseQrSvg('A', level, 'left');
  assert.equal(id, 0); assert.equal(widthMm, 40);
  assert.match(svg, /viewBox="0 0 8 8"/);
  level.drug = ''; assert.equal(warehouseQrSvg('A', level, 'left').id, id);
  level.qr.left = null; assert.throws(() => warehouseQrSvg('A', level, 'left'));
});
test('lost sync ACK queries original request identity and sends exactly one mutation', async () => {
  const calls: any[] = [];
  const op = new WarehouseOperation(async (operation, args, requestId) => {
    calls.push({ operation, args, requestId });
    if (operation === 'warehouse_sync') throw new Error('timeout');
    return { result: { status: 'succeeded', result: { synced_revision: 2 } } };
  });
  assert.deepEqual(await op.start('warehouse_sync', { base_revision: 1 }), { synced_revision: 2 });
  assert.equal(calls.length, 2); assert.equal(calls[1].args.request_id, calls[0].requestId); assert.equal(op.pending, undefined);
});
test('unknown result survives reconnect for manual query; no new sync while unresolved', async () => {
  let connected = false, syncs = 0;
  const op = new WarehouseOperation(async (name) => {
    if (name === 'warehouse_sync') { syncs++; throw new Error('disconnected'); }
    if (!connected) throw new Error('disconnected');
    return { result: { status: 'succeeded', result: { synced_revision: 8 } } };
  });
  await assert.rejects(op.start('warehouse_sync', {})); const requestId = op.pending!.requestId;
  await assert.rejects(op.start('warehouse_sync', {}));
  connected = true; assert.deepEqual(await op.resume(), { synced_revision: 8 });
  assert.ok(requestId); assert.equal(syncs, 1); assert.equal(op.pending, undefined);
});
test('known revision failure clears pending and does not query or retry mutation', async () => {
  let calls = 0;
  const op = new WarehouseOperation(async () => { calls++; throw Object.assign(new Error('warehouse_revision_changed'), { rejected: true }); });
  await assert.rejects(op.start('warehouse_sync', {}), /warehouse_revision_changed/);
  assert.equal(calls, 1); assert.equal(op.pending, undefined);
});
test('accepted asynchronous warehouse GET waits for full operation reply, not periodic summary', async () => {
  let calls = 0;
  const op = new WarehouseOperation(async name => {
    calls++;
    return name === 'warehouse_get' ? { operation_id: 'op', result: { status: 'running' } } :
      { result: { status: 'succeeded', result: { document: fixture.document, manager: { capabilities: {} } } } };
  });
  const result = await op.start('warehouse_get', {}); assert.equal(calls, 2); assert.equal(result.document.layout.shelves.length, 2);
});
