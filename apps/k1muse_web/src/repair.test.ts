import assert from 'node:assert/strict';
import { test } from 'node:test';
import { readFileSync } from 'node:fs';
import { IntentPanel } from './intent';
import { register } from 'node:module';
register('../test/ignore-css.mjs', import.meta.url);
const { WarehousePanel } = await import('./warehouse');

for (const [saved, active, actual, failed] of [
  ['rules', 'cloud', 'cloud', false], ['rules', 'cloud', 'rules', true],
  ['local', undefined, 'local', false],
] as const) {
  test(`restart compares active selection: ${saved}/${active}/${actual}`, () => {
    const panel: any = Object.create(IntentPanel.prototype);
    panel.state = { instance_id: 'old' }; panel.restartFrom = 'old';
    panel.retiredInstances = new Set();
    let failure = false;
    panel.message = (_text: string, error = false) => { failure = error; };
    panel.applyState({ instance_id: 'new', startup_mode: saved, active_startup_mode: active,
      mode: actual, inference: { phase: 'wake_wait' } });
    assert.equal(failure, failed);
  });
}
const fixture = JSON.parse(readFileSync(new URL('../../../ros2_ws/src/k1muse_manager/test/data/warehouse_cases.json', import.meta.url), 'utf8'));
for (const layer of [null, 1]) {
  test(`renaming shelf marks every layer even when selection is ${layer}`, () => {
    const page: any = Object.create(WarehousePanel.prototype);
    page.doc = structuredClone(fixture.document);page.baseline = structuredClone(page.doc);
    const shelf = page.doc.layout.shelves[0];
    page.selection = { shelfId: shelf.id, layer, qr: false };page.op = {};
    page.reprint = new Set(); page.editVersion = 0; page.render = () => {};
    page.edit((doc: any, selected: any) => {
      if (doc.layout.anchor?.shelf_id === selected.id) doc.layout.anchor.shelf_id = 'Renamed';
      selected.id = 'Renamed'; page.selection.shelfId = 'Renamed';
    });
    for (const level of shelf.levels) assert.ok(page.reprint.has(`Renamed:${level.layer}`));
  });
}
