import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { buildWarehouseTargets, parseWarehouseDocument, selectWarehouseTarget,
  validateWarehouseDocument, warehouseInitialLift, warehouseLayoutToMap,
  warehouseQrPayload, warehouseWrap, type WarehouseInputs, type WarehousePose } from "./warehouse_model";

const fixture = JSON.parse(readFileSync(fileURLToPath(new URL(
  "../../../ros2_ws/src/k1muse_manager/test/data/warehouse_cases.json", import.meta.url)), "utf8"));
const document = () => parseWarehouseDocument(JSON.stringify(fixture.document));
const inputs = (): WarehouseInputs => structuredClone(fixture.inputs);
test("runtime YAML grasp mode overrides layer metadata without mutating the document", () => {
  const doc=document(), ctx=inputs();
  const before=JSON.stringify(doc);
  ctx.grasp_mode="top";
  ctx.front.calibrated=false;ctx.top.calibrated=true;ctx.top.observation_drop_m=0;
  ctx.lift_zero_height_m=.222;
  doc.layout.shelves[0].levels[0].height_m=.4;
  const expectedDocument=JSON.stringify(doc);
  const target=buildWarehouseTargets(doc,ctx)[0];
  assert.equal(target.fetch!.grasp,"top");
  assert.ok(Math.abs(target.initial_lift_z_m!-.178)<1e-12);
  assert.deepEqual(target.unavailable_reasons,[]);
  assert.equal(JSON.stringify(doc),expectedDocument);
  assert.equal(doc.revision,JSON.parse(before).revision);
});
function nearPose(actual: WarehousePose, expected: WarehousePose) {
  assert.ok(Math.abs(actual.x_m - expected.x_m) < 1e-10);
  assert.ok(Math.abs(actual.y_m - expected.y_m) < 1e-10);
  assert.ok(Math.abs(warehouseWrap(actual.yaw_rad - expected.yaw_rad)) < 1e-10);
}
test("shared SE(2) fixtures: anchor round trip, camera offset, rotated layout and other shelf", () => {
  for (const c of fixture.cases) {
    const doc = document(), ctx = inputs(), a = doc.layout.anchor!;
    a.point_pose = c.anchor_point; doc.layout.shelves[0].pose = c.anchor_layout;
    doc.layout.shelves[1].pose = c.other_layout; ctx.camera_xy_in_base_m = c.camera_xy;
    ctx.observation_distance_m = c.observation_distance_m;
    nearPose(warehouseLayoutToMap(doc.layout.shelves[0], doc.layout.shelves[0].levels[0].qr.left!,
      a, ctx.camera_xy_in_base_m!), c.expected_transform);
    const targets = buildWarehouseTargets(doc, ctx);
    targets.forEach((target, i) => {
      assert.deepEqual(target.unavailable_reasons, [], c.name);
      nearPose(target.observation_pose!, c.expected_goals[i]);
    });
    assert.ok(Math.abs(targets[0].initial_lift_z_m! - .028) < 1e-12);
    assert.ok(Math.abs(targets[1].initial_lift_z_m! - .328) < 1e-12);
  }
  assert.equal(warehouseWrap(Math.PI), -Math.PI);
});
test("shared malformed documents are rejected without coercion", () => {
  for (const c of fixture.invalid_cases) {
    const doc = structuredClone(fixture.document);
    const keys = c.pointer.slice(1).split("/");
    let parent = doc;
    for (const k of keys.slice(0, -1)) parent = parent[k];
    parent[keys.at(-1)!] = c.value;
    assert.throws(() => validateWarehouseDocument(doc), Error, c.name);
  }
  for (const text of fixture.invalid_json) assert.throws(() => parseWarehouseDocument(text));
  assert.throws(() => parseWarehouseDocument(JSON.stringify(fixture.document).replace('"revision":1', '"revision":1e999')));
  assert.throws(() => parseWarehouseDocument(JSON.stringify(fixture.document).replace('"revision":1', '"revision":1,"revision":2')));
  assert.throws(() => parseWarehouseDocument(JSON.stringify(fixture.document).replace('"x_m":1', '"x_m":1,"\\u0078_m":1')));
});
test("incomplete models save without renumbering; selection skips unavailable first location", () => {
  const doc = document(), ctx = inputs();
  doc.layout.shelves[0].levels[0].fetch!.class_id = null;
  validateWarehouseDocument(doc);
  let targets = buildWarehouseTargets(doc, ctx);
  assert.ok(targets[0].unavailable_reasons.includes("class_id_unconfigured"));
  assert.equal(selectWarehouseTarget(targets, "酒精")?.shelf_id, "Rack-2");
  doc.layout.anchor = null;
  doc.layout.shelves[0].levels.shift();
  doc.layout.shelves[0].levels[0].fetch = null;
  doc.layout.shelves[0].levels[0].qr = { left: null, right: null };
  validateWarehouseDocument(doc);
  assert.equal(doc.layout.shelves[0].levels[0].layer, 12);
  targets = buildWarehouseTargets(doc, ctx);
  assert.ok(targets[0].unavailable_reasons.includes("left_qr_unconfigured"));
  assert.ok(targets[0].unavailable_reasons.includes("fetch_unconfigured"));
  assert.equal(selectWarehouseTarget(targets, "酒精"), undefined);
});
test("missing calibration, unknown drug and unreachable lift remain local to each target", () => {
  const doc = document(), ctx = inputs();
  doc.layout.shelves[0].levels[1].fetch!.grasp = "top";
  doc.layout.shelves[0].levels[0].height_m = .10;
  doc.layout.shelves[1].levels[0].drug = "未知药";
  let targets = buildWarehouseTargets(doc, ctx);
  assert.ok(targets[0].unavailable_reasons.includes("lift_unreachable"));
  assert.ok(targets[1].unavailable_reasons.includes("grasp_uncalibrated"));
  assert.ok(targets[2].unavailable_reasons.includes("unknown_drug"));
  assert.ok(Math.abs(targets[1].initial_lift_z_m! - .298) < 1e-12);
  ctx.camera_xy_in_base_m = null; ctx.lift_zero_height_m = null;
  targets = buildWarehouseTargets(doc, ctx);
  assert.equal(targets[0].observation_pose, null);
  assert.ok(targets[0].unavailable_reasons.includes("camera_geometry_unconfigured"));
  assert.ok(targets[0].unavailable_reasons.includes("lift_geometry_unconfigured"));
  assert.ok(warehouseInitialLift(.1, .05, .222) < .020); // Never clipped.
});
test("boundary QR identity is independent of medicine/model and both sides are required for fetching", () => {
  const doc = document(), level = doc.layout.shelves[0].levels[0];
  level.qr.left!.marker_id = 0; level.qr.right!.marker_id = 49;
  const left = warehouseQrPayload('A', level, 'left');
  assert.equal(left, '0');
  assert.equal(warehouseQrPayload('A', level, 'right'), '49');
  level.drug = '药'.repeat(100); level.fetch = null;
  assert.equal(warehouseQrPayload('A', level, 'left'), left);
  level.qr.right = null; validateWarehouseDocument(doc);
  assert.equal(warehouseQrPayload('A', level, 'right'), null);
  assert.ok(buildWarehouseTargets(doc, inputs())[0].unavailable_reasons.includes('right_qr_unconfigured'));
});

test('marker ID accepts 0 and 49, conflicts remain editable, invalid values are rejected', () => {
  const doc = document(), levels = doc.layout.shelves[0].levels;
  levels[0].qr.left!.marker_id = 0; levels[0].qr.right!.marker_id = 49;
  levels[1].qr.left!.marker_id = 0; validateWarehouseDocument(doc);
  assert.throws(() => parseWarehouseDocument(JSON.stringify(doc).replace('"marker_id":0', '"marker_id":0.0')));
  for (const bad of [-1, 50, 1.5, true, '1']) {
    (levels[0].qr.left as any).marker_id = bad;
    assert.throws(() => validateWarehouseDocument(doc));
  }
});
