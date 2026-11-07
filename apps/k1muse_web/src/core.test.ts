import { test } from "node:test";
import assert from "node:assert/strict";
import { boundedVelocity, gridToWorld, keyVelocity, worldToGrid, initialPoseAvailable, robotMotionBusy, fetchAdmission } from "./core";
test("map origin rotation and vertical image flip round trip", () => {
  const g = { width: 40, height: 30, resolution: .05, origin: { x: 2, y: -3, yaw: Math.PI / 2 } };
  const p = gridToWorld(g, 10, 20);
  assert.ok(Math.abs(p.x - 1.5) < 1e-10);
  assert.ok(Math.abs(p.y + 2.5) < 1e-10);
  const [u, v] = worldToGrid(g, p);
  assert.ok(Math.abs(u - 10) < 1e-10 && Math.abs(v - 20) < 1e-10);
});
test("diagonal and slider commands respect translation and yaw caps", () => {
  const v = keyVelocity(new Set(["KeyW", "KeyA", "KeyQ"]), .5);
  assert.ok(Math.abs(Math.hypot(v.x, v.y) - .1) < 1e-10);
  assert.equal(v.yaw, .3);
  assert.deepEqual(keyVelocity(new Set(["KeyW", "KeyS"]), 1), { x: 0, y: 0, yaw: 0 });
  const s = boundedVelocity({ x: .2, y: .2, yaw: 2 });
  assert.ok(Math.hypot(s.x, s.y) <= .20000001);
  assert.equal(s.yaw, .6);
});
test('ready navigation permits idle relocalization but business gaps and pending ACKs block it', () => {
  const state = { mode: 'NAVIGATION', initial_pose_available: true, navigation_ready: true,
    navigation_state: 'succeeded', operation_result: JSON.stringify({ web_busy: false, motion_busy: false }) };
  assert.equal(initialPoseAvailable(state), true);
  state.operation_result = JSON.stringify({ web_busy: false, motion_busy: true });
  assert.equal(robotMotionBusy(state), true); assert.equal(initialPoseAvailable(state), false);
  state.operation_result = JSON.stringify({ web_busy: true, motion_busy: false, phase: 'admission_unresolved' });
  assert.equal(initialPoseAvailable(state), false);
});
test('configuration capability does not imply current or selected-drug admission', () => {
  const manager = { fetch_available: true, fetch_admission: {
    '酒精': { ready: false, reason: 'holding_confirmed' },
    '感冒灵颗粒': { ready: true, reason: '' } } };
  assert.deepEqual(fetchAdmission(manager, '酒精'), { ready: false, reason: 'holding_confirmed' });
  assert.deepEqual(fetchAdmission(manager, '感冒灵颗粒'), { ready: true, reason: '' });
  assert.equal(fetchAdmission({ fetch_available: true }, '酒精').ready, false);
});
