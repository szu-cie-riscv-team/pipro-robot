export type Target = { x: number; y: number; yaw: number };
export type Geometry = { width: number; height: number; resolution: number; origin: Target };
export function webOperation(state: any): any {
  try { return JSON.parse(state?.operation_result || '{}'); } catch { return {}; }
}
export function robotMotionBusy(state: any): boolean {
  return !!webOperation(state).motion_busy ||
    ['starting', 'pending', 'running', 'stopping', 'canceling', 'blocked'].includes(state?.navigation_state);
}
export function initialPoseAvailable(state: any): boolean {
  return state?.mode === 'NAVIGATION' && !!state.initial_pose_available &&
    !webOperation(state).web_busy && !robotMotionBusy(state);
}
export function fetchAdmission(manager: any, drug: string | undefined): { ready: boolean; reason: string } {
  const selected = drug ? manager?.fetch_admission?.[drug] : undefined;
  if (selected?.ready === true) return { ready: true, reason: '' };
  return { ready: selected?.ready === true,
    reason: selected?.reason || manager?.fetch_blocked_reason || manager?.fetch_flow_error || 'status_unknown' };
}
export function id(): string {
  return Array.from(crypto.getRandomValues(new Uint32Array(4)), n => n.toString(16)).join("-");
}
export function gridToWorld(g: Geometry, u: number, v: number): Target {
  const x = u * g.resolution, y = (g.height - v) * g.resolution;
  const c = Math.cos(g.origin.yaw), s = Math.sin(g.origin.yaw);
  return { x: g.origin.x + c * x - s * y, y: g.origin.y + s * x + c * y, yaw: 0 };
}
export function worldToGrid(g: Geometry, p: Target): [number, number] {
  const x = p.x - g.origin.x, y = p.y - g.origin.y;
  const c = Math.cos(g.origin.yaw), s = Math.sin(g.origin.yaw);
  return [(c * x + s * y) / g.resolution, g.height - (-s * x + c * y) / g.resolution];
}
export function keyVelocity(keys: Set<string>, ratio: number): Target {
  let x = Number(keys.has("KeyW")) - Number(keys.has("KeyS"));
  let y = Number(keys.has("KeyA")) - Number(keys.has("KeyD"));
  const length = Math.max(1, Math.hypot(x, y));
  x = x / length * .2 * ratio; y = y / length * .2 * ratio;
  return { x, y, yaw: (Number(keys.has("KeyQ")) - Number(keys.has("KeyE"))) * .6 * ratio };
}
export function boundedVelocity(v: Target, ratio = 1): Target {
  const length = Math.max(.2, Math.hypot(v.x, v.y));
  return { x: v.x / length * .2 * ratio, y: v.y / length * .2 * ratio,
    yaw: Math.max(-.6, Math.min(.6, v.yaw)) * ratio };
}
export function yaw(q: { x: number; y: number; z: number; w: number }): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
export function escapeHTML(value: unknown): string {
  return String(value).replace(/[&<>"']/g, char =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[char]!);
}
