/** P01 pure model and preview maths. All geometry uses metres/radians. */
export interface WarehousePose { x_m: number; y_m: number; yaw_rad: number }
export type QrSide = "left" | "right";
export interface WarehouseQr { x_m: number; z_m: number; marker_id?: number | null }
export interface WarehouseFetch {
  class_id: string | null; grasp: "front" | "top" | "" | null;
  roi: [number, number, number, number] | null;
  search_min_m: number | null; search_max_m: number | null;
}
export interface WarehouseLevel {
  layer: number; height_m: number; drug: string;
  qr: Record<QrSide, WarehouseQr | null>; fetch: WarehouseFetch | null;
}
export interface WarehouseShelf {
  id: string; name: string; width_m: number; depth_m: number;
  pose: WarehousePose; levels: WarehouseLevel[];
}
export interface WarehouseAnchor {
  shelf_id: string; layer: number; point_name: string;
  point_pose: WarehousePose; camera_distance_m: number;
}
export interface WarehouseDocument {
  schema_version: 2; revision: number; map_id: string;
  layout: { anchor: WarehouseAnchor | null; shelves: WarehouseShelf[] };
}
export interface WarehouseGraspSettings { calibrated: boolean; observation_drop_m: number | null }
export interface WarehouseInputs {
  grasp_mode?: "front" | "top";
  camera_xy_in_base_m: [number, number] | null;
  lift_zero_height_m: number | null; observation_distance_m: number;
  front: WarehouseGraspSettings; top: WarehouseGraspSettings; canonical_drugs: string[];
}
export interface WarehouseTarget {
  shelf_id: string; shelf_name: string; layer: number; height_m: number; drug: string;
  fetch: WarehouseFetch | null; observation_pose: WarehousePose | null; initial_lift_z_m: number | null;
  unavailable_reasons: string[];
}

function invalid(field: string): never { throw new Error(`warehouse_invalid: ${field}`); }
function object(value: unknown, names: string[]): Record<string, unknown> {
  if (!value || typeof value !== "object" || Array.isArray(value)) invalid("object");
  const result = value as Record<string, unknown>;
  if (Object.keys(result).length !== names.length || names.some(n => !Object.hasOwn(result, n))) invalid("object fields");
  return result;
}
function string(value: unknown): string {
  if (typeof value !== "string") invalid("string");
  // JSON permits isolated surrogate escapes, UTF-8 does not.
  for (let i = 0; i < value.length; ++i) {
    const c = value.charCodeAt(i);
    if (c >= 0xd800 && c <= 0xdbff) {
      const next = value.charCodeAt(++i);
      if (!(next >= 0xdc00 && next <= 0xdfff)) invalid("UTF-8 string");
    } else if (c >= 0xdc00 && c <= 0xdfff) invalid("UTF-8 string");
  }
  return value;
}
function identity(value: unknown): string {
  const result = string(value);
  if (!/^[A-Za-z0-9_-]{1,32}$/.test(result)) invalid("identity");
  return result;
}
function number(value: unknown, positive = false): number {
  if (typeof value !== "number" || !Number.isFinite(value) || (positive && value <= 0)) invalid("finite number/range");
  return value;
}
function integer(value: unknown, min: number, max: number): number {
  const result = number(value);
  if (!Number.isSafeInteger(result) || result < min || result > max) invalid("integer/range");
  return result;
}
function pose(value: unknown): void {
  const p = object(value, ["x_m", "y_m", "yaw_rad"]);
  Object.values(p).forEach(v => number(v));
}
function array(value: unknown): unknown[] {
  if (!Array.isArray(value)) invalid("array");
  return value;
}

/** Use this for raw files; JSON.parse alone silently overwrites duplicate keys. */
export function parseWarehouseDocument(text: string): WarehouseDocument {
  const value: unknown = JSON.parse(text);
  // Syntax is already checked. Walk only tokens to detect duplicate decoded keys.
  const tokens = text.match(/"(?:[^"\\]|\\.)*"|[{}\[\]:,]|[^\s{}\[\]:,]+/g)!;
  let i = 0;
  function walk(field?: string): void {
    const token = tokens[i++];
    if (field === "marker_id" && token !== "null" && !/^(0|[1-9][0-9]*)$/.test(token)) invalid("marker_id integer");
    if (token === "{") {
      const keys = new Set<string>();
      while (tokens[i] !== "}") {
        const key: string = JSON.parse(tokens[i++]);
        if (keys.has(key)) invalid("duplicate key");
        keys.add(key);
        ++i; // colon
        walk(key);
        if (tokens[i] === ",") ++i;
      }
      ++i;
    } else if (token === "[") {
      while (tokens[i] !== "]") { walk(); if (tokens[i] === ",") ++i; }
      ++i;
    }
  }
  walk();
  return validateWarehouseDocument(value);
}

/** Returns source data unchanged; incomplete fetch fields are explicitly nullable. */
export function validateWarehouseDocument(value: unknown): WarehouseDocument {
  const doc = object(value, ["schema_version", "revision", "map_id", "layout"]);
  if (doc.schema_version === 1) invalid("schema_v1_requires_web_migration");
  integer(doc.schema_version, 2, 2);
  integer(doc.revision, 0, Number.MAX_SAFE_INTEGER);
  if (!/^[0-9a-f]{64}$/.test(string(doc.map_id))) invalid("map_id");
  const layout = object(doc.layout, ["anchor", "shelves"]);
  const ids = new Set<string>(), qrRefs = new Set<string>();
  for (const s of array(layout.shelves)) {
    const shelf = object(s, ["id", "name", "width_m", "depth_m", "pose", "levels"]);
    const id = identity(shelf.id);
    if (ids.has(id)) invalid("duplicate shelf id");
    ids.add(id);
    string(shelf.name);
    const width = number(shelf.width_m, true);
    number(shelf.depth_m, true); pose(shelf.pose);
    const layers = new Set<number>();
    for (const l of array(shelf.levels)) {
      const level = object(l, ["layer", "height_m", "drug", "qr", "fetch"]);
      const layer = integer(level.layer, 1, 2147483647);
      if (layers.has(layer)) invalid("duplicate layer");
      layers.add(layer);
      number(level.height_m, true); string(level.drug);
      const markers = object(level.qr, ["left", "right"]);
      for (const side of ["left", "right"]) {
        if (markers[side] === null) continue;
        const markerValue = markers[side] as Record<string, unknown>;
        const qr = object(markerValue, Object.hasOwn(markerValue, "marker_id") ? ["x_m", "z_m", "marker_id"] : ["x_m", "z_m"]);
        if (qr.marker_id !== undefined && qr.marker_id !== null) integer(qr.marker_id, 0, 49);
        number(qr.x_m); number(qr.z_m, true);
        if (side === "left") qrRefs.add(`${id}:${layer}`);
      }
      if (markers.left && markers.right &&
        (markers.left as WarehouseQr).x_m >= (markers.right as WarehouseQr).x_m) invalid("QR left/right order");
      if (level.fetch === null) continue;
      const fetch = object(level.fetch, ["class_id", "grasp", "roi", "search_min_m", "search_max_m"]);
      for (const name of ["class_id", "grasp"]) if (fetch[name] !== null) string(fetch[name]);
      if (![null, "", "front", "top"].includes(fetch.grasp as string | null)) invalid("grasp");
      if (fetch.roi !== null) {
        const roi = array(fetch.roi);
        if (roi.length !== 4) invalid("roi");
        const r = roi.map(v => number(v));
        if (r.some(v => v < 0 || v > 1) || r[0] >= r[2] || r[1] >= r[3]) invalid("roi range/order");
      }
      for (const name of ["search_min_m", "search_max_m"]) {
        if (fetch[name] !== null) {
          const v = number(fetch[name]);
          if (v < 0 || v > width) invalid("search range");
        }
      }
      if (fetch.search_min_m !== null && fetch.search_max_m !== null &&
        (fetch.search_min_m as number) > (fetch.search_max_m as number)) invalid("search order");
    }
  }
  if (layout.anchor !== null) {
    const a = object(layout.anchor, ["shelf_id", "layer", "point_name", "point_pose", "camera_distance_m"]);
    const id = identity(a.shelf_id), layer = integer(a.layer, 1, 2147483647);
    if (!string(a.point_name)) invalid("point_name");
    pose(a.point_pose); number(a.camera_distance_m, true);
    if (!qrRefs.has(`${id}:${layer}`)) invalid("anchor QR reference");
  }
  return value as WarehouseDocument;
}

export function warehouseWrap(angle: number): number {
  return ((angle + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
}
function rotate(yaw: number, x: number, y: number): [number, number] {
  return [Math.cos(yaw) * x - Math.sin(yaw) * y, Math.sin(yaw) * x + Math.cos(yaw) * y];
}
export function warehouseLayoutToMap(shelf: WarehouseShelf, qr: WarehouseQr,
  anchor: WarehouseAnchor, camera: [number, number]): WarehousePose {
  const theta = warehouseWrap(anchor.point_pose.yaw_rad - Math.PI / 2);
  const yaw = warehouseWrap(theta - shelf.pose.yaw_rad);
  const c = rotate(anchor.point_pose.yaw_rad, ...camera);
  const q = rotate(theta, qr.x_m, -anchor.camera_distance_m);
  const r = rotate(yaw, shelf.pose.x_m, shelf.pose.y_m);
  return { x_m: anchor.point_pose.x_m + c[0] - q[0] - r[0],
    y_m: anchor.point_pose.y_m + c[1] - q[1] - r[1], yaw_rad: yaw };
}
export function warehouseObservationPose(transform: WarehousePose, shelf: WarehouseShelf,
  qr: WarehouseQr, camera: [number, number], distance: number): WarehousePose {
  const r = rotate(transform.yaw_rad, shelf.pose.x_m, shelf.pose.y_m);
  const theta = warehouseWrap(transform.yaw_rad + shelf.pose.yaw_rad);
  const yaw = warehouseWrap(theta + Math.PI / 2);
  const q = rotate(theta, qr.x_m, -distance), c = rotate(yaw, ...camera);
  return { x_m: transform.x_m + r[0] + q[0] - c[0],
    y_m: transform.y_m + r[1] + q[1] - c[1], yaw_rad: yaw };
}
export function warehouseInitialLift(height: number, drop: number, zero: number): number {
  return height - drop - zero;
}
/** Count each configured identity across the current map, including incomplete layers. */
export function markerIdCount(doc: WarehouseDocument, id: number): number {
  let count = 0;
  for (const shelf of doc.layout.shelves) for (const level of shelf.levels)
    for (const side of ["left", "right"] as const) if (level.qr[side]?.marker_id === id) ++count;
  return count;
}
export function warehouseQrPayload(_shelfId: string, level: WarehouseLevel, side: QrSide): string | null {
  const id = level.qr[side]?.marker_id;
  return id === null || id === undefined ? null : String(id);
}

/** Preview only: inputs come from the manager, never browser installation constants. */
export function buildWarehouseTargets(doc: WarehouseDocument, inputs: WarehouseInputs): WarehouseTarget[] {
  const camera = inputs.camera_xy_in_base_m;
  const cameraValid = camera !== null && camera.every(Number.isFinite);
  const dockValid = Number.isFinite(inputs.observation_distance_m) && inputs.observation_distance_m > 0;
  const a = doc.layout.anchor;
  const anchorShelf = a && doc.layout.shelves.find(s => s.id === a.shelf_id);
  const anchorQr = anchorShelf && anchorShelf.levels.find(l => l.layer === a!.layer)?.qr.left;
  const transform = a && anchorShelf && anchorQr && cameraValid
    ? warehouseLayoutToMap(anchorShelf, anchorQr, a, camera!) : null;
  const drugs = new Set(inputs.canonical_drugs);
  return doc.layout.shelves.flatMap(shelf => shelf.levels.map(level => {
    const reasons: string[] = [];
    const target: WarehouseTarget = { shelf_id: shelf.id, shelf_name: shelf.name,
      layer: level.layer, height_m: level.height_m, drug: level.drug,
      fetch: level.fetch && inputs.grasp_mode ? { ...level.fetch, grasp: inputs.grasp_mode } : level.fetch,
      observation_pose: null, initial_lift_z_m: null,
      unavailable_reasons: reasons };
    if (!level.drug) reasons.push("drug_unconfigured");
    else if (!drugs.has(level.drug)) reasons.push("unknown_drug");
    if (!level.qr.left) reasons.push("left_qr_unconfigured");
    if (!level.qr.right) reasons.push("right_qr_unconfigured");
    if (!a) reasons.push("anchor_unconfigured");
    if (!cameraValid) reasons.push("camera_geometry_unconfigured");
    if (!dockValid) reasons.push("observation_distance_invalid");
    if (transform && level.qr.left && dockValid) {
      const p = warehouseObservationPose(transform, shelf, level.qr.left!, camera!, inputs.observation_distance_m);
      if (Object.values(p).every(Number.isFinite)) target.observation_pose = p;
      else reasons.push("observation_pose_nonfinite");
    }
    const f = target.fetch;
    if (!f) reasons.push("fetch_unconfigured");
    else {
      if (!f.class_id) reasons.push("class_id_unconfigured");
      if (!f.roi) reasons.push("roi_unconfigured");
      if (f.search_min_m === null) reasons.push("search_min_unconfigured");
      if (f.search_max_m === null) reasons.push("search_max_unconfigured");
      if (!f.grasp) reasons.push("grasp_unconfigured");
      else {
        const grasp = inputs[f.grasp], drop = grasp.observation_drop_m;
        if (!grasp.calibrated) reasons.push("grasp_uncalibrated");
        if (drop === null || !Number.isFinite(drop) || drop < 0) reasons.push("observation_drop_unconfigured");
        else if (inputs.lift_zero_height_m === null || !Number.isFinite(inputs.lift_zero_height_m) ||
          inputs.lift_zero_height_m < 0) reasons.push("lift_geometry_unconfigured");
        else {
          const z = warehouseInitialLift(level.height_m, drop, inputs.lift_zero_height_m);
          target.initial_lift_z_m = z;
          if (!Number.isFinite(z) || z < .020 || z > .500) reasons.push("lift_unreachable");
        }
      }
    }
    return target;
  }));
}

export function selectWarehouseTarget(targets: WarehouseTarget[], drug: string): WarehouseTarget | undefined {
  return targets.find(t => t.drug === drug && t.unavailable_reasons.length === 0);
}
