/** Editor-only identity operations. Never infer execution parameters from labels. */
import { WarehouseDocument, WarehouseShelf, WarehouseLevel, WarehouseInputs } from './warehouse_model';

export function nextIdentity(prefix: string, used: Iterable<string>): string {
  const ids = new Set(used);
  for (let n = 1; ; n++) if (!ids.has(prefix + n)) return prefix + n;
}
export function newLevel(layer: number, height_m: number): WarehouseLevel {
  return { layer, height_m, drug: '', qr: { left: null, right: null }, fetch: null };
}
export function duplicateShelf(doc: WarehouseDocument, source: WarehouseShelf): WarehouseShelf {
  const copy = structuredClone(source);
  copy.id = nextIdentity('shelf_', doc.layout.shelves.map(s => s.id));
  copy.name += ' 副本'; copy.pose.x_m += source.width_m + .2;
  for (const l of copy.levels) {
    l.fetch = null;
    for (const side of ["left", "right"] as const) if (l.qr[side]) l.qr[side]!.marker_id = null;
  }
  return copy;
}
export function warehouseInputs(capabilities: any): WarehouseInputs {
  const grasp = (name: string) => ({ calibrated: capabilities?.grasp?.[name]?.calibration_confirmed === true,
    observation_drop_m: capabilities?.grasp?.[name]?.observation_drop_m ?? null });
  return { grasp_mode: capabilities?.grasp?.mode,
    camera_xy_in_base_m: capabilities?.geometry?.color_optical_origin_xy_in_base_m ?? null,
    lift_zero_height_m: capabilities?.geometry?.lift_zero_height_m ?? null,
    observation_distance_m: capabilities?.navigation?.observation_distance_m ?? NaN,
    front: grasp('front'), top: grasp('top'), canonical_drugs: capabilities?.canonical_drugs ?? [] };
}
export const reasonLabels: Record<string, string> = {
  drug_unconfigured:'未填写药品', unknown_drug:'药品目录未配置', left_qr_unconfigured:'未设置左标记', right_qr_unconfigured:'未设置右标记',
  anchor_unconfigured:'未绑定地图标点', camera_geometry_unconfigured:'相机安装位置未标定',
  observation_distance_invalid:'观察距离未配置', observation_pose_nonfinite:'观察位计算无效',
  fetch_unconfigured:'抓取参数未配置', class_id_unconfigured:'模型类别未配置',
  roi_unconfigured:'识别区域未配置', search_min_unconfigured:'搜索起点未配置', search_max_unconfigured:'搜索终点未配置',
  grasp_unconfigured:'未选择抓法', grasp_uncalibrated:'抓法未标定', observation_drop_unconfigured:'观察高度偏置未配置',
  lift_geometry_unconfigured:'升降零位未配置', lift_unreachable:'升降目标超出行程'
};
