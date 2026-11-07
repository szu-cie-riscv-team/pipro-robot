export type InventoryLayer = {
  shelf_id: string; layer: number; drug: string; quantity: number | null;
  source: 'scan' | 'manual' | 'fetch' | null;
  updated_at: string | null; observed_at: string | null; stale: boolean;
  last_scan: null | { status: 'complete' | 'incomplete'; attempted_at: string; reason: string };
};
export type Inventory = {
  map_id: string; layers: InventoryLayer[]; persisted: boolean;
  load_error: string; save_error: string;
};
export function layerInventory(data: Inventory | undefined, map: string, shelf: string, layer: number, drug: string): InventoryLayer | undefined {
  if (!drug || data?.map_id !== map) return;
  return data.layers.find(r => r.shelf_id === shelf && r.layer === layer && r.drug === drug);
}
export function inventoryTotals(rows: { drug: string; quantity: number | null | undefined }[]) {
  const totals = new Map<string, { known: number; unknown: number }>();
  for (const row of rows) {
    if (!row.drug) continue;
    const total = totals.get(row.drug) ?? { known: 0, unknown: 0 };
    if (Number.isInteger(row.quantity) && row.quantity! >= 0) total.known += row.quantity!;
    else total.unknown++;
    totals.set(row.drug, total);
  }
  return totals;
}
