import { ARUCO_4X4_50 } from './aruco_4x4_50';
import { escapeHTML as esc } from './core';
import { QrSide, WarehouseLevel } from './warehouse_model';

export function warehouseQrSvg(_shelfId: string, level: WarehouseLevel, side: QrSide): { svg: string; widthMm: number; id: number } {
  const id = level.qr[side]?.marker_id;
  if (id === null || id === undefined || !Number.isInteger(id) || id < 0 || id >= ARUCO_4X4_50.length) throw new Error('请先配置有效的标记 ID');
  const bits = ARUCO_4X4_50[id]; let path = '';
  // An 8x8 sheet: white one-module quiet zone, black 6x6 marker, 4x4 data.
  for (let y = 0; y < 6; y++) for (let x = 0; x < 6; x++) {
    if (x === 0 || x === 5 || y === 0 || y === 5 || bits[(y - 1) * 4 + x - 1] === '1') path += `M${x+1} ${y+1}h1v1h-1z`;
  }
  return { id, widthMm: 40, svg: `<svg xmlns="http://www.w3.org/2000/svg" width="40mm" height="40mm" viewBox="0 0 8 8" shape-rendering="crispEdges" aria-label="ArUco 边界标记"><rect width="8" height="8" fill="white"/><path d="${path}" fill="black"/></svg>` };
}
export function printWarehouseQr(shelfId: string, level: WarehouseLevel, side: QrSide): void {
  const code = warehouseQrSvg(shelfId, level, side);
  const frame = document.createElement('iframe'); frame.title = 'ArUco 标记打印';
  frame.style.cssText = 'position:fixed;left:-10000px;width:600px;height:600px;border:0'; document.body.append(frame);
  const doc = frame.contentDocument!;
  doc.open(); doc.write('<!doctype html><meta charset="utf-8"><title>货架边界标记</title><style>@page{margin:15mm}body{margin:0;color:#000;background:#fff;font:12pt sans-serif}.label{width:65mm;break-inside:avoid}svg{display:block;max-width:none}p{margin:3mm 0}</style><div class="label">' + code.svg + '<p>' + esc(shelfId) + ' · ' + level.layer + '层 · ID ' + code.id + '</p><p>' + (side === 'left' ? '左边界 left' : '右边界 right') + '</p><p>纸面40×40 mm · 打印比例100%</p></div>'); doc.close();
  const cleanup = () => frame.remove(); frame.contentWindow!.addEventListener('afterprint', cleanup, { once: true });
  frame.contentWindow!.focus(); frame.contentWindow!.print(); cleanup();
}
