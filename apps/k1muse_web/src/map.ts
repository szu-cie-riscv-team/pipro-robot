import { escapeHTML, Geometry, gridToWorld, Target, worldToGrid, yaw } from "./core";
export class MapView {
  canvas: HTMLCanvasElement;
  geometry?: Geometry;
  image?: HTMLCanvasElement;
  pose?: Target;
  poseAt = 0;
  paths: Record<string, Target[]> = {};
  points: Record<string, Target> = {};
  scan: any;
  scanPose: any;
  target?: Target;
  tool = "point";
  showOrigin = false;
  onTarget: (target: Target) => void = () => {};
  private scale = 1;
  private offset = [0, 0];
  private start?: [number, number];
  private pending = false;
  private disposed = false;
  private observer: ResizeObserver;
  private events = new AbortController();
  constructor(canvas: HTMLCanvasElement, interactive = true) {
    this.canvas = canvas;
    this.observer = new ResizeObserver(() => this.draw());
    this.observer.observe(canvas);
    canvas.addEventListener("pointerdown", e => {
      if (!interactive || !this.geometry) return;
      this.start = this.at(e); canvas.setPointerCapture(e.pointerId);
    }, { signal: this.events.signal });
    canvas.addEventListener("pointerup", e => {
      if (!interactive || !this.geometry || !this.start) return;
      const a = gridToWorld(this.geometry, ...this.start);
      const b = gridToWorld(this.geometry, ...this.at(e));
      a.yaw = Math.hypot(a.x - b.x, a.y - b.y) > this.geometry.resolution ?
        Math.atan2(b.y - a.y, b.x - a.x) : 0;
      this.start = undefined; this.target = a; this.onTarget(a); this.draw();
    }, { signal: this.events.signal });
    canvas.addEventListener("pointercancel", () => { this.start = undefined; }, { signal: this.events.signal });
  }
  dispose() { this.disposed = true; this.observer.disconnect(); this.events.abort(); this.clear(); }
  private at(e: PointerEvent): [number, number] {
    const r = this.canvas.getBoundingClientRect();
    return [(e.clientX - r.left - this.offset[0]) / this.scale,
      (e.clientY - r.top - this.offset[1]) / this.scale];
  }
  clear() {
    this.geometry = undefined; this.image = undefined; this.pose = undefined;
    this.paths = {}; this.scan = this.scanPose = undefined; this.target = undefined; this.draw();
  }
  message(topic: string, msg: any) {
    if (topic === "map") {
      const info = msg.info;
      if (!info.width || !info.height || info.width * info.height > 16_000_000) return;
      this.geometry = { width: info.width, height: info.height, resolution: info.resolution,
        origin: { x: info.origin.position.x, y: info.origin.position.y, yaw: yaw(info.origin.orientation) } };
      const image = document.createElement("canvas"); image.width = info.width; image.height = info.height;
      const context = image.getContext("2d")!, pixels = context.createImageData(info.width, info.height);
      for (let y = 0; y < info.height; y++) for (let x = 0; x < info.width; x++) {
        const value = msg.data[y * info.width + x];
        const index = ((info.height - 1 - y) * info.width + x) * 4;
        const gray = value < 0 ? 122 : Math.round(255 * (1 - value / 100));
        pixels.data[index] = gray;
        pixels.data[index + 1] = gray;
        pixels.data[index + 2] = gray;
        pixels.data[index + 3] = 255;
      }
      context.putImageData(pixels, 0, 0); this.image = image;
    } else if (topic === "pose") {
      this.pose = this.position(msg.pose); this.poseAt = performance.now();
    }
    else if (topic.endsWith("_path")) {
      if (msg.header.frame_id !== "map") return;
      this.paths[topic] = msg.poses.map((p: any) => this.position(p.pose));
    } else if (topic === "scan") this.scan = msg;
    else if (topic === "scan_pose") this.scanPose = msg;
    this.draw();
  }
  private position(p: any): Target { return { x: p.position.x, y: p.position.y, yaw: yaw(p.orientation) }; }
  draw() {
    if (this.pending || this.disposed) return;
    this.pending = true;
    requestAnimationFrame(() => { this.pending = false; if (!this.disposed) this.paint(); });
  }
  private paint() {
    const c = this.canvas, width = c.clientWidth, height = c.clientHeight;
    const dpr = window.devicePixelRatio || 1;
    c.width = width * dpr; c.height = height * dpr;
    const ctx = c.getContext("2d")!; ctx.scale(dpr, dpr);
    ctx.fillStyle = "#17283b"; ctx.fillRect(0, 0, width, height);
    if (!this.geometry || !this.image) {
      ctx.fillStyle = "#a7b8cc"; ctx.font = "15px system-ui"; ctx.textAlign = "center";
      ctx.fillText("等待地图 · 启动建图或选择已保存地图", width / 2, height / 2); return;
    }
    const g = this.geometry; this.scale = Math.min((width - 40) / g.width, (height - 40) / g.height);
    this.offset = [(width - g.width * this.scale) / 2, (height - g.height * this.scale) / 2];
    ctx.save(); ctx.translate(...this.offset as [number, number]); ctx.scale(this.scale, this.scale);
    ctx.imageSmoothingEnabled = false; ctx.drawImage(this.image, 0, 0);
    const xy = (p: Target) => worldToGrid(g, p);
    for (const [name, path] of Object.entries(this.paths)) {
      ctx.strokeStyle = name === "global_path" ? "#05cba2" : "#ed982d"; ctx.lineWidth = 2 / this.scale;
      ctx.beginPath(); path.forEach((p, i) => { const [x, y] = xy(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
    }
    if (this.scan && this.scanPose && this.scan.header.stamp.sec === this.scanPose.header.stamp.sec &&
        this.scan.header.stamp.nanosec === this.scanPose.header.stamp.nanosec) {
      const p = this.position(this.scanPose.pose); ctx.fillStyle = "#d84b69";
      this.scan.ranges.forEach((range: number, i: number) => {
        if (!Number.isFinite(range) || range < this.scan.range_min || range > this.scan.range_max) return;
        const angle = this.scan.angle_min + i * this.scan.angle_increment + p.yaw;
        const [x, y] = xy({ x: p.x + Math.cos(angle) * range, y: p.y + Math.sin(angle) * range, yaw: 0 });
        ctx.fillRect(x, y, 2 / this.scale, 2 / this.scale);
      });
    }
    const arrow = (p: Target, color: string) => {
      const [x, y] = xy(p); ctx.save(); ctx.translate(x, y); ctx.rotate(-(p.yaw - g.origin.yaw));
      ctx.scale(1 / this.scale, 1 / this.scale);
      ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(13, 0); ctx.lineTo(-8, -7); ctx.lineTo(-4, 0); ctx.lineTo(-8, 7);
      ctx.closePath(); ctx.fill(); ctx.restore();
    };
    if (this.showOrigin) {
      const [ox, oy] = xy({ x: 0, y: 0, yaw: 0 });
      ctx.strokeStyle = "#ed982d"; ctx.fillStyle = "#ed982d"; ctx.lineWidth = 2 / this.scale;
      ctx.font = `${12 / this.scale}px system-ui`;
      for (const [p, label] of [[{ x: .4, y: 0, yaw: 0 }, "+X"], [{ x: 0, y: .4, yaw: 0 }, "+Y"]] as const) {
        const [x, y] = xy(p); ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(x, y); ctx.stroke();
        ctx.fillText(label, x + 3 / this.scale, y - 3 / this.scale);
      }
      ctx.fillText("建图起点 (0,0)", ox + 5 / this.scale, oy + 16 / this.scale);
    }
    for (const [name, p] of Object.entries(this.points)) {
      arrow(p, "#8368da"); const [x, y] = xy(p);
      ctx.fillStyle = "#302150"; ctx.font = `${12 / this.scale}px system-ui`;
      ctx.fillText(name, x + 14 / this.scale, y - 8 / this.scale);
    }
    if (this.target) arrow(this.target, "#ed982d");
    if (this.pose) arrow(this.pose, "#127bd0");
    ctx.restore();
  }
  description(p: Target): string {
    return `${escapeHTML(p.x.toFixed(2))}, ${p.y.toFixed(2)} m · ${(p.yaw * 180 / Math.PI).toFixed(0)}°`;
  }
}
