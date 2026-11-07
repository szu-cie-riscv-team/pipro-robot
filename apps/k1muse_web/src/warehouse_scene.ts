import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { QrSide, WarehouseDocument, WarehouseInputs, buildWarehouseTargets, warehouseLayoutToMap } from './warehouse_model';
export interface WarehouseSelection { shelfId: string; layer: number | null; qr: false | QrSide }
export type WarehouseSceneEdit = { kind: 'shelf'; x: number; y: number; yaw: number } | { kind: 'qr'; x: number; z: number };

/** Z-up editor. No render loop, physics, telemetry subscription or model download. */
export class WarehouseScene {
  private renderer = new THREE.WebGLRenderer({ antialias: true });
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(45, 1, .01, 1000);
  private orbit: OrbitControls;
  private transform: TransformControls;
  private observer: ResizeObserver;
  private content = new THREE.Group();
  private box = new THREE.BoxGeometry(1, 1, 1);
  private sphere = new THREE.SphereGeometry(.012, 10, 6);
  private materials = {
    post: new THREE.MeshStandardMaterial({ color: 0x718393, roughness: .75 }),
    board: new THREE.MeshStandardMaterial({ color: 0xdde5e9, roughness: .85 }),
    active: new THREE.MeshStandardMaterial({ color: 0x75afe5 }),
    qr: new THREE.MeshBasicMaterial({ color: 0xf5c843 }),
    origin: new THREE.MeshBasicMaterial({ color: 0xdc6655 }),
    parking: new THREE.MeshBasicMaterial({ color: 0x17a693 }),
  };
  private frame = 0;
  private disposed = false;
  private selection?: WarehouseSelection;
  private mode: 'move' | 'rotate' | 'qr' = 'move';
  private cameraSet = false;
  private dragged = false;
  private down?: [number, number];
  constructor(private host: HTMLElement, private select: (s: WarehouseSelection) => void,
    private edit: (change: WarehouseSceneEdit) => void) {
    this.scene.background = new THREE.Color(0xe9eff3);
    this.camera.up.set(0, 0, 1); this.camera.position.set(1, -1.8, 1.5);
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.domElement.setAttribute('aria-label', '仓库三维模型；可拖动旋转视角，选择货架或二维码');
    host.append(this.renderer.domElement);
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.enableDamping = false; this.orbit.target.set(.25, 0, .3);
    this.orbit.update(); this.orbit.addEventListener('change', this.draw);
    this.transform = new TransformControls(this.camera, this.renderer.domElement);
    this.transform.addEventListener('change', this.draw);
    this.transform.addEventListener('dragging-changed', e => { this.orbit.enabled = !e.value; this.dragged ||= !!e.value; });
    this.transform.addEventListener('objectChange', () => {
      if (this.mode === 'qr' && this.transform.object) this.transform.object.position.y = 0;
      this.draw();
    });
    this.transform.addEventListener('mouseUp', () => {
      const o = this.transform.object;
      if (!o || !this.dragged) return;
      const change: WarehouseSceneEdit = this.mode === 'qr' ? { kind: 'qr', x: o.position.x, z: o.position.z } :
        { kind: 'shelf', x: o.position.x, y: o.position.y, yaw: o.rotation.z };
      queueMicrotask(() => { if (!this.disposed) this.edit(change); });
    });
    this.scene.add(this.transform.getHelper(), this.content);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x75828e, 2.4));
    const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(-2, -3, 5); this.scene.add(light);
    const grid = new THREE.GridHelper(20, 100, 0xb6c7d3, 0xd6e0e7); grid.rotation.x = Math.PI / 2; grid.position.z = -.025;
    this.scene.add(grid);
    this.observer = new ResizeObserver(this.draw); this.observer.observe(host);
    this.renderer.domElement.addEventListener('pointerdown', this.pointerDown);
    this.renderer.domElement.addEventListener('pointerup', this.pointerUp);
    this.draw();
  }
  private pointerDown = (e: PointerEvent) => { this.down = [e.clientX, e.clientY]; this.dragged = this.transform.dragging; };
  private pointerUp = (e: PointerEvent) => {
    if (!this.down || this.dragged || Math.hypot(e.clientX - this.down[0], e.clientY - this.down[1]) > 5) return;
    this.down = undefined;
    const r = this.renderer.domElement.getBoundingClientRect(), ray = new THREE.Raycaster();
    ray.setFromCamera(new THREE.Vector2(2 * (e.clientX - r.left) / r.width - 1, 1 - 2 * (e.clientY - r.top) / r.height), this.camera);
    for (const hit of ray.intersectObjects(this.content.children, true)) {
      let o: THREE.Object3D | null = hit.object;
      while (o && !o.userData.selection) o = o.parent;
      if (o) { this.select(o.userData.selection); return; }
    }
  };
  private mesh(parent: THREE.Object3D, size: [number, number, number], pos: [number, number, number], material: THREE.Material) {
    const m = new THREE.Mesh(this.box, material); m.scale.set(...size); m.position.set(...pos); parent.add(m); return m;
  }
  private label(parent: THREE.Object3D, text: string, x: number, y: number, z: number, color = '#176754') {
    const canvas = document.createElement('canvas'); canvas.width = 512; canvas.height = 80;
    const c = canvas.getContext('2d')!; c.fillStyle = '#ffffff'; c.fillRect(0, 0, 512, 80);
    c.strokeStyle = color; c.lineWidth = 6; c.strokeRect(0, 0, 512, 80);
    c.font = '32px sans-serif'; c.textAlign = 'center'; c.textBaseline = 'middle'; c.fillStyle = color;
    c.fillText(text, 256, 40, 490);
    const texture = new THREE.CanvasTexture(canvas); texture.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, toneMapped: false }));
    sprite.position.set(x, y, z); sprite.scale.set(.25, .039, 1); sprite.renderOrder = 2; parent.add(sprite);
  }
  private clear() {
    this.transform.detach();
    this.content.traverse(o => {
      if (o instanceof THREE.Sprite) { o.material.map?.dispose(); o.material.dispose(); }
      if (o instanceof THREE.Box3Helper) { o.geometry.dispose(); (o.material as THREE.Material).dispose(); }
      // ArrowHelper geometries are shared by Three.js; only its cloned materials are owned here.
      if (o.parent instanceof THREE.ArrowHelper && ('material' in o)) (o.material as THREE.Material).dispose();
    });
    this.content.clear();
  }
  update(doc: WarehouseDocument, selection: WarehouseSelection, inputs: WarehouseInputs) {
    this.selection = selection; this.clear();
    let selectedShelf: THREE.Group | undefined, selectedQr: THREE.Object3D | undefined;
    for (const shelf of doc.layout.shelves) {
      const group = new THREE.Group(); group.position.set(shelf.pose.x_m, shelf.pose.y_m, 0); group.rotation.z = shelf.pose.yaw_rad;
      group.userData.selection = { shelfId: shelf.id, layer: null, qr: false }; this.content.add(group);
      const top = Math.max(.1, ...shelf.levels.map(l => l.height_m)) + .08;
      for (const x of [0, shelf.width_m]) for (const y of [0, shelf.depth_m])
        this.mesh(group, [.018, .018, top], [x, y, top / 2], this.materials.post);
      for (const l of shelf.levels) {
        const active = shelf.id === selection.shelfId && l.layer === selection.layer;
        const board = this.mesh(group, [shelf.width_m, shelf.depth_m, .012], [shelf.width_m / 2, shelf.depth_m / 2, l.height_m - .006], active ? this.materials.active : this.materials.board);
        board.userData.selection = { shelfId: shelf.id, layer: l.layer, qr: false };
        this.mesh(group, [shelf.width_m, .012, .025], [shelf.width_m / 2, shelf.depth_m, l.height_m - .025], this.materials.post);
        this.label(group, l.layer + '层 · ' + (l.drug || '未标药品') + ' · ' + Math.round(l.height_m * 100) + 'cm', shelf.width_m / 2, -.045, l.height_m + .045);
        for (const side of ['left', 'right'] as const) {
          const marker = l.qr[side]; if (!marker) continue;
          const qr = this.mesh(group, [.03, .006, .03], [marker.x_m, 0, marker.z_m], this.materials.qr);
          qr.userData.selection = { shelfId: shelf.id, layer: l.layer, qr: side };
          this.label(group, side === 'left' ? '左边界' : '右边界', marker.x_m, -.015, marker.z_m + .085, '#9c7300');
          if (active && selection.qr === side) selectedQr = qr;
        }
      }
      const origin = new THREE.Mesh(this.sphere, this.materials.origin); group.add(origin);
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(.05, -.17, .005), .13, 0x2876bd, .04, .025); group.add(arrow);
      this.label(group, shelf.name || shelf.id, shelf.width_m / 2, shelf.depth_m / 2, top + .1, '#244c74');
      if (shelf.id === selection.shelfId) {
        selectedShelf = group;
        group.updateWorldMatrix(true, false);
        const bounds = new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(shelf.width_m, shelf.depth_m, top)).applyMatrix4(group.matrixWorld);
        const outline = new THREE.Box3Helper(bounds, 0x1683df);
        this.content.add(outline);
      }
    }
    const a = doc.layout.anchor, ref = a && doc.layout.shelves.find(s => s.id === a.shelf_id);
    const qr = ref?.levels.find(l => l.layer === a!.layer)?.qr.left;
    if (a && ref && qr && inputs.camera_xy_in_base_m) {
      const t = warehouseLayoutToMap(ref, qr, a, inputs.camera_xy_in_base_m);
      const points = buildWarehouseTargets(doc, inputs);
      for (const p of points) if (p.observation_pose) {
        const dx = p.observation_pose.x_m - t.x_m, dy = p.observation_pose.y_m - t.y_m;
        const x = Math.cos(t.yaw_rad) * dx + Math.sin(t.yaw_rad) * dy;
        const y = -Math.sin(t.yaw_rad) * dx + Math.cos(t.yaw_rad) * dy;
        const yaw = p.observation_pose.yaw_rad - t.yaw_rad;
        this.content.add(new THREE.ArrowHelper(new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0), new THREE.Vector3(x, y, .015), .13, 0x17a693, .04, .025));
        this.label(this.content, p.shelf_id + '/' + p.layer + ' 机器人观察位', x, y, .09);
      }
    }
    const o = this.mode === 'qr' ? selectedQr : selectedShelf;
    if (o) this.transform.attach(o);
    this.configure();
    if (!this.cameraSet && doc.layout.shelves.length) { this.fit(); this.cameraSet = true; }
    this.draw();
  }
  private configure() {
    this.transform.setMode(this.mode === 'rotate' ? 'rotate' : 'translate');
    this.transform.setSpace(this.mode === 'qr' ? 'local' : 'world');
    this.transform.showX = this.mode !== 'rotate';
    this.transform.showY = this.mode === 'move'; this.transform.showZ = this.mode !== 'move';
  }
  setEditable(enabled: boolean) { this.transform.enabled = enabled; this.transform.getHelper().visible = enabled; }
  setMode(mode: 'move' | 'rotate' | 'qr') { this.mode = mode; }
  view(top: boolean) {
    const target = this.orbit.target, distance = Math.max(1.5, this.camera.position.distanceTo(target));
    this.camera.position.copy(target).add(top ? new THREE.Vector3(0, -.0001, distance) : new THREE.Vector3(distance * .5, -distance * .8, distance * .65));
    this.orbit.update(); this.draw();
  }
  fit() {
    const b = new THREE.Box3();
    for (const g of this.content.children) if (g instanceof THREE.Group) b.expandByObject(g);
    if (b.isEmpty()) return;
    const center = b.getCenter(new THREE.Vector3()), size = b.getSize(new THREE.Vector3());
    this.orbit.target.copy(center); const d = Math.max(1.2, size.length() * .9);
    this.camera.position.copy(center).add(new THREE.Vector3(d * .5, -d * .9, d * .65));
    this.orbit.update(); this.draw();
  }
  private draw = () => {
    if (this.disposed || this.frame) return;
    this.frame = requestAnimationFrame(() => {
      this.frame = 0; if (this.disposed) return;
      const w = this.host.clientWidth, h = this.host.clientHeight;
      if (!w || !h) return;
      this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
      this.renderer.render(this.scene, this.camera);
    });
  };
  dispose() {
    this.disposed = true; cancelAnimationFrame(this.frame); this.observer.disconnect();
    this.renderer.domElement.removeEventListener('pointerdown', this.pointerDown);
    this.renderer.domElement.removeEventListener('pointerup', this.pointerUp);
    this.orbit.dispose(); this.transform.dispose(); this.clear();
    this.scene.traverse(o => { if (o instanceof THREE.GridHelper) { o.geometry.dispose(); (o.material as THREE.Material).dispose(); } });
    this.box.dispose(); this.sphere.dispose(); Object.values(this.materials).forEach(m => m.dispose());
    this.renderer.dispose(); this.renderer.forceContextLoss(); this.renderer.domElement.remove(); this.scene.clear();
  }
}
