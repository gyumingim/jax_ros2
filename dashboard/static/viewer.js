// 팔 3D 뷰 (three.js r160, static/vendor 에 저장 — 오프라인에서도 됨).
// 모델: static/robot/arm.xml (team-jax/IK motion2/arm, MJCF). 관절 이름 = calibration.json 의 sim 관절명, 각은 rad.
// 좌표: 로봇 기준(vision_node·pick_place 와 같음) — 원점 = shoulder_roll 관절 원점, x 로봇 왼쪽+, y 뒤+(앞 = -y), z 위+.
//   이 틀은 arm.xml base_link(Shoulder_Pitch) 틀을 x 로 +0.052 m 옮긴 것과 같다(IK realcoords.py + env.py robot_yaw -90° 에서 계산,
//   pick_place 궤적의 손끝이 큐브 2 cm 안을 지나는 것으로 확인).
// 메시: 서버 /robot/meshes/<obj> (JAX_IK_DIR 등). 없으면 관절 원점을 잇는 막대 + 관절 구.
import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";
import { OBJLoader } from "three/addons/OBJLoader.js";

const REAL_FROM_BASE = new THREE.Vector3(0.052, 0, 0);
const CUBE = 0.05;
let xmlP = null;
const meshCache = new Map();

function loadXml() {
  if (!xmlP) xmlP = fetch("/static/robot/arm.xml").then((r) => { if (!r.ok) throw new Error("arm.xml " + r.status); return r.text(); })
    .then((t) => new DOMParser().parseFromString(t, "application/xml"));
  return xmlP;
}
function loadObj(file) {
  if (!meshCache.has(file)) {
    meshCache.set(file, new OBJLoader().loadAsync("/robot/meshes/" + encodeURIComponent(file)).then((g) => {
      const geos = [];
      g.traverse((o) => { if (o.isMesh) { if (!o.geometry.attributes.normal) o.geometry.computeVertexNormals(); geos.push(o.geometry); } });
      return geos;
    }).catch(() => null));
  }
  return meshCache.get(file);
}
const nums = (s, d) => (s == null ? d : s.trim().split(/\s+/).map(Number));
function placeFrom(el, obj) {
  const p = nums(el.getAttribute("pos"), [0, 0, 0]);
  const q = nums(el.getAttribute("quat"), [1, 0, 0, 0]);   // MJCF: w x y z
  obj.position.set(p[0], p[1], p[2]);
  obj.quaternion.set(q[1], q[2], q[3], q[0]).normalize();
}
const kids = (el, tag) => Array.from(el.children).filter((c) => c.tagName === tag);

// 로봇 한 벌. ghost = 반투명 파랑(슬라이더 목표).
class ArmModel {
  constructor(xml, { meshes, ghost }) {
    this.group = new THREE.Group();
    this.joints = new Map();
    this.ghost = !!ghost;
    this.meshes = meshes;
    this.pending = [];
    const assets = new Map();
    for (const m of xml.querySelectorAll("asset > mesh")) assets.set(m.getAttribute("name"), m.getAttribute("file"));
    const column = Array.from(xml.querySelectorAll("worldbody > body")).find((b) => b.getAttribute("name") === "column");
    const base = column && kids(column, "body").find((b) => b.getAttribute("name") === "base_link");
    if (!column || !base) throw new Error("arm.xml: column/base_link 없음");
    // 로봇 틀(real) = base_link 틀 + 0.052 x. _body(column) 이 column 자신의 pos 도 넣으므로 (column · base_link) 의 역만큼 옮긴다.
    const tc = new THREE.Object3D(); placeFrom(column, tc); tc.updateMatrix();
    const tb = new THREE.Object3D(); placeFrom(base, tb); tb.updateMatrix();
    const wb = tc.matrix.clone().multiply(tb.matrix);
    const m = new THREE.Matrix4().makeTranslation(REAL_FROM_BASE.x, REAL_FROM_BASE.y, REAL_FROM_BASE.z).multiply(wb.invert());
    m.decompose(this.group.position, this.group.quaternion, this.group.scale);
    const glass = (color, opacity) => new THREE.MeshStandardMaterial({ color, transparent: true, opacity, depthWrite: false });
    this.matStick = this.ghost ? glass(0x3b7bf0, 0.35) : new THREE.MeshStandardMaterial({ color: 0x4b5563, metalness: 0.2, roughness: 0.6 });
    this.matJoint = this.ghost ? glass(0x3b7bf0, 0.45) : new THREE.MeshStandardMaterial({ color: 0xf59e0b, metalness: 0.1, roughness: 0.5 });
    this.ghostMat = glass(0x3b7bf0, 0.28);
    this._body(column, this.group, assets, true);
  }
  _material(rgba) {
    if (this.ghost) return this.ghostMat;
    const c = nums(rgba, [0.6, 0.6, 0.6, 1]);
    // 짙은 회색(0.098) 부품은 밝은 배경에서 잘 보이게 조금 밝힌다(색만, 모양 그대로)
    const dark = Math.max(c[0], c[1], c[2]) < 0.2;
    const col = dark ? new THREE.Color(0.26, 0.28, 0.32) : new THREE.Color(c[0], c[1], c[2]);
    return new THREE.MeshStandardMaterial({ color: col, metalness: 0.15, roughness: 0.65 });
  }
  _body(el, parent, assets, isColumn) {
    const g = new THREE.Group();
    placeFrom(el, g);
    parent.add(g);
    let content = g;
    const j = kids(el, "joint").find((x) => (x.getAttribute("type") || "hinge") === "hinge");
    if (j) {
      const jg = new THREE.Group();
      g.add(jg);
      this.joints.set(j.getAttribute("name"), { obj: jg, axis: new THREE.Vector3(...nums(j.getAttribute("axis"), [0, 0, 1])).normalize() });
      content = jg;
    }
    if (this.meshes) {
      for (const geom of kids(el, "geom")) {
        if (geom.getAttribute("type") !== "mesh" || geom.getAttribute("group") === "3") continue;
        const file = assets.get(geom.getAttribute("mesh"));
        if (!file) continue;
        const holder = new THREE.Group();
        placeFrom(geom, holder);
        content.add(holder);
        const mat = this._material(geom.getAttribute("rgba"));
        this.pending.push(loadObj(file).then((geos) => {
          if (!geos) return false;
          for (const ge of geos) { const mm = new THREE.Mesh(ge, mat); mm.renderOrder = this.ghost ? 2 : 0; holder.add(mm); }
          return true;
        }));
      }
    } else {
      // 막대 모델: 관절 구 + 자식 몸체 원점까지 막대
      if (j) content.add(new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 12), this.matJoint));
      for (const c of kids(el, "body")) {
        const p = new THREE.Vector3(...nums(c.getAttribute("pos"), [0, 0, 0]));
        const len = p.length();
        if (len < 1e-4) continue;
        const r = isColumn ? 0.02 : 0.008;
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 10), this.matStick);
        cyl.position.copy(p).multiplyScalar(0.5);
        cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), p.clone().normalize());
        content.add(cyl);
      }
    }
    for (const s of kids(el, "site")) {
      if (s.getAttribute("name") !== "end_effector") continue;
      const sp = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 8),
        new THREE.MeshBasicMaterial({ color: this.ghost ? 0x3b7bf0 : 0x16a34a, transparent: this.ghost, opacity: this.ghost ? 0.5 : 1 }));
      placeFrom(s, sp);
      content.add(sp);
    }
    for (const c of kids(el, "body")) this._body(c, content, assets, false);
  }
  setPose(q) {
    for (const [name, j] of this.joints) {
      const v = q && Number.isFinite(q[name]) ? q[name] : 0;
      j.obj.quaternion.setFromAxisAngle(j.axis, v);
    }
  }
}

function textSprite(text, color) {
  const c = document.createElement("canvas");
  c.width = 256; c.height = 64;
  const x = c.getContext("2d");
  x.font = "bold 34px sans-serif"; x.fillStyle = color; x.textBaseline = "middle"; x.fillText(text, 6, 32);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), transparent: true }));
  s.scale.set(0.16, 0.04, 1);
  s.center.set(0, 0.5);
  return s;
}

export class Viewer {
  constructor(container, opts = {}) {
    this.el = container;
    this.opts = opts;
    this.d = 0.4775;
    this.status = document.createElement("div");
    this.status.className = "v3d-status";
    container.append(this.status);
    try {
      this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    } catch (e) {
      this.status.textContent = "3D 를 못 그림: 이 브라우저에서 WebGL 을 못 씀 (" + e.message + ")";
      this.dead = true;
      return;
    }
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    container.prepend(this.renderer.domElement);
    const scene = this.scene = new THREE.Scene();
    scene.background = new THREE.Color(0xeef2f7);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.8); sun.position.set(1, -1.5, 2); scene.add(sun);
    const cam = this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 20);
    cam.up.set(0, 0, 1);
    cam.position.set(-0.75, -1.25, 0.2);   // 앞(-y)·오른쪽(-x) 위에서 (받침대가 팔 뒤로 가게)
    this.controls = new OrbitControls(cam, this.renderer.domElement);
    this.controls.target.set(0.0, -0.12, -0.3);
    this.controls.update();
    this.controls.addEventListener("change", () => this.dirty());
    // 책상 높이 격자(어깨 축 아래 d) + 축
    this.grid = new THREE.GridHelper(1.2, 24, 0x94a3b8, 0xcbd5e1);
    this.grid.rotation.x = Math.PI / 2;
    scene.add(this.grid);
    scene.add(new THREE.AxesHelper(0.15));
    const lx = textSprite("x 왼쪽", "#dc2626"); lx.position.set(0.16, 0, 0); scene.add(lx);
    const ly = textSprite("y 뒤", "#16a34a"); ly.position.set(0, 0.16, 0); scene.add(ly);
    const lz = textSprite("z 위", "#2563eb"); lz.position.set(0, 0, 0.17); scene.add(lz);
    const lf = textSprite("앞 (-y)", "#64748b"); this.frontLabel = lf; scene.add(lf);
    this.setTableD(this.d);
    // 큐브·놓을 자리
    this.cube = new THREE.Mesh(new THREE.BoxGeometry(CUBE, CUBE, CUBE), new THREE.MeshStandardMaterial({ color: 0xf97316 }));
    this.cube.visible = false; scene.add(this.cube);
    const tg = new THREE.Group();
    tg.add(new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(CUBE, CUBE, CUBE)), new THREE.LineBasicMaterial({ color: 0x16a34a })));
    tg.add(new THREE.Mesh(new THREE.BoxGeometry(CUBE, CUBE, CUBE), new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.18, depthWrite: false })));
    this.target = tg; tg.visible = false; scene.add(tg);
    this.objMark = new THREE.Mesh(new THREE.BoxGeometry(CUBE, CUBE, CUBE), new THREE.MeshStandardMaterial({ color: 0xa855f7, transparent: true, opacity: 0.6 }));
    this.objMark.visible = false; scene.add(this.objMark);
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this._loop = this._loop.bind(this);
    requestAnimationFrame(this._loop);
  }
  setTableD(d) {
    if (!Number.isFinite(d) || (d === this.d && this._dset)) return;
    this.d = d; this._dset = true;
    this.grid.position.set(0, -0.25, -d);
    this.frontLabel.position.set(0, -0.62, -d + 0.02);
    this.dirty();
  }
  async load(meshOk) {
    if (this.dead) return;
    try {
      const xml = await loadXml();
      this.actual = new ArmModel(xml, { meshes: meshOk, ghost: false });
      this.scene.add(this.actual.group);
      if (this.opts.ghost) { this.ghost = new ArmModel(xml, { meshes: meshOk, ghost: true }); this.ghost.group.visible = false; this.scene.add(this.ghost.group); }
      this.status.textContent = meshOk ? "메시 불러오는 중…" : "막대 모델 (메시 없음: JAX_IK_DIR 에 team-jax/IK 를 두면 메시로 보임)";
      this.actual.setPose({});
      if (this.ghost) this.ghost.setPose({});
      if (meshOk) {
        const r = await Promise.all(this.actual.pending.concat(this.ghost ? this.ghost.pending : []));
        const bad = r.filter((x) => !x).length;
        this.status.textContent = bad ? `메시 ${r.length - bad}/${r.length} 불러옴 (나머지 못 읽음)` : "메시 (team-jax/IK motion2/arm)";
      }
      this.ready = true;
      this.dirty();
    } catch (e) {
      this.status.textContent = "3D 모델을 못 읽음: " + e.message;
    }
  }
  setActual(q) { if (this.actual) { this.actual.setPose(q || {}); this.dirty(); } }
  setGhost(q) {
    if (!this.ghost) return;
    this.ghost.group.visible = !!q;
    if (q) this.ghost.setPose(q);
    this.dirty();
  }
  setCube(c) {
    this.cube.visible = !!c;
    if (c) { this.cube.position.set(c.x, c.y, -this.d + CUBE / 2); this.cube.rotation.set(0, 0, (c.yaw || 0) * Math.PI / 180); }
    this.dirty();
  }
  setTarget(t) {
    this.target.visible = !!t;
    if (t) this.target.position.set(t.x, t.y, -this.d + CUBE / 2);
    this.dirty();
  }
  setObject(o) {   // /object_pose (shoulder_roll_axis 틀): 보라 반투명 상자
    this.objMark.visible = !!o;
    if (o) { this.objMark.position.set(o.x, o.y, Number.isFinite(o.z) ? o.z : -this.d + CUBE / 2); this.objMark.rotation.set(0, 0, (o.yaw || 0) * Math.PI / 180); }
    this.dirty();
  }
  dirty() { this._dirty = true; }
  resize() {
    if (this.dead) return;
    const w = this.el.clientWidth || 300, h = this.el.clientHeight || 300;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = "100%";
    this.renderer.domElement.style.height = "100%";
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.dirty();
  }
  _loop() {
    requestAnimationFrame(this._loop);
    if (!this._dirty || this.el.offsetParent === null) return;   // 숨은 탭은 안 그린다
    this._dirty = false;
    this.renderer.render(this.scene, this.camera);
  }
}

window.JaxViewer = { create: (el, opts) => new Viewer(el, opts) };
window.dispatchEvent(new Event("jaxviewer-ready"));
