// 큐브 실시간 인식 (live_api.py) 을 휴대폰 카메라 · 탑 쌓기 · 강화학습 탭이 같이 쓴다. 사진 올리기는 없다.
//   LQ.r      = 마지막 GET /api/live/cubes 결과 (0.5초마다, 이 세 탭이나 역기구학 탭이 보일 때만)
//   LQ.on(f)  = 새 결과마다 f(r)
//   LQ.sel    = 고른 큐브 번호 (탭끼리 같이 씀: 휴대폰 탭이 집을 큐브, 강화학습 '영상 인식' 의 큐브)
//   lvMapAttach(svg, opt) = 위에서 본 지도에 큐브를 그리고 누르기·끌기를 받는다 (가상 모드면 놓기·끌기·돌리기·지우기 = /api/live/scene)
//   lvTools()  = 가상 장면 고치기 줄 (고른 큐브의 각 · 지우기 · 놓을 때 어긋남) — 가상 모드에서만 보인다
//   lvBoxes(view, d, skip) = 3D 에 가상 큐브 상자들 (가상 모드)
// app.js 뒤, phone.js·stack.js·rl.js 앞에 읽힌다.
"use strict";
const LQ = { r: null, subs: [], busy: false, sel: null, drag: null, tools: [] };
LQ.on = (f) => LQ.subs.push(f);
LQ.cubes = () => (LQ.r && LQ.r.cubes) || [];
LQ.virtual = () => !!(LQ.r && LQ.r.mode === "virtual");
LQ.byId = (id) => LQ.cubes().find((c) => c.id === id) || null;
LQ.isTop = (c) => !LQ.cubes().some((o) => o !== c && o.level > c.level && Math.hypot(o.x - c.x, o.y - c.y) <= 0.03);
async function lvPoll() {
  if (LQ.busy) return;
  LQ.busy = true;
  let r;
  try {
    const q = "d=" + encodeURIComponent(tableD()) + (LQ.sel != null ? "&pick=" + LQ.sel : "");
    r = await (await fetch("/api/live/cubes?" + q)).json();
  } catch (e) { r = { ok: false, stale: true, reason: "서버 연결 실패", cubes: [], pick: { ok: false, reason: "서버 연결 실패", warn: [], note: [] } }; }
  LQ.busy = false;
  LQ.r = r;
  if (LQ.sel != null && !LQ.byId(LQ.sel) && r.mode === "virtual") LQ.sel = null;
  for (const f of LQ.subs) { try { f(r); } catch (e) { console.error(e); } }
  for (const t of LQ.tools) t.render();
}
setInterval(() => { if (!document.hidden && ["phone", "stack", "rl", "ik"].includes(curTab)) lvPoll(); }, 500);
async function lvScene(body) {
  const r = await post("/api/live/scene", body, true);
  await lvPoll();
  return r;
}

const LV_SVG = (tag, at, ...kids) => { const e = document.createElementNS("http://www.w3.org/2000/svg", tag); for (const [k, v] of Object.entries(at)) e.setAttribute(k, v); e.append(...kids); return e; };
// 큐브 상자 그림 (지도 공통): 책상 위 = 주황, 쌓인 것 = 파랑, 고른 것 = 굵은 테두리. 끄는 중이면 끄는 자리에
function lvDrawCubes(px, py, opt) {
  const g = [], o = opt || {};
  const list = LQ.cubes().slice().sort((a, b) => a.level - b.level);
  for (const c of list) {
    const d = LQ.drag && LQ.drag.id === c.id && LQ.drag.moved ? LQ.drag : c;
    const t = `rotate(${-c.yaw_deg} ${px(d.x)} ${py(d.y)})`, sel = c.id != null && c.id === LQ.sel;
    g.push(LV_SVG("rect", { x: px(d.x) - 25, y: py(d.y) - 25, width: 50, height: 50, rx: 3, transform: t,
      fill: c.source === "depth" ? "#a8a29e" : c.level ? "#2563eb" : "#f97316", stroke: sel ? "#111827" : "#fff", "stroke-width": sel ? 5 : 2,
      "fill-opacity": LQ.drag && LQ.drag.id === c.id && LQ.drag.moved ? 0.6 : 1 }),
      LV_SVG("text", { x: px(d.x), y: py(d.y) + 6, "font-size": 17, "font-weight": 700, fill: "#fff", "text-anchor": "middle", "pointer-events": "none" },
        c.id == null ? "뎁스" : String(c.id) + (c.level ? "↑" + (c.level + 1) : "")));
  }
  if (o.empty && !list.length) g.push(LV_SVG("text", { x: px(o.empty[0]), y: py(o.empty[1]), "font-size": 18, fill: "#64748b", "text-anchor": "middle" }, o.emptyText || ""));
  return g;
}
// 지도 누르기·끌기. opt = { box: [x0, x1, y0, y1] (실좌표 m, 그림 viewBox 와 같게), redraw(), onEmpty(x, y) (큐브가 아닌 곳을 누름; true 면 처리함),
//   onCube(c) (큐브를 누름), useCubes() (false 면 큐브를 안 봄 — 누른 곳은 빈 곳), edit() (지금 가상 장면을 고치나 — 빈 곳 = 큐브 놓기, 끌기 = 옮기기) }
function lvMapAttach(svg, opt) {
  const at = (ev) => {
    const b = svg.getBoundingClientRect(), [x0, x1, y0, y1] = opt.box;
    return [x0 + (ev.clientX - b.left) / b.width * (x1 - x0), y1 - (ev.clientY - b.top) / b.height * (y1 - y0)];
  };
  const hit = (x, y) => opt.useCubes && !opt.useCubes() ? null : LQ.cubes().filter((c) => Math.hypot(c.x - x, c.y - y) < 0.03).sort((a, b) => b.level - a.level)[0] || null;
  svg.addEventListener("pointerdown", (ev) => {
    const [x, y] = at(ev), c = hit(x, y);
    if (c && opt.edit && opt.edit() && LQ.isTop(c)) { LQ.drag = { id: c.id, x0: x, y0: y, x, y, moved: false }; svg.setPointerCapture(ev.pointerId); ev.preventDefault(); }
  });
  svg.addEventListener("pointermove", (ev) => {
    const g = LQ.drag; if (!g) return;
    const [x, y] = at(ev);
    g.moved = g.moved || Math.hypot(x - g.x0, y - g.y0) > 0.008;
    if (g.moved) { const c = LQ.byId(g.id); g.x = c.x + x - g.x0; g.y = c.y + y - g.y0; opt.redraw(); }
  });
  svg.addEventListener("pointerup", async (ev) => {
    const g = LQ.drag; LQ.drag = null;
    const [x, y] = at(ev);
    if (g && g.moved) { await lvScene({ op: "move", id: g.id, x: Math.round(g.x * 1000) / 1000, y: Math.round(g.y * 1000) / 1000 }); opt.redraw(); return; }
    const c = hit(x, y);
    if (c) { if (c.id != null) LQ.sel = c.id; if (opt.onCube) opt.onCube(c); opt.redraw(); lvPoll(); return; }
    if (opt.onEmpty && opt.onEmpty(x, y)) return;
    if (opt.edit && opt.edit()) {
      const r = await lvScene({ op: "add", x: Math.round(x * 100) / 100, y: Math.round(y * 100) / 100, yaw_deg: 0 });
      if (r.ok) { const ids = r.scene.cubes.map((k) => k.id); LQ.sel = Math.max(...ids); }
      opt.redraw();
    }
  });
  svg.addEventListener("pointercancel", () => { LQ.drag = null; opt.redraw(); });
}
// 가상 장면 고치기 줄 (가상 모드에서만 보임)
function lvTools() {
  const t = {};
  t.yaw = el("input", { inputmode: "decimal", style: "width:70px", onchange: () => { if (LQ.sel != null) lvScene({ op: "yaw", id: LQ.sel, yaw_deg: parseFloat(t.yaw.value) || 0 }); } });
  t.del = el("button", { class: "btn line small", onclick: () => { if (LQ.sel != null) lvScene({ op: "delete", id: LQ.sel }); } }, "지우기");
  t.jit = el("input", { type: "checkbox", onchange: () => lvScene({ op: "jitter", on: t.jit.checked }) });
  t.what = el("b", {});
  t.root = el("div", { class: "lv-tools", hidden: true },
    el("span", { class: "vbadge" }, "가상"), t.what, el("label", {}, "각 ", t.yaw, "°"), t.del,
    el("label", { title: "가상 팔이 큐브를 놓을 때 자리 ±1 cm · 각 ±5° 를 무작위로 더한다" }, t.jit, " 놓을 때 어긋남"));
  t.render = () => {
    const v = LQ.virtual(), c = LQ.sel != null ? LQ.byId(LQ.sel) : null;
    t.root.hidden = !v;
    if (!v) return;
    t.what.textContent = c ? `큐브 ${c.id}` : "빈 곳 = 큐브 놓기 · 끌기 = 옮기기";
    t.yaw.disabled = t.del.disabled = !c;
    if (c && document.activeElement !== t.yaw) t.yaw.value = c.yaw_deg.toFixed(0);
    t.jit.checked = !!(LQ.r.scene && LQ.r.scene.jitter);
  };
  LQ.tools.push(t);
  return t.root;
}
// 3D: 가상 장면의 큐브 상자 (가상 모드). skip = [[x, y], …] 이 자리 (1 cm) 의 책상 위 큐브는 그 뷰가 따로 그리므로 뺀다
function lvBoxes(v, d, skip) {
  if (!v || v.dead || !v.cube) return;
  v._lv = v._lv || [];
  const list = LQ.virtual() ? LQ.cubes() : [];
  const want = list.filter((c) => !(c.level === 0 && (skip || []).some((s) => s && Math.hypot(s[0] - c.x, s[1] - c.y) < 0.01)));
  while (v._lv.length > want.length) { const m = v._lv.pop(); v.scene.remove(m); m.material.dispose(); }
  while (v._lv.length < want.length) {
    const m = v.cube.clone(); m.material = v.cube.material.clone(); m.material.color.setHex(0xa16207); m.visible = true; v.scene.add(m); v._lv.push(m);
  }
  want.forEach((c, i) => { const m = v._lv[i]; m.position.set(c.x, c.y, -d + 0.05 * (c.level + 0.5)); m.rotation.set(0, 0, c.yaw_deg * Math.PI / 180); m.material.color.setHex(c.level ? 0x2563eb : 0xa16207); });
  v.dirty();
}
// 역기구학 탭 3D 에도 가상 큐브들 (app.js 는 안 고침: updateIkView 를 감싼다)
(function () {
  const _u = updateIkView;
  updateIkView = function () {
    _u();
    if (typeof VW !== "undefined" && VW.ik) { const sc = trajScene(); lvBoxes(VW.ik, sc.d, [sc.cube && [sc.cube.x, sc.cube.y]]); }
  };
  LQ.on(() => { if (curTab === "ik") updateIkView(); });
  document.head.append(el("style", {}, `
.lv-tools { display: flex; flex-wrap: wrap; gap: 8px 12px; align-items: center; margin-top: 8px; font-size: 14px; }
.lv-tools label { display: inline-flex; align-items: center; gap: 4px; }
.lv-src { font-size: 13px; color: var(--sub); margin-top: 6px; }`));
})();
// 지금 인식 상태 한 줄 (출처·빠르기·나이)
function lvSrcText(r) {
  if (!r) return "";
  if (r.mode === "virtual") return "가상 모드 — 카메라는 안 쓴다 (가상 큐브만)";
  if (r.stale) return r.reason || "영상 없음";
  const src = r.src === "live.jpg" ? "실시간 영상 (live.jpg)" : r.src === "inference.png" ? "폰 프로그램 그림 (inference.png, 1초쯤 간격)" : r.src;
  return `${src} · ${r.rate_hz != null ? r.rate_hz + " Hz · " : ""}${r.age_s.toFixed(1)}초 전 프레임`;
}
