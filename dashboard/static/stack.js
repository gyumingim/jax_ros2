// 탑 쌓기 탭: 폰 사진 한 장 → 큐브 여러 개(큐브마다 다른 ArUco, stack_api.py) → 한 번에 한 큐브씩 탑 위로 옮긴다.
// 옮기기 = 역기구학 탭과 같은 우리 IK 계획 (강화학습 아님): 계획 = POST /api/svc/pick_dry/start (역기구학 탭 칸 + stack 칸 = 놓는 층·다른 큐브),
// 재생 = 역기구학 탭의 원래 버튼(가상 팔로 재생 / 실물로 재생)을 그대로 누른다 → 확인창·검사·force 확인이 전부 원래 것.
// 한 큐브를 놓을 때마다 사진을 다시 찍어야 다음 큐브로 간다 (실제로 놓인 자리·탑 높이를 사진으로 확인).
// app.js·direct.js·phone.js 는 안 고친다: 탭 버튼도 여기서 단다 (phone.js 와 같은 방식).
"use strict";
(function () {
  const page = document.getElementById("page-stack");
  if (!page) return;
  const LOCK_KEY = "jaxdash.stack";
  const ST = { det: null, grasp: {}, busy: false, going: false, needPhoto: false, played: false, sawRun: false, lock: null, view: null, extra: [], att: null };
  try { ST.lock = JSON.parse(lsGet(LOCK_KEY) || "null"); } catch (e) { ST.lock = null; }

  TABS.push(["stack", "탑 쌓기", "큐브 여러 개"]);
  TAB_IDS.add("stack");
  const tb = el("a", { class: "tab", href: "#stack", role: "tab" }, "탑 쌓기", el("small", { class: "tsub" }, "큐브 여러 개"), el("span", { class: "tdot" }));
  tabBtns.stack = tb;
  $("#tabs").append(tb);
  const _showTab = showTab;
  showTab = function (id) { _showTab(id); if (curTab === "stack") onShow(); };

  // ---------- 화면 ----------
  const pick = (capture) => {
    const inp = el("input", { type: "file", accept: "image/*", hidden: true });
    if (capture) inp.setAttribute("capture", "environment");
    inp.addEventListener("change", () => { if (inp.files[0]) detect(inp.files[0]); inp.value = ""; });
    return inp;
  };
  const cam = pick(true), gal = pick(false);
  const show = (node, text, cls, small) => { node.hidden = false; node.className = "result " + (cls || ""); node.replaceChildren(text, small ? el("small", {}, small) : ""); };
  ST.bShot = el("button", { class: "btn st-big", onclick: () => cam.click() }, "사진 찍기");
  ST.img = el("img", { class: "st-img", hidden: true, alt: "찾은 큐브" });
  ST.res = el("div", { class: "result", hidden: true });
  ST.list = el("div", { class: "st-list", hidden: true });
  ST.order = el("input", { placeholder: "예: 12, 10, 11", inputmode: "numeric", onchange: () => { ST.orderTouched = true; mapDraw(); spot(); plan(); } });
  const xy = () => { mapDraw(); spot(); plan(); };
  ST.sx = el("input", { inputmode: "decimal", value: lsGet("jaxdash.stack.x") || "-0.16", onchange: xy });
  ST.sy = el("input", { inputmode: "decimal", value: lsGet("jaxdash.stack.y") || "-0.28", onchange: xy });
  ST.syaw = el("input", { inputmode: "decimal", value: "0", onchange: xy });
  ST.map = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  ST.map.setAttribute("class", "st-map");
  ST.map.addEventListener("click", mapClick);
  ST.mapNote = el("div", { class: "help", style: "margin:6px 0 0" }, "지도를 눌러 탑 자리를 고른다 (위 = 로봇, 아래 = 앞, 오른쪽 = 로봇 왼쪽 +x).");
  ST.spotNote = el("div", { class: "result", hidden: true });
  ST.lockNote = el("div", { class: "help", style: "margin:8px 0 0" });
  ST.bReset = el("button", { class: "btn line", onclick: reset }, "처음부터");
  ST.bGo = el("button", { class: "btn st-big", disabled: true, onclick: go }, "탑 쌓기");
  ST.bStop = el("button", { class: "btn red", onclick: () => post(VIRT() ? "/api/ik/stop_virtual" : "/api/ik/stop_real") }, "■ 멈춤");
  ST.note = el("div", { class: "result" }, "사진을 찍으면 큐브 목록이 나온다");
  ST.bPlay = el("button", { class: "btn", onclick: () => { if (!T.traj) return; if (T.frame >= T.traj.positions.length - 1) T.frame = 0; T.playing = !T.playing; T.last = performance.now(); } }, "▶ 재생");
  ST.slider = el("input", { type: "range", min: 0, max: 0, step: 1, value: 0 });
  ST.slider.addEventListener("input", () => { T.frame = parseFloat(ST.slider.value); T.playing = false; draw(); });
  ST.num = el("span", { class: "num" });
  document.head.append(el("style", {}, `
#page-stack .st-big { width: 100%; min-height: 60px; font-size: 18px; border-radius: 14px; }
#page-stack .st-row { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 12px; }
#page-stack .st-row > .btn { flex: 1 1 130px; min-height: 48px; }
#page-stack .st-img { display: block; width: 100%; max-height: 46vh; object-fit: contain; border-radius: 12px; background: var(--fill); margin-top: 14px; }
#page-stack .result { margin-top: 12px; }
#page-stack .result small { display: block; font-weight: 500; margin-top: 4px; }
#page-stack .st-list { margin-top: 12px; overflow-x: auto; }
#page-stack .st-list table.jt td, #page-stack .st-list table.jt th { padding: 8px 6px; }
#page-stack .st-list tr.top td { font-weight: 700; }
#page-stack .frow .field { width: 92px; }
#page-stack .frow .field.wide { width: 170px; }
#page-stack .st-map { display: block; width: 100%; max-width: 460px; margin: 12px auto 0; border-radius: 12px; background: var(--fill); touch-action: manipulation; cursor: crosshair; }
#page-stack .frow .field input, #page-stack .frow .field select { font-size: 16px; }
#page-stack ol.help { padding-left: 20px; margin: 10px 0 0; line-height: 1.6; }
@media (max-width: 980px) { #page-stack .split-r { order: 1; } }
@media (max-width: 640px) { #page-stack .frow .field.wide { width: 100%; } }`));
  $("#stack-left").append(
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "1"), "사진 찍기"),
    el("div", { class: "card" }, ST.bShot, cam, gal,
      el("div", { class: "help", style: "margin:10px 0 0" }, "기준 종이와 큐브 윗면 마커가 다 나오게, 비스듬히 위에서 한 장 (바로 위에서 찍으면 층을 못 가린다). ",
        el("a", { href: "#", onclick: (e) => { e.preventDefault(); gal.click(); } }, "찍어 둔 사진 고르기")),
      ST.img, ST.res, ST.list),
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "2"), "탑"),
    el("div", { class: "card" },
      el("div", { class: "frow" }, el("b", {}, "순서"), el("label", { class: "field wide" }, el("span", {}, "옮길 큐브 번호", el("small", {}, " 차례대로")), ST.order)),
      el("div", { class: "frow" }, el("b", {}, "탑 자리"),
        el("label", { class: "field" }, el("span", {}, "x", el("small", {}, " m")), ST.sx), el("label", { class: "field" }, el("span", {}, "y", el("small", {}, " m")), ST.sy),
        el("label", { class: "field" }, el("span", {}, "각", el("small", {}, " °")), ST.syaw)),
      el("div", { class: "help", style: "margin:8px 0 0" }, "역기구학 탭과 같은 좌표: 어깨 축 바로 아래가 (0, 0), +x = 로봇 왼쪽, 앞(책상 쪽) = −y. 순서의 첫 큐브를 이 자리 바닥에 놓고 그 위로 쌓는다."),
      ST.map, ST.mapNote, ST.spotNote, ST.lockNote,
      ST.bGo,
      el("div", { class: "st-row" }, ST.bStop, ST.bReset),
      ST.note,
      el("div", { class: "help", style: "margin:8px 0 0" }, "한 번 누르면 큐브 하나: 계획(역기구학 탭과 같은 IK, 손은 수직) → 지금 모드의 팔로 재생할지 확인창 → 놓은 뒤 사진을 다시 찍으면 다음 큐브.")),
    el("details", { class: "adv" }, el("summary", {}, "준비 — 인쇄물 · 놓는 법"),
      el("div", { class: "card" },
        el("a", { class: "btn line", href: "/api/stack/sheet", target: "_blank" }, "큐브 마커 PDF 열기 (ID 10~15)"),
        el("ol", { class: "help" },
          el("li", {}, "실제 크기 100% 로 인쇄해 큐브마다 다른 번호를 윗면에 붙인다. 기준 종이는 휴대폰 카메라 탭의 것 그대로 (자리 설정도 그 탭)."),
          el("li", {}, "큐브끼리 15 cm 이상 띄운다 — 벌린 손가락이 닫히며 큐브 중심에서 12 cm 쯤까지 나온다."),
          el("li", {}, "탑 자리는 로봇 오른쪽 앞 (x −0.16, y −0.28 근처, 지도의 진한 초록) 이 좋다: 3층째를 놓을 수 있는 곳이 좁고, 팔이 드나드는 자리 (x 0, y −0.34) 에서 15 cm 넘게 떨어져야 손가락이 탑에 안 닿는다."),
          el("li", {}, "이미 놓여 있는 큐브 위에 쌓으려면 탑 자리를 그 큐브 자리로 (지도에서 그 큐브를 누름) — 그 큐브는 순서에서 빼도 된다."),
          el("li", {}, "기본 책상 높이에서는 3층까지 (4층째는 팔이 못 닿는다).")))));
  $("#stack-ctl").append(el("div", { class: "tbar" }, ST.bPlay, ST.slider), el("div", { class: "tbar" }, el("span", { class: "spacer" }), ST.num));

  // ---------- 사진 → 큐브 목록 ----------
  async function shrink(file) {   // phone.js 와 같게: 긴 변 2000 px JPEG
    try {
      const bmp = await createImageBitmap(file);
      const k = Math.min(1, 2000 / Math.max(bmp.width, bmp.height));
      const cv = el("canvas", { width: Math.round(bmp.width * k), height: Math.round(bmp.height * k) });
      cv.getContext("2d").drawImage(bmp, 0, 0, cv.width, cv.height);
      return await new Promise((ok) => cv.toBlob((b) => ok(b || file), "image/jpeg", 0.9));
    } catch (e) { return file; }
  }
  async function detect(file) {
    if (ST.busy) return;
    ST.busy = true;
    show(ST.res, "큐브 찾는 중…", "run");
    let r;
    try {
      r = await (await fetch("/api/stack/detect", { method: "POST", headers: { "content-type": "application/octet-stream" }, body: await shrink(file) })).json();
    } catch (e) { r = { ok: false, reason: "서버 연결 실패" }; }
    ST.busy = false;
    setDet(r, true);
  }
  function setDet(r, fresh) {
    ST.det = r; ST.grasp = {}; ST.step = null;
    if (fresh) { ST.needPhoto = false; ST.played = false; ST.sawRun = false; }
    if (r.image) { ST.img.src = r.image; ST.img.hidden = false; }
    const warn = (r.warn || []).join(" · ");
    if (!r.ok) { show(ST.res, r.reason || "못 찾음", "bad", warn); ST.list.hidden = true; baseUI(); plan(); return; }
    const when = r.t ? new Date(r.t * 1000).toLocaleTimeString() : "";
    show(ST.res, `큐브 ${r.cubes.length}개 찾음`, warn ? "warn" : "ok", `기준 마커 ${r.refs.length}개 · ${when}` + (warn ? " · " + warn : ""));
    baseUI();
    spot();
    for (const c of r.cubes) if (c.level === 0) graspable(c);
    table();
    plan();
  }
  async function graspable(c) {
    const det = ST.det;
    let g;
    try {
      g = await (await fetch("/api/ik/graspable", { method: "POST", headers: { "content-type": "application/json" },
        body: JSON.stringify({ x: c.x, y: c.y, d: tableD(), grasp: "vertical" }) })).json();
    } catch (e) { g = { ok: false }; }
    if (ST.det !== det) return;
    ST.grasp[c.id] = g;
    table(); if (!ST.needPhoto && ST.step) note();
  }
  function table() {
    const r = ST.det;
    if (!r || !r.ok) { ST.list.hidden = true; return; }
    const top = ST.step && ST.step.top, nxt = ST.step && ST.step.cube;
    const rows = r.cubes.slice().sort((a, b) => a.id - b.id).map((c) => {
      const g = ST.grasp[c.id];
      const what = top && top.id === c.id ? "탑 맨 위" : nxt && nxt.id === c.id ? "다음" : "";
      const gt = c.level > 0 ? "—" : !g ? "…" : !g.ok ? "?" : g.graspable ? "✓" : "✗";
      return el("tr", { class: what ? "top" : "", title: g && g.ok && !g.graspable ? g.reason || "" : "" },
        el("td", {}, String(c.id), what ? el("small", {}, " " + what) : ""), el("td", { class: "n" }, c.x.toFixed(3)), el("td", { class: "n" }, c.y.toFixed(3)),
        el("td", { class: "n" }, c.yaw_deg.toFixed(0) + "°"), el("td", { class: "n" }, `${c.level + 1}층`),
        el("td", { class: c.level > 0 || !g || !g.ok ? "" : g.graspable ? "ok" : "no" }, gt));
    });
    ST.list.hidden = false;
    ST.list.replaceChildren(el("table", { class: "jt" },
      el("thead", {}, el("tr", {}, ...["큐브", "x (m)", "y (m)", "각", "높이", "집기"].map((h, i) => el("th", { class: i > 0 && i < 5 ? "n" : "", style: i > 0 && i < 5 ? "text-align:right" : i === 5 ? "text-align:center" : "" }, h)))), el("tbody", {}, ...rows)));
  }

  // ---------- 탑 자리·순서 ----------
  const ids = () => (ST.det && ST.det.ok ? ST.det.cubes.map((c) => c.id).sort((a, b) => a - b) : []);
  function baseUI() {
    const lk = ST.lock, all = ids();
    if (!lk) {
      if (!ST.orderTouched && all.length && ST.orderFor !== all.join(",")) { ST.order.value = all.slice(0, 3).join(", "); ST.orderFor = all.join(","); }   // 기본 = 번호 오름차순, 3층까지라 앞 3개
    } else {
      ST.order.value = lk.order.join(", ");
      ST.sx.value = lk.base.x.toFixed(3); ST.sy.value = lk.base.y.toFixed(3); ST.syaw.value = String(lk.base.yaw_deg);
    }
    ST.order.disabled = ST.sx.disabled = ST.sy.disabled = ST.syaw.disabled = !!lk;
    ST.lockNote.textContent = lk ? "탑 자리·순서 고정 (쌓는 중) — 바꾸려면 '처음부터'" : "";
    ST.bReset.disabled = !lk;
    mapDraw();
  }
  function chosen() {   // 지금 고른 탑 자리·순서 (고정됐으면 그것). 틀리면 { error }
    if (ST.lock) return ST.lock;
    const seq = ST.order.value.split(/[\s,]+/).filter(Boolean).map(Number);
    if (!seq.length || seq.some((i) => !Number.isInteger(i)) || new Set(seq).size !== seq.length) return { error: "순서는 큐브 번호를 쉼표로 (예: 12, 10, 11), 같은 번호는 한 번만" };
    const x = parseFloat(ST.sx.value), y = parseFloat(ST.sy.value), yaw = parseFloat(ST.syaw.value) || 0;
    if (!Number.isFinite(x) || !Number.isFinite(y)) return { error: "탑 자리 x, y 숫자 (m)" };
    return { base: { x, y, yaw_deg: yaw }, order: seq };
  }
  function listed() {   // 쌓기 전 확인: 순서의 큐브가 다 사진에 있고 집을 수 있나. 문제 글 또는 ""
    if (ST.lock || !ST.det || !ST.det.ok) return "";
    const ch = chosen();
    if (ch.error) return ch.error;
    if (ch.order.length > 3) return `순서는 3개까지 — 탑은 3층까지만 (지금 ${ch.order.length}개)`;
    for (const i of ch.order) {
      const c = ST.det.cubes.find((k) => k.id === i), g = ST.grasp[i];
      if (!c) return `순서의 큐브 ${i} 가 사진에 없음 (찾은 큐브: ${ids().join(", ") || "없음"})`;
      if (c.level === 0 && Math.hypot(c.x - ch.base.x, c.y - ch.base.y) > 0.025 && g && g.ok && !g.graspable) return `큐브 ${i} 는 못 집는 자리: ${g.reason || ""}`;
    }
    return "";
  }

  // ---------- 탑 자리: 지도 (놓을 수 있는 곳) + 그 자리 확인 ----------
  const MX0 = -0.27, MX1 = 0.23, MY0 = -0.45, MY1 = -0.13, K = 1000;   // 실좌표 m → 그림: x 오른쪽 = +x, 아래 = −y (앞)
  const sv = (tag, at, ...kids) => { const e = document.createElementNS("http://www.w3.org/2000/svg", tag); for (const [k, v] of Object.entries(at)) e.setAttribute(k, v); e.append(...kids); return e; };
  const px = (x) => (x - MX0) * K, py = (y) => (MY1 - y) * K;
  ST.map.setAttribute("viewBox", `0 0 ${(MX1 - MX0) * K} ${(MY1 - MY0) * K}`);
  function mapLoad() {
    const d = tableD();
    if (ST.mapD === d || ST.mapBusy) return;
    ST.mapBusy = true; ST.mapNote.textContent = "놓을 수 있는 곳 계산 중… (처음 한 번 10~40초)";
    fetch("/api/stack/map?d=" + encodeURIComponent(d)).then((r) => r.json()).then((r) => {
      ST.mapBusy = false; ST.mapD = d; ST.mapData = r.ok ? r : null;
      ST.mapNote.textContent = r.ok ? "지도를 눌러 탑 자리를 고른다. 초록 = 놓을 수 있는 곳 (진할수록 높이: 1·2·3층), 위 = 로봇 · 아래 = 앞 · 오른쪽 = 로봇 왼쪽(+x)." : "지도를 못 만듦: " + (r.error || "");
      mapDraw();
    }).catch(() => { ST.mapBusy = false; ST.mapD = d; });
  }
  function mapDraw() {
    const g = [], m = ST.mapData, need = (chosen().order || []).length || 1;
    if (m) m.rows.forEach((row, j) => { for (let i = 0; i < row.length; i++) { const n = +row[i]; if (n) g.push(sv("rect", { x: px(m.xs[i] - m.step / 2), y: py(m.ys[j] + m.step / 2), width: m.step * K, height: m.step * K,
      fill: n >= Math.min(need, 3) ? "#16a34a" : "#86efac", "fill-opacity": 0.25 + 0.2 * Math.min(n, 3) })); } });
    g.push(sv("circle", { cx: px(0), cy: py(-0.145), r: 7, fill: "#64748b" }), sv("text", { x: px(0) + 12, y: py(-0.145) + 5, "font-size": 15, fill: "#64748b" }, "로봇 쪽 (어깨 = x 0, y 0)"));
    for (const c of (ST.det && ST.det.ok ? ST.det.cubes : [])) {
      const t = `rotate(${-c.yaw_deg} ${px(c.x)} ${py(c.y)})`;
      g.push(sv("rect", { x: px(c.x) - 25, y: py(c.y) - 25, width: 50, height: 50, fill: c.level ? "#2563eb" : "#f97316", transform: t, rx: 3 }),
        sv("text", { x: px(c.x), y: py(c.y) + 6, "font-size": 17, "font-weight": 700, fill: "#fff", "text-anchor": "middle" }, String(c.id)));
    }
    const x = parseFloat(ST.sx.value), y = parseFloat(ST.sy.value);
    if (Number.isFinite(x) && Number.isFinite(y)) g.push(sv("rect", { x: px(x) - 29, y: py(y) - 29, width: 58, height: 58, fill: "none", stroke: "#1d4ed8", "stroke-width": 4, "stroke-dasharray": "9 5", rx: 4,
      transform: `rotate(${-(parseFloat(ST.syaw.value) || 0)} ${px(x)} ${py(y)})` }), sv("text", { x: px(x), y: py(y) - 36, "font-size": 15, "font-weight": 700, fill: "#1d4ed8", "text-anchor": "middle" }, "탑"));
    ST.map.replaceChildren(...g);
  }
  function mapClick(ev) {
    if (ST.lock) { toast("쌓는 중에는 탑 자리가 고정 — 바꾸려면 '처음부터'"); return; }
    const b = ST.map.getBoundingClientRect();
    let x = MX0 + (ev.clientX - b.left) / b.width * (MX1 - MX0), y = MY1 - (ev.clientY - b.top) / b.height * (MY1 - MY0);
    const near = (ST.det && ST.det.ok ? ST.det.cubes : []).find((c) => Math.hypot(c.x - x, c.y - y) < 0.03);   // 큐브를 누르면 그 큐브 자리 (그 위에 쌓기)
    if (near) { x = near.x; y = near.y; ST.syaw.value = near.yaw_deg.toFixed(0); } else { x = Math.round(x * 100) / 100; y = Math.round(y * 100) / 100; }
    ST.sx.value = x.toFixed(3); ST.sy.value = y.toFixed(3);
    xy();
  }
  let spotSeq = 0;
  async function spot() {   // 그 자리에 순서의 큐브 수만큼 층을 놓을 수 있나 (계획기와 같은 판정)
    const seq = ++spotSeq, ch = chosen();
    ST.spotOk = null;
    if (ch.error) { ST.spotNote.hidden = true; return; }
    lsSet("jaxdash.stack.x", ST.sx.value); lsSet("jaxdash.stack.y", ST.sy.value);
    let r;
    try {
      r = await (await fetch("/api/stack/spot", { method: "POST", headers: { "content-type": "application/json" },
        body: JSON.stringify({ x: ch.base.x, y: ch.base.y, yaw_deg: ch.base.yaw_deg, levels: Math.min(8, Math.max(1, ch.order.length)), d: tableD() }) })).json();
    } catch (e) { r = { ok: false, text: "탑 자리 확인 못 함 (서버 연결)" }; }
    if (seq !== spotSeq) return;
    ST.spotOk = !!r.ok;
    show(ST.spotNote, `탑 자리 (${ch.base.x.toFixed(3)}, ${ch.base.y.toFixed(3)}): ` + (r.text || ""), r.ok ? "ok" : "bad");
  }
  function reset() {
    if (!confirm("탑 자리·순서를 지우고 처음부터 할까요? (팔은 안 움직인다)")) return;
    ST.lock = null; lsSet(LOCK_KEY, "null"); ST.needPhoto = false; ST.played = false; ST.sawRun = false; ST.orderTouched = false; ST.orderFor = null;
    baseUI(); spot(); plan();
  }

  // ---------- 다음 한 걸음 (서버가 사진으로 판단) ----------
  let planSeq = 0;
  async function plan() {
    const seq = ++planSeq;
    ST.step = null;
    baseUI();
    if (!ST.det || !ST.det.ok) { table(); return; }
    const ch = chosen();
    if (ch.error) { show(ST.note, ch.error, "warn"); table(); return; }
    let r;
    try {
      r = await (await fetch("/api/stack/next", { method: "POST", headers: { "content-type": "application/json" },
        body: JSON.stringify({ cubes: ST.det.cubes, base: ch.base, order: ch.order }) })).json();
    } catch (e) { r = { ok: false, text: "서버 연결 실패" }; }
    if (seq !== planSeq) return;
    ST.step = r;
    if (!ST.needPhoto) note();
    table(); scene(); draw();
  }
  function note() {
    const r = ST.step, bad = listed(), warn = (r.warn || []).join(" · ");
    if (bad) show(ST.note, bad, "bad");
    else show(ST.note, r.text || "", !r.ok ? "bad" : r.done ? "ok" : warn ? "warn" : "", (r.ok && !r.done ? `탑 지금 ${r.height}층 · ` : "") + warn);
  }

  // ---------- 탑 쌓기 = 한 큐브: 역기구학 탭의 칸을 채우고 ▶ 계획 → (통과하면) 그 탭의 재생 버튼 ----------
  const sleep = (ms) => new Promise((ok) => setTimeout(ok, ms));
  async function wait(cond, ms) {
    for (const t0 = Date.now(); Date.now() - t0 < ms; await sleep(200)) if (cond()) return true;
    return false;
  }
  async function go() {
    const c = cards.pick_dry, st = ST.step;
    if (!st || !st.ok || st.done || ST.going || ST.needPhoto) return;
    if (!c) { toast("역기구학 계획 항목이 없다", "error"); return; }
    if (listed() || (!ST.lock && ST.spotOk === false)) return;
    ST.going = true;
    try {
      if (!ST.lock) {   // 첫 큐브: 탑 자리·순서를 고정 (바닥 큐브는 곧 가려져 사진에 안 보인다)
        const ch = chosen();
        if (ch.error) { show(ST.note, ch.error, "warn"); return; }
        ST.lock = { base: ch.base, order: ch.order }; lsSet(LOCK_KEY, JSON.stringify(ST.lock)); baseUI();
      }
      if (!c.fields.stack) c.fields.stack = el("input", { type: "hidden" });   // 역기구학 탭·휴대폰 탭의 계획은 이 칸을 비워 보낸다
      for (const [k, v] of Object.entries(st.params)) c.fields[k].value = v;
      mapUpdate();
      const params = Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.type === "checkbox" ? (i.checked ? "1" : "") : i.value]));
      c.fields.stack.value = "";
      const m0 = S.traj_mtime, st0 = svcById("pick_dry").started;
      const head = `큐브 ${st.cube.id} → ${st.level + 1}층`;
      show(ST.note, head + ": 계획 중…", "run");
      const r = await post("/api/svc/pick_dry/start", { params });
      if (!r.ok) { show(ST.note, head + ": 계획을 못 시작함", "bad", r.error || ""); return; }
      const done = await wait(() => { const v = svcById("pick_dry"); return v.started !== st0 && !v.alive && v.exit_code != null; }, 180000);
      const sv = svcById("pick_dry");
      if (!done) { show(ST.note, head + ": 계획이 3분 넘게 안 끝남", "bad", "역기구학 탭에서 확인"); return; }
      const [txt, cls] = pickSummary(sv);
      const got = sv.exit_code !== 3 && await wait(() => S.traj_mtime != null && S.traj_mtime !== m0 && T.mtime === S.traj_mtime && T.info && T.info.mtime === S.traj_mtime, 4000);
      if (!got || !T.traj) { show(ST.note, head + ": 계획 실패 — 팔은 안 움직임", "bad", txt || `종료코드 ${sv.exit_code}`); return; }
      // 이 계획이 정말 이 걸음(놓는 층·다른 큐브)으로 돌았나 — 아니면 탑 속으로 놓으러 가는 계획일 수 있다
      const real = T.traj.real || {};
      if ((real.place_level || 0) !== st.level || (real.obstacles || []).length !== st.obstacles.length) {
        show(ST.note, head + ": 계획이 탑 쌓기 값 없이 돌았다 — 재생 안 함", "bad", "다시 '탑 쌓기' (역기구학 탭에서 그 사이 계획을 돌렸는지 확인)"); return;
      }
      scene(); T.frame = 0; draw();
      if (T.traj.status !== "playable") { show(ST.note, head + ": 계획이 검사에 걸림 — 재생 안 함", "bad", txt || T.traj.reason || ""); return; }
      const pass = head + ": " + (txt || "계획 나옴");
      await sleep(600);
      const kind = S.mode && S.mode.kind;
      const btn = kind === "virtual" ? T.vplay : kind === "direct" ? (typeof D !== "undefined" && D.play && D.play.go) : null;
      if (!btn) { show(ST.note, pass, cls, "이 모드의 재생은 역기구학 탭에서"); return; }
      if (btn.disabled) { show(ST.note, pass, "warn", "재생은 아직 못 함: " + (btn.title || (kind === "direct" ? "연결 탭에서 팔 연결·원점 먼저" : "이미 재생 중"))); return; }
      show(ST.note, pass, cls, "확인창에서 재생을 고르면 팔이 움직인다");
      ST.played = true; ST.sawRun = false; ST.head = head;
      btn.click();   // 원래 버튼 = 원래 확인창 (가상: 1번, 실물: 2번 + 검사 문제면 한 번 더)
      setTimeout(() => {   // 확인창을 취소했거나 원래 검사가 거절하면 (팔 꺼짐·원점 없음 …) 재생이 안 시작된다 — 이유는 화면 위 알림
        const k = S.mode && S.mode.kind, p = (k === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
        if (ST.played && !ST.sawRun && !p.running) { ST.played = false; show(ST.note, pass, "warn", "재생이 시작되지 않았다 — 확인창을 취소했거나 거절됨 (화면 아래 알림). 고친 뒤 다시 '탑 쌓기'"); }
      }, 4000);
    } finally { ST.going = false; }
  }

  // ---------- 3D: 사진의 큐브들 + 지금 계획 다시 보기 ----------
  function onShow() {
    if (!ST.view && window.JaxViewer && S && S.robot) {
      ST.view = window.JaxViewer.create($("#v3d-stack"), {});
      ST.view.load(S.robot.mesh_ok).then(() => { scene(); draw(); });
    }
    if (ST.view && ST.view.resize) ST.view.resize();
    if (!ST.det && !ST.asked) {   // 다른 기기(폰)에서 찍은 마지막 사진 결과
      ST.asked = true;
      fetch("/api/stack/last").then((r) => r.json()).then((r) => { if (!ST.det && !r.none) setDet(r, false); }).catch(() => {});
    }
    scene(); draw(); mapLoad(); if (ST.spotOk == null) spot();
  }
  window.addEventListener("jaxviewer-ready", () => { if (curTab === "stack") onShow(); });
  const CUBE = 0.05, COL = { tower: 0x2563eb, loose: 0xf97316, next: 0xf97316, other: 0x94a3b8 };
  function scene() {   // 큐브 상자들: 사진에서 본 것 + 탑 속에 가려진 것. 다음에 옮길 큐브는 v.cube (주황), 놓을 자리는 v.target (초록)
    const v = ST.view;
    if (!v || v.dead || !v.cube) return;
    const d = tableDNum();
    v.setTableD(d);
    for (const m of ST.extra) { v.scene.remove(m); m.material.dispose(); }
    ST.extra = []; ST.att = null;
    const st = ST.step, det = ST.det;
    const box = (x, y, yaw, level, color, opacity) => {
      const m = v.cube.clone();
      m.material = v.cube.material.clone(); m.material.color.setHex(color);
      if (opacity < 1) { m.material.transparent = true; m.material.opacity = opacity; }
      m.position.set(x, y, -d + CUBE * (level + 0.5)); m.rotation.set(0, 0, yaw * Math.PI / 180); m.visible = true;
      v.scene.add(m); ST.extra.push(m);
    };
    const nid = st && st.cube ? st.cube.id : null;
    if (st && st.ok && !st.done) for (const o of st.obstacles) box(o[0], o[1], o[2], o[3], st.target && Math.hypot(o[0] - st.target.x, o[1] - st.target.y) < 1e-3 ? COL.tower : COL.other, 1);
    else if (det && det.ok) for (const c of det.cubes) for (let k = 0; k <= c.level; k++) box(c.x, c.y, c.yaw_deg, k, k === c.level ? COL.tower : COL.other, 1);
    v.cube.visible = nid != null; v.target.visible = nid != null;
    v.dirty();
  }
  function trajIsStep() {   // 지금 궤적이 화면의 걸음(다음 큐브)의 계획인가
    const st = ST.step, r = T.traj && T.traj.real;
    return !!(st && st.ok && !st.done && r && Math.abs(r.cube[0] - st.cube.x) < 1e-3 && Math.abs(r.cube[1] - st.cube.y) < 1e-3 && (r.place_level || 0) === st.level);
  }
  function draw() {
    const v = ST.view, st = ST.step;
    if (curTab !== "stack") return;
    const t = trajIsStep() ? T.traj : null;
    ST.bPlay.disabled = !t; ST.bPlay.textContent = T.playing && t ? "⏸ 일시정지" : "▶ 재생";
    ST.slider.max = t ? t.positions.length - 1 : 0; ST.slider.disabled = !t;
    if (t) { ST.slider.value = Math.round(T.frame); rangeFill(ST.slider); }
    ST.num.textContent = t ? `${(T.frame * t.dt).toFixed(1)} / ${((t.positions.length - 1) * t.dt).toFixed(1)}초` : "계획을 돌리면 여기서 다시 본다";
    if (!v || v.dead || !v.ready) return;
    const kind = S.mode && S.mode.kind;
    const pb = (kind === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
    const k = t ? Math.max(0, Math.min(t.positions.length - 1, Math.round(T.frame))) : 0;
    v.setActual(pb.running || !t ? actualRad() : trajPose(k));
    if (!st || !st.ok || st.done || !st.cube) { v.dirty(); return; }
    const d = tableDNum(), c = st.cube, tg = st.target;
    v.target.position.set(tg.x, tg.y, -d + CUBE * (st.level + 0.5)); v.target.rotation.set(0, 0, tg.yaw_deg * Math.PI / 180);
    const home = () => { v.cube.position.set(c.x, c.y, -d + CUBE / 2); v.cube.quaternion.set(0, 0, Math.sin(c.yaw_deg * Math.PI / 360), Math.cos(c.yaw_deg * Math.PI / 360)); v.cube.scale.set(1, 1, 1); };
    home();
    if (t && !pb.running) {   // 손가락을 닫은 프레임에 큐브를 손에 붙여 따라가게 (그림만 — 물리 아님), 놓은 뒤엔 놓을 자리에
      const f = (lab) => { const w = (t.waypoints || []).find((x) => x.label === lab); return w ? w.frame : null; };
      const f0 = f("손가락 닫기"), f1 = f("내려감: 놓는 자세");
      const hand = v.actual && v.actual.joints.get("hand_roll");
      if (f0 != null && f1 != null && hand && hand.obj) {
        if (!ST.att || ST.att.m !== T.mtime) {
          v.actual.setPose(trajPose(f0)); v.actual.group.updateMatrixWorld(true);
          v.cube.updateMatrix();
          ST.att = { m: T.mtime, rel: hand.obj.matrixWorld.clone().invert().multiply(v.cube.matrix) };
          v.actual.setPose(trajPose(k));
        }
        if (k > f1) { v.cube.position.set(tg.x, tg.y, -d + CUBE * (st.level + 0.5)); v.cube.rotation.set(0, 0, tg.yaw_deg * Math.PI / 180); }
        else if (k >= f0) { v.actual.group.updateMatrixWorld(true); hand.obj.matrixWorld.clone().multiply(ST.att.rel).decompose(v.cube.position, v.cube.quaternion, v.cube.scale); }
      }
    }
    v.dirty();
  }

  // ---------- 0.3초마다: 버튼·재생 상태 ----------
  function tick() {
    if (!S || curTab !== "stack") return;
    const kind = S.mode && S.mode.kind;
    const pb = (kind === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
    if (ST.played && pb.running) ST.sawRun = true;
    if (ST.played && ST.sawRun && !pb.running) {   // 우리가 누른 재생이 끝남 (다 갔든 멈췄든) → 사진으로 확인해야 다음
      ST.played = false; ST.sawRun = false; ST.needPhoto = true;
      show(ST.note, `${ST.head}: 재생 끝 — 사진을 다시 찍기`, "warn", (pb.reason ? `마지막: ${pb.reason} · ` : "") + "놓인 자리와 탑 높이를 사진으로 확인한 뒤 다음 큐브로 간다");
    }
    const st = ST.step;
    ST.bShot.disabled = ST.busy || !!pb.running;
    ST.bShot.textContent = ST.busy ? "찾는 중…" : ST.det ? "사진 다시 찍기" : "사진 찍기";
    ST.bGo.disabled = !st || !st.ok || st.done || ST.going || !!pb.running || ST.needPhoto || !!listed() || (!ST.lock && ST.spotOk === false);
    if (ST.mapD !== tableD()) mapLoad();
    ST.bGo.textContent = pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1}` : ST.going ? "계획 중…" : ST.needPhoto ? "사진을 다시 찍기" : st && st.ok && st.done ? "탑 완성"
      : st && st.ok ? `탑 쌓기: 큐브 ${st.cube.id} → ${st.level + 1}층 (${kind === "virtual" ? "가상 팔" : "실물 팔"})` : "탑 쌓기";
    ST.bStop.disabled = !pb.running;
    draw();
  }
  setInterval(tick, 300);
  setInterval(() => { if (curTab === "stack" && T.playing) draw(); }, 60);
  baseUI();
  if (location.hash === "#stack" && curTab && curTab !== "stack") showTab("stack");
  else if (curTab === "stack") onShow();
})();
