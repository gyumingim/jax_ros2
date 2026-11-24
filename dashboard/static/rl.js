// '강화학습' 탭: 팀원이 학습한 집기 정책(team-jax/IK sim2sim, MuJoCo)을 한 곳에서 돌려 보고 3D 로 다시 본다.
// 버튼 둘: [강화학습으로 들기] = 그 정책(끝 = 큐브를 쥔 ㄴ자 자세), [역기구학으로 내리기] = 거기서 이어 우리 IK 계획(pick_plan.plan_place_from)으로
// 놓을 자리에 내려놓고 HOME. 3D 는 들기 + 내리기를 이어서 재생한다.
// 서버: rl_api.py (/api/rl/*) → rl_runner.py. 계산은 시뮬만 — 팔에는 맨 아래 '팔로 재생' 블록의 버튼에서만 보낸다.
// 큐브 자리는 둘 중 하나 (작은 스위치): '영상 인식' = 실시간 인식의 큐브 (live.js, 여럿이면 지도에서 골라 누름; 가상 모드면 가상 큐브) 를
// 정우 님 틀로 옮겨 칸을 채운다 / '지도로 고르기' = 예전처럼 지도·숫자 칸.
// app.js 다음에 읽혀 탭 하나를 덧붙인다(app.js 는 그대로). 이 파일이 실패해도 다른 탭은 그대로 돈다.
"use strict";
(function () {
  const MX0_ = -0.38, MX1_ = 0.30, MY0_ = -0.50, MY1_ = 0.03;   // 지도 범위 (실좌표 m)
  const R = { info: null, res: null, frame: 0, playing: false, last: 0, view: null, busy: false, timer: null };
  const PH = { RAISE: "들어올리기", SETTLE: "대기", INSERT: "밀어 넣기", GRASP_LIFT: "쥐고 들기(정책)", RETURN: "ㄴ자 복귀", DONE: "완료",
    CARRY: "놓을 자리 위로(IK)", LOWER: "내려놓기(IK)", OPEN: "손 펴기", RETREAT: "손 빼기", HOME: "HOME 으로" };
  const WHY = {
    "not all three digits touched the cube": "손가락 셋이 다 닿지 않음",
    "cube never 4 cm up": "큐브가 4 cm 못 올라감",
    "cube not 4 cm up for 0.5 s (no L return)": "4 cm 위에서 0.5초 못 버팀",
    "cube dropped after rising": "들었다가 떨어뜨림",
    "done check never held 0.5 s (grip/still)": "완료 판정(쥠·정지 0.5초)을 못 채움",
  };
  const num = (i) => parseFloat(i.value);

  // ---------- 탭 달기 ----------
  TABS.push(["rl", "강화학습", "정책 돌려보기"]);
  TAB_IDS.add("rl");
  const tb = el("a", { class: "tab", href: "#rl", role: "tab" }, "강화학습", el("small", { class: "tsub" }, "정책 돌려보기"), el("span", { class: "tdot" }));
  tabBtns.rl = tb;
  $("#tabs").append(tb);
  const _showTab = showTab;
  showTab = function (id) { _showTab(id); if (curTab === "rl") onShow(); };

  // ---------- 왼쪽 카드 ----------
  R.preset = el("select", {}, el("option", { value: "" }, "자리 고르기…"));
  R.x = el("input", { type: "number", step: "1", value: "0" });
  R.y = el("input", { type: "number", step: "1", value: "0" });
  R.yaw = el("input", { type: "number", step: "5", value: "0" });
  R.reach = el("div", { class: "note" });
  R.run = el("button", { class: "btn", onclick: run }, "강화학습으로 들기");
  R.tx = el("input", { type: "number", step: "0.01", value: "0" });
  R.ty = el("input", { type: "number", step: "0.01", value: "-0.33" });
  R.tyaw = el("input", { type: "number", step: "5", value: "0" });
  R.place = el("button", { class: "btn", onclick: place, disabled: true, title: "들기가 성공한 뒤에 누른다" }, "역기구학으로 내리기");
  R.out = el("div", { class: "result" }, "아직 안 돌림");
  R.evalBtn = el("button", { class: "btn line small", onclick: evalAll }, "평가 (27곳)");
  R.evalOut = el("div", { class: "note" });
  // 지도: 눌러서 자리 고르기. '큐브 자리' / '놓을 자리' 중 고른 쪽을 누른 곳으로 옮긴다 (숫자 칸과 같이 움직임)
  R.mode = "cube";
  R.src = "map";   // "live" = 영상 인식, "map" = 지도로 고르기
  try { if (localStorage.getItem("jaxdash.rl.src") === "live") R.src = "live"; } catch (e) { /* 없어도 됨 */ }
  R.bLive = el("button", { onclick: () => setSrc("live") }, "영상 인식");
  R.bMap = el("button", { onclick: () => setSrc("map") }, "지도로 고르기");
  R.liveNote = el("div", { class: "note" });
  R.bCube = el("button", { onclick: () => setMode("cube") }, el("i", { class: "sw cube" }), "큐브 자리");
  R.bPlace = el("button", { onclick: () => setMode("place") }, el("i", { class: "sw tgt" }), "놓을 자리");
  R.map = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  R.map.setAttribute("class", "rl-map");
  lvMapAttach(R.map, { box: [MX0_, MX1_, MY0_, MY1_], redraw: () => mapDraw(), useCubes: () => R.src === "live" && R.mode === "cube",
    edit: () => R.src === "live" && R.mode === "cube" && LQ.virtual(), onCube: () => liveSpot(), onEmpty: (x, y) => {
      if (R.src === "live" && R.mode === "cube") return false;   // 영상 인식: 빈 곳은 (가상이면) 큐브 놓기
      mapClick(x, y); return true; } });
  R.mapNote = el("div", { class: "help", style: "margin:6px 0 0" }, "지도를 눌러 자리를 고른다 (위 = 로봇, 아래 = 앞, 오른쪽 = 로봇 왼쪽 +x).");
  document.head.append(el("style", {}, `
#page-rl .rl-map { display: block; width: 100%; max-width: 460px; margin: 10px auto 0; border-radius: 12px; background: var(--fill); touch-action: manipulation; cursor: crosshair; }
#page-rl .rl-seg { margin-top: 4px; }
#page-rl .rl-seg i.sw.tgt { border-color: #1d4ed8; }`));
  R.card = el("div", { class: "card" },
    el("h3", {}, "정책 돌려보기"),
    el("p", { class: "help" }, "팀원이 학습한 집기 정책 ", (R.policy = el("b", {}, "grasp_lift_25550")), " 을 MuJoCo 시뮬에서 돌린다. 팔은 아래 재생 버튼을 눌러야만 움직인다."),
    el("div", { class: "modesw seg rl-seg", role: "group", "aria-label": "큐브 자리를 어디서" }, R.bLive, R.bMap),
    el("div", { class: "modesw seg rl-seg", role: "group", "aria-label": "지도를 누르면 옮길 것" }, R.bCube, R.bPlace),
    R.map, lvTools(), R.mapNote, R.liveNote,
    el("div", { class: "frow" },
      el("label", { class: "field", style: "width:auto" }, "미리 정한 자리", R.preset),
      el("label", { class: "field", title: "기준 자리에서 앞뒤. − = 책상 안쪽(로봇에서 멀리)" }, "x (cm)", R.x),
      el("label", { class: "field", title: "기준 자리에서 좌우. + = 로봇 오른쪽" }, "y (cm)", R.y),
      el("label", { class: "field", title: "큐브가 세로축으로 돈 각도 −45~45°" }, "각도 (°)", R.yaw)),
    R.reach,
    el("div", { class: "frow" },
      el("label", { class: "field", title: "놓을 자리, 역기구학 탭과 같은 좌표: 어깨에서 로봇 왼쪽 +" }, "놓을 자리 x (m)", R.tx),
      el("label", { class: "field", title: "어깨에서 뒤 + (책상은 앞 = −)" }, "y (m)", R.ty),
      el("label", { class: "field", title: "놓인 큐브가 세로축으로 돈 각도" }, "각도 (°)", R.tyaw)),
    el("div", { class: "row" }, R.run, R.place),
    R.out,
    el("details", { class: "log" }, el("summary", {}, "더보기"),
      el("p", { class: "help" }, "x·y 는 정우 님 기준 자리(책상 앞 모서리에서 6 cm 안, 어깨에서 10 cm 바깥)에서 벗어난 양. 27곳 평가는 1분쯤 걸린다."),
      el("div", { class: "row" }, R.evalBtn), R.evalOut));
  $("#rl-left").append(R.card);
  for (const i of [R.x, R.y, R.yaw]) i.addEventListener("input", () => { R.preset.value = ""; reach(); });
  for (const i of [R.tx, R.ty, R.tyaw]) i.addEventListener("input", mapDraw);
  R.preset.addEventListener("change", () => {
    const s = R.info && R.info.spots[parseInt(R.preset.value, 10)];
    if (s) { R.x.value = s[0]; R.y.value = s[1]; R.yaw.value = s[2]; setMode("cube"); }
  });

  // ---------- 재생 막대 ----------
  R.play = el("button", { class: "btn", onclick: () => { if (!R.res) return; if (R.frame >= R.res.t.length - 1) R.frame = 0; R.playing = !R.playing; R.last = performance.now(); } }, "▶ 재생");
  R.slider = el("input", { type: "range", min: 0, max: 0, step: 1, value: 0 });
  R.slider.addEventListener("input", () => { R.frame = parseFloat(R.slider.value); R.playing = false; show(); });
  R.num = el("span", { class: "num" }, "실행하면 여기서 다시 볼 수 있다");
  $("#rl-ctl").append(el("div", { class: "tbar" }, R.play, R.slider), el("div", { class: "tbar" }, R.num));

  // 놓는 범위(reach_mask_v65): (x, y)를 품는 칸 중 네 꼭짓점이 모두 유효한 칸이 있으면 안 (sim2sim chain.Tables.valid 와 같은 규칙)
  function inside(x, y) {
    const I = R.info, gx = I.gx_cm, gy = I.gy_cm, m = I.valid;
    if (x < gx[0] - 1e-7 || x > gx[gx.length - 1] + 1e-7 || y < gy[0] - 1e-7 || y > gy[gy.length - 1] + 1e-7) return false;
    const fx = (x - gx[0]) / (gx[1] - gx[0]), fy = (y - gy[0]) / (gy[1] - gy[0]);
    for (const i of new Set([Math.floor(fx - 1e-4), Math.floor(fx + 1e-4)])) for (const j of new Set([Math.floor(fy - 1e-4), Math.floor(fy + 1e-4)])) {
      if (i >= 0 && i <= gx.length - 2 && j >= 0 && j <= gy.length - 2 && m[i][j] && m[i + 1][j] && m[i][j + 1] && m[i + 1][j + 1]) return true;
    }
    return false;
  }
  function reach() {
    mapDraw();
    if (!R.info) return;
    const x = num(R.x), y = num(R.y), a = num(R.yaw);
    if (![x, y, a].every(Number.isFinite)) { R.reach.className = "note bad"; R.reach.textContent = "x, y, 각도에 숫자를 넣는다"; return; }
    const ok = inside(x, y);
    R.reach.className = "note" + (ok ? "" : " warn");
    R.reach.textContent = (ok ? "✓ 놓는 범위 안 (학습한 곳)" : "⚠ 놓는 범위 밖 — 학습하지 않은 곳이라 실패할 수 있다 (돌려 볼 수는 있음)")
      + (Math.abs(a) > 45 ? " · 각도는 −45~45° 로 학습" : "");
  }

  // ---------- 지도 (위에서 본 책상) ----------
  // 틀 = 대시보드 실좌표 (역기구학·탑 쌓기 탭 지도와 같은 방향): 원점 = 어깨, 오른쪽 = +x(로봇 왼쪽), 아래 = −y(앞).
  // 큐브 칸(정우 님 틀: 기준 큐브에서 dx, dy cm)은 서버가 준 변환으로 옮긴다: 실좌표 = ref + dx·ax + dy·ay (rl_runner.info 의 map — 3D 의 큐브와 같은 변환)
  const MX0 = MX0_, MX1 = MX1_, MY0 = MY0_, MY1 = MY1_, K = 1000, EDGE = 0.035;   // EDGE = 놓을 자리가 책상 끝에서 떨어져야 하는 거리 (rl_runner.place 와 같게)
  const sv = (tag, at, ...kids) => { const e = document.createElementNS("http://www.w3.org/2000/svg", tag); for (const [k, v] of Object.entries(at)) e.setAttribute(k, v); e.append(...kids); return e; };
  const px = (x) => (x - MX0) * K, py = (y) => (MY1 - y) * K;
  R.map.setAttribute("viewBox", `0 0 ${(MX1 - MX0) * K} ${(MY1 - MY0) * K}`);
  function toReal(xc, yc) { const m = R.info.map; return [m.ref[0] + (xc * m.ax[0] + yc * m.ay[0]) / 100, m.ref[1] + (xc * m.ax[1] + yc * m.ay[1]) / 100]; }
  function toSpot(x, y) {   // 실좌표 m → 정우 님 dx, dy cm (ax, ay 는 서로 수직인 단위 벡터)
    const m = R.info.map, u = x - m.ref[0], v = y - m.ref[1];
    return [(u * m.ax[0] + v * m.ax[1]) * 100, (u * m.ay[0] + v * m.ay[1]) * 100];
  }
  function placeBox() {   // 놓을 수 있는 책상 안쪽 (실좌표) — 정우 님 장면의 책상에서 EDGE 만큼 안
    const t = R.info && R.info.map && R.info.map.table;
    return t ? [t.pos[0] - t.size[0] / 2 + EDGE, t.pos[0] + t.size[0] / 2 - EDGE, t.pos[1] - t.size[1] / 2 + EDGE, t.pos[1] + t.size[1] / 2 - EDGE] : [-9, 9, -9, 9];
  }
  function placeOk(x, y) {   // 그 자리가 초록인가 (지도 칸 + 책상 안). 지도가 아직 없으면 null
    const g = R.gmap, b = placeBox();
    if (!g) return null;
    const i = Math.round((x - g.xs[0]) / g.step), j = Math.round((y - g.ys[0]) / g.step);
    return x >= b[0] - 1e-9 && x <= b[1] + 1e-9 && y >= b[2] - 1e-9 && y <= b[3] + 1e-9 && j >= 0 && j < g.rows.length && g.rows[j][i] === "2";
  }
  function setMode(m) {
    R.mode = m;
    R.bCube.classList.toggle("on", m === "cube"); R.bPlace.classList.toggle("on", m === "place");
    R.bCube.setAttribute("aria-pressed", m === "cube"); R.bPlace.setAttribute("aria-pressed", m === "place");
    if (m === "place") gmapLoad();
    reach();
  }
  function gmapLoad() {   // 놓을 수 있는 곳 = 역기구학 탭의 지도 (계획기 판정, /api/ik/graspmap) 를 이 장면의 책상 높이로. 서버가 기억한다
    if (R.gmap || R.gmapBusy || !R.info) return;
    R.gmapBusy = true; mapDraw();
    fetch("/api/ik/graspmap?shoulder_yaw=0&grasp=auto&d=" + encodeURIComponent(R.info.table_d)).then((r) => r.json()).then((r) => {
      R.gmapBusy = false;
      if (r.ok) R.gmap = r; else R.gmapErr = r.reason || r.error || "실패";
      mapDraw();
    }).catch(() => { R.gmapBusy = false; R.gmapErr = "서버 연결 실패"; mapDraw(); });
  }
  function mapDraw() {
    const I = R.info, g = [];
    if (!I || !I.map) { R.map.replaceChildren(); return; }
    const t = I.map.table, cube = R.mode === "cube";
    if (t) g.push(sv("rect", { x: px(t.pos[0] - t.size[0] / 2), y: py(t.pos[1] + t.size[1] / 2), width: t.size[0] * K, height: t.size[1] * K, fill: "#fff", "fill-opacity": 0.55, stroke: "#94a3b8", "stroke-width": 2 }),
      sv("text", { x: px(MX1) - 10, y: py(t.pos[1] + t.size[1] / 2) + 20, "font-size": 19, fill: "#64748b", "text-anchor": "end" }, "책상 (로봇 쪽 끝)"));
    if (cube) {   // 초록 = 정책이 학습한 곳 (reach_mask_v65: 네 꼭짓점이 모두 유효한 2 cm 칸)
      const gx = I.gx_cm, gy = I.gy_cm, m = I.valid;
      for (let i = 0; i < gx.length - 1; i++) for (let j = 0; j < gy.length - 1; j++) {
        if (!(m[i][j] && m[i + 1][j] && m[i][j + 1] && m[i + 1][j + 1])) continue;
        const a = toReal(gx[i], gy[j]), b = toReal(gx[i + 1], gy[j + 1]);
        g.push(sv("rect", { x: px(Math.min(a[0], b[0])), y: py(Math.max(a[1], b[1])), width: Math.abs(a[0] - b[0]) * K, height: Math.abs(a[1] - b[1]) * K, fill: "#16a34a", "fill-opacity": 0.55 }));
      }
      for (const s of I.spots) { const p = toReal(s[0], s[1]); g.push(sv("circle", { cx: px(p[0]), cy: py(p[1]), r: 4, fill: "#14532d", "fill-opacity": 0.6 })); }
    } else if (R.gmap) {   // 초록 = 우리 IK 계획이 놓을 수 있는 곳 (책상 안쪽만)
      const m = R.gmap, b = placeBox();
      m.rows.forEach((row, j) => { for (let i = 0; i < row.length; i++) {
        if (row[i] !== "2" || m.xs[i] < b[0] - 1e-9 || m.xs[i] > b[1] + 1e-9 || m.ys[j] < b[2] - 1e-9 || m.ys[j] > b[3] + 1e-9) continue;
        g.push(sv("rect", { x: px(m.xs[i] - m.step / 2), y: py(m.ys[j] + m.step / 2), width: m.step * K, height: m.step * K, fill: "#16a34a", "fill-opacity": 0.55 }));
      } });
    }
    g.push(sv("circle", { cx: px(0), cy: py(0), r: 9, fill: "#64748b" }), sv("text", { x: px(0) + 15, y: py(0) + 5, "font-size": 19, fill: "#64748b" }, "로봇 (어깨 = x 0, y 0)"));
    const tx = num(R.tx), ty = num(R.ty), x = num(R.x), y = num(R.y);
    if (Number.isFinite(tx) && Number.isFinite(ty)) g.push(sv("rect", { x: px(tx) - 29, y: py(ty) - 29, width: 58, height: 58, fill: "none", stroke: "#1d4ed8", "stroke-width": cube ? 3 : 5, "stroke-dasharray": "9 5", rx: 4,
      transform: `rotate(${-(num(R.tyaw) || 0)} ${px(tx)} ${py(ty)})` }), sv("text", { x: px(tx), y: py(ty) + 48, "font-size": 19, "font-weight": 700, fill: "#1d4ed8", "text-anchor": "middle" }, "놓을 자리"));
    if (R.src === "live") g.push(...lvDrawCubes(px, py));
    else if (Number.isFinite(x) && Number.isFinite(y)) { const p = toReal(x, y);
      g.push(sv("rect", { x: px(p[0]) - 25, y: py(p[1]) - 25, width: 50, height: 50, fill: "#f97316", stroke: "#fff", "stroke-width": cube ? 3 : 0, rx: 3, transform: `rotate(${-(num(R.yaw) || 0)} ${px(p[0])} ${py(p[1])})` }),
        sv("text", { x: px(p[0]), y: py(p[1]) - 32, "font-size": 19, "font-weight": 700, fill: "#c2410c", "text-anchor": "middle" }, "큐브")); }
    R.map.replaceChildren(...g);
    const ok = cube ? null : (Number.isFinite(tx) && Number.isFinite(ty) ? placeOk(tx, ty) : null);
    R.mapNote.textContent = cube && R.src === "live" ? "영상 인식: 큐브 자리 = 지금 보이는 큐브 (여럿이면 눌러서 고른다" + (LQ.virtual() ? ", 가상 모드: 빈 곳 = 가상 큐브 놓기 · 끌기 = 옮기기)." : ").")
      : cube ? "지도를 눌러 큐브 자리를 고른다. 초록 = 정책이 집을 수 있는 곳(학습한 범위), 점 = 평가 27곳. 위 = 로봇 · 아래 = 앞 · 오른쪽 = 로봇 왼쪽(+x)."
      : R.gmapBusy ? "놓을 수 있는 곳 계산 중… (처음 한 번 10~40초)" : R.gmap ? "지도를 눌러 놓을 자리를 고른다. 초록 = 역기구학 계획이 놓을 수 있는 곳 (초록 가장자리는 손에 든 큐브가 기울어 안 될 수도 있다)."
        + (ok === false ? " ⚠ 지금 놓을 자리는 초록 밖 — 계획이 안 될 수 있다." : "") : "놓을 수 있는 곳 지도를 못 만듦: " + (R.gmapErr || "");
  }
  function mapClick(x, y) {
    if (!R.info || !R.info.map) return;
    if (R.mode === "cube") {   // 1 cm 칸에 맞춤 (정우 님 틀)
      const s = toSpot(x, y);
      R.x.value = Math.round(s[0]); R.y.value = Math.round(s[1]); R.preset.value = "";
      reach();
    } else { R.tx.value = (Math.round(x * 100) / 100).toFixed(2); R.ty.value = (Math.round(y * 100) / 100).toFixed(2); mapDraw(); }
  }

  // ---------- 큐브 자리: 영상 인식 / 지도로 고르기 ----------
  function setSrc(m) {
    R.src = m;
    try { localStorage.setItem("jaxdash.rl.src", m); } catch (e) { /* 없어도 됨 */ }
    R.bLive.classList.toggle("on", m === "live"); R.bMap.classList.toggle("on", m === "map");
    for (const i of [R.x, R.y, R.yaw, R.preset]) i.disabled = m === "live";
    if (m === "live") { setMode("cube"); lvPoll(); }
    liveSpot();
  }
  function liveCube() {   // 영상 인식의 큐브: 고른 번호, 없으면 책상 위의 가장 작은 번호 (뎁스만 보인 큐브도 됨)
    const cs = LQ.cubes().filter((c) => c.level === 0 && LQ.isTop(c));
    return (LQ.sel != null && cs.find((c) => c.id === LQ.sel)) || cs.slice().sort((a, b) => (a.id ?? 99) - (b.id ?? 99))[0] || null;
  }
  function liveSpot() {   // 영상 인식이면 큐브 칸 = 지금 큐브 (정우 님 틀 dx, dy cm · 각은 90° 마다 같아 틀 회전만큼 돌려 −45~45)
    if (R.src !== "live") { R.liveNote.hidden = true; R.liveOk = true; setBusy(R.busy); mapDraw(); return; }
    const r = LQ.r, c = r && !r.stale ? liveCube() : null;
    R.liveNote.hidden = false;
    R.liveOk = !!(c && R.info && R.info.map);
    if (R.liveOk) {
      const s = toSpot(c.x, c.y), m = R.info.map, rot = Math.atan2(m.ax[1], m.ax[0]) * 180 / Math.PI;
      const yaw = ((c.yaw_deg - rot + 45) % 90 + 90) % 90 - 45;
      R.x.value = (Math.round(s[0] * 10) / 10).toString(); R.y.value = (Math.round(s[1] * 10) / 10).toString(); R.yaw.value = (Math.round(yaw * 10) / 10).toString();
      R.liveNote.className = "note";
      R.liveNote.textContent = `영상 인식: 큐브 ${c.id ?? "(뎁스)"} — 실좌표 (${c.x.toFixed(3)}, ${c.y.toFixed(3)}) ${c.yaw_deg.toFixed(0)}° → 정우 님 틀 (${R.x.value}, ${R.y.value}) cm ${R.yaw.value}° · ${lvSrcText(r)}`;
      reach();
    } else {
      R.liveNote.className = "note bad";
      R.liveNote.textContent = !r ? "인식 결과 기다리는 중…" : r.stale ? `영상 없음 — 들기 막음 (${r.reason || ""})` : (r.mode === "virtual" ? "가상 큐브가 없음 — 지도를 눌러 놓기" : "책상 위 큐브가 안 보임");
      mapDraw();
    }
    setBusy(R.busy);
  }
  LQ.on(() => { if (curTab === "rl") { liveSpot(); show(); } });

  // ---------- 서버 ----------
  async function get(url) {
    try { const r = await fetch(url); if (!r.ok) return { ok: false, error: r.status === 404 ? "404" : "서버 응답 " + r.status }; return await r.json(); }
    catch (e) { return { ok: false, error: "서버 연결 실패" }; }
  }
  function setBusy(b, text) {
    R.busy = b;
    R.run.disabled = b || (R.src === "live" && !R.liveOk); R.evalBtn.disabled = b;
    R.run.title = R.src === "live" && !R.liveOk ? "영상 인식: 큐브가 안 보임 (영상 없음)" : "";
    R.place.disabled = b || !(R.res && R.res.summary.done);
    if (b && text) { R.out.className = "result run"; R.out.textContent = text; }
  }
  async function loadInfo() {
    if (R.info || R.loading) return;
    R.loading = true;
    const i = await get("/api/rl/info");
    R.loading = false;
    if (!i.ok) {
      R.run.disabled = true; R.evalBtn.disabled = true;
      R.out.className = "result bad";
      R.out.textContent = i.error === "404" ? "이 탭은 대시보드를 다시 시작해야 켜진다 (서버가 옛 버전)" : (i.error || "sim2sim 을 못 읽음");
      return;
    }
    R.info = i;
    if (i.policy) R.policy.textContent = i.policy;
    i.spots.forEach((s, k) => R.preset.append(el("option", { value: k }, `${k + 1}. x ${s[0]}, y ${s[1]}, ${s[2]}°`)));
    setMode(R.mode);
    setSrc(R.src);
    showEval(i.status.eval);
    if (i.status.busy) { setBusy(true, "돌리는 중…"); wait(); }
    else if (i.status.has_result) loadResult();
  }
  async function run() {
    if (R.busy) return;
    if (R.src === "live") { await lvPoll(); liveSpot(); if (!R.liveOk) { toast("영상 인식: 큐브가 안 보임 — 들기 안 함", "error"); return; } }
    const b = { x_cm: num(R.x), y_cm: num(R.y), yaw_deg: num(R.yaw) };
    if (![b.x_cm, b.y_cm, b.yaw_deg].every(Number.isFinite)) { toast("x, y, 각도에 숫자를 넣는다", "error"); return; }
    setBusy(true, "돌리는 중… (몇 초)");
    const r = await post("/api/rl/run", b, true);
    if (!r.ok) { setBusy(false); R.out.className = "result bad"; R.out.textContent = r.error || "실패"; return; }
    wait();
  }
  async function place() {
    if (R.busy) return;
    const b = { x: num(R.tx), y: num(R.ty), yaw_deg: num(R.tyaw) };
    if (![b.x, b.y, b.yaw_deg].every(Number.isFinite)) { toast("놓을 자리 x, y, 각도에 숫자를 넣는다", "error"); return; }
    setBusy(true, "내리기 계획 중… (몇 초)");
    const r = await post("/api/rl/place", b, true);
    if (!r.ok) { setBusy(false); R.out.className = "result bad"; R.out.textContent = r.error || "실패"; return; }
    wait();
  }
  async function evalAll() {
    if (R.busy) return;
    const r = await post("/api/rl/eval", {}, true);
    if (!r.ok) return;
    setBusy(true);
    R.evalOut.textContent = "27곳 돌리는 중… (1분쯤)";
    wait();
  }
  function wait() {
    clearTimeout(R.timer);
    R.timer = setTimeout(async () => {
      const s = await get("/api/rl/status");
      if (s.ok && s.busy) { if (s.kind === "eval") R.evalOut.textContent = `27곳 돌리는 중… ${Math.round(s.elapsed)}초`; return wait(); }
      setBusy(false);
      if (!s.ok || s.error) {
        const msg = "실패: " + (s.error || "서버 응답 없음");
        if (s.kind === "eval") R.evalOut.textContent = msg; else { R.out.className = "result bad"; R.out.textContent = msg; }
        return;
      }
      if (s.kind === "eval") showEval(s.eval); else loadResult(true);
    }, 500);
  }
  function showEval(e) {
    if (!e) { R.evalOut.textContent = ""; return; }
    const bad = e.rows.filter((r) => !r.done).map((r) => `(${r.x_cm}, ${r.y_cm}, ${r.yaw_deg}°) ${WHY[r.fail] || r.fail}`);
    const old = R.info && e.policy && e.policy !== R.info.policy ? ` — 예전 정책 ${e.policy} 결과, 다시 돌려야 함` : "";
    R.evalOut.textContent = `완료 ${e.done}/${e.runs}곳 (${new Date(e.mtime * 1000).toLocaleString("ko-KR")})` + old + (bad.length ? " — 실패: " + bad.join(" · ") : "");
  }
  async function loadResult(autoplay) {
    const d = await get("/api/rl/result");
    if (!d.ok) { if (autoplay) { R.out.className = "result bad"; R.out.textContent = d.error || "결과를 못 읽음"; } return; }
    R.res = d;
    const s = d.summary, p = s.phase_t_s, sec = (v) => v == null ? "—" : v.toFixed(1) + "초";
    const where = `(${d.spot.x_cm}, ${d.spot.y_cm}) cm ${d.spot.yaw_deg}°`;
    const steps = `닫기 ${sec(p.GRASP_LIFT)} → 복귀 ${sec(p.RETURN)} → 완료 ${sec(p.DONE)}`;
    const pl = d.place;
    R.out.className = "result " + (s.done && (!pl || pl.ok) ? "ok" : "bad");
    if (pl) {
      const at = `(${pl.target.x}, ${pl.target.y}) m`;
      R.out.textContent = pl.ok ? `들기 성공 → 내리기 계획 OK ${at} · 총 ${pl.total_s.toFixed(0)}초 (들기 ${(pl.total_s - pl.seconds).toFixed(0)} + 내리기 ${pl.seconds.toFixed(0)})`
          + (pl.checks && pl.checks.release_tilt_deg > 5 ? ` · 큐브가 손 안에서 ${pl.checks.release_tilt_deg.toFixed(0)}° 기운 채 놓임` : "")
        : `들기 ${s.done ? "성공" : "실패"} → 내리기 안 됨 ${at}: ${pl.reason}`;
    } else R.out.textContent = (s.done ? `성공 ${where} — 큐브 ${(s.max_rise_m * 100).toFixed(1)} cm 들림 · ${steps}`
      : `실패 ${where} — ${WHY[s.fail_reason] || s.fail_reason} · 최대 ${(s.max_rise_m * 100).toFixed(1)} cm 들림 · ${steps}`)
      + (s.valid_spot ? "" : " · 놓는 범위 밖");
    R.slider.max = d.t.length - 1;
    R.frame = autoplay && pl && pl.ok ? pl.start_frame : 0;   // 내리기를 막 계획했으면 들기 끝에서부터 재생 (처음부터는 슬라이더·다시 재생)
    R.playing = !!autoplay; R.last = performance.now();
    if (pl) { R.tx.value = pl.target.x; R.ty.value = pl.target.y; R.tyaw.value = pl.target.yaw_deg; }
    if (!R.preset.value && R.src !== "live") { R.x.value = d.spot.x_cm; R.y.value = d.spot.y_cm; R.yaw.value = d.spot.yaw_deg; }   // 지도의 큐브 = 3D 의 큐브 (다시 열었을 때도)
    if (autoplay && s.done && !pl) setMode("place"); else reach();   // 들기가 막 성공했으면 다음은 놓을 자리 고르기
    R.place.disabled = R.busy || !s.done;
    scene();
    show();
  }

  // ---------- 3D ----------
  function onShow() {
    loadInfo(); lvPoll();
    if (!R.view && window.JaxViewer && S && S.robot) {
      R.view = window.JaxViewer.create($("#v3d-rl"), {});
      R.view.load(S.robot.mesh_ok).then(() => { scene(); show(); });
    }
    if (R.view && R.view.resize) R.view.resize();
  }
  window.addEventListener("jaxviewer-ready", () => { if (curTab === "rl") onShow(); });
  function scene() {   // 정우 님 배치의 책상 (viewer 의 책상 윗판을 복제해 크기·자리만 바꿈)
    const v = R.view, d = R.res;
    if (!v || v.dead || !d) return;
    v.setTableD(d.table_d);
    if (d.table && !R.tableMesh && v.table && v.table.children[0]) {
      const t = v.table.children[0].clone(), g = t.geometry.parameters;
      t.scale.set(d.table.size[0] / g.width, d.table.size[1] / g.height, d.table.size[2] / g.depth);
      t.position.set(d.table.pos[0], d.table.pos[1], d.table.pos[2]);
      v.scene.add(t); R.tableMesh = t;
    }
  }
  function show() {
    const v = R.view, d = R.res;
    if (!d) return;
    const k = Math.max(0, Math.min(d.t.length - 1, Math.round(R.frame)));
    R.slider.value = k; rangeFill(R.slider);
    R.play.textContent = R.playing ? "❚❚ 멈춤" : "▶ 재생";
    R.num.textContent = `${d.t[k].toFixed(1)} / ${d.t[d.t.length - 1].toFixed(1)}초 · ${PH[d.phases[d.phase[k]]] || ""} · 큐브 높이 ${d.rise_cm[k] >= 0 ? "+" : ""}${d.rise_cm[k].toFixed(1)} cm`;
    if (!v || v.dead || !v.ready) return;
    const q = {};
    d.joint_names.forEach((n, i) => { q[n] = d.q[k][i]; });
    v.setActual(q);
    // 엄지 받침(RotationBody): 3D 모델(arm.xml)엔 관절이 없고 −180° 로 굳어 있다 → 몸체 틀 = Rx(180°)·Rz(각) 로 직접 돌린다
    const f3 = v.actual && v.actual.joints.get("finger3_joint1");
    const rb = f3 && f3.obj.parent && f3.obj.parent.parent;
    if (rb) { const h = d.rot[k] / 2; rb.quaternion.set(Math.cos(h), -Math.sin(h), 0, 0); }
    const c = d.cube[k];
    v.cube.visible = true;
    v.cube.position.set(c[0], c[1], c[2]);
    v.cube.quaternion.set(c[3], c[4], c[5], c[6]);
    lvBoxes(v, d.table_d, [[d.cube[0][0], d.cube[0][1]]]);   // 가상 모드: 장면의 다른 가상 큐브들
    const pt = d.place && d.place.ok && d.place.target;   // 놓을 자리 표시 (초록)
    v.target.visible = !!pt;
    if (pt) { v.target.position.set(pt.x, pt.y, -d.table_d + 0.025); v.target.rotation.set(0, 0, pt.yaw_deg * Math.PI / 180); }
    v.dirty();
  }
  function tick(now) {
    requestAnimationFrame(tick);
    if (!R.playing || !R.res || curTab !== "rl") { R.last = now; return; }
    const hz = (R.res.t.length - 1) / (R.res.t[R.res.t.length - 1] - R.res.t[0] || 1);
    R.frame += (now - R.last) / 1000 * hz;
    R.last = now;
    if (R.frame >= R.res.t.length - 1) { R.frame = R.res.t.length - 1; R.playing = false; }
    show();
  }
  requestAnimationFrame(tick);

  // ===== 팔로 재생 (2026-11-24): 들기 + 내리기를 실물 / 가상 팔로 — 이 블록만 (서버 rl_api.py /api/rl/traj·play_real·play_virtual) =====
  // 실물 = direct_api.play_real 그대로 (IK 탭 '실물로 재생'과 같은 검사·확인창), 가상 = server._play_virtual 그대로. mtime = 화면이 본 궤적.
  const A = { info: null, res: null, msg: "" };
  A.btn = el("button", { class: "btn red", onclick: armPlay, disabled: true }, "실물로 재생");
  A.out = el("div", { class: "note" });
  A.help = el("p", { class: "help" }, "팔로 재생 = 시뮬 결과(들기 + 내리기)를 그대로 되풀이 (열린 루프). 실물 손엔 손가락 압력 센서(정책 관측 37–42)가 없고 큐브를 매 순간 따라가지 않는다. 먼저 ⌂ HOME — 지금 자세가 HOME(첫 프레임)에서 5° 넘게 다르면 거절.");
  R.place.after(A.btn);
  R.out.after(A.out, A.help);
  async function armInfo() { A.res = R.res; A.msg = ""; A.info = await get("/api/rl/traj"); armRender(); }
  function armRender() {
    if (R.res !== A.res) { armInfo(); return; }
    const real = DIRECT(), virt = VIRT(), i = A.info;
    const pb = (virt ? ((S.ros || {}).virtual_state || {}).playback : DS().playback) || {};
    A.btn.hidden = A.help.hidden = A.out.hidden = !(real || virt);
    A.btn.textContent = virt ? "가상 팔로 재생" : "실물로 재생";
    A.btn.className = virt ? "btn" : "btn red";
    const why = R.busy ? "계산 중" : !i || !i.ok ? (i && i.error) || "들기 + 내리기 먼저" : real && !DC() ? "연결 탭 ② 팔 연결 먼저" : pb.running ? "재생 중" : "";
    A.btn.disabled = !!why; A.btn.title = why;
    const plan = i && i.ok ? `재생 ${i.play_s.toFixed(0)}초 (시뮬 ${i.sim_s.toFixed(0)}초의 ${i.factor.toFixed(1)}배 — 빠른 구간 최대 ${i.stretch_max.toFixed(1)}배 늘림, ${i.speed_deg_s}°/s)` : "재생 안 됨: " + ((i && i.error) || "…");
    A.out.className = "note" + (i && i.ok ? "" : " warn");
    A.out.textContent = pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1} (${(pb.slow || 1).toFixed(1)}배 더 느리게, 약 ${(pb.seconds || 0).toFixed(0)}초) · 멈춤 = ■ STOP / Esc`
      : (A.msg ? A.msg + " · " : "") + (pb.reason ? "마지막 재생: " + pb.reason + " · " : "") + plan;
  }
  async function armPlay() {
    const i = A.info, real = DIRECT();
    if (!i || !i.ok) return;
    const what = `강화학습 들기 + 역기구학 내리기 (${i.play_s.toFixed(0)}초, 시뮬 결과 그대로 — 열린 루프)`;
    if (!confirm(real ? `⚠ 실물 팔로 ${what} 를 재생한다.\n\n먼저 ⌂ HOME: 지금 자세가 첫 프레임(HOME, 모든 관절 0)에서 5° 넘게 다르면 거절. 원점 4개·서보 한 번씩 보낸 뒤. ` +
      "AK45 가 명령에서 15° 넘게 0.5초 벌어지면 멈추고 그 자리 홀드.\n\n팔 주변을 비웠나요?" : `(가상) 가상 팔로 ${what} 를 재생할까요?`)) return;
    if (real && !confirm("정말 실물로 재생할까요? (멈춤 = ■ STOP / Esc)")) return;
    const url = real ? "/api/rl/play_real" : "/api/rl/play_virtual";
    let r = await post(url, { force: false, mtime: i.mtime }, true);
    if (!r.ok && r.need_force) {
      if (!confirm("검사에서 문제가 나왔다 (play_trajectory 면 --force 가 필요):\n\n- " + r.problems.join("\n- ") + "\n\n그래도 재생할까요?")) { A.msg = "취소: 검사 " + r.problems.length + "건"; armRender(); return; }
      r = await post(url, { force: true, mtime: i.mtime }, true);
    }
    A.msg = r.ok ? (r.msg || "재생 시작") : "거절: " + (r.error || "실패");
    if (r.ok) toast(A.msg);
    armRender();
  }
  setInterval(() => { if (curTab === "rl") armRender(); }, 500);
  // ===== 팔로 재생 끝 =====

  if (curTab === "rl") onShow();
})();
