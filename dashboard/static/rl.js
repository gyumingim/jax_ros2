// '강화학습' 탭: 팀원이 학습한 집기 정책(team-jax/IK sim2sim, MuJoCo)을 한 곳에서 돌려 보고 3D 로 다시 본다.
// 버튼 둘: [강화학습으로 들기] = 그 정책(끝 = 큐브를 쥔 ㄴ자 자세), [역기구학으로 내리기] = 거기서 이어 우리 IK 계획(pick_plan.plan_place_from)으로
// 놓을 자리에 내려놓고 HOME. 3D 는 들기 + 내리기를 이어서 재생한다.
// 서버: rl_api.py (/api/rl/*) → rl_runner.py. 시뮬만 — 실물·가상 팔에는 아무것도 안 보낸다.
// app.js 다음에 읽혀 탭 하나를 덧붙인다(app.js 는 그대로). 이 파일이 실패해도 다른 탭은 그대로 돈다.
"use strict";
(function () {
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
  R.card = el("div", { class: "card" },
    el("h3", {}, "정책 돌려보기"),
    el("p", { class: "help" }, "팀원이 학습한 집기 정책 ", el("b", {}, "grasp_lift_23025"), " 을 MuJoCo 시뮬에서 돌린다. 실물 팔은 안 움직인다."),
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
  R.preset.addEventListener("change", () => {
    const s = R.info && R.info.spots[parseInt(R.preset.value, 10)];
    if (s) { R.x.value = s[0]; R.y.value = s[1]; R.yaw.value = s[2]; reach(); }
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
    if (!R.info) return;
    const x = num(R.x), y = num(R.y), a = num(R.yaw);
    if (![x, y, a].every(Number.isFinite)) { R.reach.className = "note bad"; R.reach.textContent = "x, y, 각도에 숫자를 넣는다"; return; }
    const ok = inside(x, y);
    R.reach.className = "note" + (ok ? "" : " warn");
    R.reach.textContent = (ok ? "✓ 놓는 범위 안 (학습한 곳)" : "⚠ 놓는 범위 밖 — 학습하지 않은 곳이라 실패할 수 있다 (돌려 볼 수는 있음)")
      + (Math.abs(a) > 45 ? " · 각도는 −45~45° 로 학습" : "");
  }

  // ---------- 서버 ----------
  async function get(url) {
    try { const r = await fetch(url); if (!r.ok) return { ok: false, error: r.status === 404 ? "404" : "서버 응답 " + r.status }; return await r.json(); }
    catch (e) { return { ok: false, error: "서버 연결 실패" }; }
  }
  function setBusy(b, text) {
    R.busy = b;
    R.run.disabled = b; R.evalBtn.disabled = b;
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
    i.spots.forEach((s, k) => R.preset.append(el("option", { value: k }, `${k + 1}. x ${s[0]}, y ${s[1]}, ${s[2]}°`)));
    reach();
    showEval(i.status.eval);
    if (i.status.busy) { setBusy(true, "돌리는 중…"); wait(); }
    else if (i.status.has_result) loadResult();
  }
  async function run() {
    if (R.busy) return;
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
    R.evalOut.textContent = `완료 ${e.done}/${e.runs}곳 (${new Date(e.mtime * 1000).toLocaleString("ko-KR")})` + (bad.length ? " — 실패: " + bad.join(" · ") : "");
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
    R.place.disabled = R.busy || !s.done;
    scene();
    show();
  }

  // ---------- 3D ----------
  function onShow() {
    loadInfo();
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
  if (curTab === "rl") onShow();
})();
