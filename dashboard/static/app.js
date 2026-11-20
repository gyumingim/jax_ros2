// JAX 팔 대시보드 — 상태는 WS(/ws, 0.5초마다 전체 스냅샷), 안 되면 /api/state 폴링.
// 모터 각도 명령 버튼은 없음(읽기 + 프로세스 관리 + homing_guard 원점 잡기/쉬게 하기 서비스).
"use strict";
const $ = (s, r = document) => r.querySelector(s);
const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k === "class") e.className = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v !== undefined && v !== null && v !== false) e.setAttribute(k, v);
  }
  for (const k of kids.flat()) if (k !== null && k !== undefined) e.append(k.nodeType ? k : document.createTextNode(String(k)));
  return e;
};
let S = null, sock = null, pollTimer = null;

function toast(text, level = "info") {
  const t = el("div", { class: "toast " + level }, text);
  $("#toasts").append(t);
  setTimeout(() => t.remove(), level === "error" ? 9000 : 4000);
}
function chip(id, text, cls) { const c = document.getElementById(id); c.className = "chip " + (cls || ""); c.innerHTML = '<span class="dot"></span>'; c.append(text); }
async function post(url, body) {
  try {
    const r = await fetch(url, { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(body || {}), keepalive: true });
    const j = await r.json();
    if (!j.ok) toast(j.error || "실패", "error"); else if (j.msg) toast(j.msg);
    return j;
  } catch (e) { toast("서버 연결 실패", "error"); return { ok: false }; }
}
function stopAll() { post("/api/stop"); }
$("#btn-stop").onclick = stopAll;
document.addEventListener("keydown", (e) => { if (e.key === "Escape") stopAll(); });

// ---------- 연결 ----------
function connect() {
  sock = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  sock.onopen = () => { chip("chip-ws", "연결됨", "on"); if (pollTimer) { clearInterval(pollTimer); pollTimer = null; } };
  sock.onmessage = (ev) => { S = JSON.parse(ev.data); render(); };
  sock.onclose = () => {
    chip("chip-ws", "WS 끊김 — 폴링", "warn");
    if (!pollTimer) pollTimer = setInterval(poll, 1000);
    setTimeout(connect, 3000);
  };
}
async function poll() {
  try { S = await (await fetch("/api/state")).json(); render(); chip("chip-ws", "폴링 연결", "warn"); }
  catch (e) { chip("chip-ws", "서버 끊김", "bad"); }
}

// ---------- 그리기 ----------
const fmtDur = (s) => s == null ? "" : s < 60 ? `${s.toFixed(0)}초` : s < 3600 ? `${Math.floor(s / 60)}분 ${Math.floor(s % 60)}초` : `${Math.floor(s / 3600)}시간 ${Math.floor(s % 3600 / 60)}분`;
const LV = ["on", "warn", "bad", "warn"];   // diagnostic level 0 OK 1 WARN 2 ERROR 3 STALE
const LVT = ["정상", "주의", "에러", "오래됨"];

function render() {
  if (!S) return;
  const r = S.ros;
  chip("chip-ros", r.available ? "ROS 구독 중" : "ROS 없음", r.available ? "on" : "");
  const c = S.can;
  chip("chip-can", c.text || "can0", !c.available ? "" : c.up ? (c.state && c.state !== "ERROR-ACTIVE" ? "warn" : "on") : "bad");
  const running = S.services.filter((s) => s.alive).length;
  chip("chip-run", `실행 ${running}`, running ? "on" : "");
  const sb = $("#straybar");
  if (S.strays.length) {
    sb.hidden = false;
    sb.textContent = "대시보드 밖에서 떠 있는 노드(전부 정지로 안 꺼짐): " + S.strays.map((x) => `${x.name} pid ${x.pid}`).join(", ") +
      " — 그 터미널에서 끄거나 pkill -INT -f <이름>";
  } else sb.hidden = true;
  renderMotors();
  renderServices();
  renderCalib();
  renderHoming();
  $("#foot").textContent = `워크스페이스 ${S.info.ws} · ROS ${S.info.distro} · 레포 ${S.info.repo} · 로그 ${S.info.logs} · ${S.info.platform}`;
}

function renderMotors() {
  const r = S.ros, box = $("#motors");
  box.replaceChildren();
  if (!r.available) {
    $("#ros-note").textContent = "";
    box.append(el("div", { class: "card muted" }, r.reason || "ROS 없음",
      el("div", { class: "help" }, "Linux 에서 ROS 를 source 한 뒤 서버를 켜야 보인다. 켜기/끄기는 그대로 된다.")));
    return;
  }
  $("#ros-note").textContent = r.diag_age == null ? "/diagnostics 아직 없음 (ak45_node 꺼짐?)" :
    `/diagnostics ${r.diag_age.toFixed(1)}초 전 · /joint_states ${r.joint_age == null ? "없음" : r.joint_age.toFixed(1) + "초 전"}`;
  if (!r.motors.length) { box.append(el("div", { class: "card muted" }, "ak45_node 진단을 기다리는 중")); return; }
  for (const m of r.motors) {
    const v = m.values || {}, stale = m.age > 3;
    const lv = stale ? 3 : m.level;
    box.append(el("div", { class: "card" },
      el("h3", {}, m.name.replace("ak45/", ""), el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv]),
        el("span", { class: "help", style: "margin:0" }, m.hardware_id)),
      el("div", { class: "big" }, v.position_deg != null ? `${v.position_deg}°` : "—"),
      el("div", { class: "note " + (lv === 2 ? "bad" : lv ? "warn" : "") }, stale ? `진단 ${m.age.toFixed(0)}초 전 (ak45_node 멈춤?)` : m.message),
      el("div", { class: "kv" },
        kv("온도", `${v.temperature_c ?? "—"} °C`), kv("에러 코드", v.error_code ?? "—"),
        kv("워치독", v.watchdog_ok === "1" ? "OK" : "끊김"), kv("target", v.target_deg === "none" ? "없음" : `${v.target_deg}°`),
        kv("command_active", v.command_active === "1" ? "예 (홀드 중)" : "아니오"), kv("오차", v.position_error_deg === "none" ? "—" : `${v.position_error_deg}°`),
        kv("전류", `${v.current_a ?? "—"} A`), kv("속도", `${v.speed_erpm ?? "—"} erpm`),
        kv("/joint_states", m.joint_deg == null ? "없음" : `${m.joint_deg.toFixed(2)}°`))));
  }
  if (r.bus) {
    const v = r.bus.values || {}, stale = r.bus.age > 3, lv = stale ? 3 : r.bus.level;
    box.append(el("div", { class: "card" },
      el("h3", {}, "CAN 버스", el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv])),
      el("div", { class: "big" }, v.state || "—"),
      el("div", { class: "note " + (lv === 2 ? "bad" : lv ? "warn" : "") }, r.bus.message),
      el("div", { class: "kv" }, kv("에러 프레임", v.err_frames), kv("bit-stuff", v.bit_stuff_errors), kv("form", v.form_errors),
        kv("no-ack", v.ack_errors), kv("버스오프", v.busoff_count), kv("재시작", v.restart_count))));
  }
}
const kv = (k, v) => [el("span", { class: "k" }, k), el("span", { class: "v" }, v ?? "—")];

// 항목 카드는 한 번 만들고 바뀌는 부분만 고친다(입력칸을 쓰는 중에 지워지지 않게)
const cards = {};
function renderServices() {
  const box = $("#services");
  for (const s of S.services) {
    let c = cards[s.id];
    if (!c) { c = cards[s.id] = buildService(s); box.append(c.root); }
    updateService(c, s);
  }
}
function buildService(s) {
  const c = { fields: {} };
  c.chip = el("span", { class: "chip" });
  c.status = el("div", { class: "note" });
  c.note = el("div", { class: "note" });
  c.cmd = el("div", { class: "cmd", hidden: true });
  c.log = el("pre", { class: "logbox" });
  c.logbox = el("details", { class: "log" }, el("summary", {}, "로그 (마지막 30줄)"), c.log,
    el("div", { class: "row" }, el("a", { href: `/api/log/${s.id}?n=1000`, target: "_blank", class: "help" }, "로그 1000줄 보기")));
  const fields = s.params.map((p) => {
    const inp = el("input", { value: p.value, placeholder: p.hint || "" });
    c.fields[p.key] = inp;
    return el("label", { class: "field" }, `${p.label}${p.hint ? " (" + p.hint + ")" : ""}`, inp);
  });
  const params = () => Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.value]));
  const once = s.kind === "once";
  c.bStart = el("button", { class: "btn", onclick: () => post(`/api/svc/${s.id}/start`, { params: params() }) }, once ? "▶ 실행" : "▶ 켜기");
  c.bStop = el("button", { class: "btn red", onclick: () => {
    if (s.stop_warn && !confirm(s.stop_warn)) return;
    post(`/api/svc/${s.id}/stop`);
  } }, once ? "■ 중단" : "■ 끄기");
  c.bRestart = once ? null : el("button", { class: "btn line", onclick: () => {
    if (s.stop_warn && !confirm(s.stop_warn)) return;
    post(`/api/svc/${s.id}/restart`, { params: params() });
  } }, "↻ 재시작");
  c.root = el("div", { class: "card" },
    el("h3", {}, s.label, c.chip), el("p", { class: "help" }, `[${s.group}] ${s.help}`),
    fields, el("div", { class: "row" }, c.bStart, c.bStop, c.bRestart), c.status, c.note, c.cmd, c.logbox);
  return c;
}
function updateService(c, s) {
  let cls = "", txt = "꺼짐";
  if (!s.available) { txt = "못 씀"; }
  else if (s.stopping) { cls = "warn"; txt = "끄는 중"; }
  else if (s.alive) { cls = "on"; txt = s.kind === "once" ? "실행 중" : "켜짐"; }
  else if (s.exit_code != null) { cls = s.exit_code === 0 ? "" : "bad"; txt = s.exit_code === 0 ? "끝남 (0)" : `죽음 (코드 ${s.exit_code})`; }
  c.chip.className = "chip " + cls; c.chip.innerHTML = '<span class="dot"></span>'; c.chip.append(txt);
  c.bStart.disabled = !s.available || s.alive || s.stopping;
  c.bStop.disabled = !s.alive || s.stopping;
  if (c.bRestart) c.bRestart.disabled = !s.available || s.stopping;
  for (const inp of Object.values(c.fields)) inp.disabled = s.alive;
  let st = "";
  if (!s.available) st = s.why;
  else if (s.alive) st = `pid ${s.pid} · 가동 ${fmtDur(s.uptime)}${s.adopted ? " · 이전 대시보드에서 켠 것(다시 붙잡음)" : ""}`;
  else if (s.ended) st = `끝난 시각 ${new Date(s.ended * 1000).toLocaleTimeString()}`;
  c.status.textContent = st;
  const note = s.error || s.stop_note;
  c.note.textContent = note;
  c.note.className = "note " + (s.error || /SIGKILL|살아 있음/.test(s.stop_note) ? "bad" : /SIGTERM/.test(s.stop_note) ? "warn" : "");
  c.cmd.hidden = !s.cmd; c.cmd.textContent = s.cmd ? "$ " + s.cmd : "";
  const atBottom = c.log.scrollTop + c.log.clientHeight >= c.log.scrollHeight - 4;
  const text = s.log.join("\n");
  if (c.log.textContent !== text) { c.log.textContent = text; if (atBottom) c.log.scrollTop = c.log.scrollHeight; }
  if (s.alive && !c.opened) { c.logbox.open = true; c.opened = true; }
}

let calibBuilt = false;
function renderCalib() {
  const g = S.calibrate, box = $("#calib");
  const on = (id) => (S.services.find((x) => x.id === id) || {}).alive;
  if (!calibBuilt) {
    calibBuilt = true;
    box.append(el("div", { class: "card" },
      el("h3", {}, "보정은 터미널에서"),
      el("p", { class: "help" }, g.note),
      g.commands.map((x) => el("div", { class: "cmd" }, x)),
      el("div", { class: "row" }, el("button", { class: "btn line", onclick: async () => {
        for (const id of g.needs) if (!on(id)) await post(`/api/svc/${id}/start`, {});
      } }, "보정 준비: ak45_node + stm_bridge_node 켜기")),
      el("div", { class: "note", id: "calib-check" })));
  }
  const bad = g.forbid.filter(on).map((id) => S.services.find((x) => x.id === id).label);
  const miss = g.needs.filter((id) => !on(id)).map((id) => S.services.find((x) => x.id === id)?.label || id);
  const n = $("#calib-check");
  n.className = "note " + (bad.length ? "bad" : miss.length ? "warn" : "");
  n.textContent = bad.length ? `끄세요: ${bad.join(", ")} (calibrate 와 같은 모터에 명령)` :
    miss.length ? `아직 안 켬: ${miss.join(", ")}` : "준비됨: 터미널에서 calibrate --ros 실행";
}

// ---------- 원점 (homing_guard) ----------
// 카드는 한 번 만들고(확인창 중에 다시 그려져도 버튼이 안 사라지게) 글자만 바꾼다.
const H = { rows: {} };
function homeAsk(joint) {
  const poses = S.home_poses || {};
  const pose = joint === "all"
    ? Object.entries(poses).map(([k, v]) => `${k} ${v}°`).join(", ") || "전부 sim 0°"
    : `${joint} sim ${poses[joint] ?? 0}°`;
  return `${joint === "all" ? "원점 없는 관절 전부" : joint}: 관절을 기준 자세(homing.json home_sim_deg: ${pose})에 두었나요?
` +
    "지금 위치를 그 자세로 삼는다. 다른 자세에서 누르면 그만큼 틀어진 원점이 된다.";
}
const RELEASE_HELP = "쉬게 하기: guard 가 이 관절 명령 전달만 끊고 원점을 지운다. ak45_node 는 마지막 target 을 계속 잡고 있다 " +
  "(실제로 힘을 빼려면 ak45_node 종료 — 팔이 떨어질 수 있음).";
async function homingDo(action, joint) {
  if (action === "home" && !confirm(homeAsk(joint))) return;
  if (action === "release" && !confirm(`${joint === "all" ? "전부" : joint} 쉬게 하기?

${RELEASE_HELP}`)) return;
  await post(`/api/homing/${action}`, { joint });
}
function buildHoming() {
  const box = $("#homing");
  H.msg = el("div", { class: "note" });
  H.bHomeAll = el("button", { class: "btn", onclick: () => homingDo("home", "all") }, "전부 원점");
  H.bRelAll = el("button", { class: "btn line", onclick: () => homingDo("release", "all") }, "전부 쉬게");
  H.list = el("div", { class: "hlist" });
  H.card = el("div", { class: "card wide" },
    el("h3", {}, "AK45 원점", H.chip = el("span", { class: "chip" })),
    el("p", { class: "help" }, "homing_guard(arm_homed.launch.py)가 원점 잡기 전·전원 재투입 뒤·홀드가 끝난 뒤에는 AK45 명령을 막는다. " +
      "관절을 기준 자세에 두고 '원점 잡기'. " + RELEASE_HELP),
    el("div", { class: "row" }, H.bHomeAll, H.bRelAll), H.msg, H.list);
  box.append(H.card);
}
function homingRow(name) {
  const r = { };
  r.badge = el("span", { class: "chip" });
  r.reason = el("div", { class: "note" });
  r.kv = el("div", { class: "kv" });
  r.bHome = el("button", { class: "btn", onclick: () => homingDo("home", name) }, "원점 잡기");
  r.bRel = el("button", { class: "btn line", onclick: () => homingDo("release", name) }, "쉬게 하기");
  r.root = el("div", { class: "hrow" },
    el("div", { class: "hhead" }, el("b", {}, name), r.badge, el("span", { class: "spacer" }), r.bHome, r.bRel),
    r.reason, r.kv);
  return r;
}
const f2 = (x, u = "°") => x == null ? "—" : `${Number(x).toFixed(2)}${u}`;
function renderHoming() {
  if (!H.card) buildHoming();
  const h = (S.ros && S.ros.homing) || { available: false };
  const guardOn = h.available && h.seen && h.age <= 3;
  let chipCls = "", chipTxt = "guard 없음", msg = "", msgCls = "warn";
  if (!h.available) { msg = `${h.reason || "ROS 없음"} — 원점 칸은 ROS 를 source 하고 서버를 켜야 보인다.`; msgCls = ""; chipTxt = "ROS 없음"; }
  else if (!h.seen) msg = "homing_guard 가 안 켜짐: 위 '켜기/끄기'에서 arm_homed.launch.py 를 켜세요.";
  else if (h.age > 3) { msg = `원점 상태가 ${h.age.toFixed(0)}초 전 것 (homing_guard 꺼짐?). arm_homed.launch.py 를 켜세요.`; chipTxt = "오래됨"; chipCls = "warn"; }
  else if (h.all_homed) { chipTxt = "전부 원점"; chipCls = "on"; }
  else { chipTxt = `원점 ${h.joints.filter((j) => j.homed).length}/${h.joints.length}`; chipCls = "bad"; }
  H.chip.className = "chip " + chipCls; H.chip.innerHTML = '<span class="dot"></span>'; H.chip.append(chipTxt);
  H.msg.textContent = msg; H.msg.className = "note " + (msg ? msgCls : "");
  $("#homing-note").textContent = guardOn ? `/arm_control/homing ${h.age.toFixed(1)}초 전` : "";
  H.bHomeAll.disabled = H.bRelAll.disabled = !guardOn;
  const joints = h.seen ? h.joints : Object.keys(S.home_poses || {}).map((j) => ({ joint: j }));
  const seenNames = new Set(joints.map((j) => j.joint));
  for (const [n, r] of Object.entries(H.rows)) if (!seenNames.has(n)) { r.root.remove(); delete H.rows[n]; }
  for (const j of joints) {
    let r = H.rows[j.joint];
    if (!r) { r = H.rows[j.joint] = homingRow(j.joint); H.list.append(r.root); }
    r.bHome.disabled = r.bRel.disabled = !guardOn;
    r.badge.innerHTML = "";
    if (!guardOn || j.homed == null) { r.badge.className = "chip"; r.badge.append("—"); r.reason.textContent = ""; r.kv.replaceChildren(); continue; }
    r.badge.className = "chip " + (j.homed ? "on" : "bad");
    r.badge.append(j.homed ? "✅ 원점" : "⛔ 원점 없음");
    r.reason.textContent = j.reason || "";
    r.reason.className = "note " + (j.homed ? "" : "bad");
    const age = j.feedback_lost ? "끊김" : j.feedback_age_s == null ? "없음" : `${j.feedback_age_s.toFixed(2)}초`;
    r.kv.replaceChildren(...[
      kv("모터", j.motor), kv("기준 자세", `sim ${j.home_sim_deg ?? 0}°`),
      kv("오프셋", j.homed ? f2(j.offset_deg) : "—"), kv("세션 zero", f2(j.session_zero_deg)),
      kv("calibration zero", f2(j.calibration_zero_deg)), kv("피드백", `${f2(j.raw_deg)} · ${age}`),
      kv("command_active", j.command_active == null ? "—" : j.command_active ? "예" : "아니오"),
    ].flat());
  }
}

connect();
poll();
