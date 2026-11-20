// JAX 팔 대시보드 — 상태는 WS(/ws, 0.25초마다 전체 스냅샷), 안 되면 /api/state 폴링.
// 탭 3개: 연결 / 각도 제어 / 역기구학. 위 칩·모드 스위치·전부 정지·경고 띠는 모든 탭 공통.
// 모터를 움직이는 길은 각도 제어 탭의 '보내기' 하나(→ POST /api/joint/send → /arm_control/joint_command).
// 3D 뷰는 viewer.js(ES 모듈, three.js)가 window.JaxViewer 로 준다. 못 읽어도 나머지 화면은 돈다.
"use strict";
const $ = (s, r = document) => r.querySelector(s);
const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k === "class") e.className = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v !== undefined && v !== null && v !== false) e.setAttribute(k, v);
  }
  for (const k of kids.flat()) if (k !== null && k !== undefined && k !== false) e.append(k.nodeType ? k : document.createTextNode(String(k)));
  return e;
};
let S = null, sock = null, pollTimer = null;
const VIRT = () => !!(S && S.mode && S.mode.virtual);

function toast(text, level = "info") {
  const t = el("div", { class: "toast " + level }, text);
  $("#toasts").append(t);
  setTimeout(() => t.remove(), level === "error" ? 9000 : 4500);
}
function setChip(c, text, cls) { c.className = "chip " + (cls || ""); c.innerHTML = '<span class="dot"></span>'; c.append(text); }
function chip(id, text, cls) { setChip(document.getElementById(id), text, cls); }
async function post(url, body, quiet) {
  try {
    const r = await fetch(url, { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(body || {}), keepalive: true });
    const j = await r.json();
    if (!j.ok) toast(j.error || "실패", "error"); else if (j.msg && !quiet) toast(j.msg);
    return j;
  } catch (e) { toast("서버 연결 실패", "error"); return { ok: false }; }
}
function stopAll() { post("/api/stop"); }
$("#btn-stop").onclick = stopAll;
document.addEventListener("keydown", (e) => { if (e.key === "Escape") stopAll(); });
const vb = () => VIRT() ? el("span", { class: "vbadge" }, "가상") : null;

// ---------- 모드 (실물 / 가상) ----------
async function setMode(v) {
  if (VIRT() === v) return;
  if (v && !confirm("가상 모드로 바꿀까요?\n\n화면의 노드·모터·원점이 전부 가상 팔이 된다(실물과 상관없음). 실물 노드가 켜져 있으면 거절된다.")) return;
  if (!v && !confirm("실물(ROS) 모드로 바꿀까요?\n\n가상 노드는 전부 꺼지고, 이제부터 버튼이 실제 로봇에 간다.")) return;
  await post("/api/mode", { virtual: v });
}
$("#mode-real").onclick = () => setMode(false);
$("#mode-virtual").onclick = () => setMode(true);
let rosBusy = false;
async function rosConnect() {
  if (rosBusy) return;
  rosBusy = true;
  try { await post("/api/ros/connect"); } finally { rosBusy = false; if (S) render(); }
}
const rosLabel = () => (S && S.ros.conn && S.ros.conn.state === "connected") ? "ROS 다시 연결" : "ROS 연결(source)";
$("#btn-ros").onclick = rosConnect;

// ---------- 탭 ----------
const TABS = [["conn", "연결"], ["angle", "각도 제어"], ["ik", "역기구학"]];
const OLD_TABS = { ak: "conn", servo: "angle", finger: "angle", arm: "angle", vision: "ik" };   // 예전 주소
const TAB_IDS = new Set(TABS.map((t) => t[0]));
const TAB_KEY = "jaxdash.tab";
const tabBtns = {};
let curTab = null;
function lsGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
function lsSet(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* 저장 못 해도 화면은 그대로 */ } }
function showTab(id) {
  id = OLD_TABS[id] || id;
  if (!TAB_IDS.has(id)) id = "conn";
  curTab = id;
  for (const [t] of TABS) {
    document.getElementById("page-" + t).hidden = t !== id;
    tabBtns[t].classList.toggle("active", t === id);
    tabBtns[t].setAttribute("aria-selected", t === id ? "true" : "false");
  }
  lsSet(TAB_KEY, id);
  if (S) { initViewers(); renderViewers(); updateIkView(); }
  for (const v of Object.values(VW)) if (v && v.resize) { v.resize(); }
}
(function buildTabs() {
  const nav = $("#tabs");
  for (const [id, label] of TABS) {
    const b = el("a", { class: "tab", href: "#" + id, role: "tab" }, label, el("span", { class: "tdot" }));
    tabBtns[id] = b;
    nav.append(b);
  }
  window.addEventListener("hashchange", () => showTab(location.hash.slice(1)));
  const topH = () => document.documentElement.style.setProperty("--topbar-h", $(".topbar").offsetHeight + "px");
  topH(); window.addEventListener("resize", topH);
})();
function renderTabDots() {
  for (const [id] of TABS) {
    const svcs = S.services.filter((s) => s.tab === id);
    const on = svcs.filter((s) => s.alive).length;
    const dead = svcs.some((s) => !s.alive && s.exit_code != null && s.exit_code !== 0);
    const d = tabBtns[id].querySelector(".tdot");
    d.className = "tdot " + (on ? "on" : dead ? "bad" : "");
    d.textContent = on ? String(on) : "";
  }
}

// ---------- 서버 연결 ----------
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
  try { S = await (await fetch("/api/state")).json(); render(); if (!sock || sock.readyState !== WebSocket.OPEN) chip("chip-ws", "폴링 연결", "warn"); }
  catch (e) { chip("chip-ws", "서버 끊김", "bad"); }
}

// ---------- 그리기 ----------
const fmtDur = (s) => s == null ? "" : s < 60 ? `${s.toFixed(0)}초` : s < 3600 ? `${Math.floor(s / 60)}분 ${Math.floor(s % 60)}초` : `${Math.floor(s / 3600)}시간 ${Math.floor(s % 3600 / 60)}분`;
const fmtAge = (a) => a == null ? "—" : a < 60 ? `${a.toFixed(1)}초 전` : `${fmtDur(a)} 전`;
const LV = ["on", "warn", "bad", "warn"];
const LVT = ["정상", "주의", "에러", "오래됨"];
const kv = (k, v) => [el("span", { class: "k" }, k), el("span", { class: "v" }, v ?? "—")];
const f1 = (x, u = "°") => x == null || !Number.isFinite(x) ? "—" : `${Number(x).toFixed(1)}${u}`;
const f2 = (x, u = "°") => x == null ? "—" : `${Number(x).toFixed(2)}${u}`;
const svcById = (id) => (S.services.find((x) => x.id === id) || {});

function render() {
  if (!S) return;
  if (!curTab) { const h = location.hash.slice(1); showTab(OLD_TABS[h] || (TAB_IDS.has(h) ? h : (lsGet(TAB_KEY) || "conn"))); }
  const v = VIRT(), r = S.ros, cs = (r.conn || {}).state;
  document.body.classList.toggle("virtual", v);
  $("#brand-virtual").hidden = !v;
  $("#virtbar").hidden = !v;
  $("#mode-real").className = v ? "" : "on";
  $("#mode-virtual").className = v ? "on v" : "";
  $("#warnbar").innerHTML = v ? "⚠ (가상) 전부 정지 = 가상 노드 종료. 가상 ak45_node 가 끝나면 가상 팔이 떨어진다 — 실물에서는 <b>홀드가 풀려 팔이 떨어질 수 있다.</b>"
    : "⚠ 전부 정지 = 노드 종료. ak45_node 가 끝나면 AK45 에 브레이크 0A → <b>홀드가 풀려 팔이 떨어질 수 있다.</b> 먼저 팔을 받친다.";
  const sg = $("#suggestbar");
  if (!v && S.mode && !S.mode.ros_possible) {
    if (sg.hidden) { sg.hidden = false; sg.replaceChildren(`이 PC엔 ROS 없음 (${S.mode.ros_setup} 없음) — 실물 모드는 못 쓴다. `,
      el("button", { class: "btn small", onclick: () => setMode(true) }, "가상 모드로 바꾸기")); }
  } else sg.hidden = true;
  if (v) chip("chip-ros", "가상 팔", "on");
  else chip("chip-ros", r.available ? "ROS 구독 중" : cs === "starting" ? "ROS 연결 중" : cs === "dead" ? "ROS 자식 죽음" : cs === "stale" ? "ROS 응답 없음" : "ROS 없음",
    r.available ? "on" : (cs === "dead" || cs === "stale") ? "bad" : cs === "starting" ? "warn" : "");
  const rb = $("#btn-ros"); rb.hidden = v; rb.textContent = rosBusy ? "연결 중…" : rosLabel(); rb.disabled = rosBusy;
  const c = S.can;
  chip("chip-can", c.text || "can0", !c.available ? "" : c.up ? (c.state && c.state !== "ERROR-ACTIVE" ? "warn" : "on") : "bad");
  const running = S.services.filter((s) => s.alive).length;
  chip("chip-run", `실행 ${running}${v ? " (가상)" : ""}`, running ? "on" : "");
  const sb = $("#straybar");
  if (S.strays.length) {
    sb.hidden = false;
    sb.textContent = "대시보드 밖에서 떠 있는 노드(전부 정지로 안 꺼짐): " + S.strays.map((x) => `${x.name} pid ${x.pid}`).join(", ") +
      " — 그 터미널에서 끄거나 pkill -INT -f <이름>";
  } else sb.hidden = true;
  renderTabDots();
  renderConn();
  renderServices();
  renderSteps();
  renderMotors();
  renderCalib();
  renderHoming();
  renderAngles();
  renderPose();
  renderTraj();
  renderViewers();
  $("#foot").textContent = `${v ? "가상 모드 · " : ""}워크스페이스 ${S.info.ws} · ROS ${S.info.distro} · 레포 ${S.info.repo} · 로그 ${S.info.logs} · ${S.info.platform}` +
    ` · 메시 ${S.robot.mesh_ok ? S.robot.mesh_dir : "없음"}`;
}

// ---------- 연결 탭: ROS / 가상 카드 ----------
const C = {};
function renderConn() {
  const box = $("#conn-ros"), v = VIRT();
  const key = v ? "v" : "r";
  if (C.key !== key) {
    C.key = key;
    if (v) {
      C.vnodes = el("div", { class: "kv" }); C.vpower = el("span", { class: "chip" }); C.vmsg = el("div", { class: "note" });
      box.replaceChildren(el("div", { class: "card wide" },
        el("h3", {}, el("span", { class: "vbadge" }, "가상"), "가상 로봇팔", C.vpower),
        el("p", { class: "help" }, "실물 없이 같은 규칙으로 도는 가상 노드(virtual_arm.py): ak45_node(첫 명령 전 안 잡음, 마지막 목표 홀딩, 60 °/s 모터), " +
          "stm_bridge(보낸 각 그대로), arm_control_node(calibration 변환 + max_speed 램프, arm_control 모듈 그대로), homing_guard(homing.py 그대로). " +
          "아래 '노드 켜기/끄기' 로 가상 노드를 켠다(arm_homed.launch.py 하나면 전부)."),
        el("div", { class: "row" },
          el("button", { class: "btn red", onclick: () => { if (confirm("가상 AK45 전원을 끈다: 피드백이 끊기고 가상 팔이 떨어진다.")) post("/api/virtual/power", { on: false }); } }, "가상 전원 끄기"),
          el("button", { class: "btn line", onclick: () => post("/api/virtual/power", { on: true }) }, "가상 전원 켜기 (영점 틀어짐)")),
        el("p", { class: "help" }, "전원을 다시 켜면 엔코더 영점이 15~40° 틀어진다(단일 엔코더 흉내): homing_guard 가 원점을 지우고 그 관절 명령을 막는지 본다."),
        C.vmsg, C.vnodes));
    } else {
      C.rchip = el("span", { class: "chip" }); C.rnote = el("div", { class: "note" }); C.rhelp = el("div", { class: "help" });
      C.rbtn = el("button", { class: "btn", onclick: rosConnect }, "ROS 연결(source)");
      box.replaceChildren(el("div", { class: "card wide" }, el("h3", {}, "ROS 연결", C.rchip),
        el("p", { class: "help" }, "ROS 구독·발행은 ROS 를 source 한 자식 프로세스(ros_monitor_proc.py)에서 돈다. 노드 켜기/끄기는 ROS 연결 없이도 된다."),
        C.rnote, el("div", { class: "row" }, C.rbtn), C.rhelp));
    }
  }
  if (v) {
    const vs = S.ros.virtual_state || {};
    setChip(C.vpower, vs.power ? "AK45 전원 켜짐" : "AK45 전원 꺼짐", vs.power ? "on" : "bad");
    $("#conn-note").textContent = "· 가상";
    C.vmsg.textContent = `켜진 가상 노드: ${(vs.nodes || []).join(", ") || "없음"}`;
    const rows = Object.entries(vs.phys_deg || {}).map(([n, p]) => kv(`${n} 실제 축 / 엔코더 영점 어긋남`, `${p.toFixed(1)}° / ${(vs.enc_off_deg || {})[n]?.toFixed(1)}°`));
    C.vnodes.replaceChildren(...rows.flat());
  } else {
    const r = S.ros, c = r.conn || {};
    setChip(C.rchip, r.available ? "연결됨" : c.state === "starting" ? "연결 중" : "없음", r.available ? "on" : c.state === "starting" ? "warn" : "bad");
    $("#conn-note").textContent = "";
    C.rnote.textContent = r.available ? `ROS ${c.distro} · 자식 pid ${c.pid}` : (r.reason || "ROS 없음");
    C.rnote.className = "note " + (r.available ? "" : ["dead", "stale"].includes(c.state) ? "bad" : "warn");
    C.rbtn.textContent = rosBusy ? "연결 중…" : rosLabel(); C.rbtn.disabled = rosBusy;
    C.rhelp.textContent = `source ${c.setup || "?"} (+ ${c.ws_setup || "?"} 있으면) · 로그 ${c.log || ""}`;
  }
}
function rosMissing(box) {
  const c = S.ros.conn || {}, key = [S.ros.reason, c.state, c.pid, rosBusy].join("|");
  if (box.dataset.rosKey === key) return;
  box.dataset.rosKey = key;
  box.replaceChildren(el("div", { class: "card muted" },
    el("div", { class: "note " + (["dead", "stale"].includes(c.state) ? "bad" : "") }, S.ros.reason || "ROS 없음"),
    el("div", { class: "help" }, "위 'ROS 연결' 카드 또는 화면 위 '가상' 스위치.")));
}

// ---------- 연결 탭: 순서대로 켜기 (① can0 ② ROS ③ 팔) ----------
const ST = {};
function svcState(s) {
  if (!s) return ["없음", ""];
  if (!s.available) return ["못 씀", ""];
  if (s.stopping) return ["끄는 중", "warn"];
  if (s.alive) return [s.kind === "once" ? "실행 중" : "켜짐", "on"];
  if (s.exit_code != null && s.exit_code !== 0) return [`죽음 (코드 ${s.exit_code})`, "bad"];
  return ["꺼짐", ""];
}
function svcParams(id) {
  const c = cards[id];
  return c ? Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.value])) : {};
}
function renderSteps() {
  const box = $("#conn-steps"), v = VIRT();
  if (!ST.built) {
    ST.built = true;
    const step = (n, title) => {
      const o = { chip: el("span", { class: "chip" }), note: el("div", { class: "note" }), row: el("div", { class: "row" }) };
      o.root = el("div", { class: "card step" }, el("h3", {}, el("span", { class: "num" }, n), title, o.chip), o.note, o.row);
      return o;
    };
    ST.can = step("1", "CAN 켜기");
    ST.can.b = el("button", { class: "btn", onclick: () => post("/api/svc/can_up/start", { params: {} }) }, "can0 올리기");
    ST.can.row.append(ST.can.b);
    ST.ros = step("2", "ROS 연결");
    ST.ros.b = el("button", { class: "btn", onclick: rosConnect }, "연결");
    ST.ros.row.append(ST.ros.b);
    ST.arm = step("3", "팔 켜기");
    ST.arm.on = el("button", { class: "btn", onclick: () => post("/api/svc/arm_homed/start", { params: svcParams("arm_homed") }) }, "▶ 켜기");
    ST.arm.off = el("button", { class: "btn red", onclick: () => {
      const s = S.services.find((x) => x.id === "arm_homed");
      if (s && s.stop_warn && !VIRT() && !confirm(s.stop_warn)) return;
      post("/api/svc/arm_homed/stop");
    } }, "■ 끄기");
    ST.arm.row.append(ST.arm.on, ST.arm.off);
    box.append(ST.can.root, ST.ros.root, ST.arm.root);
  }
  const can = S.can, canSvc = S.services.find((x) => x.id === "can_up");
  if (v) { setChip(ST.can.chip, "가상", "on"); ST.can.note.textContent = "가상 모드에선 필요 없음"; ST.can.b.disabled = true; }
  else {
    setChip(ST.can.chip, !can.available ? "못 봄" : can.up ? "UP" : "꺼짐", !can.available ? "" : can.up ? "on" : "bad");
    ST.can.note.textContent = !can.available ? (canSvc && !canSvc.available ? canSvc.why : "can0 상태를 못 읽음") : can.up ? (can.state || "") : "재부팅마다 1번";
    ST.can.b.disabled = !canSvc || !canSvc.available || canSvc.alive || can.up;
  }
  const r = S.ros, cs = (r.conn || {}).state;
  if (v) { setChip(ST.ros.chip, "가상 팔", "on"); ST.ros.note.textContent = "가상 모드에선 필요 없음"; ST.ros.b.disabled = true; }
  else {
    setChip(ST.ros.chip, r.available ? "연결됨" : cs === "starting" ? "연결 중" : "안 됨", r.available ? "on" : cs === "starting" ? "warn" : "bad");
    ST.ros.note.textContent = r.available ? `ROS ${(r.conn || {}).distro || ""}` : (r.reason || "ROS 없음");
    ST.ros.b.textContent = rosBusy ? "연결 중…" : r.available ? "다시 연결" : "연결"; ST.ros.b.disabled = rosBusy;
  }
  const arm = S.services.find((x) => x.id === "arm_homed"), [t, cls] = svcState(arm);
  setChip(ST.arm.chip, (arm && arm.virtual ? "가상 · " : "") + t, cls);
  ST.arm.note.textContent = !arm ? "" : !arm.available ? arm.why : arm.error || "ak45 · 서보 · 원점 guard · arm_control 한 번에 (arm_homed.launch.py)";
  ST.arm.on.disabled = !arm || !arm.available || arm.alive || arm.stopping;
  ST.arm.off.disabled = !arm || !arm.alive || arm.stopping;
}

// ---------- AK45 (한 줄씩) ----------
function renderMotors() {
  const r = S.ros, box = $("#motors");
  if (!r.available) { $("#ros-note").textContent = ""; rosMissing(box); return; }
  delete box.dataset.rosKey;
  box.replaceChildren();
  $("#ros-note").textContent = (VIRT() ? "(가상) " : "") + (r.diag_age == null ? "ak45_node 응답 없음 (팔 꺼짐?)" : `${r.diag_age.toFixed(1)}초 전`);
  if (!r.motors.length) { box.append(el("div", { class: "card muted" }, "팔을 켜면 AK45 4개가 여기 나온다")); return; }
  const rows = r.motors.map((m) => {
    const v = m.values || {}, stale = m.age > 3, lv = stale ? 3 : m.level;
    return el("tr", {},
      el("td", {}, el("b", {}, m.name.replace("ak45/", ""))),
      el("td", {}, el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv])),
      el("td", { class: "num" }, v.position_deg != null ? `${v.position_deg}°` : "—"),
      el("td", { class: "num" }, `${v.temperature_c ?? "—"} °C`),
      el("td", {}, v.command_active === "1" ? "홀드 중" : "힘 없음"),
      el("td", { class: "note " + (lv === 2 ? "bad" : lv ? "warn" : "") }, stale ? "응답 끊김" : lv ? m.message : ""));
  });
  let bus = null;
  if (r.bus) {
    const lv = r.bus.age > 3 ? 3 : r.bus.level;
    bus = el("div", { class: "note" }, "CAN 버스 ", el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv]), " ", lv ? r.bus.message : "");
  }
  box.append(el("div", { class: "card mtable" }, el("table", {},
    el("thead", {}, el("tr", {}, ...["모터", "상태", "위치", "온도", "힘", ""].map((h) => el("th", {}, h)))),
    el("tbody", {}, ...rows)), bus || ""));
}

// ---------- AK45 진단 (예전 카드, 안 씀) ----------
function renderMotorsFull() {
  const r = S.ros, box = $("#motors");
  if (!r.available) { $("#ros-note").textContent = ""; rosMissing(box); return; }
  delete box.dataset.rosKey;
  box.replaceChildren();
  $("#ros-note").textContent = (VIRT() ? "(가상) " : "") + (r.diag_age == null ? "/diagnostics 아직 없음 (ak45_node 꺼짐?)" :
    `/diagnostics ${r.diag_age.toFixed(1)}초 전 · /joint_states ${r.joint_age == null ? "없음" : r.joint_age.toFixed(1) + "초 전"}`);
  if (!r.motors.length) { box.append(el("div", { class: "card muted" }, "ak45_node 진단을 기다리는 중")); return; }
  for (const m of r.motors) {
    const v = m.values || {}, stale = m.age > 3;
    const lv = stale ? 3 : m.level;
    box.append(el("div", { class: "card" },
      el("h3", {}, vb(), m.name.replace("ak45/", ""), el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv]),
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
      el("h3", {}, vb(), "CAN 버스", el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv])),
      el("div", { class: "big" }, v.state || "—"),
      el("div", { class: "note " + (lv === 2 ? "bad" : lv ? "warn" : "") }, r.bus.message),
      el("div", { class: "kv" }, kv("에러 프레임", v.err_frames), kv("bit-stuff", v.bit_stuff_errors), kv("form", v.form_errors),
        kv("no-ack", v.ack_errors), kv("버스오프", v.busoff_count), kv("재시작", v.restart_count))));
  }
}

// ---------- 노드 카드 (한 번 만들고 바뀌는 부분만) ----------
const cards = {};
function renderServices() {
  for (const s of S.services) {
    let c = cards[s.id];
    if (!c) {
      c = cards[s.id] = buildService(s);
      (document.getElementById("svc-" + s.tab) || $("#svc-conn")).append(c.root);
    }
    updateService(c, s);
  }
}
function buildService(s) {
  const c = { fields: {} };
  c.chip = el("span", { class: "chip" });
  c.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  c.status = el("div", { class: "note" });
  c.note = el("div", { class: "note" });
  c.cmd = el("div", { class: "cmd", hidden: true });
  c.log = el("pre", { class: "logbox" + (s.tab === "ik" ? " tall" : "") });
  c.logbox = el("details", { class: "log" },
    el("summary", {}, s.kind === "once" ? `출력 (마지막 실행, 최대 ${s.tail}줄)` : `로그 (마지막 ${s.tail}줄)`), c.log,
    el("div", { class: "row" }, el("a", { href: `/api/log/${s.id}?n=1000`, target: "_blank", class: "help" }, "로그 1000줄 보기")));
  const fields = s.params.map((p) => {
    const inp = el("input", { value: p.value, placeholder: p.hint || "" });
    c.fields[p.key] = inp;
    return el("label", { class: "field" }, `${p.label}${p.hint ? " (" + p.hint + ")" : ""}`, inp);
  });
  const params = () => Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.value]));
  const once = s.kind === "once";
  const warn = () => s.stop_warn && !VIRT() && !confirm(s.stop_warn);
  c.bStart = el("button", { class: "btn", onclick: () => post(`/api/svc/${s.id}/start`, { params: params() }) }, once ? "▶ 실행" : "▶ 켜기");
  c.bStop = el("button", { class: "btn red", onclick: () => { if (warn()) return; post(`/api/svc/${s.id}/stop`); } }, once ? "■ 중단" : "■ 끄기");
  c.bRestart = once ? null : el("button", { class: "btn line", onclick: () => { if (warn()) return; post(`/api/svc/${s.id}/restart`, { params: params() }); } }, "↻ 재시작");
  c.root = el("div", { class: "card" + (s.tab === "ik" ? " wide" : "") },
    el("h3", {}, c.vb, s.label, c.chip), el("p", { class: "help" }, `[${s.group}] ${s.help}`),
    fields, el("div", { class: "row" }, c.bStart, c.bStop, c.bRestart), c.status, c.note, c.cmd, c.logbox);
  return c;
}
function updateService(c, s) {
  let cls = "", txt = "꺼짐";
  if (!s.available) { txt = "못 씀"; }
  else if (s.stopping) { cls = "warn"; txt = "끄는 중"; }
  else if (s.alive) { cls = "on"; txt = s.kind === "once" ? "실행 중" : "켜짐"; }
  else if (s.exit_code != null) { cls = s.exit_code === 0 ? "" : "bad"; txt = s.exit_code === 0 ? "끝남 (0)" : `죽음 (코드 ${s.exit_code})`; }
  setChip(c.chip, txt, cls);
  c.vb.hidden = !s.virtual;
  c.bStart.disabled = !s.available || s.alive || s.stopping;
  c.bStop.disabled = !s.alive || s.stopping;
  if (c.bRestart) c.bRestart.disabled = !s.available || s.stopping;
  for (const inp of Object.values(c.fields)) inp.disabled = s.alive;
  let st = "";
  if (!s.available) st = s.why;
  else if (s.alive) st = s.virtual ? `가상 노드 · 가동 ${fmtDur(s.uptime)}` : `pid ${s.pid} · 가동 ${fmtDur(s.uptime)}${s.adopted ? " · 이전 대시보드에서 켠 것(다시 붙잡음)" : ""}`;
  else if (s.ended) st = `끝난 시각 ${new Date(s.ended * 1000).toLocaleTimeString()}`;
  c.status.textContent = st;
  const note = s.error || s.stop_note;
  c.note.textContent = note;
  c.note.className = "note " + (s.error || /SIGKILL|살아 있음/.test(s.stop_note) ? "bad" : /SIGTERM/.test(s.stop_note) ? "warn" : "");
  c.cmd.hidden = !s.cmd; c.cmd.textContent = s.cmd ? "$ " + (s.cmd.length > 400 ? s.cmd.slice(0, 400) + " …" : s.cmd) : "";
  const atBottom = c.log.scrollTop + c.log.clientHeight >= c.log.scrollHeight - 4;
  let lines = s.log;
  if (s.kind === "once") {
    let k = -1;
    for (let i = lines.length - 1; i >= 0; i--) if (/^===== .* 켜기: /.test(lines[i])) { k = i; break; }
    if (k >= 0) lines = lines.slice(k);
  }
  const text = lines.join("\n");
  if (c.log.textContent !== text) { c.log.textContent = text; if (atBottom) c.log.scrollTop = c.log.scrollHeight; }
  if ((s.alive || (s.tab === "ik" && text)) && !c.opened) { c.logbox.open = true; c.opened = true; }
}

// ---------- 보정 안내 ----------
let calibBuilt = false;
function renderCalib() {
  const g = S.calibrate, box = $("#calib");
  const on = (id) => svcById(id).alive;
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
  const bad = g.forbid.filter(on).map((id) => svcById(id).label);
  const miss = g.needs.filter((id) => !on(id)).map((id) => svcById(id).label || id);
  const n = $("#calib-check");
  n.className = "note " + (bad.length ? "bad" : miss.length ? "warn" : "");
  n.textContent = (VIRT() ? "(가상 모드: calibrate 는 실물 전용) " : "") + (bad.length ? `끄세요: ${bad.join(", ")} (calibrate 와 같은 모터에 명령)` :
    miss.length ? `아직 안 켬: ${miss.join(", ")}` : "준비됨: 터미널에서 calibrate --ros 실행");
}

// ---------- 원점 (homing_guard) ----------
const H = { rows: {} };
function homeAsk(joint) {
  const poses = S.home_poses || {};
  const pose = joint === "all" ? Object.entries(poses).map(([k, v]) => `${k} ${v}°`).join(", ") || "전부 sim 0°" : `${joint} sim ${poses[joint] ?? 0}°`;
  return `${VIRT() ? "(가상) " : ""}${joint === "all" ? "원점 없는 관절 전부" : joint}: 관절을 기준 자세(homing.json home_sim_deg: ${pose})에 두었나요?\n` +
    "지금 위치를 그 자세로 삼는다. 다른 자세에서 누르면 그만큼 틀어진 원점이 된다.";
}
const RELEASE_HELP = "쉬게 하기: guard 가 이 관절 명령 전달만 끊고 원점을 지운다. ak45_node 는 마지막 target 을 계속 잡고 있다 " +
  "(실제로 힘을 빼려면 ak45_node 종료 — 팔이 떨어질 수 있음).";
async function homingDo(action, joint) {
  if (action === "home" && !confirm(homeAsk(joint))) return;
  if (action === "release" && !confirm(`${joint === "all" ? "전부" : joint} 쉬게 하기?\n\n${RELEASE_HELP}`)) return;
  await post(`/api/homing/${action}`, { joint });
}
function buildHoming() {
  H.msg = el("div", { class: "note" });
  H.bHomeAll = el("button", { class: "btn", onclick: () => homingDo("home", "all") }, "전부 원점");
  H.bRelAll = el("button", { class: "btn line", onclick: () => homingDo("release", "all") }, "전부 쉬게");
  H.list = el("div", { class: "hlist" });
  H.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  H.card = el("div", { class: "card" },
    el("h3", {}, H.vb, "AK45 원점", H.chip = el("span", { class: "chip" })),
    el("p", { class: "help" }, "homing_guard(arm_homed.launch.py)가 원점 잡기 전·전원 재투입 뒤·홀드가 끝난 뒤에는 AK45 명령을 막는다. " +
      "관절을 기준 자세에 두고 '원점 잡기'. " + RELEASE_HELP),
    el("div", { class: "row" }, H.bHomeAll, H.bRelAll), H.msg, H.list);
  $("#homing").append(H.card);
}
function homingRow(name) {
  const r = {};
  r.badge = el("span", { class: "chip" });
  r.reason = el("div", { class: "note" });
  r.kv = el("div", { class: "kv" });
  r.more = el("details", { class: "log" }, el("summary", {}, "자세히"), r.kv);
  r.bHome = el("button", { class: "btn", onclick: () => homingDo("home", name) }, "원점 잡기");
  r.bRel = el("button", { class: "btn line", onclick: () => homingDo("release", name) }, "쉬게 하기");
  r.root = el("div", { class: "hrow" }, el("div", { class: "hhead" }, el("b", {}, name), r.badge, el("span", { class: "spacer" }), r.bHome, r.bRel), r.reason, r.more);
  return r;
}
function renderHoming() {
  if (!H.card) buildHoming();
  H.vb.hidden = !VIRT();
  const h = (S.ros && S.ros.homing) || { available: false };
  const guardOn = h.available && h.seen && h.age <= 3;
  let chipCls = "", chipTxt = "guard 없음", msg = "", msgCls = "warn";
  if (!h.available) { msg = `${h.reason || "ROS 없음"} — 연결 탭에서 ROS 연결(또는 가상 모드).`; msgCls = ""; chipTxt = "ROS 없음"; }
  else if (!h.seen) msg = "homing_guard 가 안 켜짐: 연결 탭에서 arm_homed.launch.py 를 켜세요.";
  else if (h.age > 3) { msg = `원점 상태가 ${h.age.toFixed(0)}초 전 것 (homing_guard 꺼짐?). 연결 탭에서 arm_homed.launch.py 를 켜세요.`; chipTxt = "오래됨"; chipCls = "warn"; }
  else if (h.all_homed) { chipTxt = "전부 원점"; chipCls = "on"; }
  else { chipTxt = `원점 ${h.joints.filter((j) => j.homed).length}/${h.joints.length}`; chipCls = "bad"; }
  setChip(H.chip, chipTxt, chipCls);
  H.msg.textContent = msg; H.msg.className = "note " + (msg ? msgCls : "");
  $("#homing-note").textContent = guardOn ? `/arm_control/homing ${h.age.toFixed(1)}초 전` : "";
  H.bHomeAll.disabled = H.bRelAll.disabled = !guardOn;
  const joints = h.seen ? h.joints : Object.keys(S.home_poses || {}).map((j) => ({ joint: j }));
  const seen = new Set(joints.map((j) => j.joint));
  for (const [n, r] of Object.entries(H.rows)) if (!seen.has(n)) { r.root.remove(); delete H.rows[n]; }
  for (const j of joints) {
    let r = H.rows[j.joint];
    if (!r) { r = H.rows[j.joint] = homingRow(j.joint); H.list.append(r.root); }
    r.bHome.disabled = r.bRel.disabled = !guardOn;
    if (!guardOn || j.homed == null) { setChip(r.badge, "—", ""); r.reason.textContent = ""; r.kv.replaceChildren(); continue; }
    setChip(r.badge, j.homed ? "✅ 원점" : "⛔ 원점 없음", j.homed ? "on" : "bad");
    r.reason.textContent = j.reason || "";
    r.reason.className = "note " + (j.homed ? "" : "bad");
    const age = j.feedback_lost ? "끊김" : j.feedback_age_s == null ? "없음" : `${j.feedback_age_s.toFixed(2)}초`;
    r.kv.replaceChildren(...[
      kv("모터", j.motor), kv("기준 자세", `sim ${j.home_sim_deg ?? 0}°`),
      kv("오프셋", j.homed ? f2(j.offset_deg) : "—"), kv("세션 zero", f2(j.session_zero_deg)),
      kv("피드백", `${f2(j.raw_deg)} · ${age}`), kv("command_active", j.command_active == null ? "—" : j.command_active ? "예" : "아니오"),
    ].flat());
  }
}

// ---------- 각도 제어: 관절 14개 슬라이더 ----------
// 값 = 시뮬 관절각(°). 범위 = calibration.json sim_range_deg ∩ 모터 범위(서버 cmd_range_deg). 보내기 버튼으로만 나간다.
const A = { rows: {}, val: {}, base: {}, sent: {}, confirmed: false, link: false };
const WRIST = ["wrist_pitch", "wrist_yaw", "hand_roll"];
const FINGER_SETS = [["손가락 1", ["finger1_joint1", "finger1_joint2"]], ["손가락 2", ["finger2_joint1", "finger2_joint2"]],
  ["손가락 3", ["finger3_joint1", "finger3_joint2"]], ["손가락 회전", ["finger_rotation"]]];
const calJ = () => Object.fromEntries(((S.calibration || {}).joints || []).map((j) => [j.joint, j]));
const clampR = (j, v) => { const r = j.cmd_range_deg; return r ? Math.min(r[1], Math.max(r[0], v)) : v; };
function confirmFirst() {
  if (A.confirmed) return true;
  const unver = ((S.calibration || {}).joints || []).filter((j) => !j.verified).map((j) => j.joint);
  const sp = (svcById("arm_homed").alive ? svcById("arm_homed") : svcById("arm_launch").alive ? svcById("arm_launch") : svcById("arm_control"));
  const speed = ((sp.params || []).find((p) => p.key === "max_speed_deg_s") || {}).value || "?";
  const ok = confirm((VIRT() ? "(가상 팔) " : "⚠ 실제 모터가 움직인다.\n\n") +
    `calibration verified=false (${unver.length}개 관절: ${unver.join(", ") || "없음"}) — 방향·영점이 아직 가정이다.\n` +
    "팔 주변을 비우고, 팔을 받칠 준비를 한다.\n\n" +
    `arm_control_node 가 max_speed_deg_s ${speed} °/s 로 천천히 보낸다. 서보는 첫 명령에 램프 없이 바로 간다.\n` +
    "멈추려면 위 ■ 전부 정지(Esc) — 노드가 꺼져 팔이 떨어질 수 있다.\n\n보낼까요?");
  if (ok) A.confirmed = true;
  return ok;
}
async function sendJoints(names) {
  const g = S.gate || {}, cal = calJ();
  const ok = names.filter((n) => (g.joints[n] || {}).ok);
  const skip = names.filter((n) => !(g.joints[n] || {}).ok);
  if (!ok.length) { toast("보낼 수 있는 관절이 없음: " + (skip.map((n) => `${n} (${(g.joints[n] || {}).reason || g.reason})`).join(", ") || "바뀐 것 없음"), "error"); return; }
  if (!confirmFirst()) return;
  const joints = Object.fromEntries(ok.map((n) => [n, Math.round(clampR(cal[n], A.val[n]) * 100) / 100]));
  const r = await post("/api/joint/send", { joints });
  if (r.ok) for (const n of ok) { A.base[n] = A.val[n]; A.sent[n] = joints[n]; }
  if (skip.length) toast("안 보낸 관절: " + skip.map((n) => `${n} (${(g.joints[n] || {}).reason})`).join(", "), "error");
}
function syncToActual() {
  let k = 0;
  for (const [n, j] of Object.entries(calJ())) {
    const p = (S.pose || {})[n];
    const v = p ? p.deg : A.sent[n];
    if (v == null) continue;
    setVal(n, clampR(j, v), true);
    k++;
  }
  toast(k ? `현재 자세로 ${k}개 관절 맞춤 (AK45 = 피드백, 서보 = 마지막으로 보낸 값)` : "현재 자세 값이 없음 (피드백·보낸 값 없음)");
}
function setVal(n, v, asBase) {
  A.val[n] = v;
  if (asBase) A.base[n] = v;
  const r = A.rows[n];
  if (r) { r.range.value = v; r.num.value = (Math.round(v * 10) / 10).toString(); }
}
function onUser(n, v) {
  const cal = calJ(), j = cal[n];
  if (!Number.isFinite(v)) return;
  v = clampR(j, v);
  setVal(n, v);
  if (A.link && /^finger\d_joint[12]$/.test(n)) {   // 손가락 1·2·3 같이: 같은 번호 관절을 같은 값으로
    const idx = n.slice(-1);
    for (const f of [1, 2, 3]) { const m = `finger${f}_joint${idx}`; if (m !== n && cal[m]) setVal(m, clampR(cal[m], v)); }
  }
  renderAngles();
}
function angleRow(j) {
  const r = {}, rg = j.cmd_range_deg || [0, 0];
  const motor = j.motor === "ak45" ? `AK45 ID ${j.id}` : `서보 ${j.name} · ID ${j.id}`;
  r.range = el("input", { type: "range", min: rg[0], max: rg[1], step: 0.5, value: 0 });
  r.num = el("input", { type: "number", min: rg[0], max: rg[1], step: 0.5, value: 0 });
  r.range.addEventListener("input", () => onUser(j.joint, parseFloat(r.range.value)));
  r.num.addEventListener("change", () => onUser(j.joint, parseFloat(r.num.value)));
  r.send = el("button", { class: "btn", onclick: () => sendJoints([j.joint]) }, "보내기");
  r.cur = el("span", {}); r.sentT = el("span", {}); r.why = el("span", {});
  r.root = el("div", { class: "jrow" },
    el("div", { class: "jn" }, j.joint, el("small", {}, `${motor} · ${rg[0]}~${rg[1]}°${j.verified ? "" : " · 미확인"}`)),
    r.range, r.num, r.send, el("div", { class: "jinfo" }, r.cur, r.sentT, r.why));
  return r;
}
function buildAngles() {
  const cal = calJ(), box = $("#angles");
  if (!Object.keys(cal).length) return false;
  A.gate = el("div", { class: "gate" });
  A.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  A.bAll = el("button", { class: "btn", onclick: () => sendJoints(Object.keys(cal).filter((n) => Math.abs((A.val[n] ?? 0) - (A.base[n] ?? 0)) > 1e-9)) }, "바뀐 것 전부 보내기");
  A.bSync = el("button", { class: "btn line", onclick: syncToActual }, "현재 자세로 슬라이더 맞추기");
  A.bReset = el("button", { class: "btn line", onclick: () => { for (const n of Object.keys(cal)) setVal(n, A.base[n] ?? 0); renderAngles(); } }, "바꾼 것 되돌리기");
  const linkBox = el("input", { type: "checkbox" });
  linkBox.addEventListener("change", () => { A.link = linkBox.checked; });
  const grp = (title, names, extra) => {
    const card = el("div", { class: "card jgroup" }, el("h3", {}, title), extra || null);
    for (const n of names) if (cal[n]) { const r = A.rows[n] = angleRow(cal[n]); card.append(r.root); }
    return card;
  };
  const ak = Object.values(cal).filter((j) => j.motor === "ak45").map((j) => j.joint);
  const fingers = el("div", { class: "card jgroup" }, el("h3", {}, "손가락 (서보 7개)"),
    el("label", { class: "help", style: "display:flex;gap:6px;align-items:center" }, linkBox, "손가락 1·2·3 같이 움직이기 (joint1 끼리, joint2 끼리)"));
  for (const [label, names] of FINGER_SETS) {
    fingers.append(el("div", { class: "help", style: "margin:8px 0 0;font-weight:700;color:#1f2a44" }, label));
    for (const n of names) if (cal[n]) { const r = A.rows[n] = angleRow(cal[n]); fingers.append(r.root); }
  }
  box.append(el("div", { class: "card jgroup" },
    el("h3", {}, A.vb, "보내기"),
    el("p", { class: "help" }, "슬라이더는 3D 의 파란 목표만 바꾼다. 모터로 가는 것은 '보내기' 뿐: /arm_control/joint_command (sim 관절명, rad) → " +
      "arm_control_node(calibration 변환·클립, max_speed 램프) → arm_homed 면 homing_guard(원점 검사) → ak45_node / stm_bridge_node."),
    A.gate, el("div", { class: "row" }, A.bAll, A.bSync, A.bReset)),
    grp("AK45 (어깨·팔꿈치)", ak), grp("손목 (서보)", WRIST), fingers);
  for (const [n, j] of Object.entries(cal)) setVal(n, clampR(j, (S.pose || {})[n] ? S.pose[n].deg : 0), true);
  return true;
}
function renderAngles() {
  if (!A.built) A.built = buildAngles();
  if (!A.built) return;
  const g = S.gate || { joints: {} }, cal = calJ();
  A.vb.hidden = !VIRT();
  A.gate.className = "gate " + (g.ok ? "on" : "bad");
  A.gate.textContent = g.ok ? `${VIRT() ? "(가상) " : ""}보낼 수 있음 — arm_control_node 구독 중${g.homed_mode ? " · homing_guard 사용 중(원점 없는 AK45 는 막힘)" : " · homing_guard 없음(arm.launch / arm_control: AK45 원점 검사 없음)"}`
    : `보내기 꺼짐: ${g.reason}`;
  let dirty = 0;
  for (const [n, r] of Object.entries(A.rows)) {
    const jg = g.joints[n] || {}, p = (S.pose || {})[n];
    const isDirty = Math.abs((A.val[n] ?? 0) - (A.base[n] ?? 0)) > 1e-9;
    if (isDirty) dirty++;
    r.root.classList.toggle("dirty", isDirty);
    r.send.disabled = !jg.ok;
    r.send.title = jg.ok ? "이 관절만 보내기" : jg.reason || "";
    r.cur.textContent = p ? `현재 ${f1(p.deg)} (${p.src}${p.age > 3 ? ", " + fmtAge(p.age) : ""})` : "현재 —";
    r.sentT.textContent = A.sent[n] != null ? `보낸 값 ${f1(A.sent[n])}` : "";
    r.why.textContent = !jg.ok ? (jg.reason || "") : jg.note || "";
    r.why.className = !jg.ok ? "bad" : jg.note ? "warn" : "";
    if (cal[n] && cal[n].note && !r.noted) { r.noted = true; r.root.title = cal[n].note; }
  }
  A.bAll.disabled = !g.ok || !dirty;
  A.bAll.textContent = dirty ? `바뀐 것 ${dirty}개 보내기` : "바뀐 것 전부 보내기";
}

// ---------- 역기구학 탭: 비전 / 가상 큐브 ----------
const P = {};
function renderPose() {
  const r = S.ros, box = $("#objpose");
  if (!P.root) {
    P.chip = el("span", { class: "chip" }); P.vchip = el("span", { class: "chip" }); P.big = el("div", { class: "big", style: "font-size:18px" });
    P.note = el("div", { class: "note" });
    P.bUse = el("button", { class: "btn line", onclick: usePose }, "/object_pose 값을 cube 칸에 넣기");
    P.vin = {};
    const fld = (k, v, lab) => { const i = el("input", { value: v }); P.vin[k] = i; return el("label", { class: "field" }, lab, i); };
    P.virt = el("div", { hidden: true },
      el("div", { class: "help", style: "margin-top:8px" }, el("span", { class: "vbadge" }, "가상"), " 가상 큐브: 가상 vision_node 가 이 값을 /object_pose 로 1 Hz 발행"),
      el("div", { class: "minirow" }, fld("x", "0.10", "x m"), fld("y", "-0.30", "y m"), fld("z", "-0.4525", "z m"), fld("yaw", "20", "yaw °"),
        el("button", { class: "btn line", onclick: () => post("/api/virtual/cube", Object.fromEntries(Object.entries(P.vin).map(([k, i]) => [k, parseFloat(i.value)]))) }, "가상 큐브 놓기")));
    P.root = el("div", { class: "card" }, el("h3", {}, "vision_node", P.vchip, "/object_pose", P.chip), P.big, P.note,
      el("div", { class: "row" }, P.bUse), P.virt,
      el("p", { class: "help" }, "frame shoulder_roll_axis = 로봇 좌표(pick_place --cube 와 같은 틀). camera = 카메라 좌표 그대로라 cube 칸에 못 넣음. vision_node 켜기/끄기는 연결 탭."));
    box.replaceChildren(P.root);
  }
  const vs = svcById("vision");
  setChip(P.vchip, vs.alive ? "켜짐" : "꺼짐", vs.alive ? "on" : "");
  P.virt.hidden = !VIRT();
  const o = r.available ? r.object_pose : null;
  $("#pose-note").textContent = !r.available ? "· ROS 없음" : o ? `· ${fmtAge(o.age)} 받음` : "· 아직 없음";
  const stale = o && o.age > 3;
  setChip(P.chip, !o ? "없음" : stale ? "오래됨" : "받는 중", !o ? "" : stale ? "warn" : "on");
  P.big.textContent = o ? `x ${o.x.toFixed(3)} · y ${o.y.toFixed(3)} · z ${o.z.toFixed(3)} · yaw ${o.yaw_deg.toFixed(1)}° · ${o.frame_id || "(빈 frame)"}` : "—";
  P.note.textContent = !r.available ? (r.reason || "ROS 없음") : !o ? "vision_node 가 안 켜졌거나 아직 발행 전 (1 Hz)" : stale ? `${o.age.toFixed(0)}초 전 값 (vision_node 꺼짐?)` : "";
  P.note.className = "note " + (stale ? "warn" : "");
  P.bUse.disabled = !o || o.frame_id !== "shoulder_roll_axis";
}
function usePose() {
  const o = S.ros.object_pose, c = cards.pick_dry;
  if (!o || !c) return;
  const set = (k, v) => { if (c.fields[k] && !c.fields[k].disabled) c.fields[k].value = v; };
  set("cube_x", o.x.toFixed(3)); set("cube_y", o.y.toFixed(3)); set("cube_yaw", o.yaw_deg.toFixed(1));
  toast("cube 칸을 채웠다. ▶ 실행(--dry-run)");
}

// ---------- 역기구학 탭: 궤적 재생 ----------
const T = { traj: null, mtime: undefined, frame: 0, playing: false, speed: 1, view: "traj" };
async function loadTraj() {
  try {
    const d = await (await fetch("/api/ik/trajectory")).json();
    T.info = d;
    T.traj = d.ok ? d.traj : null;
    T.frame = 0; T.playing = false;
    if (T.slider) { T.slider.max = T.traj ? T.traj.positions.length - 1 : 0; T.slider.value = 0; }
  } catch (e) { T.info = { ok: false, error: "궤적을 못 받음" }; }
}
function trajPose(k) {
  const t = T.traj; if (!t) return null;
  const f = t.positions[Math.max(0, Math.min(t.positions.length - 1, Math.round(k)))];
  return Object.fromEntries(t.joint_names.map((n, i) => [n, f[i]]));
}
function buildTraj() {
  const box = $("#traj-ctl");
  T.status = el("div", { class: "note" });
  T.bPlay = el("button", { class: "btn", onclick: () => { if (!T.traj) return; if (T.frame >= T.traj.positions.length - 1) T.frame = 0; T.playing = !T.playing; T.last = performance.now(); } }, "▶ 재생");
  T.slider = el("input", { type: "range", min: 0, max: 0, step: 1, value: 0 });
  T.slider.addEventListener("input", () => { T.frame = parseFloat(T.slider.value); T.playing = false; });
  T.num = el("span", { class: "num" });
  const sp = el("select", {}, ...[1, 2, 5, 10].map((x) => el("option", { value: x }, `${x}×`)));
  sp.addEventListener("change", () => { T.speed = parseFloat(sp.value); });
  const vsel = el("select", {}, el("option", { value: "traj" }, "궤적 프레임 보기"), el("option", { value: "actual" }, "실제 현재 자세 보기"));
  vsel.addEventListener("change", () => { T.view = vsel.value; });
  T.vplay = el("button", { class: "btn", onclick: async () => {
    if (!confirm("가상 팔로 이 궤적을 재생할까요? (가상 arm_control_node 로 프레임을 보낸다. 실물과 상관없음)")) return;
    vsel.value = T.view = "actual"; await post("/api/ik/play_virtual");
  } }, "가상 팔로 재생");
  T.vstop = el("button", { class: "btn line", onclick: () => post("/api/ik/stop_virtual") }, "재생 멈춤");
  T.vrow = el("div", { class: "tbar", hidden: true }, el("span", { class: "vbadge" }, "가상"), T.vplay, T.vstop, T.vinfo = el("span", { class: "num" }));
  box.append(T.status, el("div", { class: "tbar" }, T.bPlay, T.slider, T.num), el("div", { class: "tbar" }, "속도", sp, vsel), T.vrow);
  setInterval(stepTraj, 50);
}
function stepTraj() {
  if (!T.playing || !T.traj) return;
  const now = performance.now(), dt = (now - (T.last || now)) / 1000; T.last = now;
  T.frame = Math.min(T.traj.positions.length - 1, T.frame + dt * T.speed / T.traj.dt);
  if (T.frame >= T.traj.positions.length - 1) T.playing = false;
  T.slider.value = Math.round(T.frame);
  updateIkView();
}
function trajScene() {
  // 큐브·놓을 자리: 궤적의 real(계획에 쓴 값), 없으면 마지막 실행 칸
  const t = T.traj, p = (T.info && T.info.params) || {};
  const num = (x) => { const v = parseFloat(x); return Number.isFinite(v) ? v : null; };
  let cube = null, target = null, d = num(p.d) ?? 0.4775;
  if (t && t.real) { cube = { x: t.real.cube[0], y: t.real.cube[1], yaw: t.real.cube[2] }; target = { x: t.real.target[0], y: t.real.target[1] }; d = t.real.d ?? d; }
  else if (num(p.cube_x) != null) { cube = { x: num(p.cube_x), y: num(p.cube_y), yaw: num(p.cube_yaw) || 0 }; target = { x: num(p.target_x), y: num(p.target_y) }; }
  return { cube, target, d };
}
function renderTraj() {
  if (!T.status) buildTraj();
  if (S.traj_mtime !== T.mtime) { T.mtime = S.traj_mtime; loadTraj().then(updateIkView); }
  const t = T.traj, i = T.info || {};
  if (t) {
    const n = t.positions.length;
    T.status.textContent = `궤적: ${t.status} (${t.reason || ""}) · ${n} 프레임 · sim ${(n * t.dt).toFixed(1)}초` +
      (t.final_error_m != null ? ` · 놓은 오차 ${(t.final_error_m * 1000).toFixed(1)} mm` : "") + (i.mtime ? ` · ${new Date(i.mtime * 1000).toLocaleTimeString()}` : "");
    T.status.className = "note " + (t.status === "playable" ? "" : "warn");
    T.num.textContent = `${Math.round(T.frame)}/${n - 1} · ${(T.frame * t.dt).toFixed(2)}초`;
  } else {
    T.status.textContent = `궤적 없음 — ${i.error || "pick_place --dry-run 을 실행하면 여기서 재생된다"}. 큐브·놓을 자리만 표시.`;
    T.status.className = "note warn";
    T.num.textContent = "";
  }
  T.bPlay.disabled = !t; T.bPlay.textContent = T.playing ? "⏸ 일시정지" : "▶ 재생";
  const v = VIRT(), pb = ((S.ros || {}).virtual_state || {}).playback || {};
  T.vrow.hidden = !v;
  T.vplay.disabled = !t || pb.running; T.vstop.disabled = !pb.running;
  T.vinfo.textContent = pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1} (${pb.slow.toFixed(1)}배 느리게, 약 ${pb.seconds.toFixed(0)}초)` : pb.reason ? `마지막: ${pb.reason}` : "";
  updateIkView();
}

// ---------- 3D 뷰 ----------
const VW = {};
const toRad = (degMap) => Object.fromEntries(Object.entries(degMap).map(([k, v]) => [k, v * Math.PI / 180]));
function actualRad() { return toRad(Object.fromEntries(Object.entries(S.pose || {}).map(([k, p]) => [k, p.deg]))); }
function initViewers() {
  // 탭을 처음 열 때 그 탭의 3D 만 만든다(숨은 탭에 WebGL 을 미리 만들면 headless 등에서 안 그려지는 경우가 있었고, 메시 읽기도 줄인다)
  if (!window.JaxViewer || !S) return;
  if (curTab === "angle" && !VW.angle) {
    VW.angle = window.JaxViewer.create($("#v3d-angle"), { ghost: true });
    VW.angle.load(S.robot.mesh_ok).then(renderViewers);
  }
  if (curTab === "ik" && !VW.ik) {
    VW.ik = window.JaxViewer.create($("#v3d-ik"), { ghost: false });
    VW.ik.load(S.robot.mesh_ok).then(updateIkView);
  }
}
window.addEventListener("jaxviewer-ready", initViewers);
function renderViewers() {
  initViewers();
  if (!VW.angle || !S) return;
  const act = actualRad();
  VW.angle.setActual(act);
  // 목표 유령: 슬라이더가 실제와 0.5° 넘게 다르거나 바꾼 것이 있으면
  let show = false;
  for (const [n, v] of Object.entries(A.val)) {
    const p = (S.pose || {})[n];
    // 바꿨는데 안 보냄, 또는 이 화면에서 보냈고 아직 그 자리에 안 감(램프 중)
    if (Math.abs(v - (A.base[n] ?? v)) > 1e-9 || (A.sent[n] != null && p && Math.abs(p.deg - A.sent[n]) > 0.5)) { show = true; break; }
  }
  VW.angle.setGhost(show ? toRad(A.val) : null);
  const o = S.ros.available ? S.ros.object_pose : null;
  VW.angle.setObject(o && o.frame_id === "shoulder_roll_axis" ? { x: o.x, y: o.y, z: o.z, yaw: o.yaw_deg } : null);
  const nPose = Object.keys(S.pose || {}).length;
  $("#v3d-angle-note").textContent = (VIRT() ? "(가상) " : "") + (nPose ? `현재 자세: ${nPose}개 관절 값 (AK45 = 피드백, 서보 = 마지막으로 보낸 값). 값 없는 관절은 0°.`
    : "현재 자세 값 없음 — 0° 로 그림 (ROS·노드 꺼짐)");
}
function updateIkView() {
  if (!VW.ik || !S) return;
  const sc = trajScene();
  VW.ik.setTableD(sc.d);
  VW.ik.setCube(sc.cube);
  VW.ik.setTarget(sc.target);
  const o = S.ros.available ? S.ros.object_pose : null;
  VW.ik.setObject(o && o.frame_id === "shoulder_roll_axis" ? { x: o.x, y: o.y, z: o.z, yaw: o.yaw_deg } : null);
  const pbRun = VIRT() && ((((S.ros || {}).virtual_state || {}).playback) || {}).running;   // 가상 재생 중엔 가상 팔(실제 자세)을 보인다
  VW.ik.setActual(T.view === "actual" || pbRun || !T.traj ? actualRad() : trajPose(T.frame));
}

connect();
poll();
