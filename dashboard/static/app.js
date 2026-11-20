// JAX 팔 대시보드 — 상태는 WS(/ws, 0.5초마다 전체 스냅샷), 안 되면 /api/state 폴링.
// 탭 6개: AK / 서보 / 손가락 / 팔 / 역기구학 / 비전. 위 칩·전부 정지·경고 띠는 모든 탭 공통.
// 모터 각도 명령 버튼은 없음(읽기 + 프로세스 관리 + homing_guard 원점 잡기/쉬게 하기 + pick_place --dry-run).
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
// ROS 연결: 서버가 ROS 를 source 한 자식 프로세스(ros_monitor_proc.py)를 (다시) 띄운다
let rosBusy = false;
async function rosConnect() {
  if (rosBusy) return;
  rosBusy = true; document.querySelectorAll(".btn-ros").forEach((b) => { b.disabled = true; });
  try { await post("/api/ros/connect"); } finally { rosBusy = false; if (S) render(); }
}
const rosLabel = () => (S && S.ros.conn && S.ros.conn.state === "connected") ? "ROS 다시 연결" : "ROS 연결(source)";
$("#btn-ros").classList.add("btn-ros");
$("#btn-ros").onclick = rosConnect;
function rosConnBox() {
  const c = S.ros.conn || {};
  return [el("div", { class: "row" }, el("button", { class: "btn btn-ros", onclick: rosConnect, disabled: rosBusy }, rosBusy ? "연결 중…" : rosLabel())),
    el("div", { class: "help", style: "margin-top:6px" }, `source ${c.setup || "?"} (+ ${c.ws_setup || "?"} 있으면) → 자식 프로세스 ${c.pid ? "pid " + c.pid : "없음"} · 로그 ${c.log || ""}`)];
}

// ---------- 탭 ----------
const TABS = [["ak", "AK"], ["servo", "서보"], ["finger", "손가락"], ["arm", "팔"], ["ik", "역기구학"], ["vision", "비전"]];
const TAB_IDS = new Set(TABS.map((t) => t[0]));
const TAB_KEY = "jaxdash.tab";
const tabBtns = {};
let curTab = null;
function lsGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
function lsSet(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* 저장 못 해도 화면은 그대로 */ } }
function showTab(id) {
  if (!TAB_IDS.has(id)) id = "ak";
  curTab = id;
  for (const [t] of TABS) {
    document.getElementById("page-" + t).hidden = t !== id;
    tabBtns[t].classList.toggle("active", t === id);
    tabBtns[t].setAttribute("aria-selected", t === id ? "true" : "false");
  }
  lsSet(TAB_KEY, id);
  const nav = $("#tabs"), b = tabBtns[id];   // 폰 폭: 고른 탭이 가로 스크롤 밖이면 보이게
  if (b.offsetLeft < nav.scrollLeft || b.offsetLeft + b.offsetWidth > nav.scrollLeft + nav.clientWidth) nav.scrollLeft = b.offsetLeft - 16;
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
  const h = location.hash.slice(1);
  showTab(TAB_IDS.has(h) ? h : (lsGet(TAB_KEY) || "ak"));
})();
function renderTabDots() {
  // 탭 이름 옆 숫자: 초록 = 그 탭에서 켜진 항목 수, 빨간 점 = 그 탭의 항목이 죽음(종료코드 ≠ 0)
  for (const [id] of TABS) {
    const svcs = S.services.filter((s) => s.tab === id || (id === "finger" && s.id === "stm_bridge"));
    const on = svcs.filter((s) => s.alive).length;
    const dead = svcs.some((s) => !s.alive && s.exit_code != null && s.exit_code !== 0);
    const d = tabBtns[id].querySelector(".tdot");
    d.className = "tdot " + (on ? "on" : dead ? "bad" : "");
    d.textContent = on ? String(on) : "";
  }
}
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
  try { S = await (await fetch("/api/state")).json(); render(); if (!sock || sock.readyState !== WebSocket.OPEN) chip("chip-ws", "폴링 연결", "warn"); }
  catch (e) { chip("chip-ws", "서버 끊김", "bad"); }
}

// ---------- 그리기 ----------
const fmtDur = (s) => s == null ? "" : s < 60 ? `${s.toFixed(0)}초` : s < 3600 ? `${Math.floor(s / 60)}분 ${Math.floor(s % 60)}초` : `${Math.floor(s / 3600)}시간 ${Math.floor(s % 3600 / 60)}분`;
const LV = ["on", "warn", "bad", "warn"];   // diagnostic level 0 OK 1 WARN 2 ERROR 3 STALE
const LVT = ["정상", "주의", "에러", "오래됨"];

function render() {
  if (!S) return;
  const r = S.ros;
  const cs = (r.conn || {}).state;
  chip("chip-ros", r.available ? "ROS 구독 중" : cs === "starting" ? "ROS 연결 중" : cs === "dead" ? "ROS 자식 죽음" : cs === "stale" ? "ROS 응답 없음" : "ROS 없음",
    r.available ? "on" : (cs === "dead" || cs === "stale") ? "bad" : cs === "starting" ? "warn" : "");
  const rb = $("#btn-ros"); rb.textContent = rosBusy ? "연결 중…" : rosLabel(); rb.disabled = rosBusy;
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
  renderTabDots();
  renderMotors();
  renderServices();
  renderCalib();
  renderHoming();
  renderServos();
  renderFingerStm();
  renderJoints();
  renderPose();
  $("#foot").textContent = `워크스페이스 ${S.info.ws} · ROS ${S.info.distro} · 레포 ${S.info.repo} · 로그 ${S.info.logs} · ${S.info.platform}`;
}

function renderMotors() {
  const r = S.ros, box = $("#motors");
  if (!r.available) {
    $("#ros-note").textContent = "";
    rosMissing(box);
    return;
  }
  delete box.dataset.rosKey;
  box.replaceChildren();
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
  for (const s of S.services) {
    let c = cards[s.id];
    if (!c) {
      c = cards[s.id] = buildService(s);
      (document.getElementById("svc-" + s.tab) || $("#svc-arm")).append(c.root);
    }
    updateService(c, s);
  }
}
function buildService(s) {
  const c = { fields: {} };
  c.chip = el("span", { class: "chip" });
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
  c.bStart = el("button", { class: "btn", onclick: () => post(`/api/svc/${s.id}/start`, { params: params() }) }, once ? "▶ 실행" : "▶ 켜기");
  c.bStop = el("button", { class: "btn red", onclick: () => {
    if (s.stop_warn && !confirm(s.stop_warn)) return;
    post(`/api/svc/${s.id}/stop`);
  } }, once ? "■ 중단" : "■ 끄기");
  c.bRestart = once ? null : el("button", { class: "btn line", onclick: () => {
    if (s.stop_warn && !confirm(s.stop_warn)) return;
    post(`/api/svc/${s.id}/restart`, { params: params() });
  } }, "↻ 재시작");
  c.root = el("div", { class: "card" + (s.tab === "ik" ? " wide" : "") },
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
  let lines = s.log;
  if (s.kind === "once") {
    // 한 번 실행형: 로그 파일에 이어 쓰므로 마지막 '켜기:' 줄부터만 보인다
    let k = -1;
    for (let i = lines.length - 1; i >= 0; i--) if (/^===== .* 켜기: /.test(lines[i])) { k = i; break; }
    if (k >= 0) lines = lines.slice(k);
  }
  const text = lines.join("\n");
  if (c.log.textContent !== text) { c.log.textContent = text; if (atBottom) c.log.scrollTop = c.log.scrollHeight; }
  if ((s.alive || (s.tab === "ik" && text)) && !c.opened) { c.logbox.open = true; c.opened = true; }
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
  if (!h.available) { msg = `${h.reason || "ROS 없음"} — 위 'ROS 연결(source)' 버튼으로 연결하면 보인다.`; msgCls = ""; chipTxt = "ROS 없음"; }
  else if (!h.seen) msg = "homing_guard 가 안 켜짐: 팔 탭에서 arm_homed.launch.py 를 켜세요.";
  else if (h.age > 3) { msg = `원점 상태가 ${h.age.toFixed(0)}초 전 것 (homing_guard 꺼짐?). 팔 탭에서 arm_homed.launch.py 를 켜세요.`; chipTxt = "오래됨"; chipCls = "warn"; }
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

// ---------- 서보 (/servo/state = stm_bridge_node 가 마지막으로 보낸 모터 각, 측정값 아님) ----------
const WRIST = ["W_PITCH", "W_YAW", "W_X"];
const FINGERS = [["손가락 1", ["F1_1", "F1_2"]], ["손가락 2", ["F2_1", "F2_2"]], ["손가락 3", ["F3_1", "F3_2"]], ["손가락 회전", ["F_ROT"]]];
const calByName = () => Object.fromEntries(((S.calibration || {}).joints || []).map((j) => [j.name, j]));
// sim 각 = (모터 각 - zero_deg) / (sign * scale)  (calibration.json, verified=false 면 가정)
const toSim = (j, motorDeg) => (j && j.sign && j.scale) ? (motorDeg - (j.zero_deg || 0)) / (j.sign * j.scale) : null;
const fmtAge = (a) => a == null ? "—" : a < 60 ? `${a.toFixed(1)}초 전` : `${fmtDur(a)} 전`;
function rosMissing(box) {
  // 같은 내용이면 다시 안 그린다(0.5초마다 버튼을 새로 만들면 누르는 중에 사라질 수 있음)
  const c = S.ros.conn || {}, key = [S.ros.reason, c.state, c.pid, rosBusy].join("|");
  if (box.dataset.rosKey === key) return;
  box.dataset.rosKey = key;
  const bad = ["dead", "stale"].includes(c.state);
  box.replaceChildren(el("div", { class: "card" },
    el("div", { class: "note " + (bad ? "bad" : "") }, S.ros.reason || "ROS 없음"),
    el("div", { class: "help" }, "ROS 구독은 ROS 를 source 한 자식 프로세스에서 돈다. 버튼으로 (다시) 연결. 켜기/끄기는 ROS 없이도 된다."),
    rosConnBox()));
}
function servoRow(name, cal) {
  const r = S.ros, v = (r.servos || {})[name], j = cal[name];
  const sim = v ? toSim(j, v.deg) : null;
  return el("div", { class: "hrow" },
    el("div", { class: "hhead" }, el("b", {}, name), el("span", { class: "help", style: "margin:0" }, j ? `${j.joint} · ID ${j.id}` : "calibration 에 없음"),
      el("span", { class: "spacer" }),
      el("span", { class: "chip " + (v ? "on" : "") }, el("span", { class: "dot" }), v ? "보낸 값" : "아직 안 보냄")),
    el("div", { class: "big" }, v ? `${v.deg.toFixed(1)}°` : "—", el("small", { class: "unit" }, " 모터 각 (보낸 값)")),
    el("div", { class: "kv" },
      kv("sim 환산", sim == null ? "—" : `${sim.toFixed(1)}°${j && !j.verified ? " (보정 미확인)" : ""}`),
      kv("zero_deg", j ? `${j.zero_deg}°` : "—"),
      kv("sim 범위", j && j.sim_range_deg ? `${j.sim_range_deg[0]} ~ ${j.sim_range_deg[1]}°` : "—"),
      kv("보낸 시각", v ? fmtAge(v.age) : "—")));
}
function servoNote(id) {
  const r = S.ros;
  $(id).textContent = !r.available ? "" : r.servo_age == null ? "· /servo/state 아직 없음 (stm_bridge_node 꺼짐 또는 명령 없음)"
    : `· /servo/state ${fmtAge(r.servo_age)}`;
}
const SERVO_HELP = "stm_bridge_node 가 STM32 로 보낸 마지막 각도. 서보는 피드백이 없어 실제 위치는 모른다. 명령이 없으면 발행도 없다(오래돼도 이상 아님).";
function renderServos() {
  const wbox = $("#servo-wrist"), fbox = $("#servo-finger");
  servoNote("#servo-note"); servoNote("#finger-note");
  if (!S.ros.available) { rosMissing(wbox); rosMissing(fbox); return; }
  delete wbox.dataset.rosKey; delete fbox.dataset.rosKey;
  const cal = calByName();
  wbox.replaceChildren(el("div", { class: "card wide" }, el("h3", {}, "손목 (W_PITCH · W_YAW · W_X)"),
    el("p", { class: "help" }, SERVO_HELP), el("div", { class: "hlist" }, WRIST.map((n) => servoRow(n, cal)))));
  fbox.replaceChildren(el("div", { class: "card wide muted" }, el("p", { class: "help", style: "margin:0" }, SERVO_HELP)),
    ...FINGERS.map(([label, names]) => el("div", { class: "card" }, el("h3", {}, label),
      el("div", { class: "vlist" }, names.map((n) => servoRow(n, cal))))));
}
// 손가락 탭: stm_bridge_node 상태 칩 + 로그(켜기/끄기는 서보 탭)
const FS = {};
function renderFingerStm() {
  const s = S.services.find((x) => x.id === "stm_bridge");
  if (!s) return;
  if (!FS.root) {
    FS.chip = el("span", { class: "chip" }); FS.status = el("div", { class: "note" }); FS.log = el("pre", { class: "logbox" });
    FS.root = el("div", { class: "card" }, el("h3", {}, "stm_bridge_node", FS.chip),
      el("p", { class: "help" }, "손가락 서보도 stm_bridge_node 가 보낸다. 켜기/끄기는 ", el("a", { href: "#servo" }, "서보 탭"), "."),
      FS.status, el("details", { class: "log" }, el("summary", {}, `로그 (마지막 ${s.tail}줄)`), FS.log));
    $("#finger-stm").append(FS.root);
  }
  const cls = s.stopping ? "warn" : s.alive ? "on" : (s.exit_code != null && s.exit_code !== 0) ? "bad" : "";
  const txt = !s.available ? "못 씀" : s.stopping ? "끄는 중" : s.alive ? "켜짐" : s.exit_code != null ? `끝남 (코드 ${s.exit_code})` : "꺼짐";
  FS.chip.className = "chip " + cls; FS.chip.innerHTML = '<span class="dot"></span>'; FS.chip.append(txt);
  const lab = (id) => (S.services.find((x) => x.id === id) || {});
  const inside = ["arm_launch", "arm_homed"].filter((id) => lab(id).alive).map((id) => lab(id).label);
  FS.status.textContent = !s.available ? s.why : s.alive ? `pid ${s.pid} · 가동 ${fmtDur(s.uptime)}` :
    inside.length ? `${inside.join(", ")} 안에서 stm_bridge_node 가 돈다 (팔 탭)` : "";
  const t = s.log.join("\n");
  if (FS.log.textContent !== t) FS.log.textContent = t;
}

// ---------- 팔: 관절 14개 표 (calibration.json + /joint_states + /servo/state) ----------
const J = {};
function renderJoints() {
  const cal = S.calibration || {}, box = $("#joints"), r = S.ros;
  $("#joints-note").textContent = cal.file ? `· ${cal.file}` : "";
  if (!J.root) {
    J.tbody = el("tbody");
    J.msg = el("div", { class: "note" });
    J.root = el("div", { class: "card wide" },
      el("p", { class: "help" }, "모터 각: AK45 = /joint_states(측정), 서보 = /servo/state(마지막으로 보낸 값, 측정 아님). " +
        "sim 환산 = (모터 각 − zero_deg) / (sign × scale). verified ✗ = 관절↔모터 짝·방향·영점이 아직 가정."),
      J.msg,
      el("div", { class: "tablewrap" }, el("table", { class: "jt" },
        el("thead", {}, el("tr", {}, ["관절", "모터", "이름", "ID", "verified", "모터 각", "sim 환산", "sim 범위 °", "나이"].map((h) => el("th", {}, h)))),
        J.tbody)));
    box.append(J.root);
  }
  J.msg.textContent = cal.error ? `calibration.json 못 읽음: ${cal.error}` : !r.available ? `${r.reason || "ROS 없음"} — 현재값은 비어 있다.` : "";
  J.msg.className = "note " + (cal.error ? "bad" : "");
  const rows = (cal.joints || []).map((j) => {
    const isAk = j.motor === "ak45";
    const v = r.available ? (isAk ? (r.joints || {})[String(j.name).toLowerCase()] : (r.servos || {})[j.name]) : null;
    const sim = v ? toSim(j, v.deg) : null;
    const stale = v && isAk && v.age > 3;
    return el("tr", { class: stale ? "stale" : "" },
      el("td", {}, el("b", {}, j.joint)), el("td", {}, isAk ? "AK45" : "서보"), el("td", {}, j.name), el("td", { class: "n" }, j.id),
      el("td", { class: j.verified ? "ok" : "no" }, j.verified ? "✓" : "✗"),
      el("td", { class: "n" }, v ? `${v.deg.toFixed(2)}°` : "—", isAk || !v ? null : el("small", {}, " 보낸 값")),
      el("td", { class: "n" }, sim == null ? "—" : `${sim.toFixed(2)}°`),
      el("td", { class: "n" }, j.sim_range_deg ? `${j.sim_range_deg[0]} ~ ${j.sim_range_deg[1]}` : "—"),
      el("td", { class: "n" }, v ? fmtAge(v.age) + (stale ? " (오래됨)" : "") : "—"));
  });
  J.tbody.replaceChildren(...rows);
}

// ---------- 비전: /object_pose ----------
const P = {};
function renderPose() {
  const r = S.ros, box = $("#objpose");
  if (!r.available) { $("#pose-note").textContent = ""; rosMissing(box); P.root = null; return; }
  if (!P.root) {
    delete box.dataset.rosKey;
    P.chip = el("span", { class: "chip" }); P.big = el("div", { class: "big", style: "font-size:20px" }); P.note = el("div", { class: "note" }); P.kv = el("div", { class: "kv" });
    P.bUse = el("button", { class: "btn line", onclick: usePose }, "역기구학 탭 cube 칸에 넣기");
    P.root = el("div", { class: "card" }, el("h3", {}, "물체 위치", P.chip), P.big, P.note, P.kv,
      el("div", { class: "row" }, P.bUse),
      el("p", { class: "help" }, "frame shoulder_roll_axis = 로봇 좌표(pick_place --cube 와 같은 틀). camera = 카메라 좌표 그대로(yaw 도 카메라 기준이라 cube 칸에 못 넣음)."));
    box.replaceChildren(P.root);
  }
  const o = r.object_pose;
  $("#pose-note").textContent = o ? `· ${fmtAge(o.age)} 받음` : "· 아직 없음";
  const stale = o && o.age > 3;
  P.chip.className = "chip " + (!o ? "" : stale ? "warn" : "on"); P.chip.innerHTML = '<span class="dot"></span>';
  P.chip.append(!o ? "없음" : stale ? "오래됨" : "받는 중");
  P.big.textContent = o ? `x ${o.x.toFixed(3)} · y ${o.y.toFixed(3)} · yaw ${o.yaw_deg.toFixed(1)}°` : "—";
  P.note.textContent = !o ? "vision_node 가 안 켜졌거나 아직 발행 전 (1 Hz)" : stale ? `${o.age.toFixed(0)}초 전 값 (vision_node 꺼짐?)` : "";
  P.note.className = "note " + (stale ? "warn" : "");
  P.kv.replaceChildren(...(o ? [kv("x", `${o.x.toFixed(4)} m`), kv("y", `${o.y.toFixed(4)} m`), kv("z", `${o.z.toFixed(4)} m`),
    kv("yaw", `${o.yaw_deg.toFixed(2)}°`), kv("frame_id", o.frame_id || "(빈칸)"), kv("나이", fmtAge(o.age)),
    kv("stamp", o.stamp ? new Date(o.stamp * 1000).toLocaleTimeString() : "—")] : []).flat());
  P.bUse.disabled = !o || o.frame_id !== "shoulder_roll_axis";
}
function usePose() {
  // 입력칸만 채운다(실행은 역기구학 탭에서 직접, 그것도 --dry-run 뿐)
  const o = S.ros.object_pose, c = cards.pick_dry;
  if (!o || !c) return;
  const set = (k, v) => { if (c.fields[k] && !c.fields[k].disabled) c.fields[k].value = v; };
  set("cube_x", o.x.toFixed(3)); set("cube_y", o.y.toFixed(3)); set("cube_yaw", o.yaw_deg.toFixed(1));
  location.hash = "#ik";
  toast("cube 칸을 채웠다. 역기구학 탭에서 ▶ 실행(--dry-run)");
}

connect();
poll();
