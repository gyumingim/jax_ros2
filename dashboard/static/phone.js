// 휴대폰 카메라 탭: 큐브 자리를 서버가 찾고 (phone_api.py) → 역기구학 탭과 같은 계획·재생으로 집으러 간다.
// 찾는 흐름은 하나고 사진 올리기는 없다: 로봇에 거치한 S25 의 폰 스테레오 프로그램이 켜져 있으면 서버가 그 프레임의 ArUco 와 뎁스 결과를
// 합쳐 (phone_fuse.py: 둘 다면 ArUco 자리 + 뎁스로 맞춰 봄, 안 맞으면 막음) 1초마다 준다 (GET /api/phone/live). 결과 줄에 어느 것을 썼는지 적힌다.
// 폰 프로그램이 꺼져 있으면 큰 '폰 프로그램 켜기' 버튼 하나만 보인다. 끄기와 설정은 아래 접힌 '준비' 에 있다.
// app.js·direct.js 는 안 고친다: 탭 버튼도 여기서 단다. 계획 = POST /api/svc/pick_dry/start (역기구학 탭 칸 그대로, 큐브 칸만 바꿈),
// 재생 = 역기구학 탭의 원래 버튼(가상 팔로 재생 / 실물로 재생)을 그대로 누른다 → 확인창·검사·force 확인이 전부 원래 것.
"use strict";
const PH = { cube: null, grasp: null, going: false, dp: null, dBusy: false, block: "", last: null, d: null };

(function phInstall() {
  const page = document.getElementById("page-phone");
  if (!page) return;
  document.head.append(el("link", { rel: "stylesheet", href: "/static/phone.css" }));
  // 탭 버튼 (app.js buildTabs 와 같은 모양)
  TABS.push(["phone", "휴대폰 카메라", "사진으로 집기"]);
  TAB_IDS.add("phone");
  const b = el("a", { class: "tab", href: "#phone", role: "tab" }, "휴대폰 카메라", el("small", { class: "tsub" }, "사진으로 집기"), el("span", { class: "tdot" }));
  tabBtns.phone = b;
  $("#tabs").append(b);

  PH.bTh = el("button", { class: "btn line", onclick: phTable }, "책상 높이 재기");
  PH.bThFill = el("button", { class: "btn", disabled: true, onclick: phTableFill }, "역기구학 탭에 넣기");
  PH.thImg = el("img", { class: "ph-img", hidden: true, alt: "책상 높이를 잰 사진" });
  PH.thRes = el("div", { class: "result", hidden: true });
  PH.bOn = el("button", { class: "btn ph-big", hidden: true, onclick: phPipe }, "폰 프로그램 켜기");
  PH.dStat = el("div", { class: "help ph-stat" });
  PH.img = el("img", { class: "ph-img", hidden: true, alt: "찾은 큐브" });
  PH.res = el("div", { class: "result", hidden: true });
  PH.grasp_el = el("div", { class: "result", hidden: true });
  PH.bGo = el("button", { class: "btn ph-big", disabled: true, onclick: phGo }, "집으러 가기");
  PH.bStop = el("button", { class: "btn red", onclick: () => post(VIRT() ? "/api/ik/stop_virtual" : "/api/ik/stop_real") }, "■ 멈춤");
  PH.goNote = el("div", { class: "result", hidden: true });
  PH.target = el("div", { class: "help", style: "margin:10px 0 0" });
  PH.f = {}; PH.df = {};
  PH.bPipe = el("button", { class: "btn line", onclick: phPipe }, "폰 프로그램 켜기");
  PH.pipeNote = el("div", { class: "help", style: "margin:10px 0 0" });
  PH.cmd = el("code", { class: "ph-cmd" });
  const dfld = (k, lab, unit, wide) => el("label", { class: "field" + (wide ? " ph-wide" : "") }, el("span", {}, lab, el("small", {}, " " + unit)), PH.df[k] = el("input", wide ? {} : { inputmode: "decimal" }));
  const fld = (k, lab, unit) => el("label", { class: "field" }, el("span", {}, lab, el("small", {}, " " + unit)), PH.f[k] = el("input", { inputmode: "decimal" }));
  page.append(
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "1"), "큐브 찾기"),
    el("div", { class: "card" },
      PH.bOn, PH.dStat, PH.img, PH.res, PH.grasp_el,
      el("div", { class: "help", style: "margin:10px 0 0" }, "로봇에 거치한 폰의 화면에서 1초마다 찾는다. 기준 종이와 큐브 윗면 마커가 보이면 ArUco 자리를 쓰고 뎁스로 맞춰 본다. 마커가 없으면 뎁스만 쓴다.")),
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "2"), "집으러 가기"),
    el("div", { class: "card" },
      PH.bGo,
      el("div", { class: "ph-row" }, PH.bStop, el("a", { class: "btn line", href: "#ik" }, "역기구학 탭에서 보기")),
      PH.goNote, PH.target,
      el("div", { class: "help", style: "margin:6px 0 0" }, "계획(역기구학 탭과 같은 것)을 돌린 뒤, 지금 모드의 팔로 재생할지 확인창이 묻는다.")),
    el("h2", { class: "sec" }, "책상 높이"),
    el("div", { class: "card" },
      el("div", { class: "ph-row", style: "margin-top:0" }, PH.bTh, PH.bThFill),
      PH.thImg, PH.thRes,
      el("div", { class: "help", style: "margin:10px 0 0" }, "거치한 폰의 화면으로 잰다 (폰 프로그램이 켜져 있어야 한다): 기준 종이 마커가 보이면 ArUco 로, 뎁스 맵이 새것이면 뎁스로도 재서 차이를 보여 준다.",
        el("br"), "보정 없이 CAD 거치 자리를 믿는다 — 렌즈 자리·초점거리 때문에 1~3 cm 틀릴 수 있다. 처음 한 번은 줄자로 잰 값(어깨 축 ~ 책상 윗면)과 맞춰 보기.")),
    el("details", { class: "adv" }, el("summary", {}, "준비 — 인쇄물 · 폰 프로그램(뎁스) · 자리 설정"),
      el("h3", { class: "ph-h3" }, "ArUco — 인쇄물 · 기준 종이 자리"),
      el("div", { class: "card" },
        el("a", { class: "btn line", href: "/api/phone/sheet", target: "_blank" }, "인쇄물 PDF 열기 (A4 가로 2쪽)"),
        el("ol", { class: "help" },
          el("li", {}, "실제 크기 100% 로 인쇄 (1쪽의 100 mm 선을 자로 확인)."),
          el("li", {}, "1쪽(기준 종이)을 책상에 평평하게 붙인다: '로봇 쪽' 글자가 로봇을 향하게, 가운데 십자가 아래 자리에 오게."),
          el("li", {}, "2쪽의 마커 한 장을 50 mm 선으로 오려 큐브 윗면에 붙인다.")),
        el("div", { class: "frow" }, el("b", {}, "종이 가운데"), fld("x", "x", "m"), fld("y", "y", "m"), fld("rot_deg", "돌아간 각", "°"), fld("cube_h", "큐브 높이", "m")),
        el("div", { class: "help", style: "margin:8px 0 0" }, "로봇 좌표: 원점 = 어깨 축 바로 아래, +x = 로봇 왼쪽, 앞(책상 쪽) = −y. 각 = 위에서 볼 때 종이가 반시계로 돌아간 만큼."),
        el("div", { class: "ph-row" }, el("button", { class: "btn line", onclick: phSaveCfg }, "종이 자리 저장"))),
      el("h3", { class: "ph-h3" }, "뎁스 — 폰 프로그램 · 거치 자세"),
      el("div", { class: "card" },
        el("div", { class: "help", style: "margin:0" }, "로봇에 거치한 Galaxy S25 (USB) 의 화면과 두 렌즈 깊이로 책상 위 큐브를 찾는다 (표시지가 없어도 뎁스로)."),
        el("div", { class: "ph-row" }, PH.bPipe), PH.pipeNote,
        el("label", { class: "help", style: "display:flex;gap:8px;align-items:center;margin:14px 0 0" }, PH.df.save_rgbd = el("input", { type: "checkbox" }),
          "뎁스 맵 저장 (책상 높이를 뎁스로도 재려면 — 저장한 뒤 폰 프로그램을 껐다 켜기)"),
        el("div", { class: "frow" }, el("b", {}, "거치 자세"), dfld("c0", "tx", "m"), dfld("c1", "ty", "m"), dfld("c2", "tz", "m"), dfld("c3", "roll", "°"), dfld("c4", "pitch", "°"), dfld("c5", "yaw", "°")),
        el("div", { class: "help", style: "margin:8px 0 0" }, "카메라 좌표 → 로봇 좌표 (팀 vision_node 의 camera_to_robot 과 같은 숫자 6개). 처음 값은 CAD 임시값이라 실물에서 보정해야 한다: ",
          "큐브를 아는 자리 3곳 이상에 놓고 결과 줄의 '카메라 좌표' 를 적어 dashboard/tools/solve_phone_mount.py 로 푼다 (README)."),
        el("div", { class: "frow" }, el("b", {}, "큐브 각"), dfld("yaw", "직접 넣기", "° (비우면 추정)")),
        el("div", { class: "help", style: "margin:8px 0 0" }, "뎁스만으로 찾을 때 쓰는 각. 추정한 각은 보이는 면의 방향에서 구한 근삿값이다 (작은 큐브에서는 10° 안팎 틀릴 수 있다). 큐브는 90° 마다 같아서 −45~45° 로만 나온다. ArUco 가 같이 있으면 ArUco 각을 쓴다."),
        el("div", { class: "frow" }, dfld("repo_dir", "폰 스테레오 저장소", "폴더", true), dfld("run_dir", "결과 폴더", "(비우면 저장소/phone_stereo/runs/dash)", true)),
        el("div", { class: "ph-row" }, el("button", { class: "btn line", onclick: phSaveDepthCfg }, "폰 뎁스 설정 저장")),
        el("div", { class: "help", style: "margin:10px 0 0" }, "터미널에서 직접 켤 때:"), PH.cmd)));
  fetch("/api/phone/config").then((r) => r.json()).then((d) => { for (const k in PH.f) PH.f[k].value = d.cfg[k]; }).catch(() => {});
  phLoadDepthCfg();
  phLive();
  setInterval(phTick, 500);
  setInterval(() => { if (curTab === "phone" && !document.hidden) phLive(); }, 1000);
  if (location.hash === "#phone" && curTab && curTab !== "phone") showTab("phone");   // 첫 화면이 이 파일보다 먼저 그려졌을 때
})();

async function phSaveCfg() {
  const body = Object.fromEntries(Object.entries(PH.f).map(([k, i]) => [k, parseFloat(i.value)]));
  await post("/api/phone/config", body);
}

function phShow(node, text, cls, small) {
  node.hidden = false; node.className = "result " + (cls || "");
  node.replaceChildren(text, small ? el("small", {}, small) : "");
}

const PH_YAW_FROM = { side: "옆면 방향으로 추정", top: "윗면 모서리로 추정", mixed: "추정(두 면이 섞임)", manual: "직접 넣은 값" };
// 1초마다: 서버가 지금 프레임의 ArUco + 지금 뎁스를 합친 결과 (GET /api/phone/live, phone_fuse.py) → 결과 줄·그림·폰 프로그램 상태
async function phLive() {
  if (PH.dBusy) return;
  PH.dBusy = true;
  const d = tableD();
  let r;
  try { r = await (await fetch("/api/phone/live?d=" + encodeURIComponent(d))).json(); } catch (e) { r = { ok: false, reason: "서버 연결 실패", warn: [] }; }
  PH.dBusy = false;
  PH.dp = r;
  const pl = r.pipeline || {}, fresh = !!r.fresh;
  PH.bPipe.textContent = pl.alive ? "폰 프로그램 끄기" : "폰 프로그램 켜기";
  PH.bPipe.classList.toggle("red", !!pl.alive); PH.bPipe.classList.toggle("line", !pl.alive);
  PH.bOn.hidden = !!pl.alive || fresh;
  PH.pipeNote.textContent = pl.alive ? `폰 프로그램 켜짐 (pid ${pl.pid})` + (fresh ? "" : " — 아직 결과 없음: 폰 잠금을 풀고 몇 초 가만히. " + (pl.log || []).slice(-1).join(""))
    : fresh ? "결과가 들어오는 중 (다른 곳에서 켠 프로그램)" : "폰 프로그램 꺼짐" + ((PH.missing || []).length ? " — 이 PC 에 없는 것: " + PH.missing.join(" / ") : "") + ((pl.log || []).length > 1 ? " · 마지막 기록: " + pl.log.slice(-1)[0] : "");
  PH.dStat.hidden = fresh;
  PH.dStat.textContent = pl.alive ? "폰 프로그램 켜는 중 — 폰 잠금을 풀고 몇 초 가만히 (끄기는 아래 준비)" : PH.pipeNote.textContent;
  const src = !fresh ? "" : r.image || (r.depth_image_mtime ? "/api/phone/depth/image?t=" + r.depth_image_mtime : "");
  PH.img.hidden = !src;
  if (src && PH.imgSrc !== src) { PH.imgSrc = src; PH.img.src = src; }
  const tail = (r.note || []).concat(r.warn || []).join(" · ");
  if (!r.ok) {
    PH.cube = null; PH.grasp = null; PH.block = ""; PH.last = null; PH.grasp_el.hidden = true;
    phShow(PH.res, fresh ? "큐브를 못 찾음" : "결과 없음", fresh ? "bad" : "", (r.reason || "") + (tail ? " · " + tail : ""));
    return;
  }
  const a = r.aruco && r.aruco.ok ? r.aruco : null, p = r.depth && r.depth.ok ? r.depth : null;
  const det = [];
  if (a) det.push(`ArUco: 기준 마커 ${a.refs.length}개 · 큐브 마커 ${a.cube_marker_mm} mm 로 재짐 (원래 40)`);
  if (p) det.push(`뎁스: ${p.age_s.toFixed(0)}초 전 · 크기 ${p.dims_mm[0].toFixed(0)}×${p.dims_mm[1].toFixed(0)} mm · 책상 위 ${p.height_cm.toFixed(1)} cm · 각 ${PH_YAW_FROM[p.yaw_from] || "모름"}` +
    ` · 카메라 좌표 ${p.cam.map((v) => v.toFixed(4)).join(", ")}`);
  if (tail) det.push(tail);
  PH.last = r; PH.block = r.blocked ? r.block : "";
  phShow(PH.res, el("span", {}, el("span", { class: "ph-xy" }, `큐브  x ${r.cube.x.toFixed(3)}  y ${r.cube.y.toFixed(3)} m  yaw ${r.cube.yaw_deg.toFixed(0)}°`), el("b", { class: "ph-src" }, r.text)),
    r.blocked ? "bad" : (r.warn || []).length ? "warn" : "", det.join(" · "));
  const c = PH.cube;   // 거의 같은 자리면 그대로 둔다 (1초마다 집을 수 있는지 다시 묻지 않게)
  if (!c || Math.hypot(c.x - r.cube.x, c.y - r.cube.y) > 0.003 || Math.abs(c.yaw_deg - r.cube.yaw_deg) > 2 || PH.d !== d) { PH.d = d; phGrasp(r.cube, " — 큐브를 옮기기"); }
}

// 찾은 큐브를 PH.cube 로 두고, 그 자리를 집을 수 있는지 (지도와 같은 판정) 묻는다
async function phGrasp(cube, hint) {
  PH.cube = cube; PH.grasp = null;
  phShow(PH.grasp_el, "집을 수 있는지 확인 중…", "run");
  let g;
  try {
    g = await (await fetch("/api/ik/graspable", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ x: cube.x, y: cube.y, d: tableD() }) })).json();
  } catch (e) { g = { ok: false, reason: "서버 연결 실패" }; }
  if (PH.cube !== cube) return;   // 그 사이 새 결과
  PH.grasp = g;
  if (g.ok && g.graspable) phShow(PH.grasp_el, "집을 수 있는 자리", "ok");
  else phShow(PH.grasp_el, g.ok ? "여기는 못 집음" : "집을 수 있는지 확인 못 함", g.ok ? "bad" : "warn", (g.reason || "") + (g.ok ? hint : ""));
}

// ---- 책상 높이 재기 (phone_table.py): 거치한 폰의 지금 프레임(기준 종이 ArUco) 과 폰 뎁스 맵 → d. 넣는 것은 사람이 버튼으로 ----
async function phTable() {
  if (PH.thBusy) return;
  PH.thBusy = true; PH.th = null; PH.bTh.disabled = PH.bThFill.disabled = true;
  phShow(PH.thRes, "책상 높이 재는 중…", "run");
  let r;
  try {
    r = await (await fetch("/api/phone/table_height", { method: "POST" })).json();
  } catch (e) { r = { ok: false, reason: "서버 연결 실패" }; }
  PH.thBusy = false; PH.bTh.disabled = false;
  PH.thImg.hidden = !r.image;
  if (r.image) PH.thImg.src = r.image;
  const warn = (r.warn || []).join(" · ");
  if (!r.ok) { phShow(PH.thRes, "책상 높이를 못 잼", "bad", (r.reason || "") + (warn ? " · " + warn : "")); return; }
  let text = r.text;
  const a = r.aruco && r.aruco.ok ? r.aruco : null, p = r.depth && r.depth.ok ? r.depth : null;
  if (a) PH.thA = { d: a.d, t: Date.now() };
  else if (PH.thA && Date.now() - PH.thA.t < 600000)    // 이번엔 뎁스만: 10분 안에 ArUco 로 잰 값이 있으면 같이
    text += ` · ArUco ${PH.thA.d.toFixed(3)} m (${Math.round((Date.now() - PH.thA.t) / 60000)}분 전) · 차이 ${Math.round(Math.abs(PH.thA.d - r.d) * 1000)} mm`;
  const m = a || p, now = tableDNum();
  const note = [`기울기 ${m.tilt_deg.toFixed(1)}° (CAD 와 차이, 0 이어야)`, `카메라가 책상 위 ${m.cam_above_table.toFixed(3)} m`,
    a ? `마커 ${a.refs.length}개 · 초점거리 추정 흔들림 ± ${a.d_pm_mm.toFixed(0)} mm` : `평면 점 ${Math.round(p.inlier * 100)} % · 잔차 ${p.rms_mm} mm`,
    `지금 칸 ${now.toFixed(4)} m 와 ${Math.round((r.d - now) * 1000)} mm 차이`];
  if (!r.in_range) note.unshift(r.reason);
  if (warn) note.push(warn);
  PH.th = r;
  phShow(PH.thRes, el("span", { class: "ph-xy" }, text), !r.in_range ? "bad" : warn ? "warn" : "ok", note.join(" · "));
  PH.bThFill.disabled = !r.in_range;
}

function phTableFill() {
  const f = M.c && M.c.fields.d;
  if (!f || !PH.th || !PH.th.in_range) return;
  f.value = PH.th.d.toFixed(3);
  f.dispatchEvent(new Event("input", { bubbles: true }));   // 지도·집을 수 있는 곳이 새 높이로 다시 그려진다 (app.js 의 칸 처리 그대로)
  PH.bThFill.disabled = true;
  toast(`역기구학 탭 책상 높이 = ${f.value} m (어깨 축 아래)`);   // 큐브는 다음 phLive 가 새 높이로 다시 본다
}

// ---- 폰 뎁스: 설정 · 폰 프로그램 켜기·끄기 · 상태 한 줄 ----
async function phLoadDepthCfg() {
  try {
    const d = await (await fetch("/api/phone/depth/config")).json();
    d.cfg.camera_to_robot.forEach((v, i) => { PH.df["c" + i].value = v; });
    PH.df.yaw.value = d.cfg.yaw_manual_deg ?? "";
    PH.df.repo_dir.value = d.cfg.repo_dir; PH.df.run_dir.value = d.cfg.run_dir; PH.df.save_rgbd.checked = !!d.cfg.save_rgbd;
    PH.cmd.textContent = `cd ${d.cfg.repo_dir} && ${d.command}`;
    PH.missing = d.missing || [];
  } catch (e) { /* 다음에 다시 */ }
}

async function phSaveDepthCfg() {
  const c2r = [0, 1, 2, 3, 4, 5].map((i) => parseFloat(PH.df["c" + i].value));
  const yaw = PH.df.yaw.value.trim();
  const r = await post("/api/phone/depth/config", { camera_to_robot: c2r, yaw_manual_deg: yaw === "" ? null : parseFloat(yaw), repo_dir: PH.df.repo_dir.value, run_dir: PH.df.run_dir.value, save_rgbd: PH.df.save_rgbd.checked });
  if (r.ok) { await phLoadDepthCfg(); phLive(); }
}

async function phPipe() {
  const on = PH.dp && PH.dp.pipeline && PH.dp.pipeline.alive;
  PH.bPipe.disabled = true;
  await post(on ? "/api/phone/depth/stop" : "/api/phone/depth/start");
  PH.bPipe.disabled = false;
  phLive();
}

const phSleep = (ms) => new Promise((ok) => setTimeout(ok, ms));
async function phWait(cond, ms) {
  for (const t0 = Date.now(); Date.now() - t0 < ms; await phSleep(200)) if (cond()) return true;
  return false;
}

// 집으러 가기 = 역기구학 탭에서 사람이 하는 순서 그대로: 큐브 칸 채움 → ▶ 계획 돌려 보기 → (통과하면) 재생 버튼
async function phGo() {
  const c = cards.pick_dry, cube = PH.cube;
  if (!cube || PH.going) return;
  if (!c) { toast("역기구학 계획 항목이 없다", "error"); return; }
  PH.going = true;
  try {
    const set = (k, v) => { c.fields[k].value = v; };
    set("cube_x", cube.x.toFixed(3)); set("cube_y", cube.y.toFixed(3)); set("cube_yaw", cube.yaw_deg.toFixed(1));
    mapUpdate();
    const params = Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.type === "checkbox" ? (i.checked ? "1" : "") : i.value]));
    const m0 = S.traj_mtime;
    const st0 = svcById("pick_dry").started;
    phShow(PH.goNote, "계획 중…", "run");
    const r = await post("/api/svc/pick_dry/start", { params });
    if (!r.ok) { phShow(PH.goNote, "계획을 못 시작함", "bad", r.error || ""); return; }
    // 끝남 = 이번 실행(started 가 바뀜)이 꺼지고 종료코드가 나옴 (꺼진 직후 한 틱은 종료코드가 아직 없다)
    const done = await phWait(() => { const v = svcById("pick_dry"); return v.started !== st0 && !v.alive && v.exit_code != null; }, 180000);
    const sv = svcById("pick_dry");
    if (!done) { phShow(PH.goNote, "계획이 3분 넘게 안 끝남", "bad", "역기구학 탭에서 확인"); return; }
    const [txt, cls] = pickSummary(sv);   // 역기구학 탭의 결과 한 줄 그대로
    // 궤적이 나왔으면(종료코드 1 = 검사 거절이어도 역기구학 탭처럼) 재생 버튼으로 — 문제는 그 버튼의 검사·확인창이 다시 묻는다
    const got = sv.exit_code !== 3 && await phWait(() => S.traj_mtime != null && S.traj_mtime !== m0 && T.mtime === S.traj_mtime && T.info && T.info.mtime === S.traj_mtime, 4000);
    if (!got || !T.traj) { phShow(PH.goNote, "계획 실패 — 팔은 안 움직임", "bad", txt || `종료코드 ${sv.exit_code}`); return; }
    const pass = txt || "계획 나옴";
    await phSleep(600);   // 재생 버튼이 새 궤적으로 켜질 때까지 (다음 화면 갱신)
    const kind = S.mode && S.mode.kind;
    const btn = kind === "virtual" ? T.vplay : kind === "direct" ? (typeof D !== "undefined" && D.play && D.play.go) : null;
    if (!btn) { phShow(PH.goNote, pass, cls, "이 모드의 재생은 역기구학 탭에서"); return; }
    if (btn.disabled) { phShow(PH.goNote, pass, "warn", "재생은 아직 못 함: " + (btn.title || (kind === "direct" ? "연결 탭에서 팔 연결·원점 먼저" : "이미 재생 중"))); return; }
    phShow(PH.goNote, pass, cls);
    btn.click();   // 원래 버튼 = 원래 확인창 (가상: 1번, 실물: 2번 + 검사 문제면 한 번 더)
  } finally { PH.going = false; }
}

function phTick() {
  if (!S || curTab !== "phone") return;
  const kind = S.mode && S.mode.kind;
  const pb = (kind === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
  PH.bGo.disabled = !PH.cube || PH.going || !!pb.running || !PH.grasp || (PH.grasp.ok && !PH.grasp.graspable) || !!PH.block;   // 못 집는 자리로 확인되면 막음 (확인 못 했으면 계획이 판정)
  PH.bGo.textContent = pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1}` : PH.going ? "계획 중…" : PH.block && PH.cube ? PH.block : `집으러 가기 (${kind === "virtual" ? "가상 팔" : "실물 팔"})`;
  PH.bStop.disabled = !pb.running;
  const v = mapVals();
  PH.target.textContent = (v.target ? `놓을 자리: x ${v.target.x.toFixed(3)}, y ${v.target.y.toFixed(3)} m (역기구학 탭에서 바꾼다)` : "놓을 자리: 역기구학 탭에서 정한다") +
    ` · 책상 높이 ${tableDNum().toFixed(3)} m`;
}
