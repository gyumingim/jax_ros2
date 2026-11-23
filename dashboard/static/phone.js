// 휴대폰 카메라 탭: 폰으로 책상 사진 → 서버가 큐브 자리(ArUco, phone_api.py) → 역기구학 탭과 같은 계획·재생으로 집으러 간다.
// app.js·direct.js 는 안 고친다: 탭 버튼도 여기서 단다. 계획 = POST /api/svc/pick_dry/start (역기구학 탭 칸 그대로, 큐브 칸만 바꿈),
// 재생 = 역기구학 탭의 원래 버튼(가상 팔로 재생 / 실물로 재생)을 그대로 누른다 → 확인창·검사·force 확인이 전부 원래 것.
"use strict";
const PH = { cube: null, grasp: null, busy: false, going: false };

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

  const pick = (capture) => {
    const inp = el("input", { type: "file", accept: "image/*", hidden: true });
    if (capture) inp.setAttribute("capture", "environment");
    inp.addEventListener("change", () => { if (inp.files[0]) phDetect(inp.files[0]); inp.value = ""; });
    return inp;
  };
  const cam = pick(true), gal = pick(false);
  PH.bShot = el("button", { class: "btn ph-big", onclick: () => cam.click() }, "사진 찍기");
  PH.img = el("img", { class: "ph-img", hidden: true, alt: "찾은 마커" });
  PH.res = el("div", { class: "result", hidden: true });
  PH.grasp_el = el("div", { class: "result", hidden: true });
  PH.bGo = el("button", { class: "btn ph-big", disabled: true, onclick: phGo }, "집으러 가기");
  PH.bStop = el("button", { class: "btn red", onclick: () => post(VIRT() ? "/api/ik/stop_virtual" : "/api/ik/stop_real") }, "■ 멈춤");
  PH.goNote = el("div", { class: "result", hidden: true });
  PH.target = el("div", { class: "help", style: "margin:10px 0 0" });
  PH.f = {};
  const fld = (k, lab, unit) => el("label", { class: "field" }, el("span", {}, lab, el("small", {}, " " + unit)), PH.f[k] = el("input", { inputmode: "decimal" }));
  page.append(
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "1"), "사진 찍기"),
    el("div", { class: "card" },
      PH.bShot, cam, gal,
      el("div", { class: "help", style: "margin:10px 0 0" }, "기준 종이와 큐브 윗면 마커가 다 나오게, 위쪽에서 한 장. ",
        el("a", { href: "#", onclick: (e) => { e.preventDefault(); gal.click(); } }, "찍어 둔 사진 고르기")),
      PH.img, PH.res, PH.grasp_el),
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "2"), "집으러 가기"),
    el("div", { class: "card" },
      PH.bGo,
      el("div", { class: "ph-row" }, PH.bStop, el("a", { class: "btn line", href: "#ik" }, "역기구학 탭에서 보기")),
      PH.goNote, PH.target,
      el("div", { class: "help", style: "margin:6px 0 0" }, "계획(역기구학 탭과 같은 것)을 돌린 뒤, 지금 모드의 팔로 재생할지 확인창이 묻는다.")),
    el("details", { class: "adv" }, el("summary", {}, "준비 — 인쇄물 · 기준 종이 자리"),
      el("div", { class: "card" },
        el("a", { class: "btn line", href: "/api/phone/sheet", target: "_blank" }, "인쇄물 PDF 열기 (A4 가로 2쪽)"),
        el("ol", { class: "help" },
          el("li", {}, "실제 크기 100% 로 인쇄 (1쪽의 100 mm 선을 자로 확인)."),
          el("li", {}, "1쪽(기준 종이)을 책상에 평평하게 붙인다: '로봇 쪽' 글자가 로봇을 향하게, 가운데 십자가 아래 자리에 오게."),
          el("li", {}, "2쪽의 마커 한 장을 50 mm 선으로 오려 큐브 윗면에 붙인다.")),
        el("div", { class: "frow" }, el("b", {}, "종이 가운데"), fld("x", "x", "m"), fld("y", "y", "m"), fld("rot_deg", "돌아간 각", "°"), fld("cube_h", "큐브 높이", "m")),
        el("div", { class: "help", style: "margin:8px 0 0" }, "로봇 좌표: 원점 = 어깨 축 바로 아래, +x = 로봇 왼쪽, 앞(책상 쪽) = −y. 각 = 위에서 볼 때 종이가 반시계로 돌아간 만큼."),
        el("div", { class: "ph-row" }, el("button", { class: "btn line", onclick: phSaveCfg }, "종이 자리 저장")))));
  fetch("/api/phone/config").then((r) => r.json()).then((d) => { for (const k in PH.f) PH.f[k].value = d.cfg[k]; }).catch(() => {});
  setInterval(phTick, 500);
  if (location.hash === "#phone" && curTab && curTab !== "phone") showTab("phone");   // 첫 화면이 이 파일보다 먼저 그려졌을 때
})();

async function phSaveCfg() {
  const body = Object.fromEntries(Object.entries(PH.f).map(([k, i]) => [k, parseFloat(i.value)]));
  await post("/api/phone/config", body);
}

// 폰 사진(보통 12 MP)을 긴 변 2000 px JPEG 로 줄여 올린다 (느린 와이파이에서도 1초 안팎). 못 줄이면 원본 그대로
async function phShrink(file) {
  try {
    const bmp = await createImageBitmap(file);
    const k = Math.min(1, 2000 / Math.max(bmp.width, bmp.height));
    const cv = el("canvas", { width: Math.round(bmp.width * k), height: Math.round(bmp.height * k) });
    cv.getContext("2d").drawImage(bmp, 0, 0, cv.width, cv.height);
    return await new Promise((ok) => cv.toBlob((b) => ok(b || file), "image/jpeg", 0.9));
  } catch (e) { return file; }
}

function phShow(node, text, cls, small) {
  node.hidden = false; node.className = "result " + (cls || "");
  node.replaceChildren(text, small ? el("small", {}, small) : "");
}

async function phDetect(file) {
  if (PH.busy) return;
  PH.busy = true; PH.cube = null; PH.grasp = null;
  PH.grasp_el.hidden = true; PH.goNote.hidden = true;
  phShow(PH.res, "큐브 찾는 중…", "run");
  let r;
  try {
    r = await (await fetch("/api/phone/detect", { method: "POST", headers: { "content-type": "application/octet-stream" }, body: await phShrink(file) })).json();
  } catch (e) { r = { ok: false, reason: "서버 연결 실패" }; }
  PH.busy = false;
  if (r.image) { PH.img.src = r.image; PH.img.hidden = false; }
  const warn = (r.warn || []).join(" · ");
  if (!r.ok) { phShow(PH.res, r.reason || "못 찾음", "bad", warn); return; }
  PH.cube = r.cube;
  phShow(PH.res, el("span", { class: "ph-xy" }, `큐브  x ${r.cube.x.toFixed(3)}  y ${r.cube.y.toFixed(3)} m  yaw ${r.cube.yaw_deg.toFixed(0)}°`), warn ? "warn" : "",
    `기준 마커 ${r.refs.length}개 · 큐브 마커 ${r.cube_marker_mm} mm 로 재짐 (원래 40)` + (warn ? " · " + warn : ""));
  phShow(PH.grasp_el, "집을 수 있는지 확인 중…", "run");
  let g;
  try {
    g = await (await fetch("/api/ik/graspable", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ x: r.cube.x, y: r.cube.y, d: tableD() }) })).json();
  } catch (e) { g = { ok: false, reason: "서버 연결 실패" }; }
  if (PH.cube !== r.cube) return;   // 그 사이 새 사진
  PH.grasp = g;
  if (g.ok && g.graspable) phShow(PH.grasp_el, "집을 수 있는 자리", "ok");
  else phShow(PH.grasp_el, g.ok ? "여기는 못 집음" : "집을 수 있는지 확인 못 함", g.ok ? "bad" : "warn", (g.reason || "") + (g.ok ? " — 큐브를 옮기고 다시 찍기" : ""));
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
  PH.bShot.disabled = PH.busy;
  PH.bShot.textContent = PH.busy ? "찾는 중…" : PH.cube ? "다시 찍기" : "사진 찍기";
  PH.bGo.disabled = !PH.cube || PH.going || !!pb.running || !PH.grasp || (PH.grasp.ok && !PH.grasp.graspable);   // 못 집는 자리로 확인되면 막음 (확인 못 했으면 계획이 판정)
  PH.bGo.textContent = pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1}` : PH.going ? "계획 중…" : `집으러 가기 (${kind === "virtual" ? "가상 팔" : "실물 팔"})`;
  PH.bStop.disabled = !pb.running;
  const v = mapVals();
  PH.target.textContent = (v.target ? `놓을 자리: x ${v.target.x.toFixed(3)}, y ${v.target.y.toFixed(3)} m (역기구학 탭에서 바꾼다)` : "놓을 자리: 역기구학 탭에서 정한다") +
    ` · 책상 높이 ${tableDNum().toFixed(3)} m`;
}
