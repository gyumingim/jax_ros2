# dashboard — 노드 켜기/끄기 + AK45 상태 웹 화면

터미널 여러 개 대신 브라우저 한 화면에서 `can0` 올리기, `ak45_node`·`stm_bridge_node`·`arm_control_node`(또는 `arm.launch.py` / `arm_homed.launch.py` 전체)·`vision_node` 켜기/끄기/재시작, 로그 보기, vcan 시험, AK45 원점 잡기/쉬게 하기를 한다. **모터 각도를 명령하는 버튼은 없다**(읽기 + 프로세스 관리 + homing_guard 의 원점 서비스만).

## 실행 (로봇 PC, Ubuntu 24.04 / Jazzy)

```bash
sudo apt install -y python3-fastapi python3-uvicorn python3-websockets   # 또는 pip install fastapi uvicorn websockets
python3 dashboard/server.py      # ROS 를 source 하지 않아도 된다(아래 'ROS 연결')
```

브라우저에서 **http://localhost:8080** 을 연다. 기본은 **이 PC에서만** 열린다(127.0.0.1).
같은 네트워크의 폰·노트북에서 보려면 `python3 dashboard/server.py --host 0.0.0.0` (암호가 없어 같은 네트워크 누구나 노드를 켜고 끌 수 있다. 필요할 때만).

설정(환경변수):

| 변수 | 기본 | 뜻 |
|---|---|---|
| `JAX_WS` | 레포가 `…/src/` 안에 있으면 그 위 워크스페이스, 아니면 `~/ros2_ws` | `install/setup.bash` 를 찾는 곳 |
| `JAX_ROS_DISTRO` | `$ROS_DISTRO`, 없으면 `jazzy` | `/opt/ros/<distro>/setup.bash` |
| `JAX_DASH_PORT` / `--port` | 8080 | |
| `JAX_DASH_HOST` / `--host` | 127.0.0.1 | |
| `JAX_DASH_SIGINT_WAIT`, `JAX_DASH_SIGTERM_WAIT` | 5, 3 초 | 끄기 단계 사이 대기 |
| `JAX_DASH_DEMO=1` | | Linux 에서도 가짜 시험 항목을 보인다 |

## ROS 연결 (자식 프로세스)

서버는 rclpy 를 import 하지 않는다. ROS 구독(`ros_monitor.RosMonitor`)은 `ros_bridge.py` 가 띄우는 자식 프로세스에서 돈다:
`bash -c "source /opt/ros/$JAX_ROS_DISTRO/setup.bash; source $JAX_WS/install/setup.bash(있으면); exec python3 -u dashboard/ros_monitor_proc.py"`.
자식은 0.25초마다 상태를 JSON 줄로 stdout 에 쓰고, 원점 서비스 호출은 stdin JSON 명령으로 받는다. stdin 이 닫히면(서버 종료) 끝난다. stderr 는 `logs/ros_monitor.log`.

- 서버가 켜질 때 한 번 자동 시도. 실패하면 'ROS 없음' 카드에 이유 + **ROS 연결(source)** 버튼, 연결되면 위 칩 옆 버튼이 **ROS 다시 연결**. 자식이 죽으면 'ROS 자식 죽음 (코드 N)', 3초 넘게 줄이 없으면 'ROS 응답 없음'.
- `/opt/ros/<distro>/setup.bash` 가 없으면(Windows 등) 바로 '이 PC엔 ROS 없음 (… 없음)'. API: `POST /api/ros/connect`.

## 화면

탭 6개(위 칩·전부 정지·경고 띠는 모든 탭 공통). 고른 탭은 주소 해시(`#ak` `#servo` `#finger` `#arm` `#ik` `#vision`)와 브라우저 localStorage 에 기억. 폰 폭에서는 탭 바가 가로로 스크롤된다. 탭 이름 옆 초록 숫자 = 그 탭에서 켜진 항목 수, 빨간 점 = 죽은 항목.

| 탭 | 내용 |
|---|---|
| AK | can0 올리기, ak45_node, AK45 카드 4개 + CAN 버스 카드, 원점(homing) 패널 |
| 서보 | stm_bridge_node, 손목 서보 W_PITCH·W_YAW·W_X 의 `/servo/state`(**마지막으로 보낸 값, 측정 아님**) + sim 환산 |
| 손가락 | stm_bridge_node 상태 칩·로그(켜기/끄기는 서보 탭), F1_1~F3_2·F_ROT 마지막 보낸 값(손가락별 묶음) |
| 팔 | arm.launch.py / arm_homed.launch.py / arm_control_node, vcan 시험, 보정 안내, 관절 14개 표(calibration.json 의 관절·모터·ID·verified + 현재 모터 각: AK = `/joint_states`, 서보 = `/servo/state`, sim 환산 = (모터 각 − zero_deg)/(sign×scale)) |
| 역기구학 | `ros2 run arm_control pick_place --cube X Y YAW --target X Y --max-speed V [--d] [--ik] --dry-run` 한 번 실행형. **--dry-run 고정**(계획·검사만, 모터 안 움직임). 출력은 마지막 실행분만 최대 200줄 |
| 비전 | vision_node(fixed_pose 또는 result_json), `/object_pose` 최근 값(x, y, z, yaw, frame_id, 나이; 3초 넘으면 '오래됨'). frame 이 shoulder_roll_axis 이면 '역기구학 탭 cube 칸에 넣기'(칸만 채움) |

- 위: 연결·ROS·can0 상태 칩, 실행 개수, 큰 **■ 전부 정지**(Esc). 아래 띠에 경고: 정지하면 `ak45_node` 가 끝나며 브레이크 0A → **홀드가 풀려 팔이 떨어질 수 있다.** 터미널에서 켠 노드가 있으면 빨간 띠로 알린다(전부 정지로 안 꺼짐).
- **AK45 상태**: `/diagnostics`·`/joint_states` 구독. 모터 4개 카드(위치°, 온도, 에러 코드, 워치독, target, command_active, 오차, 전류, 속도) + CAN 버스 카드(state, 에러 프레임, 버스오프). 3초 넘게 안 오면 '오래됨'. rclpy 가 없으면 'ROS 없음'만 보이고 나머지는 그대로 돈다.
- **켜기/끄기**: 항목마다 상태(켜짐 / 죽음+종료코드 / 가동 시간 / pid), 파라미터 칸, 켜기·끄기·재시작, 실제 실행한 명령, 로그 마지막 30줄(`/api/log/<id>?n=1000` 더 보기).
- **보정**: `calibrate --ros` 는 키보드 대화형이라 웹에서 돌리지 않는다. 명령 안내 + 'ak45_node + stm_bridge_node 켜기' 버튼 + `arm_control_node`/`arm.launch.py` 가 켜져 있으면 빨간 경고.
- **원점(homing)**: `homing_guard`(`arm_homed.launch.py`)의 `/arm_control/homing`(std_msgs/String JSON, transient local) 구독. 관절 4개 줄마다 ✅ 원점 / ⛔ 원점 없음 + 이유, 모터, 기준 자세(sim °), 오프셋, 세션 zero, calibration zero, 피드백 값·나이, command_active.
  - **원점 잡기**(관절마다, 위에 **전부 원점**): 확인창 "관절을 기준 자세(homing.json home_sim_deg)에 두었나요?" → `/arm_control/home_<관절>` / `home_all`(std_srvs/Trigger). guard 의 답(경고 포함)이 알림으로 뜬다.
  - **쉬게 하기**(관절마다, 위에 **전부 쉬게**): `/arm_control/release_<관절>` / `release_all`. guard 가 그 관절 **명령 전달만 끊고** 원점을 지운다. ak45_node 는 마지막 target 을 계속 잡고 있다 — 실제로 힘을 빼려면 ak45_node 를 끈다(팔이 떨어질 수 있음).
  - guard 가 안 켜져 있으면(상태 토픽 없음, 또는 3초 넘게 안 옴) 카드에 'arm_homed.launch.py 를 켜세요', 버튼은 꺼짐.
  - API: `POST /api/homing/home`, `POST /api/homing/release`, 몸통 `{"joint": "<관절>" | "all"}`. 관절 이름은 guard 상태 또는 `arm_control/homing.json` 에 있는 것만 받는다. 서비스 타임아웃 2초, 서비스가 없으면 'homing_guard 가 안 켜짐' 오류.

## 동작 규칙

- 항목마다 새 프로세스 그룹: `bash -lc "source /opt/ros/<distro>/setup.bash; source <ws>/install/setup.bash; exec stdbuf -oL -eL ros2 …"`. 로그는 `dashboard/logs/<id>.log`(git 무시).
- 끄기: 그룹에 **SIGINT → 5초 → SIGTERM → 3초 → SIGKILL**. 어느 단계에서 끝났는지 카드에 표시. SIGKILL 까지 갔으면 빨간 글씨(`ak45_node` 가 프레임이 끊기면 SIGINT 로 안 끝나는 알려진 문제, 루트 README·STATUS.md).
- 같이 켜면 안 되는 것은 켜기를 거절: `arm.launch.py` ↔ `arm_homed.launch.py` ↔ 개별 `ak45`/`stm_bridge`/`arm_control` (서로 전부). 보정 칸은 `arm_homed.launch.py` 가 켜져 있어도 빨간 경고. 대시보드 밖(터미널)에서 같은 노드가 이미 떠 있어도 거절.
- `can0 올리기`: 이미 UP 이면 그대로, 아니면 `sudo -n` → `pkexec`(로봇 PC 화면에 암호창) 순서. 둘 다 안 되면 로그에 복사할 명령 `sudo ip link set can0 up type can bitrate 1000000`.
- 대시보드 서버를 꺼도 노드는 **그대로 둔다**(끄면 팔이 떨어질 수 있어서). 다시 켜면 `logs/pids.json` 으로 다시 붙잡는다(Linux).
- 대시보드는 `jax_dashboard` 라는 ROS 노드로 `/diagnostics`, `/joint_states`, `/arm_control/homing`, `/servo/state`, `/object_pose` 를 구독하고, 원점 버튼을 누를 때만 homing_guard 서비스 클라이언트를 만든다. `arm_control/tests/check_topics.sh` 를 돌릴 때는 대시보드를 끄거나 ROS 없이 켠다(구독자가 하나 늘어 표와 달라질 수 있음).

## 항목 추가

`dashboard/services.py` 의 `SERVICES` 에 한 줄:

```python
Service("my_node", "내 노드", lambda p: f"exec ros2 run my_pkg my_node --ros-args -p rate:={p['rate']}",
        params=[Param("rate", "rate", "10")], conflicts=(), procs=("my_node",), help="설명"),
```

`build` 는 bash 본문(ROS 를 source 한 뒤 실행) 또는 argv 목록을 돌려준다. 값이 틀리면 `ValueError("이유")`. 끝나는 명령이면 `kind="once"`. 필드 설명은 파일 맨 위.

## 확인한 것 (2026-11-20, Windows 11, Python 3.13, ROS 없음)

- `python dashboard/server.py --port 8093` 이 뜨고 `127.0.0.1:8093` 에만 LISTEN(이 PC의 8080 은 다른 프로그램이 쓰고 있어 8093 으로 시험). `/`, `/static/app.js` 200, `/api/state` 응답: ROS 없음(rclpy ModuleNotFoundError), ROS 항목 7개는 '이 PC에서 못 씀', 가짜 항목 2개 사용 가능.
- 가짜 숫자 세기: 켜기 → 4초 뒤 로그 `2 3 4`, 두 번 켜기 거절, 재시작 두 번, 끄기 'SIGINT 로 끝남(0.1초)'.
- 가짜 SIGINT 무시: 전부 정지 → 'SIGINT 로 5초 안에 안 끝나 SIGTERM 으로 끝냄'.
- WS `/ws` 스냅샷 수신(websockets 라이브러리로 2개).
- 파라미터 검사: `fixed_pose` 형식 틀림·`; rm -rf /` 섞임·`fixed_pose`+`result_json` 둘 다 → 거절, `servo_channel:=can0;x`·`max_speed 999` → 거절. 만들어진 bash 스크립트 전부 `bash -n` 통과.
- 시험 끝에 서버·가짜 프로세스 모두 끔.

### 원점 패널 (2026-11-20)

- Windows(ROS 없음, `--port 8094`): `/api/state` 의 `ros.homing` = `{available: false, reason: "ROS 없음 (rclpy import 실패…)"}`, `home_poses` 4관절. `POST /api/homing/home {"joint":"elbow"}`·`release {"joint":"all"}` → `ok:false` 'ROS 없음 … 원점 서비스를 못 부름'. `{"joint":"../x"}`·`[1]`(빈 이름) → '모르는 관절' 거절. `app.js` `node --check` 통과.
- WSL Ubuntu 22.04 + ROS **Humble**(로봇 PC 는 Jazzy), colcon 빌드 없이 `homing_guard.HomingGuard` 를 python 으로 직접 띄우고 가짜 `/joint_states`(ak45_4 = 30°, 나머지 0°)를 보냄. 대시보드는 venv(fastapi 0.162)에서 `--port 8095`:
  - 상태 4관절 'not homed since start' 수신 → `home elbow` ok(오프셋 +30, 20° 넘음 경고 문구 그대로 전달) → `home all` → all_homed true → `release elbow` → elbow 'released (rest)' → `release all` → 전부 원점 없음. `wrist` 거절.
  - guard 를 끈 뒤: 상태 나이가 계속 늘어남(화면은 3초 넘으면 '오래됨, arm_homed.launch.py 를 켜세요'), `home elbow` → 약 1초 뒤 '/arm_control/home_elbow 없음: homing_guard 가 안 켜짐'.
- 시험 끝에 서버·guard 프로세스 모두 끔.

### 탭 6개 (2026-11-20)

- Windows `--port 8097`: `/` 200, `/api/state` 에 항목마다 `tab`, `calibration.joints` 14개. `node --check static/app.js` 통과. headless Edge 로 탭 6개(1280 폭)와 폰 폭(390 px, iframe 으로 실제 폭) 스크린샷 확인 — 가로 넘침 없음(관절 표는 카드 안에서 가로 스크롤).
- `pick_dry` 명령: 기본값 → `… pick_place --cube 0.1 -0.3 20 --target 0.05 -0.27 --max-speed 30 --dry-run`. `0.1;rm`·범위 밖·공백 든 ik 경로 거절. Windows 에서 켜기 → '이 PC에서 못 씀'.
- WSL Humble + venv(fastapi 0.162) `--port 8098`, `ros2 topic pub` 로 가짜 `/servo/state`(10개), `/object_pose`(shoulder_roll_axis, yaw 20°), `/joint_states`(ak45_1..4): `/api/state` 에 각도·yaw 20.0°·frame_id 그대로, 서보·손가락·팔 표·비전 화면에 값이 뜸.

- ROS 자식 프로세스: Windows → `/api/ros/connect` '이 PC엔 ROS 없음 (/opt/ros/jazzy/setup.bash 없음)', 원점 호출도 같은 이유로 거절. WSL: 서버를 **ROS source 없이**(`env -i`, venv python) `JAX_ROS_DISTRO=humble` 로 켬 → 자식이 자동으로 붙어 가짜 `/servo/state`·`/joint_states` 수신, homing_guard(python 직접 실행) 상태 수신, `home elbow`·`release all` 이 stdin 명령으로 성공. 자식 `kill -9` → 'ROS 자식 프로세스 끝남 (코드 -9)', 원점 호출 거절 → `/api/ros/connect` 로 새 pid 로 다시 붙음(옛 자식 안 남음). 서버 SIGINT → 자식도 끝남.

## 확인 안 한 것

- 역기구학 탭의 실제 `pick_place --dry-run` 실행(로봇 PC + team-jax/IK 필요)과 출력 표시.

- **Linux 쪽 전부**: 실제 `ros2 launch/run` 켜기·끄기, 프로세스 그룹 SIGINT/SIGTERM/SIGKILL, `ak45_node` 가 안 끝나는 경우 표시, `can0` 올리기(sudo -n / pkexec), `ip -json` can0 상태 칩, 터미널 노드 감지(/proc), `pids.json` 다시 붙잡기, rclpy 구독과 AK45 카드, `launch_vcan.sh` 버튼(레포가 `<ws>/src/` 안에 있어야 한다).
- 폰 화면은 CSS 만 맞춰 두었고 실제 폰으로 보지 않았다.
- 원점 카드 화면 자체(브라우저로 열어 보지 않음: 확인창·버튼 꺼짐·배지). Jazzy 에서 `ros2 launch arm_control arm_homed.launch.py` 를 대시보드로 켜기/끄기. 대시보드가 guard 보다 늦게 켜져도 transient local 로 바로 받는지(guard 가 2 Hz 로 다시 보내서 이번 시험으로는 구별 안 됨).
- Ubuntu apt 의 fastapi 버전(0.101)에서 도는지.

## 다음 단계

1. 로봇 PC 에서 위 '확인 안 한 것' 시험(먼저 `JAX_DASH_DEMO=1` 가짜 항목, 다음 vcan 시험 버튼, 그다음 실물).
2. **모터 명령 패널**(이번엔 일부러 안 만듦): 작은 각도(2°) 한 관절 명령, 누르는 동안만 움직이기, 원점 확인 후에만 활성, 속도 제한 표시.
3. '정지'의 뜻 정하기: 지금은 노드 종료 = 브레이크 0A(팔 떨어짐). 홀드 유지 정지(현재 위치를 target 으로)가 필요한지.
4. systemd 자동 실행과 연결.
