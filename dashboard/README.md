# dashboard — 실물 팔 제어(ROS 없이) + 각도 제어 + 팔 3D (웹 화면)

브라우저 한 화면에서 `can0` 올리기, **팔 연결(이 서버가 can0 을 직접: `real_arm.py`)**, AK45 상태, 원점(homing), 보정,
**관절 각도 보내기(모터가 실제로 움직임)**, `pick_place --dry-run` 궤적 3D 재생과 실물 재생을 한다. **ROS 설치가 필요 없다.**
모드 2개: **실물(직접 CAN, 기본)** / **가상**(Windows 에서도 끝까지 돈다). `--ros` 를 주면 '실물'이 팀 ROS 노드 경로가 된다(아래 'ROS 모드').

## 실행

```bash
sudo apt install -y can-utils python3-can python3-fastapi python3-uvicorn python3-websockets   # 또는 pip install python-can fastapi uvicorn websockets
python3 dashboard/server.py              # 실물(직접 CAN) 모드: ① CAN 켜기 → ② 팔 연결
python3 dashboard/server.py --virtual    # 가상 팔 모드 (실물·CAN·ROS 없이, Windows 도)
python3 dashboard/server.py --fake-hw    # 직접 CAN 을 가짜 AK45 4개 + 가짜 STM32 (python-can virtual) 에 — 화면 시험
python3 dashboard/server.py --ros        # 팀 ROS 버전 (ROS 2 + colcon build 필요)
```

브라우저에서 **http://localhost:8080**. 기본은 **이 PC에서만**(127.0.0.1). 같은 네트워크에서 보려면 `--host 0.0.0.0`
(암호가 없어 같은 네트워크 누구나 노드를 켜고 끄고 **모터 명령을 보낼 수 있다**. 필요할 때만).

| 변수 | 기본 | 뜻 |
|---|---|---|
| `JAX_WS` | 레포가 `…/src/` 안이면 그 워크스페이스, 아니면 `~/ros2_ws` | `install/setup.bash` 를 찾는 곳 |
| `JAX_ROS_DISTRO` | `$ROS_DISTRO`, 없으면 `jazzy` | `/opt/ros/<distro>/setup.bash` |
| `JAX_DASH_PORT` / `--port`, `JAX_DASH_HOST` / `--host` | 8080, 127.0.0.1 | |
| `JAX_DASH_VIRTUAL=1` / `--virtual` | | 가상 모드로 시작 (화면 위 스위치로도 바꿈) |
| `JAX_DASH_ROS=1` / `--ros` | | '실물' = ROS 노드 (없으면 직접 CAN) |
| `JAX_CAN_INTERFACE` / `--can-interface`, `JAX_CAN_CHANNEL` / `--can-channel` | socketcan, can0 | 직접 CAN 의 python-can 버스 |
| `JAX_IK_DIR` | 없으면 레포 옆 `../IK`, 그다음 `<ws>/src/IK` | team-jax/IK 체크아웃. 3D 메시(`motion2/arm/meshes`)와 pick_place 계획에 쓴다 |
| `JAX_DASH_PYTHONPATH` | | 로컬 pick_place(가상 모드·Windows)에서 sys.path 뒤에 붙일 경로(mujoco·gymnasium·scipy 가 따로 있을 때) |
| `JAX_DASH_SIGINT_WAIT`, `JAX_DASH_SIGTERM_WAIT` | 5, 3 초 | 끄기 단계 사이 대기 |
| `JAX_DASH_DEMO=1` | | Linux 에서도 가짜 시험 항목을 보인다 |

## 화면: 탭 3개

위 띠(모든 탭 공통): 연결·팔 연결·can0·실행 수 칩, **모드 스위치 실물 / 가상**, 큰 **■ STOP**(Esc, 직접 CAN = 지금 자리 홀드; 가상·ROS = 전부 정지), 경고 띠. 고른 탭은 주소 해시(`#conn` `#angle` `#ik`, 예전 `#ak` 등도 옮겨 감)와 localStorage 에 기억.

| 탭 | 내용 |
|---|---|
| **연결** | 직접 CAN: ① CAN 켜기 ② 팔 연결(속도 °/s, 힘 빼기, 연결 끊기), AK45 4개 표, 고급: 버스 상태(에러 프레임·송신 실패)·안전 기본값·로그·**관절 하나씩 보정**. ROS 모드면 아래 옛 화면: ROS 연결 카드(상태 + 'ROS 연결(source)') 또는 가상 팔 카드(가상 전원 끄기/켜기), 노드 켜기/끄기 전부(can0 올리기, arm.launch / arm_homed.launch / ak45 / stm_bridge / arm_control / vision, vcan 시험), AK45 진단 카드 4개 + CAN 버스, 로그, 보정 안내 |
| **각도 제어** | 원점(homing) 패널, 관절 14개 슬라이더 + 숫자칸(**시뮬 관절각 °**) + 관절마다 '보내기' / '바뀐 것 전부 보내기' / '현재 자세로 슬라이더 맞추기' / '바꾼 것 되돌리기'. 손가락은 묶어서(손가락 1·2·3 같이 움직이기 체크). 오른쪽 3D: **불투명 = 실제 현재 자세**, **파란 반투명 = 슬라이더 목표** |
| **역기구학** | **2D 지도**(위에서 본 책상, 로봇 위·앞 아래·로봇 왼쪽 = 화면 오른쪽): [큐브 놓기]·[놓을 자리 고르기] 로 눌러 고르고 yaw 슬라이더, 칸 색 = 그 yaw 의 도달 맵(team-jax/IK `reachmap_real_yaw.npz`), 고를 때마다 `realcoords.plan` 판정 한 줄(안 돼도 계획은 돌릴 수 있음). 숫자칸은 '숫자로 입력' 안(양방향). `pick_place --dry-run` 폼(+ `/object_pose` 값을 cube 칸에 넣기, vision_node 상태 작게, 가상 모드면 가상 큐브 칸). 오른쪽 3D: 계획한 궤적 재생(재생/일시정지/프레임 슬라이더/속도) + 큐브(주황)·놓을 자리(초록)·/object_pose(보라). 가상 모드면 **가상 팔로 재생** |

### 각도 제어 — 안전장치 (모터가 실제로 움직인다)

- 슬라이더·숫자칸은 3D 의 파란 목표만 바꾼다. **'보내기' 버튼을 눌러야** 나간다(관절 하나 / 바뀐 것 전부).
- 보내는 곳: `/arm_control/joint_command` (sensor_msgs/JointState, 이름 = sim 관절명, position = rad). ROS 자식 프로세스의 stdin 명령 → 자식이 발행.
  arm_control_node 가 calibration.json 변환·클립과 램프(`max_speed_deg_s`)를 하고, arm_homed 면 homing_guard 가 원점 없는 AK45 명령을 버린다(둘 다 기존 코드 그대로).
- 범위: 관절마다 `sim_range_deg` ∩ 모터 범위(서보 0~180°, AK45 `zero_deg ± limit_deg`)를 sim 각으로 바꾼 것(`cmd_range_deg`). 지금 calibration 이면 hand_roll −90~90, finger*_joint2 0~90, finger_rotation 0 만.
- 서버(`POST /api/joint/send {"joints": {관절: 도}}`)가 다시 검사: 아는 관절명, 유한한 숫자(bool·문자열 거절), 범위 안(잘라 보내지 않고 거절), 아래 게이트.
- 게이트(`state.gate`, 화면과 서버가 같은 판정): ROS(또는 가상) 연결 + `/arm_control/joint_command` 구독자 ≥ 1(arm_control_node / arm.launch / arm_homed 가 켜짐) 이 아니면 전부 꺼짐 + 이유.
  arm_homed 가 켜져 있거나 homing_guard 상태가 3초 안에 왔으면 '원점 모드': guard 상태가 없으면 AK45 전부, 원점 없는 AK45 는 그 관절만 꺼짐.
  알림(막지는 않음): AK45 피드백 없음(arm_control_node 가 기다림), 서보 첫 명령(위치를 몰라 램프 없이 바로 감 — arm_control_node 규칙).
- 첫 보내기 전에 확인창 1회(페이지마다): `verified=false` 관절 목록 — 방향·영점이 가정, 팔 주변 비우기, max_speed, 서보 첫 명령은 바로 감, 멈추는 법.
- '현재 자세로 슬라이더 맞추기': AK45 = 피드백(arm_homed 면 guard 가 오프셋을 뺀 `/ak45/joint_states_homed`) → sim 환산, 서보 = `/servo/state`(마지막으로 보낸 값). 서보는 피드백이 없다.
- 직접 CAN 모드: 보내는 곳은 `real_arm.RealArm.joint_command`(같은 calibration 변환·클립 → 원점 오프셋 → 램프). 멈춤 = ■ STOP(지금 자리 홀드, 팔 안 떨어짐).
- 가상·ROS 모드: 멈추는 것은 ■ 전부 정지(노드 종료 → AK45 브레이크 0A → **팔이 떨어질 수 있음**).

### 3D 뷰

- 모델: `static/robot/arm.xml` — team-jax/IK `motion2/arm/arm.xml`(브랜치 sim-table-friction, 커밋 925e023, 출처·해시는 `static/robot/SOURCE.txt`). IK 시뮬(motion1 pick_place 계획 포함)과 같은 생성기의 모델이라 관절명·축·영점이 시뮬 각과 같다.
- **arm.urdf 대신 arm.xml 을 쓰는 이유**: `arm.urdf` 는 ikpy 용 7관절 체인뿐이라 `<visual>`(메시)도 손가락도 없다. arm.xml 은 같은 7관절(관절 원점·축이 arm.urdf 와 같음을 확인: 위치 차 0, 회전 차 < 1e-7°) + 손가락 6관절 + 메시 배치를 갖고 있다. 그래서 urdf-loader 는 쓰지 않고, arm.xml 의 body/joint/geom 을 읽는 작은 로더(`static/viewer.js`)를 썼다. `arm.urdf` 도 참고로 같이 복사해 두었다(쓰지 않음).
- 좌표: 로봇 기준(vision_node·pick_place 와 같음) — 원점 = shoulder_roll 관절 원점, x 로봇 왼쪽+, y 뒤+(앞 = −y), z 위+. arm.xml base_link 틀을 x 로 +0.052 m 옮긴 것과 같다(IK `realcoords.py` + `env.py` robot_yaw −90° 로 계산. pick_place 궤적을 이 틀로 FK 하면 손끝이 큐브 중심 2.1 cm 안을 지나고 놓을 자리 2.5 cm 안에 간다 — grasp_dz 21.5 mm 를 생각하면 맞음). 격자 = 책상 높이(어깨 축 아래 d, 기본 0.4775 m).
- 메시(OBJ 약 8.7 MB)는 레포에 넣지 않는다. 서버가 `JAX_IK_DIR`(없으면 `../IK`, `<ws>/src/IK`)의 `motion2/arm/meshes/*.obj` 를 `/robot/meshes/<파일>` 로 준다(파일 이름 검사). 없으면 **막대 모델**(관절 원점을 잇는 막대 + 주황 관절 구 + 초록 손끝)로 그린다.
- 라이브러리: three.js r160 (`three.module.js`, `OrbitControls.js`, `OBJLoader.js`, LICENSE) 을 `static/vendor/three-0.160.0/` 에 저장(로봇 PC 가 오프라인이어도 됨, importmap). 마우스: 왼쪽 회전, 오른쪽 이동, 휠 확대.
- 현재 자세 = `state.pose`(서버가 계산): AK45 = 피드백(guard 가 있으면 원점 보정 피드백), 서보 = 마지막으로 보낸 값. 값 없는 관절은 0°. finger_rotation 은 sim 에서 고정이라 안 움직인다.
- 궤적: `pick_place` 의 기존 `--save` 옵션으로 `logs/pick_dry_traj.json`(joint_names, dt, positions rad, real/sim 좌표, status)을 쓰게 하고 `GET /api/ik/trajectory` 로 읽는다(arm_control 코드는 안 고침). 팔이 못 닿으면(종료코드 3) 계획이 없어 파일을 안 쓰므로 큐브·놓을 자리만 보인다(실행마다 옛 파일은 지움).

## 실물(직접 CAN) 모드 (`real_arm.py`, `direct_api.py`) — 기본

한 프로세스(대시보드 서버)가 can0 을 연다(python-can socketcan). ak45_node + stm_bridge_node + arm_control_node + homing_guard 가 하던 일을
스레드 2개로 한다(robot_arm v2 `class AK` 의 구조를 CAN 프레임으로):

- **rx**: `bus.recv` → AK45 피드백 0x29 파싱(`arm_control.protocol.parse_ak45_feedback`), CAN 에러 프레임(python-can socketcan 이 에러 프레임을 켠다) 집계 → bus-off·passive·warning, bit-stuff·no-ack 수. 우리가 안 보낸 AK45/서보 명령 프레임이 보이면 '다른 프로그램이 명령 중' 경고.
- **control 50 Hz**: AK45 마다 목표 → 사다리꼴 램프(30 °/s, 90 °/s², 한 스텝 3°) → carrot(측정 ±30°) → 위치 프레임(`ak45_pos_frame`), 서 있으면 20 Hz 재송신. 끊김·튐·막힘·온도·에러코드 처리. 서보는 `arm_control.motion.Mover` 램프 → `servo_frame`, 1 Hz 재전송.
- 원점: `arm_control.homing.Homing`(gap_s = 5 s). 원점을 잡는 순간 그 AK45 가 지금 자리에서 홀드를 시작한다(homing_guard 와 같음). '쉬게 하기' = 원점 지움 + 명령 막음, 홀드는 계속(힘을 빼려면 '힘 빼기').
- 보정: 연결 탭 고급의 '관절 하나씩 보정' = `calibrate.py` 의 m ±N(한 번에 ±10° 까지, 원점 없이 상대 이동) / z(지금 = sim 0; AK45 는 세션 원점도 여기로) / f / k / v / w(calibration.json 저장).
- 궤적 실물 재생: 역기구학 탭 '실물로 재생'(확인창 2번) → `play.check_trajectory`(치명 = 거절, 검사 문제 = 세 번째 확인창 = `--force`) → `play.play` + `watch.TrackingWatch`(15°, 0.5 s) → 벗어나면 멈춤·그 자리 홀드. 원점 없는 AK45 가 있으면 시작 안 함.
- 연결 거부: `/tmp/ak45_ctrl.lock` 이 잠김(ak45_node 가 떠 있음), 대시보드 밖 ROS 노드(ak45_node·stm_bridge_node·arm_control_node·homing_guard·calibrate), 대시보드가 켠 ROS 항목, can0 DOWN/없음.
- 실물 모드에서 가상으로 바꾸려면 팔 연결을 먼저 끊는다. 서버 종료(Ctrl+C) = 브레이크 0 A 후 닫음. 로그 `logs/direct.log`.
- API: `POST /api/direct/connect {max_speed}`, `/api/direct/disconnect`, `/api/direct/hold`, `/api/direct/limp {joint|"all"}`, `/api/direct/calib {action: move|zero|flip|scale|verify|save, joint, value}`, `/api/ik/play_real {force}`, `/api/ik/stop_real`, `GET /api/direct/log`. `/api/stop` = STOP(홀드), 원점·보내기는 다른 모드와 같은 API.
- 시험: `python dashboard/tools/try_direct_fake.py` (가짜 AK45 4개 + 가짜 STM32, `tools/fake_can_arm.py`). 아래 '확인한 것'.
- 검사 지적 반영(2026-11-20): 재생은 `play.play` 대신 자체 루프(첫 프레임 홀드 동안에도 STOP·고장 확인, 멈추면 지금 자리 홀드, 멈춤 확인과 보내기를 한 잠금 안에서),
  짧은 끊김 뒤 위치가 3° 넘게 다르면 원점 지움(전원 재투입·영점 변화), rx 스레드 보호·버스 다시 열기·죽은 스레드 다시 띄움(화면 CAN 버스 빨강),
  보정 ±N° 의 기준 = 측정 위치, 브레이크 2 ms 간격·재시도·연속 송신 실패 경고, 누르기 멈춤(3 A·2°·1 s, 가정 — `press_amps` 0 = 끔), 온도 경고 5 ℃ 히스테리시스, 닫는 중엔 명령·위치 프레임 막음.

### 실물 첫 시동 체크리스트 (직접 CAN)

1. 대시보드를 켜기 전, 팔을 손으로 받치고 한 모터의 지금 각(candump 의 0x29 피드백)을 읽은 뒤 **그 각으로 위치 프레임 1개만** 보낸다
   (예: elbow ID 4 가 지금 0.0° 면 `cansend can0 00000404#00000000`, 값 = int32 BE deg×10000). 1 s 뒤 살짝 밀어 홀드 토크가 남는지 본다.
   real_arm 은 "프레임이 끊기면 AK45 힘이 빠진다"(0.25 s 정도)를 **가정**한다(실측 없음). 힘이 남으면 close()·힘 빼기의 브레이크가 꼭 필요하고, 빠지면 짧은 끊김 홀드(5 s)가 팔을 붙잡는 근거가 맞다.
2. `ip -details link show can0` 로 txqueuelen 확인(기본 10). 브레이크·홀드 프레임이 ENOBUFS 로 버려지지 않게 `sudo ip link set can0 txqueuelen 100` 권장. 화면 CAN 버스에 '송신 실패'가 뜨면 멈춘다.
3. 무부하 홀드 전류와 팔 무게 홀드 전류를 기록 → `press_amps`(기본 3 A, 가정)를 그보다 넉넉히 크게. 바닥을 살짝 눌러 '누르기 멈춤'이 뜨는지 확인.
4. 피드백 끊김 시험: 한 모터 CAN 선을 1 s 뽑았다 꽂기 → 원점 유지·홀드 확인. 전원만 1 s 껐다 켜기 → 원점 지움·지금 자리 홀드 확인(엔코더 영점이 바뀌는지도 기록).

## 가상 팔 모드 (`virtual_arm.py`)

화면 위 스위치(또는 `--virtual`)로 바꾼다. 가상 모드에서는 ROS 자식 프로세스 대신 `VirtualArm` 이 같은 모양의 상태를 채우고, 노드 켜기/끄기·각도 보내기·원점·쉬게 하기·궤적 재생이 다 돈다. 화면 곳곳에 보라색 **가상** 배지·띠가 붙는다. 실물 노드가 켜져 있으면 가상으로 못 바꾼다. 로그는 `logs/virtual/<id>.log`.

| 가상 노드 | 흉내 내는 규칙 |
|---|---|
| ak45 | 첫 명령 전엔 프레임 없음, \|목표\| > 360° 거부, 마지막 목표 홀딩(timeout 없음), 피드백·워치독 없으면 안 보냄, 60 ℃ 이상 잠금, `/joint_states` 50 Hz(0.1° 양자화), `/diagnostics` 2 Hz(같은 키). 모터는 목표를 1차 지연(τ 0.08 s) + 60 °/s 로 따라감. 힘이 없으면(노드 꺼짐·전원 꺼짐) 0° 쪽으로 40 °/s 로 떨어짐. 온도는 전류²로 오르고 식음(가상 숫자) |
| stm_bridge | `/servo/command` 를 0~180° 로 잘라 `/servo/state`(보낸 값)로. 서보는 즉시 그 각(가정) |
| arm_control | `arm_control/arm_control/calibration.py`(변환·클립)와 `motion.py`(Mover 램프)를 **import 해서 그대로** 씀. 서보 첫 명령은 바로 감 |
| homing_guard | `arm_control/arm_control/homing.py`(Homing)를 import, homing_guard.py 의 on_state / on_diagnostics / on_command / on_timer / home·release 서비스 순서를 따라 함(arm_homed 일 때만) |
| vision | fixed_pose 파라미터 또는 역기구학 탭의 가상 큐브 칸 → `/object_pose`(shoulder_roll_axis) 1 Hz |

- **가상 전원 끄기/켜기**(연결 탭): 피드백이 끊기고 힘이 빠짐 → 다시 켜면 엔코더 영점이 15~40° 틀어진다. guard 가 원점을 지우고 제자리 홀드를 보내고, 그 관절 명령이 막히는지 볼 수 있다.
- 한 번 실행형: can0 올리기·vcan 시험은 '가상: 할 일 없음'(종료코드 0). `pick_place --dry-run` 은 가상 모드·Windows 에서도 **진짜로** 이 PC 의 python 으로 돈다(arm_control·stm_bridge 를 sys.path 에 넣고 `pick_place.main`; mujoco·gymnasium·scipy 와 team-jax/IK 필요).
- 역기구학 탭 **가상 팔로 재생**: `arm_control/arm_control/play.py` 의 `schedule`·`play` 를 그대로 써서 궤적 프레임을 가상 arm_control_node 로 보낸다(실물 재생은 대시보드에 없음). `check_trajectory` 의 치명 거절(not delivered 등)은 막고, --force 가 필요한 검사는 알림만.
- 가상 숫자(모터 속도·온도·떨어지는 속도·영점 틀어짐)는 가정이다. 실측 아님.

## ROS 모드 (`--ros`, 팀 ROS 버전) — ROS 연결 (자식 프로세스)

`--ros` 일 때만 쓰인다. 이때 연결 탭은 ① can0 ② ROS 연결 ③ 팔 켜기(arm_homed.launch.py), 고급에 노드 따로 켜기·보정 안내가 보인다.

서버는 rclpy 를 import 하지 않는다. 구독·발행(`ros_monitor.RosMonitor`)은 `ros_bridge.py` 가 띄우는 자식에서 돈다:
`bash -c "source /opt/ros/$JAX_ROS_DISTRO/setup.bash; source $JAX_WS/install/setup.bash(있으면); exec python3 -u dashboard/ros_monitor_proc.py"`.
자식은 0.25초마다 상태 JSON 줄을 stdout 에, 원점 서비스 호출과 관절 명령은 stdin JSON 명령으로 받는다. stdin 이 닫히면 끝난다. stderr 는 `logs/ros_monitor.log`.
구독: `/diagnostics`, `/joint_states`, `/ak45/joint_states_homed`, `/arm_control/homing`(transient local), `/servo/state`, `/object_pose`. 발행: `/arm_control/joint_command` 하나(보내기 버튼일 때만, 처음 보낼 때 publisher 를 만든다). 서비스: homing_guard 의 Trigger.

## 동작 규칙 (노드 켜기/끄기)

- 항목마다 새 프로세스 그룹: `bash -lc "source …; exec stdbuf -oL -eL ros2 …"`. 로그 `dashboard/logs/<id>.log`(git 무시).
- 끄기: 그룹에 SIGINT → 5초 → SIGTERM → 3초 → SIGKILL. 어느 단계에서 끝났는지 카드에 표시.
- 같이 켜면 안 되는 것은 거절: `arm.launch.py` ↔ `arm_homed.launch.py` ↔ 개별 `ak45`/`stm_bridge`/`arm_control`. 터미널에서 같은 노드가 떠 있어도 거절.
- `can0 올리기`: 이미 UP 이면 그대로, 아니면 `sudo -n` → `pkexec` 순서. 안 되면 로그에 명령.
- 대시보드 서버를 꺼도 노드는 그대로 둔다(끄면 팔이 떨어질 수 있음). 다시 켜면 `logs/pids.json` 으로 다시 붙잡는다(Linux).
- 보정(`calibrate --ros`)은 키보드 대화형이라 웹에서 안 돌린다(안내 + 준비 버튼만).
- 원점: `POST /api/homing/home|release {"joint": "<관절>" | "all"}` → homing_guard 서비스(가상 모드면 가상 guard).
- 기타 API: `POST /api/mode {"virtual": bool}`, `POST /api/virtual/power {"on": bool}`, `POST /api/virtual/cube {x,y,z,yaw}`, `GET /api/reachmap?yaw=<실 yaw °>`(칸 중심 실좌표 + grasp/place/feasible/되는 손 돌림, `JAX_IK_DIR` 의 motion1/src/humanoid_grasp_rl), `POST /api/reachmap/check {"cube":[x,y,yaw],"target":[x,y]}`(realcoords.plan 결과 + 큐브·놓을 자리 따로 판정), `GET /api/ik/trajectory`, `POST /api/ik/play_virtual`, `POST /api/ik/stop_virtual`.

## 항목 추가

`dashboard/services.py` 의 `SERVICES` 에 `Service(...)` 한 줄(필드 설명은 파일 맨 위). 가상 모드에서 노드로 흉내 내려면 `virtual_arm.SERVICE_NODES` 에도 넣는다(아니면 한 번 실행형은 '할 일 없음', argv 항목은 그대로 실행).

## 확인한 것

### 역기구학 탭 ③ 손 위치로 보내기 (`hand_ik.py`, `POST /api/ik/hand`, `POST /api/ik/fk`)

- 손이 쥐는 점(큐브 중심) x, y(지도 [손 위치] 모드로 누르거나 숫자) + 높이(책상 위 cm, 책상 = 어깨 아래 0.4775 m, 프리셋 2.5 cm = 큐브 중심) + 손 방향(잡을 큐브의 yaw) → [풀기] → 3D 에 파란 유령 + 분홍 목표점 → [보내기].
- 풀이: team-jax/IK `env.py`·`ik.py`(solve_site_pose) 를 서버의 자식 프로세스(`hand_ik.py --worker`, JAX_SETUP=real, 첫 호출 때 모델 읽기 약 1초)에서 그대로 쓴다. 큐브 세 면(+0, ±90°) × 손 그대로/반 바퀴 × 시작점 5개(지금 자세·home 등)를 풀고, 관절 한계 = 모델(손 비틀기 ±88°) ∩ calibration 의 보낼 수 있는 범위, 위치 5 mm·방향 1.5° 안이면 풀림, 그중 손가락이 책상에 안 묻히고 지금 자세에서 가장 가까운 해.
- 손 자세: **기본 = 큐브 잡는 자세**(실물 셋업과 같은 손가락 아래 + −40° 기울임, 파지 중심은 손 축 따라 21.5 mm 위 = env.pick_pose). **손끝 수직(0°)** 은 옵션: 큐브 높이에선 어깨 가까운 일부(격자 48점 중 15점, 기울인 손은 39점)만 되고 책상 위 10 cm 부터는 0점.
- 실패 이유: 책상 아래 / 팔이 안 닿음(손 방향 빼고도 n mm) / 손 비틀기 서보 범위 밖(±120 이면 풀림) / 그 손 방향으로는 안 됨.
- [보내기] = 각도 제어 탭과 같은 `/api/joint/send`(같은 막힘 규칙, 같은 첫 확인창) + 첫 사용 때 "관절 공간 직선 이동, 중간 경로 충돌 검사 없음" 확인창. 손가락은 그대로(체크하면 벌림). 팔이 목표에 닿으면 FK 로 손 위치를 다시 재서 목표와의 mm 차이를 보인다.
- 자동 풀기: 슬라이더(x, y, 높이, 손 방향)·지도 끌기 중에는 빠른 풀이(`fast: true`, 직전 해에서 한 번 80 반복, 같은 손 모양, 요청은 최신 것만), 손을 떼거나 숫자를 바꾸면 300 ms 뒤 전체 풀이. 빠른 풀이가 못 따라가면 유령이 빨강(마지막 좋은 해)이고 바로 전체 풀이 한 번. [다시 풀기] 는 작은 버튼.
- 어깨 옆으로 벌리기 고정(`fix_deg: {shoulder_yaw: 0}`, 기본 켬): 7 관절 → 6 관절이라 해가 거의 하나. 대신 닿는 곳이 크게 줄어든다(큐브 높이 격자 24점: 고정 없음 22, 0° 고정 4, 10° 1, 20°·30° 3). 안 풀리면 고정 없이 다시 풀어 "고정 때문 — 풀면 풀림(shoulder_yaw +19°)" 처럼 알린다.
- 팔도 바로 따라가기(기본 끔): 0.2초마다 같은 `/api/joint/send` 로, 한 번에 20° 넘게 바뀌는 해는 안 보냄.
- 속도(가상 8098): 서버 빠른 풀이 0.3~7 ms, HTTP 왕복 2.5 ms(3D 숨김) — headless Edge(소프트웨어 WebGL)에선 3D 그리기가 느려 화면 갱신 약 3 회/초, 왕복 35~47 ms. GPU 있는 실제 브라우저는 안 재 봄.
- 확인(2026-11-20, 가상 8098): 큐브 높이 3점 + 15 cm + 수직 손 1점 풀림·보냄 → FK 손 위치 목표와 0.6~1.2 mm; 30 cm·안 닿는 곳·책상 아래·수직 (0.10, −0.30) 은 이유와 함께 거절.

### 실물(직접 CAN) — 가짜 하드웨어 (2026-11-20)

- Windows 11, Python 3.13, python-can 4.6.1 `virtual` 버스, `tools/try_direct_fake.py` 34/34 통과(약 50 초, 2번): 첫 명령 전 무송신(1 s 동안 AK45 프레임 0, 피드백 50개씩), 원점 없으면 거절, 원점 → elbow 0→20° 0.91 s(최대 ~30 °/s), 피드백 1.5 s 끊김 = 홀드 프레임 계속(1 s 에 17개)·팔 안 떨어짐·원점 유지, 7 s 끊김 = 5 s 에 송신 멈춤·원점 지움·돌아오면 지금 자리 홀드, 엔코더 +70° 튐 = 지금 자리 홀드·원점 지움(축 안 움직임), 붙잡기 = 2.5 s 뒤 힘 뺌(브레이크 3개), 62 ℃ 경고 / 76 ℃ 힘 뺌·원점 거절, 에러 코드 2 힘 뺌, STOP = 그 자리 홀드, 서보 첫 명령 바로·다음 램프, 궤적 재생 끝까지 / elbow 붙잡으면 TrackingWatch 로 멈춤, 보정 ±5°·영점·방향·확인·저장(임시 복사본), 힘 빼기 = 브레이크 12개 후 위치 프레임 0, 브레이크 프레임 `00000203#00000000`.
  검사 지적 시험 14개(시험마다 새 가짜 팔): 재생 첫 프레임 홀드 중 STOP → 그 자리(90° 로 안 감) / 재생 멈춤 버튼 → 지금 자리 홀드 / 프레임 재생 중 STOP → 목표 안 덮어씀, 전원 1 s 껐다 켜며 엔코더 영점 30° → 원점 지움·축 그대로(전엔 축 50° 로 감), 5.05 s 경계 피드백 → 지금 자리, rx 처리 예외 60개에도 rx 삶·화면 빨강, rx 스레드 죽음 → 다시 띄움, recv 실패 → 버스 다시 엶·'수신 오류' 로그 한 줄, 막힌 관절 보정 +10° ×10 → 목표 +10°(전엔 +100°), 송신 3번 중 2번 실패에도 모터마다 브레이크 3개 / 계속 실패 → 화면 경고, 누르기(전류 4.3 A·오차 8°) → 목표 풀어 줌·힘·원점 유지 / 끄면 안 함, 온도 62/57/54 ℃ 경고 켜짐/유지/풀림·76 ℃ 뒤 65 ℃ 원점 거절·58 ℃ 허용, 재생 중 닫기 → 브레이크 뒤 위치 프레임 0.
- WSL Ubuntu 22.04: `/tmp/…lock` 을 `flock` 이 쥐고 있으면 `take_lock` 거절, 우리가 쥐면 `flock -n` 이 막힘(ak45_node 쪽 거부). **WSL 커널에 vcan 이 없어(CONFIG_CAN_VCAN 꺼짐) SocketCAN 경로(에러 프레임 포함)는 못 돌렸다.**
- `server.py --port 8099 --fake-hw` + curl: 연결 전 보내기 거절 → 연결 → 원점 없음 거절 → 원점 → elbow 30·shoulder_yaw 20·wrist_pitch 15 → STOP → 현재 자세 표, 연결 중 가상 모드 거절, 보정 서보 이동, 관절 힘 빼기, 연결 끊기 → 가상 → 실물. headless Edge 스크린샷(연결 탭·각도 제어 탭).

### 탭 3개 + 3D + 각도 제어 + 가상 팔 (2026-11-20)

- Windows 11, Python 3.13, `--port 8097 --virtual`, `JAX_IK_DIR` = team-jax/IK 클론(925e023): `/robot/meshes/Upper_Arm_1.obj` 200(933942 B), `..%2Fik.py` 404, vendor three 200. `node --check app.js`, viewer.js(모듈) 문법 통과.
- 가상 모드 curl: arm_homed(가상) 켬 → `elbow 40` 거절 '원점 없음' → 서보 `wrist_pitch 30` 은 보내짐 → 전부 원점 → `elbow 40` 보냄 → 피드백 0.5초 간격 14.4 → 34.8 → 40.0°(30 °/s 램프) → 가상 전원 끄기/켜기 → 4관절 '원점 없음'(feedback back after 2 s), `elbow 10` 거절 → elbow 다시 원점(오프셋 −26.8°, 20° 넘음 경고 그대로) → 전부 쉬게. 범위 밖(`wrist_pitch 120`), 모르는 이름, `"NaN"` 문자열 거절.
- headless Edge(swiftshader) 스크린샷: 연결 탭, 각도 제어 탭(메시 / 막대 모델), 각도 보내기 3초 뒤·12초 뒤 두 장(3D 자세가 램프로 움직임). 역기구학 탭은 headless 스크린샷에 3D 캔버스가 빈칸으로 찍혔다 — 같은 페이지에서 `canvas.toDataURL()` 로 뽑으면 팔·큐브·놓을 자리가 제대로 그려져 있어(그리기 호출 27개) headless 합성 문제로 본다(실제 브라우저에서 못 봄).
- 로컬 pick_place --dry-run(가상 모드, `JAX_DASH_PYTHONPATH` 에 mujoco 4.0.1): 계획 playable, 놓은 오차 2.5 mm, 1601 프레임, 검사에서 거절(종료코드 1: hand_roll 이 −113.4° 까지 가는데 서보 범위로는 −90° 까지 → 잘림, calibration 미확인). 궤적 파일 → 3D 재생, 가상 팔로 재생(3.2배 느리게 약 106초) 돌아감.
- 모드 바꾸기: 가상 → 실물(Windows: '이 PC엔 ROS 없음', 보내기 거절, 가상 전원 API 거절) → 다시 가상. 가상 vision_node(fixed_pose) → `/object_pose` 값.
- **WSL Ubuntu 22.04 + ROS Humble**(로봇 PC 는 Jazzy), colcon 없이 arm_control_node + homing_guard 를 python 으로 직접(arm_homed 와 같은 리매핑, max_speed 30) + 가짜 ak45(받은 `/ak45/command` 를 60 °/s 로 따라가는 `/joint_states`) + 가짜 stm(`/servo/command` → `/servo/state`). 대시보드는 ROS source 없이(`env -i`, venv fastapi 0.162) `JAX_ROS_DISTRO=humble`:
  구독자 1·guard 감지 → elbow 거절(원점 없음) → 전부 원점 → `elbow 30, wrist_pitch 20` 보냄 → `/ak45/command` ak45_4 가 …29.4, 30.0 으로 램프, `/servo/command` W_PITCH 110°(= 90 + 20) → 대시보드 현재 자세 elbow 9.0 → 30.0°(3D 가 쓰는 값). 스택을 끄면 구독자 0 → 보내기 거절. 시험 프로세스 모두 끔.

### 이전 (탭 6개 시절, 2026-11-20)

- 노드 켜기/끄기·가짜 항목·SIGINT/SIGTERM 단계·파라미터 검사·WS 스냅샷(Windows), 원점 패널(WSL Humble + homing_guard 직접 실행), ROS 자식 프로세스 연결·죽음·다시 연결(WSL). 자세한 기록은 git 이력(c319bc1 이전 README).

## 확인 안 한 것

- 손 위치로 보내기의 어깨 옆 고정은 이 카드에만 적용된다. 큐브 집기 계획(①②, pick_place → team-jax/IK)에는 안 들어간다(IK 레포 수정이 필요).

- **직접 CAN 실물 전부**: 실제 can0(SocketCAN)·CANable 에서 real_arm, 에러 프레임 해석, bus-off 때 송신 실패 처리, 실제 AK45 가 위치 프레임이 끊기면 힘을 놓는지(가정), 20 Hz 홀드로 충분한지, carrot·막힘 기준이 실물 무게에서 맞는지, 실제 STM32 가 1 Hz 재전송에 문제 없는지. Linux vcan 으로도 못 돌림(WSL 에 vcan 없음).
- **ROS 모드 실물 전부**: 로봇 PC(Jazzy)에서 실제 `ros2 launch/run`, 실제 모터로 '보내기'(방향·영점 미확인 상태), can0, 진단 카드, pids.json 다시 붙잡기.
- 실제(headless 아닌) 브라우저·폰에서 3D 조작감과 성능(메시 8.7 MB 를 한 번 읽음).
- Ubuntu apt 의 fastapi 0.101 에서 도는지.
- 가상 팔의 숫자(속도·온도·중력)는 실물과 대조 안 함.

### 직접 CAN 남은 문제 (2026-11-20 검사)

- 전원 재투입 직후 **첫 피드백 전에** 나간 홀드 프레임은 막지 못한다(그때는 위치를 모름). 첫 피드백이 오면 3° 검사로 원점 지우고 지금 자리 홀드 — 그 사이(부팅 → 첫 피드백, 실물 미측정) 모터가 옛 명령 쪽으로 조금 움직일 수 있다. 가짜 모터에선 축 움직임 < 1°.
- 3° 기준(`resume_rehome_deg`)·누르기 3 A 는 실물 홀드 오차·전류를 재기 전의 가정. 실물 홀드 오차가 3° 를 넘으면 짧은 끊김마다 원점이 지워진다(안전 쪽 실패).
- 누르기 멈춤은 명령 하나에 한 번만: 계속 누르면 막힘(12°/2 s)·온도(75 ℃)가 마지막 방어선. 전류 부호·단위(0.01 A)는 매뉴얼 기준, 실물 미확인.
- 버스 다시 열기는 python-can virtual 에서만 확인(실제 USB-CAN 빼고 꽂기·`ip link set can0 down/up` 은 안 해 봄).

## 다음 단계

1. 로봇 PC 에서 가상 모드 → vcan 시험 → 실물 순서로 확인. 실물 첫 보내기는 작은 각(2~5°) 한 관절, 팔 받치고.
2. calibration 확인(verified=true) 전에는 AK45 를 크게 움직이지 않는다. hand_roll 범위(서보 0~180 → sim ±90 vs 시뮬 ±120) 정리 필요 — pick_place 궤적이 −113° 를 쓴다.
3. (직접 CAN 에서 됨) STOP = 지금 자리 홀드, 힘 빼기는 따로. ROS 모드의 정지는 여전히 노드 종료(팔 떨어짐).
