# 팀 ROS 버전 — ROS 2 노드로 실물 제어 (Jazzy)

> 2026-11-20 README.md 에서 옮김. 포크(`noros`)의 기본은 ROS 없는 직접 CAN 이고(README.md), 이 문서는 팀 레포(team-jax/jax_ros2)와 같은 ROS 구조를 설명한다. 코드(ak45_can, ak45_ros2, stm_bridge, arm_control, vision_node)는 그대로 있다.

흩어져 있던 ROS 2 코드를 한 레포에 모았다. 시뮬 관절각을 실물 모터 명령으로 바꾸던 `sim2real_node`(team-jax/SimToReal)는 노드 4개로 쪼갰고, **동작은 바뀌지 않았다**(같은 입력에서 `/ak45/command`와 서보 프레임이 같은 값으로 나가는 것을 숫자로 비교했다. `STATUS.md`).

```
vision_node ─/object_pose─▶ arm_control ─┬─/ak45/command──▶ ak45_node ─CAN(확장 ID)─▶ AK45 4개 (ID 1~4)
                                          └─/servo/command─▶ stm_bridge ─CAN(StdId 5)─▶ STM32 ─▶ 서보 10개
```

| 폴더 | 내용 | 원본 |
|---|---|---|
| `ak45_can/` | AK45 CAN 라이브러리(ROS 패키지 아님. `ak45_ros2`가 `../ak45_can`을 찾는다) | team-jax/ak45_can `6db174f` |
| `ak45_ros2/` | `ak45_node`: AK45 **4개**(ID 0x01~0x04) | team-jax/ak45_ros2 `5495298` (6개→4개로 변경) |
| `stm_bridge/` | `/servo/command` → CAN_STM 서보 프레임, `/servo/state` | 새로 만듦 (SimToReal의 서보 CAN 부분) |
| `arm_control/` | 보정, 속도 제한, 감시, 계획, 재생, 보정 CLI. `arm.launch.py` | SimToReal의 나머지 |
| `vision_node/` | `fixed_pose`(시험용) 또는 `result_json`(카메라 프로그램의 `inference.json`)일 때만 `/object_pose`를 발행 | 새로 만듦 |

각 복사본 폴더의 `SOURCE.md`에 원본 URL과 커밋 SHA가 있다. 원본 레포(`SimToReal`, `ak45_ros2`, `ak45_can`)는 건드리지 않았다.

## 대시보드를 ROS 노드로 (`python3 dashboard/server.py --ros`)

포크 기본은 ROS 없는 직접 CAN 이다(README.md). 아래는 같은 대시보드를 `--ros` 로 띄워 팀 ROS 노드(ak45_node …)를 쓰는 방법이다.

### 1. 받기
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b noros https://github.com/gyumingim/jax_ros2
git clone -b handroll-limit  https://github.com/gyumingim/IK     # 역기구학: hand_roll 을 서보 범위 ±88° 로 묶은 판
```
대시보드는 IK 레포를 `../IK` 또는 `~/ros2_ws/src/IK` 에서 찾는다(다른 곳이면 `JAX_IK_DIR`).

### 2. 설치·빌드 (Ubuntu 24.04 + Jazzy. 22.04 면 jazzy → humble)
```bash
sudo apt install -y can-utils python3-can python3-colcon-common-extensions \
                    python3-fastapi python3-uvicorn python3-websockets
pip install --user --break-system-packages mujoco gymnasium scipy   # pick_place 계획(시뮬)용
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && colcon build --symlink-install
```

### 3. 켜기
```bash
cd ~/ros2_ws/src/jax_ros2 && python3 dashboard/server.py --ros  # 실물 없이 시험: --virtual
```
브라우저 **http://localhost:8080** → 위쪽 스위치 **실물(ROS)** → **연결** 탭에서 순서대로:
① CAN 켜기(CANable 은 꽂으면 `can0`, sudo 를 물을 수 있음) → ② ROS 연결 → ③ 팔 켜기(`arm_homed.launch.py` = ak45_node + stm_bridge + homing_guard + arm_control).

| 탭 | 하는 일 |
|---|---|
| 연결 | 위 ①②③, AK45 4개 상태(위치·온도·홀드), 고급: 노드 따로 켜기·로그·보정 안내 |
| 각도 제어 | ① 원점 잡기 ② 관절 슬라이더 → [보내기] (arm_control 의 30°/s 램프와 원점 guard 를 거침), 오른쪽 3D |
| 역기구학 | 지도에서 큐브 → 놓을 자리 클릭, [계획 돌려 보기] = `pick_place --dry-run`(모터 안 움직임), 3D 궤적 재생 |

실행 순서·파일 역할 그림: 대시보드가 켜져 있으면 http://localhost:8080/static/run_map.html

### 4. 처음 움직이기 전에 (꼭)
1. **팔을 받친다.** 노드를 끄거나 피드백이 끊기면 AK45 홀드가 풀려 팔이 떨어질 수 있다.
2. **보정은 전부 가정이다**(`calibration.json` 의 `verified: false`). 관절마다 `ros2 run arm_control calibrate --ros` 로 +5° 만 보내 방향을 눈으로 확인하고, 처음 보내기는 2~5° 로 작게.
3. **손목 서보 확인**: `calibrate` 에서 `j W_PITCH` → `m +5`. 손목이 **굽어야** 맞다(아래 "손목 이름"). 비틀리면 `calibration.json` 의 wrist_pitch / wrist_yaw 서보(`name`, `id`)를 되돌린다.
4. **원점**: 각도 제어 탭에서 팔을 기준 자세(시뮬 0° = 팔을 수직으로 내린 자세, `arm_control/homing.json`)에 두고 [전부 원점 잡기]. 켤 때마다·전원 재투입마다·쉬게 한 뒤마다 다시 잡는다(원점은 한 번의 홀드 동안만 유효).
5. **역기구학은 계획(dry-run)만.** 실물 재생은 보정을 다 확인한 뒤.

### 알아 둘 것
- **손목 이름**: CAD/URDF 의 손목 이름이 실제 움직임과 바뀌어 있다(시뮬 0° 에서 `wrist_pitch` 는 아래팔 축 비틀기, `wrist_yaw` 는 굽힘, `hand_roll` 도 아래팔 축 = yaw). 그래서 `calibration.json` 에서 wrist_pitch → W_YAW(8), wrist_yaw → W_PITCH(9) 로 바꿔 연결했다. 시뮬 관절 이름은 IK·궤적·토픽 호환 때문에 그대로 둔다. 대시보드 표시는 "손목 비틀기 / 손목 굽힘 / 손 yaw".
- **hand_roll ±88°**: 손 yaw 서보(W_X, MG996R)가 0~180° 라 시뮬 ±90° 만 된다. IK `handroll-limit` 은 이 범위 안에서만 계획한다. 그 대가로 집을 수 있는 칸이 60 → 36 칸이고, 대부분 손을 반 바퀴 돌려 잡는다. 서보 혼을 30° 옮기면(−58°~118°) 100/100 이지만 혼은 안 옮기기로 했다.
- **반 바퀴 잡기**: 집기와 놓기는 같은 손 모양이어야 한다(들고 가며 바꿔 잡을 수 없음). 지도에서 둘 다 파란 칸이어도 손 모양이 다르면 `pick_place` 가 "cannot serve this layout" 로 거절한다(지도가 이걸 반영하도록 고치는 중).
- **원점 guard 를 쓸 때** `play_trajectory`·`pick_place` 를 따로 띄우면 `--ros-args -r /joint_states:=/ak45/joint_states_homed` 를 붙인다(안 붙이면 오프셋이 두 번 들어가 팔이 더 움직일 수 있음).
- **손가락 회전 F_ROT(6번)**: 3번 손가락이 1·2번과 마주 보는 서보 각도(`zero_deg`)가 아직 정해지지 않았다.
- **서보 펌웨어(team-jax/CAN_STM)**: 피드백 없음(`/servo/state` 는 보낸 값), 명령이 끊겨도 마지막 자세를 계속 잡음, 500~2500 µs 가 실제 0~180° 인지는 서보마다 다를 수 있음(비율 `scale` 1.0 은 가정).
- **확인 범위**: 대시보드·guard·가상 팔은 Windows 가상 모드와 WSL Humble 에서만 시험했다. 실물 모터·Jazzy 에서는 아직 안 돌렸다.


## 토픽

| 토픽 | 형식 | 방향 | 규칙 |
|---|---|---|---|
| `/arm_control/joint_command` | JointState, **시뮬 관절 이름**(`elbow`, `wrist_pitch` …), rad | pick_place·play_trajectory·사람 → arm_control | 기존 `/sim2real/joint_command`의 새 이름 |
| `/ak45/command` | JointState, 이름 `ak45_1..4`, position = **모터 각도(rad)** | arm_control → ak45_node | 기존 규칙 그대로 |
| `/joint_states` | JointState (AK45 실측, rad) | ak45_node → 모두 | 기존 그대로 |
| `/servo/command` | JointState, 이름 = 서보 이름(`W_PITCH`, `F1_1` … `calibration.json`의 `name`), position = **서보 모터 각도(rad, 0~π)** | arm_control → stm_bridge | `/ak45/command`와 같은 규칙. 시뮬 각도 → 모터 각도 변환은 **arm_control만** 한다 |
| `/servo/state` | 위와 같은 형식, position = **마지막으로 보낸 값**(측정값 아님) | stm_bridge → 모두 | 서보는 피드백이 없다. `/joint_states`에 섞지 않는다 |
| `/object_pose` | PoseStamped, `frame_id = shoulder_roll_axis`(카메라 모드에서 `camera_to_robot`이 비면 `camera`), 위치 m | vision_node → arm_control | 아래 |

`/object_pose`: 원점 = 오른쪽 어깨 롤 모터 축 중심, x = 로봇 왼쪽(+), y = 뒤(+, 앞은 −), z = 위. **회전은 z축 yaw만**(쿼터니언의 x·y는 0). 단위: `vision_node`의 `fixed_pose:=[x, y, yaw]`는 x·y가 m, **yaw가 도(°)**(반시계, `pick_place --cube x y yaw`와 같다). **position.z는 사용하지 않는다**(`fixed_pose`에 높이가 없어 0.0). 카메라 모드(`result_json`)에서는 변환한 z가 실린다(`arm_control`은 쓰지 않는다). `arm_control_node`는 지금 이 토픽을 로그로만 찍고 로봇을 움직이지 않는다(카메라 좌표 변환이 아직 없다).

서보 프레임: 표준 ID 5, `[motor_id 0..9][각도×10 H][L]`, 0~1800 (team-jax/CAN_STM `src/main.c`의 `CAN_SERVO_STDID 5U`). stm_bridge는 `/servo/command`에서 0~180°를 벗어난 값이 하나라도 있거나 이름·길이·NaN이 틀린 메시지를 **통째로 버린다**.

## 설치 (Ubuntu 24.04, ROS 2 Jazzy)

```bash
sudo apt install -y can-utils python3-can python3-colcon-common-extensions
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone <이 레포> jax_ros      # 또는 폴더를 그대로 둔다
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
sudo ip link set can0 up type can bitrate 1000000       # 재부팅마다 1회
candump can0                                            # 00002901~00002904 확장 프레임(AK45 피드백)이 보여야 한다
```

`--symlink-install`이면 `calibrate`가 저장하는 `arm_control/calibration.json`이 저장소 파일 그대로다. `pick_place`까지 쓰려면 IK(`sim-table-friction` 브랜치)와 MuJoCo가 필요하다. 설치는 `arm_control/arm_control/planner.py` 맨 위 설명을 본다.

`ak45_node`의 CAN 인터페이스 이름은 컴파일 타임 상수 `can0`이다. `servo_channel`만 바꿀 수 있다(`ak45_ros2/README.md` §2).

## 실행

```bash
ros2 launch arm_control arm.launch.py            # ak45_node + stm_bridge_node + arm_control_node
#   servo_channel:=can0  max_speed_deg_s:=30.0  calibration_file:=<경로>
ros2 topic pub -t 3 -r 10 /arm_control/joint_command sensor_msgs/msg/JointState \
  "{name: ['elbow', 'wrist_pitch'], position: [0.0, 0.349]}"
ros2 run arm_control pick_place --cube 0.10 -0.30 20 --target 0.05 -0.27 --dry-run    # 계획과 검사만
ros2 run vision_node vision_node --ros-args -p fixed_pose:="[0.10, -0.30, 20.0]"      # 시험용 /object_pose
ros2 run vision_node vision_node --ros-args -p result_json:=<runs 폴더>/cube_test/inference.json   # 카메라 결과(camera 프레임)
```

### vision_node 카메라 모드 (`result_json`)
카메라 프로그램(SteroDepthSeg, 자체 venv)은 `--out` 폴더의 `inference.json`을 약 1초마다 덮어쓴다. `vision_node`는 그 파일만 읽는다(카메라 코드·torch를 import하지 않는다).
- 파라미터: `result_json`(SteroDepthSeg `--out` 폴더의 `inference.json` 경로, 기본 빈 문자열), `poll_hz`(2), `max_age_s`(3), `cube_labels`(["blob", "kite"], `["*"]`이면 모든 라벨), `cube_dim_mm`([40, 80]), `depth_range_m`([0.3, 1.0]), `camera_to_robot`([tx, ty, tz, roll_deg, pitch_deg, yaw_deg], 기본 빈 목록). `fixed_pose`와 같이 주면 오류, 발행 없음.
- 후보: `label`이 `cube_labels`에 있음(기본 `blob`, `kite`: COCO/YOLO가 큐브를 `kite`로 분류한 실제 사례. `["*"]`이면 전부), `status == "ok"`, `dims_mm`의 가로·세로가 `cube_dim_mm` 안, `depth_rect_m`가 `depth_range_m` 안. **정확히 1개일 때만** 발행(0개·2개 이상이면 이유만 로그: 버려진 객체마다 label / status / dims / depth 중 무엇 때문인지 포함). 파일 수정 시각이 `max_age_s`보다 오래됐거나 JSON이 깨졌으면 발행하지 않는다(로그). 같은 파일 내용은 다시 발행하지 않는다.
- 좌표: `camera_to_robot`이 비면 `frame_id = camera`(카메라 좌표 그대로, OpenCV: x 오른쪽·y 아래·z 앞, **로봇 좌표 아님**), 있으면 `p_robot = R·p_cam + t`(R = Rz(yaw)·Ry(pitch)·Rx(roll))로 바꿔 `frame_id = shoulder_roll_axis`. 둘 다 **임시**(폰 고정 방법 미정). **yaw는 카메라 기준**(`rotation.yaw_deg` 그대로, 로봇 좌표 yaw로 바꾸지 않음). 이 경우 시작할 때 "임시: 카메라 좌표, 로봇 좌표 아님"을 로그로 남긴다.

### 보정 (calibrate)

```bash
ros2 run ak45_ros2 ak45_node &                       # 또는 ros2 launch ak45_ros2 ak45_node.launch.py
ros2 run stm_bridge stm_bridge_node
ros2 run arm_control calibrate --ros                 # ROS가 없는 곳: python3 arm_control/calibrate.py (dry-run)
```

> ⚠️ **`calibrate --ros`를 쓸 때는 `arm_control_node`를 띄우지 않는다**(그래서 `arm.launch.py`도 쓰지 않는다). 둘 다 같은 모터에 명령을 보내게 된다. `ak45_node`와 `stm_bridge_node`만 띄운다.
> 서보 명령은 `/servo/command`를 받는 `stm_bridge`가 보여야 나간다. 5초 안에 안 보이면 "no subscriber on /servo/command … nothing sent"로 거절한다. 명령은 `arm_control/arm_control/calibrate.py` 맨 위에 있다. `z`/`f`/`v`/`w`로 영점·방향·확인·저장.

### 원점 잡기 (homing_guard, 전원 재투입 대비)

AK45-36이 단일 엔코더면 모터 전원이 꺼졌다 켜질 때 원점이 날아간다(듀얼 엔코더 여부 미확인, `ak45_ros2/docs/ros2.md` 10-7). `arm_homed.launch.py`는 `arm.launch.py`와 같지만 `arm_control_node`와 `ak45_node` 사이에 `homing_guard`를 끼워, **이번 실행에서 원점을 잡은 AK45만 움직이게** 한다. 기존 노드·`calibration.json`은 그대로다(`zero_deg`는 "처음 기대값"으로만 쓴다).

```bash
ros2 launch arm_control arm_homed.launch.py          # 시작하면 AK45 4개 모두 미원점
# 팔을 원점 자세(homing.json의 home_sim_deg, 기본 시뮬 0°)에 두고:
ros2 service call /arm_control/home_all std_srvs/srv/Trigger           # 또는 /arm_control/home_elbow 등 관절별
ros2 topic echo /arm_control/homing                                    # 관절별 homed, 이유, 오프셋 (JSON)
```

- 미원점 관절: 명령은 경고 후 버리고, 피드백을 `arm_control_node`에 넘기지 않는다(그래서 "no /joint_states … waits").
- 원점 = 지금 피드백 − sign·scale·home_sim_deg. 피드백이 없거나 0.5 s보다 오래되면 거부. `calibration.json`의 `zero_deg`와 20° 넘게 다르면 경고만 한다.
- **원점은 한 번의 홀드가 이어지는 동안만 유효하다**(쓸 때마다 다시 잡는다). 아래 중 하나면 미원점으로 돌아가고 **다시 원점을 잡아야 움직인다**:
  1. guard(스택)가 새로 시작됨
  2. 그 모터의 `/joint_states`가 1.0 s 넘게 끊김(`gap_s`) 또는 연속 두 값이 60° 넘게 튐(`jump_deg`) — 전원 재투입. 피드백이 돌아오면 `ak45_node`에 "지금 자리 유지" 명령을 보낸다
  3. `ak45_node`가 그 모터 명령 송신을 멈춤: `/diagnostics`의 `ak45/<이름>` `command_active`가 1→0(타임아웃·온도 잠김·피드백/워치독 끊김·송신 실패)
  4. 쉼 요청: `ros2 service call /arm_control/release_all std_srvs/srv/Trigger`(또는 `release_<관절>`). guard가 그 관절 명령 전달을 끊는다. **`ak45_node`는 마지막 목표를 계속 홀드하므로 실제로 힘을 빼려면 `ak45_node`를 종료해야 한다**(먼저 팔을 받친다, 아래 "주의")
- 원점 잡은 뒤의 명령은 guard가 피드백에서부터 `max_speed_deg_s`로 다시 램프한다(옛 목표로 튀지 않게).
- `play_trajectory`/`pick_place`를 따로 띄울 때는 `--ros-args -r /joint_states:=/ak45/joint_states_homed`를 붙인다(감시가 보정된 각도를 보게). 미원점 관절이 있어도 시작은 거부하지 않는다 — guard가 명령을 막는다.
- `calibrate --ros`는 guard를 거치지 않는다(원래대로 `ak45_node`에 직접).

### 주의

- **종료하면 AK45가 힘을 놓는다.** `ak45_node`가 끝날 때 `ak45_can`이 Current Brake 0A 프레임을 ID 1~**6**에 보낸다(`NUM_MOTORS=6`이라 5·6은 받는 모터가 없다). 먼저 팔을 받친다. AK45는 확장 29비트 ID, STM 서보 보드는 표준 11비트 프레임만 받으므로(`main.c`의 IDE 검사) 지금은 충돌하지 않지만, **나중에 ID 5·6 AK45를 추가하면 이 브레이크 프레임이 그 모터에 간다.**
- **`ak45_node`는 CAN 프레임이 더 안 들어오는 상태에서는 SIGINT/SIGTERM으로 끝나지 않을 수 있다**(`ak45_close()`가 소켓을 닫아 블로킹 읽기를 깨우려 하는데 깨어나지 않는다. 프레임이 오면 1초에 끝난다). 모터 전원이 꺼졌을 때는 `kill -KILL`이 필요할 수 있다. **실물 시험 때는 두 번째 터미널을 열어 두고 `pkill -KILL -f ak45_node`를 바로 칠 수 있게 한다.**
- `calibration.json`의 값은 전부 `verified: false`(영점·방향·관절↔모터 대응이 가정)다. `pick_place`/`play_trajectory`는 `verified`가 아닌 관절이 있으면 거절한다(`--force`로 넘길 수 있다).
- AK45 위치가 출력축 기준인지 로터 기준인지(V3)는 문서마다 다르다. **첫 실제 명령은 2° 정도로 보내 출력축이 얼마나 도는지 눈으로 확인한 뒤** 진행한다.

## 시험 (실물 없이)

CAN은 `vcan`으로 시험한다(`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`). **실제 모터로는 시험하지 않았다.**

| 명령 | 확인하는 것 |
|---|---|
| `python3 -I stm_bridge/tests/check_protocol.py` | 서보 프레임 바이트, 이름→id 표 = `calibration.json` |
| `python3 stm_bridge/tests/check_node.py` | stm_bridge_node (python-can virtual 버스) |
| `python3 -I arm_control/tests/check_protocol.py --ik ~/IK` | 원본 SimToReal과 같은 58개 검사 |
| `python3 arm_control/tests/check_node.py` | arm_control_node → stm_bridge_node 전체 체인(원본 기대값 그대로) |
| `python3 arm_control/tests/check_split.py --original <SimToReal>` | 쪼개기 전/후 같은 입력, 같은 출력(틱 단위), 서보 0~180° 왕복 |
| `python3 arm_control/tests/check_calibrate_wait.py`, `bash arm_control/tests/calibrate_vcan.sh` | calibrate가 stm_bridge를 기다림 |
| `bash arm_control/tests/chain_vcan.sh` | vcan0에서 50 Hz, 발행한 서보 수 = candump 프레임 수 |
| `bash arm_control/tests/launch_vcan.sh` | `arm.launch.py` 전체(별도 네트워크 공간에 `can0`를 만들어 sudo가 필요 없다) |
| `bash arm_control/tests/check_topics.sh` | NODES.md §4 토픽 표(타입, 발행·구독 노드, 메시지 내용)를 실행 중인 시스템과 대조 |
| `python3 vision_node/tests/check_node.py` | `fixed_pose`가 없거나 틀리면 발행 없음, 있으면 1 Hz |
| `python3 vision_node/tests/check_camera_result.py` | ROS 없이 `inference.json` 파싱·큐브 선택·오래된 파일·임시 변환(손계산 값) |
| `python3 vision_node/tests/check_camera_mode.py` | `result_json` 모드 실제 `ros2 run`(파일 변경·깨진 JSON·후보 2개·오래된 파일·잘못된 파라미터) |

시험 스크립트를 쓸 때 `pkill -f`·`pgrep -f` 패턴이 같은 셸 명령 본문에 들어 있으면 자기 자신이 매칭된다. 스크립트 파일로 실행한다.

## 문서

- `STATUS.md` — 했던 일, 하고 있는 일, 할 일, 목표, 기술, 문제점과 해결방안, 단계별 검증 결과.
- `NODES.md` — 노드 구성과 토픽 표(2026-11-07에 이 구현에 맞춰 고침). 표는 `arm_control/tests/check_topics.sh`로 실행 중인 시스템과 대조한다.
- `ARCHITECTURE.md` — **아직 이 폴더에 없다.**

