# BRINGUP — 로봇 한 번에 켜기

`tools/bringup.sh` 하나로 **이전 프로세스를 항상 끄고**, CAN을 다시 올리고, 전체 노드를 띄운다.
팔을 실제로 움직이는 명령(`go_zero.sh`, `/approach/start`)은 따로, 사람이 직접 한다.

## 0. 처음 한 번: 빌드

`~/ros2_ws/src` 아래에 같은 패키지 이름의 폴더가 두 개 있으면(예: 옛 `jax_ros`) 빌드가 충돌한다. 옛 폴더를 빼고 빌드한다.

```bash
touch ~/ros2_ws/src/jax_ros/COLCON_IGNORE          # 옛 폴더가 있을 때만
cd ~/ros2_ws && colcon build --symlink-install
```

코드를 받은 뒤(`git pull`)에도 다시 빌드한다. `bringup.sh`는 `~/ros2_ws/install/setup.bash`를 스스로 source한다.

## 1. 켜기

```bash
cd ~/ros2_ws/src/jax_ros2_model-approach

# 카메라 모드 (result_json 생략 시 ~/Desktop/StereoDepthSeg/phone_stereo/runs/cube_test/inference.json)
tools/bringup.sh camera
tools/bringup.sh camera result_json:=/경로/inference.json

# 카메라 없이 고정 위치로 시험 ([x m, y m, yaw 도], shoulder_roll_axis 좌표)
tools/bringup.sh fixed "[0.10, -0.30, 20.0]"

# 뒤에 붙인 name:=value 는 launch 에 그대로 전달된다
tools/bringup.sh camera force:=true max_speed_deg_s:=20.0
```

`bringup.sh`가 하는 일 (순서대로):

| 단계 | 내용 |
|---|---|
| 0 | `arm_control/calibration.json`의 AK45 4개 줄(sign, zero_deg, verified)을 **출력만** 한다(수정 안 함) |
| 1 | "모터 토크가 풀려 팔이 떨어질 수 있다" 확인 **y/N** — 팔을 받치고 y |
| 2 | 이전 프로세스 전부 종료: `ak45_node`, `stm_bridge_node`, `arm_control_node`, `vision_node`, `approach_node`, `calibrate`, 모든 `ros2 launch` / `ros2 run`. SIGINT 후 5초 안에 안 끝나면 SIGKILL |
| 3 | `/tmp/ak45_ctrl.lock`이 풀릴 때까지 최대 10초 대기. 안 풀리면 잡고 있는 프로세스를 보여주고 **중단** |
| 4 | `sudo ip link set can0 down` → `sudo ip link set can0 up type can bitrate 1000000` (sudo 비밀번호를 물을 수 있음) |
| 5 | `ros2 launch arm_control approach.launch.py ...` 실행 (이 터미널에 로그가 나온다. 끄려면 Ctrl+C) |

뜨는 노드 5개: `/ak45_node`, `/stm_bridge`, `/arm_control_node`, `/vision_node`, `/approach_node`.
`ak45_node`는 launch로 켜지므로 `ak45_ros2/config/ak45_node.yaml`이 적용된다(온도 상한 75℃, 각도 상한 360°).

다시 켜고 싶으면 그냥 `tools/bringup.sh ...`를 한 번 더 실행하면 된다. 앞의 것은 2단계에서 꺼진다.

## 2. 확인 (다른 터미널)

```bash
source ~/ros2_ws/install/setup.bash
ros2 node list                          # 위 5개
ros2 topic echo /object_pose --once     # frame_id shoulder_roll_axis, 큐브 위치
ros2 run ak45_ros2 ak45_deg --list      # AK45 4개 현재 각도
```

## 3. 팔 움직이기 (사람이 직접)

```bash
tools/go_zero.sh                                              # AK45 4개를 시뮬 0 자세로 (y/N 확인)
ros2 service call /approach/start std_srvs/srv/Trigger        # 접근 시작
ros2 service call /approach/stop  std_srvs/srv/Trigger        # 멈춤
```

- `approach_node`는 시작할 때 모든 시뮬 각도가 0인 자세라고 가정한다. **`go_zero.sh`를 먼저** 한다.
- AK45가 명령에서 15° 넘게 0.5초 벗어나면 `approach_node`가 멈춘다.
- `fixed` 모드는 z가 0이라 `/approach/start`가 책상 높이 검사(`max_z_error_m`)에서 거부한다. 켜지는지 확인용이다.

## launch 인자 (`approach.launch.py`)

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `result_json` | (없음) | 카메라 프로그램의 `inference.json`. `fixed_pose`와 둘 중 하나만 |
| `fixed_pose` | (없음) | `[x, y, yaw_deg]` 고정 위치 |
| `camera_to_robot` | 빈 값 = `vision_node/config/camera.yaml` | `[tx, ty, tz, roll, pitch, yaw]`로 덮어쓰기 |
| `force` | `false` | 보정 미검증이어도 시작 |
| `send_finger_rotation` | `false` | finger_rotation 명령 보내기 |
| `max_z_error_m` | `0.05` | 큐브 z 허용 오차 |
| `table_e` | `0.0` | 책상 오프셋 |
| `rate_hz` | `60.0` | 명령 주기 |
| `servo_channel` | `can0` | 서보 CAN 인터페이스 |
| `max_speed_deg_s` | `30.0` | 모든 관절 속도 상한 |
| `calibration_file` | 빈 값 = 패키지의 `calibration.json` | 보정 파일 |

## 문제가 생기면

| 증상 | 확인 |
|---|---|
| `stop: /tmp/ak45_ctrl.lock is still held by` | 출력된 PID가 무엇인지 보고 직접 종료 (`kill <pid>`) |
| `stop: can0 did not come up` | USB-CAN 연결, `ip -br link`에 can0가 있는지 |
| `ak45_init() 실패` | can0가 올라왔는지, 다른 AK45 프로그램(대시보드 등)이 떠 있는지 |
| `give exactly one of result_json ... fixed_pose` | `camera`/`fixed` 모드를 정확히 하나 |
| `/object_pose`가 안 나옴 (camera) | `inference.json`이 3초 안에 갱신되는지(카메라 프로그램이 돌고 있는지) |
| 끌 때 노드마다 `KeyboardInterrupt` Traceback | 종료 신호로 끝날 때 나오는 메시지. 동작에는 영향 없음 |

## 시험용 환경 변수

| 변수 | 뜻 |
|---|---|
| `BRINGUP_YES=1` | y/N 질문 생략 |
| `BRINGUP_CAN=skip` | can0를 건드리지 않음 (vcan 시험용) |
| `BRINGUP_SETUP=<setup.bash>` | 다른 install을 source |
