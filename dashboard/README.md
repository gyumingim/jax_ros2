# dashboard — 노드 켜기/끄기 + AK45 상태 웹 화면

터미널 여러 개 대신 브라우저 한 화면에서 `can0` 올리기, `ak45_node`·`stm_bridge_node`·`arm_control_node`(또는 `arm.launch.py` 전체)·`vision_node` 켜기/끄기/재시작, 로그 보기, vcan 시험을 한다. **모터를 움직이는 버튼은 없다**(읽기 + 프로세스 관리만).

## 실행 (로봇 PC, Ubuntu 24.04 / Jazzy)

```bash
sudo apt install -y python3-fastapi python3-uvicorn python3-websockets   # 또는 pip install fastapi uvicorn websockets
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash   # AK45 상태 칸(rclpy)이 필요할 때
python3 dashboard/server.py
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

## 화면

- 위: 연결·ROS·can0 상태 칩, 실행 개수, 큰 **■ 전부 정지**(Esc). 아래 띠에 경고: 정지하면 `ak45_node` 가 끝나며 브레이크 0A → **홀드가 풀려 팔이 떨어질 수 있다.** 터미널에서 켠 노드가 있으면 빨간 띠로 알린다(전부 정지로 안 꺼짐).
- **AK45 상태**: `/diagnostics`·`/joint_states` 구독. 모터 4개 카드(위치°, 온도, 에러 코드, 워치독, target, command_active, 오차, 전류, 속도) + CAN 버스 카드(state, 에러 프레임, 버스오프). 3초 넘게 안 오면 '오래됨'. rclpy 가 없으면 'ROS 없음'만 보이고 나머지는 그대로 돈다.
- **켜기/끄기**: 항목마다 상태(켜짐 / 죽음+종료코드 / 가동 시간 / pid), 파라미터 칸, 켜기·끄기·재시작, 실제 실행한 명령, 로그 마지막 30줄(`/api/log/<id>?n=1000` 더 보기).
- **보정**: `calibrate --ros` 는 키보드 대화형이라 웹에서 돌리지 않는다. 명령 안내 + 'ak45_node + stm_bridge_node 켜기' 버튼 + `arm_control_node`/`arm.launch.py` 가 켜져 있으면 빨간 경고.
- **원점(homing)**: 빈 자리(다음 단계).

## 동작 규칙

- 항목마다 새 프로세스 그룹: `bash -lc "source /opt/ros/<distro>/setup.bash; source <ws>/install/setup.bash; exec stdbuf -oL -eL ros2 …"`. 로그는 `dashboard/logs/<id>.log`(git 무시).
- 끄기: 그룹에 **SIGINT → 5초 → SIGTERM → 3초 → SIGKILL**. 어느 단계에서 끝났는지 카드에 표시. SIGKILL 까지 갔으면 빨간 글씨(`ak45_node` 가 프레임이 끊기면 SIGINT 로 안 끝나는 알려진 문제, 루트 README·STATUS.md).
- 같이 켜면 안 되는 것은 켜기를 거절: `arm.launch.py` ↔ 개별 `ak45`/`stm_bridge`/`arm_control`. 대시보드 밖(터미널)에서 같은 노드가 이미 떠 있어도 거절.
- `can0 올리기`: 이미 UP 이면 그대로, 아니면 `sudo -n` → `pkexec`(로봇 PC 화면에 암호창) 순서. 둘 다 안 되면 로그에 복사할 명령 `sudo ip link set can0 up type can bitrate 1000000`.
- 대시보드 서버를 꺼도 노드는 **그대로 둔다**(끄면 팔이 떨어질 수 있어서). 다시 켜면 `logs/pids.json` 으로 다시 붙잡는다(Linux).
- 대시보드는 `jax_dashboard` 라는 ROS 노드로 `/diagnostics`, `/joint_states` 를 구독한다. `arm_control/tests/check_topics.sh` 를 돌릴 때는 대시보드를 끄거나 ROS 없이 켠다(구독자가 하나 늘어 표와 달라질 수 있음).

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

## 확인 안 한 것

- **Linux 쪽 전부**: 실제 `ros2 launch/run` 켜기·끄기, 프로세스 그룹 SIGINT/SIGTERM/SIGKILL, `ak45_node` 가 안 끝나는 경우 표시, `can0` 올리기(sudo -n / pkexec), `ip -json` can0 상태 칩, 터미널 노드 감지(/proc), `pids.json` 다시 붙잡기, rclpy 구독과 AK45 카드, `launch_vcan.sh` 버튼(레포가 `<ws>/src/` 안에 있어야 한다).
- 폰 화면은 CSS 만 맞춰 두었고 실제 폰으로 보지 않았다.
- Ubuntu apt 의 fastapi 버전(0.101)에서 도는지.

## 다음 단계

1. 로봇 PC 에서 위 '확인 안 한 것' 시험(먼저 `JAX_DASH_DEMO=1` 가짜 항목, 다음 vcan 시험 버튼, 그다음 실물).
2. **원점(homing) 패널**: `homing` 브랜치의 AK45 원점 guard 노드(전원 재투입 시 원점 다시 잡기, 원점 서비스, homing 상태 토픽)가 들어오면 `services.py` 에 항목 추가, `ros_monitor.py` 의 `homing` 칸에 상태 토픽 구독, 화면 '원점' 칸에 상태와 원점 서비스 버튼.
3. **모터 명령 패널**(이번엔 일부러 안 만듦): 작은 각도(2°) 한 관절 명령, 누르는 동안만 움직이기, 원점 확인 후에만 활성, 속도 제한 표시.
4. '정지'의 뜻 정하기: 지금은 노드 종료 = 브레이크 0A(팔 떨어짐). 홀드 유지 정지(현재 위치를 target 으로)가 필요한지.
5. 서보(`/servo/state`) 칸, `/object_pose` 표시, systemd 자동 실행과 연결.
