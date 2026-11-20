"""대시보드가 켜고 끄는 항목 목록. 새 항목은 SERVICES 에 Service(...) 하나를 더하면 된다.

Service 필드
  id        영문 짧은 이름(로그 파일 이름 dashboard/logs/<id>.log)
  label     화면 이름
  kind      "proc"  계속 도는 노드(켜기/끄기/재시작)
            "once"  한 번 돌고 끝나는 명령(실행 버튼, 끝나면 종료코드 표시)
  build     params(dict) -> 문자열(bash 본문, ROS 환경을 source 한 뒤 실행) 또는 list(argv, 그대로 실행)
            값이 틀리면 ValueError("이유") 를 던진다(화면에 그대로 뜸)
  ros       True 면 /opt/ros/<distro>/setup.bash 와 <ws>/install/setup.bash 를 먼저 source
  params    화면 입력칸 [Param(key, label, default, hint)]
  conflicts 동시에 켜면 안 되는 항목 id(켜기를 거절)
  procs     이 항목이 띄우는 실행 파일 이름(대시보드 밖에서 이미 떠 있는지 pgrep 으로 검사)
  stop_warn 끌 때 확인창 문구(비우면 확인 없이 끔)
  linux     True 면 Linux 에서만(Windows 에서는 '이 PC에서 못 씀')
  demo      True 면 JAX_DASH_DEMO=1 이거나 Linux 가 아닐 때만 보임(시험용 가짜 항목)
"""
import os
import re
import shlex
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Union

REPO = Path(__file__).resolve().parent.parent


def _default_ws():
    # README: ~/ros2_ws/src/<이 레포>. 레포가 어떤 src 폴더 안에 있으면 그 워크스페이스를 쓴다.
    if REPO.parent.name == "src":
        return str(REPO.parent.parent)
    return os.path.expanduser("~/ros2_ws")


WS = os.environ.get("JAX_WS") or _default_ws()
ROS_DISTRO = os.environ.get("JAX_ROS_DISTRO") or os.environ.get("ROS_DISTRO") or "jazzy"
CAN_IFACE = "can0"   # ak45_node 의 CAN 이름은 컴파일 타임 상수 can0 (README)
CAN_UP_CMD = f"sudo ip link set {CAN_IFACE} up type can bitrate 1000000"


@dataclass
class Param:
    key: str
    label: str
    default: str = ""
    hint: str = ""


@dataclass
class Service:
    id: str
    label: str
    build: Callable[[dict], Union[str, List[str]]]
    kind: str = "proc"
    ros: bool = True
    group: str = "노드"
    help: str = ""
    params: List[Param] = field(default_factory=list)
    conflicts: tuple = ()
    procs: tuple = ()
    stop_warn: str = ""
    linux: bool = True
    demo: bool = False


# ---------- 값 검사 ----------
_NUM = r"\s*-?\d+(?:\.\d+)?\s*"


def _num(v, name, lo=None, hi=None):
    try:
        x = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: 숫자가 아님 ({v!r})")
    if (lo is not None and x < lo) or (hi is not None and x > hi):
        raise ValueError(f"{name}: {lo}~{hi} 범위 밖 ({x})")
    return x


def _iface(v, name):
    v = (v or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", v):
        raise ValueError(f"{name}: 인터페이스 이름이 이상함 ({v!r})")
    return v


# ---------- 명령 ----------
AK45_STOP_WARN = ("ak45_node 를 끄면 AK45 에 브레이크 0A 가 가서 홀드가 풀린다. 팔이 떨어질 수 있다.\n"
                  "팔을 받쳤나요?")


def _can_up(p):
    # 이미 UP 이면 아무것도 안 함. sudo -n(암호 없이 되는 경우) → pkexec(로봇 PC 화면에 암호창) 순서로 시도.
    up = f"ip link set {CAN_IFACE} up type can bitrate 1000000"
    return f"""
if ip link show {CAN_IFACE} 2>/dev/null | grep -qE '<([^>]*,)?UP(,[^>]*)?>'; then
  echo "{CAN_IFACE} 이미 UP"; ip -details -statistics link show {CAN_IFACE}; exit 0
fi
ip link show {CAN_IFACE} >/dev/null 2>&1 || {{ echo "{CAN_IFACE} 없음 (USB-CAN 연결 확인)"; exit 4; }}
if sudo -n {up} 2>/dev/null; then echo "sudo -n 으로 올림"
elif command -v pkexec >/dev/null && timeout 60 pkexec {up}; then echo "pkexec 으로 올림"
else
  echo "권한이 없어 못 올림. 터미널에서 직접:"; echo "  {CAN_UP_CMD}"; exit 5
fi
ip -details link show {CAN_IFACE}
"""


def _arm_launch(p):
    args = ["ros2", "launch", "arm_control", "arm.launch.py",
            f"servo_channel:={_iface(p.get('servo_channel'), 'servo_channel')}",
            f"max_speed_deg_s:={_num(p.get('max_speed_deg_s'), 'max_speed_deg_s', 0.1, 360)}"]
    cal = (p.get("calibration_file") or "").strip()
    if cal:
        args.append(f"calibration_file:={cal}")
    return "exec " + " ".join(shlex.quote(a) for a in args)


def _stm_bridge(p):
    ch = _iface(p.get("servo_channel"), "servo_channel")
    return f"exec ros2 run stm_bridge stm_bridge_node --ros-args -p servo_channel:={ch}"


def _arm_control(p):
    a = ["ros2", "run", "arm_control", "arm_control_node", "--ros-args",
         "-p", f"max_speed_deg_s:={_num(p.get('max_speed_deg_s'), 'max_speed_deg_s', 0.1, 360)}"]
    cal = (p.get("calibration_file") or "").strip()
    if cal:
        a += ["-p", f"calibration_file:={cal}"]
    return "exec " + " ".join(shlex.quote(x) for x in a)


def _vision(p):
    fp = (p.get("fixed_pose") or "").strip()
    rj = (p.get("result_json") or "").strip()
    if bool(fp) == bool(rj):
        raise ValueError("fixed_pose 와 result_json 중 하나만 채운다")
    a = ["ros2", "run", "vision_node", "vision_node", "--ros-args"]
    if fp:
        if not re.fullmatch(rf"\[{_NUM},{_NUM},{_NUM}\]", fp):
            raise ValueError("fixed_pose 형식: [x_m, y_m, yaw_deg] 예 [0.10, -0.30, 20.0]")
        a += ["-p", f"fixed_pose:={fp}"]
    else:
        a += ["-p", f"result_json:={rj}"]
    return "exec " + " ".join(shlex.quote(x) for x in a)


def _vcan(p):
    return f"exec bash {shlex.quote(str(REPO / 'arm_control' / 'tests' / 'launch_vcan.sh'))}"


PY = sys.executable

SERVICES = [
    Service("can_up", "can0 올리기", _can_up, kind="once", ros=False, group="준비",
            help=f"재부팅마다 1번. sudo 권한이 필요하다. 안 되면 터미널에서: {CAN_UP_CMD}"),
    Service("arm_launch", "arm.launch.py (3개 한 번에)", _arm_launch, group="노드",
            help="ak45_node + stm_bridge_node + arm_control_node. 보정(calibrate --ros) 중에는 켜지 않는다.",
            params=[Param("servo_channel", "servo_channel", "can0"),
                    Param("max_speed_deg_s", "max_speed_deg_s", "30.0", "°/s"),
                    Param("calibration_file", "calibration_file", "", "비우면 패키지 기본")],
            conflicts=("ak45", "stm_bridge", "arm_control"),
            procs=("ak45_node", "stm_bridge_node", "arm_control_node"),
            stop_warn=AK45_STOP_WARN),
    Service("ak45", "ak45_node", lambda p: "exec ros2 launch ak45_ros2 ak45_node.launch.py", group="노드",
            help="AK45 4개(ID 1~4). /joint_states, /diagnostics 발행, /ak45/command 수신.",
            conflicts=("arm_launch",), procs=("ak45_node",), stop_warn=AK45_STOP_WARN),
    Service("stm_bridge", "stm_bridge_node", _stm_bridge, group="노드",
            help="/servo/command → STM32 서보 CAN 프레임(StdId 5).",
            params=[Param("servo_channel", "servo_channel", "can0")],
            conflicts=("arm_launch",), procs=("stm_bridge_node",)),
    Service("arm_control", "arm_control_node", _arm_control, group="노드",
            help="/arm_control/joint_command → /ak45/command, /servo/command. calibrate --ros 와 같이 켜지 않는다.",
            params=[Param("max_speed_deg_s", "max_speed_deg_s", "30.0", "°/s"),
                    Param("calibration_file", "calibration_file", "", "비우면 패키지 기본")],
            conflicts=("arm_launch",), procs=("arm_control_node",)),
    Service("vision", "vision_node", _vision, group="노드",
            help="둘 중 하나만: fixed_pose(시험용) 또는 result_json(카메라 inference.json 경로).",
            params=[Param("fixed_pose", "fixed_pose", "[0.10, -0.30, 20.0]", "[x m, y m, yaw °]"),
                    Param("result_json", "result_json", "", "…/inference.json")],
            procs=("vision_node",)),
    Service("vcan_test", "vcan 시험 (launch_vcan.sh)", _vcan, kind="once", group="시험",
            help="실물 없이 arm.launch.py 전체 시험. 별도 네트워크 공간의 가짜 can0 라 실제 버스·모터와 상관없다. 약 30초.",
            ),
    # ---- 시험용 가짜 항목 (Windows 또는 JAX_DASH_DEMO=1) ----
    Service("demo_counter", "가짜: 숫자 세기", lambda p: [PY, "-u", "-c",
            "import time\nfor i in range(100): print(i, flush=True); time.sleep(1)"],
            ros=False, linux=False, demo=True, group="시험(가짜)", help="켜기·로그·끄기 시험용."),
    Service("demo_stubborn", "가짜: SIGINT 무시", lambda p: [PY, "-u", "-c",
            "import signal,time\nfor s in ('SIGINT','SIGBREAK'):\n"
            "    getattr(signal,s,None) and signal.signal(getattr(signal,s), signal.SIG_IGN)\n"
            "i=0\nwhile True: print('stubborn', i, flush=True); i+=1; time.sleep(1)"],
            ros=False, linux=False, demo=True, group="시험(가짜)",
            help="ak45_node 가 SIGINT 로 안 끝나는 경우 흉내. 끄면 SIGTERM/SIGKILL 단계가 보여야 한다.",
            stop_warn="가짜 항목 확인창 시험"),
]

# 보정은 키보드 대화형(z/f/v/w)이라 웹에서 돌리지 않는다. 화면에는 명령 안내만.
CALIBRATE_GUIDE = {
    "needs": ["ak45", "stm_bridge"],
    "forbid": ["arm_launch", "arm_control"],
    "commands": [
        f"source /opt/ros/{ROS_DISTRO}/setup.bash && source {WS}/install/setup.bash",
        "ros2 run arm_control calibrate --ros",
    ],
    "note": "ak45_node 와 stm_bridge_node 만 켜고(위 버튼), arm_control_node·arm.launch.py 는 끈 상태에서 터미널로 실행. "
            "키: z 영점 / f 방향 / v 확인 / w 저장.",
}

# 대시보드 밖(터미널)에서 떠 있는지 볼 실행 파일 이름
WATCH_PROCS = ("ak45_node", "stm_bridge_node", "arm_control_node", "vision_node", "calibrate")


def visible_services():
    demo = os.environ.get("JAX_DASH_DEMO") == "1" or not sys.platform.startswith("linux")
    return [s for s in SERVICES if demo or not s.demo]
