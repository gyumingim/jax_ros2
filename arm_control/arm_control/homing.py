"""Session homing of the AK45s, without ROS (homing_guard.py is the ROS side).

Why: an AK45-36 with a single encoder may lose its zero every time its power goes off and on (dual
encoder or not is still unknown, ak45_ros2/docs/ros2.md 10-7). calibration.json's zero_deg is then
only what the zero was the first time, not what it is now.

A zero is good only while one hold lasts (robot_arm v2's rule: rest the arm, and the next use homes
again). Every AK45 starts NOT homed, and goes back to not homed (reason in `reason`) when:
  - the guard starts (a new stack = a new session),
  - its /joint_states feedback stopped for more than gap_s, or two feedbacks in a row are more than
    jump_deg apart (power cycle),
  - ak45_node stopped sending it commands: /diagnostics ak45/<name> command_active went 1 -> 0
    (command timeout, temperature lock, feedback or watchdog lost, send error),
  - it was released (rest) on request.
  - homing = the joint is put by hand (or by the dashboard) in its known home pose, home_sim_deg (sim
    angle, default 0), and home() is asked: session zero = feedback now - sign * scale * home_sim_deg.
    Refused without feedback, or with feedback older than max_age_s.
  - offset = session zero - calibration zero_deg. arm_control keeps working with calibration.json's
    zero_deg; the guard adds the offset to its commands and takes it off the feedback it passes back.

Angles are motor degrees, as on /ak45/command and /joint_states.
"""
from dataclasses import dataclass
import json
from pathlib import Path

DEFAULT_FILE = Path(__file__).resolve().parent.parent / "homing.json"


@dataclass
class MotorHoming:
    joint: object                 # calibration.Joint (an AK45)
    home_sim_deg: float = 0.0
    homed: bool = False
    offset_deg: float = 0.0       # session zero - zero_deg
    reason: str = "not homed since start"
    raw_deg: float = None         # last feedback, raw motor degrees
    seen: float = None            # when (monotonic s)
    lost: bool = False            # feedback stopped for more than gap_s
    active: bool = None           # last /diagnostics command_active (None: none seen yet)

    @property
    def session_zero_deg(self):
        return self.joint.zero_deg + self.offset_deg if self.homed else None


def load_home_poses(path=None):
    """{joint: home_sim_deg} from homing.json; {} when the file is missing (every home pose = sim 0)."""
    if path:
        path = Path(path)        # given: it must exist
    elif DEFAULT_FILE.exists():
        path = DEFAULT_FILE      # the repository's (also under colcon build --symlink-install)
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory("arm_control")) / "homing.json"
        except Exception:
            return {}
        if not path.exists():
            return {}
    raw = json.loads(path.read_text(encoding="utf-8"))
    return {k: float(v) for k, v in raw.get("home_sim_deg", {}).items()}


class Homing:
    """Per-AK45 homing state. Keyed by ak45_node's names (ak45_1..4)."""

    def __init__(self, joints, home_poses=None, gap_s=1.0, jump_deg=60.0, max_age_s=0.5, warn_deg=20.0):
        """joints: {ak45 name: calibration.Joint}. home_poses: {joint name: home_sim_deg}."""
        home_poses = home_poses or {}
        unknown = set(home_poses) - {j.joint for j in joints.values()}
        if unknown:
            raise ValueError(f"home_sim_deg for joints that are not AK45s here: {sorted(unknown)}")
        self.m = {n: MotorHoming(j, home_poses.get(j.joint, 0.0)) for n, j in joints.items()}
        for h in self.m.values():
            lo, hi = h.joint.sim_range_deg
            if not lo <= h.home_sim_deg <= hi:
                raise ValueError(f"{h.joint.joint}: home_sim_deg {h.home_sim_deg:g} outside the sim range {lo:g}..{hi:g}")
        self.gap_s, self.jump_deg, self.max_age_s, self.warn_deg = gap_s, jump_deg, max_age_s, warn_deg

    def by_joint(self, joint_name):
        for n, h in self.m.items():
            if h.joint.joint == joint_name:
                return n
        raise KeyError(f"no AK45 joint named {joint_name!r}")

    def _unhome(self, h, reason):
        was = h.homed
        h.homed, h.offset_deg, h.reason = False, 0.0, reason
        return was

    def feedback(self, name, raw_deg, now):
        """One feedback. Returns (event, message): event None, or "power_cycle" when this feedback
        shows the motor came back (after a gap) or jumped. On "power_cycle" the joint is not homed
        any more; the guard then holds the motor where it is and drops what it was doing."""
        h = self.m.get(name)
        if h is None:
            return None, None
        event = msg = None
        if h.seen is not None:
            gap = now - h.seen
            jump = abs(raw_deg - h.raw_deg)
            if gap > self.gap_s or h.lost:
                event = "power_cycle"
                msg = f"{h.joint.joint} ({name}): feedback back after {gap:.1f} s"
            elif jump > self.jump_deg:
                event = "power_cycle"
                msg = f"{h.joint.joint} ({name}): feedback jumped {jump:.1f} deg in {gap * 1000:.0f} ms"
            if event:
                was = self._unhome(h, msg)
                msg += ("; was homed, now NOT homed: home it again" if was else "; still not homed")
        h.raw_deg, h.seen, h.lost = raw_deg, now, False
        return event, msg

    def check_lost(self, now):
        """Marks joints whose feedback stopped for more than gap_s. Returns messages for the ones that
        just got lost (homed ones go back to not homed at once: no command goes out without feedback)."""
        out = []
        for n, h in self.m.items():
            if h.seen is None or h.lost or now - h.seen <= self.gap_s:
                continue
            h.lost = True
            msg = f"{h.joint.joint} ({n}): no feedback for {now - h.seen:.1f} s (power off?)"
            was = self._unhome(h, msg)
            out.append(msg + ("; NOT homed any more" if was else ""))
        return out

    def command_active(self, name, active):
        """ak45_node's command_active for one motor. Returns a message when the hold just ended
        (1 -> 0), and the joint is then not homed."""
        h = self.m.get(name)
        if h is None:
            return None
        was_active, h.active = h.active, active
        if was_active and not active:
            msg = f"{h.joint.joint} ({name}): ak45_node stopped sending it commands (command_active 1 -> 0)"
            was = self._unhome(h, msg)
            return msg + ("; hold ended, NOT homed any more" if was else "")
        return None

    def release(self, name):
        """Rest: the joint is not homed and its commands stop at the guard. Returns whether it was homed."""
        return self._unhome(self.m[name], "released (rest): home it again before use")

    def home(self, name, now):
        """(ok, message). The motor must be at its home pose now."""
        h = self.m[name]
        j = h.joint
        if h.seen is None or h.lost or now - h.seen > self.max_age_s:
            age = "never" if h.seen is None else f"{now - h.seen:.2f} s ago"
            return False, f"{j.joint} ({name}): refused, no fresh feedback (last {age}, needs < {self.max_age_s:g} s)"
        zero = h.raw_deg - j.sign * j.scale * h.home_sim_deg
        h.offset_deg = zero - j.zero_deg
        h.homed, h.reason = True, f"homed at sim {h.home_sim_deg:g} deg (motor {h.raw_deg:.2f})"
        msg = (f"{j.joint} ({name}): homed at sim {h.home_sim_deg:g} deg, feedback {h.raw_deg:.2f} -> "
               f"session zero {zero:.2f} (calibration.json {j.zero_deg:g}, offset {h.offset_deg:+.2f})")
        if abs(h.offset_deg) > self.warn_deg:
            msg += f". WARNING: more than {self.warn_deg:g} deg from calibration.json's zero: is the joint really at its home pose?"
        return True, msg

    # the two directions of the offset
    def to_raw(self, name, deg):
        """arm_control's motor command -> the real motor's. None when not homed (do not send)."""
        h = self.m[name]
        return deg + h.offset_deg if h.homed else None

    def from_raw(self, name, raw_deg):
        """The real motor's feedback -> what arm_control expects. None when not homed (pass nothing)."""
        h = self.m[name]
        return raw_deg - h.offset_deg if h.homed else None

    def all_homed(self):
        return all(h.homed for h in self.m.values())

    def status(self, now):
        joints = {}
        for n, h in self.m.items():
            joints[h.joint.joint] = {
                "motor": n,
                "homed": h.homed,
                "reason": h.reason,
                "home_sim_deg": h.home_sim_deg,
                "offset_deg": round(h.offset_deg, 3),
                "session_zero_deg": None if h.session_zero_deg is None else round(h.session_zero_deg, 3),
                "calibration_zero_deg": h.joint.zero_deg,
                "raw_deg": None if h.raw_deg is None else round(h.raw_deg, 3),
                "feedback_age_s": None if h.seen is None else round(now - h.seen, 3),
                "feedback_lost": h.lost,
                "command_active": h.active,
            }
        return {"all_homed": self.all_homed(), "joints": joints}
