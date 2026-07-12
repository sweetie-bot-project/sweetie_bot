"""ROS RobotStateProvider: subscribe to robot-state topics, build a RobotState snapshot.

ROS-side glue. The agent core only sees the RobotState value object (RobotStateProvider seam),
so this can be swapped for a sim/TUI stub.
"""
from __future__ import annotations

import datetime
import threading
from typing import Dict, Optional

import rospy
from sensor_msgs.msg import BatteryState, JointState

from sweetie_bot_ai_core.schema import RobotState, ServoInfo, servo_info

# HerkulexState is optional (message package may not be present in all builds).
try:
    from sweetie_bot_herkulex_msgs.msg import HerkulexState  # type: ignore
    _HAS_HERKULEX = True
except Exception:  # noqa: BLE001
    _HAS_HERKULEX = False

_WEEKDAYS = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
_BATTERY_STATUS = {
    BatteryState.POWER_SUPPLY_STATUS_CHARGING: "charging",
    BatteryState.POWER_SUPPLY_STATUS_DISCHARGING: "discharging",
    BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING: "not charging",
    BatteryState.POWER_SUPPLY_STATUS_FULL: "full",
}


class ServoFaultFilter:
    """Debounce transient servo bus errors (normal noise on this robot): a servo is FAULTED
    only when errors persist (>= min_reports reports spanning >= min_span_s); one clean report
    clears it. Pure (clock injected) - unit-testable."""

    def __init__(self, min_reports: int = 4, min_span_s: float = 3.0, clock=None):
        import time as _time
        self._clock = clock or _time.monotonic
        self.min_reports = min_reports
        self.min_span_s = min_span_s
        self._err = {}     # name -> (first_t, count)

    def observe(self, name: str, has_error: bool) -> bool:
        """Feed one report; returns True while the servo counts as (persistently) faulted."""
        now = self._clock()
        if not has_error:
            self._err.pop(name, None)
            return False
        first_t, count = self._err.get(name, (now, 0))
        count += 1
        self._err[name] = (first_t, count)
        return count >= self.min_reports and (now - first_t) >= self.min_span_s


class StateCollector:
    """Caches the latest state messages; ``snapshot()`` renders a RobotState. Thread-safe."""

    def __init__(self, *, battery_topic="battery_state",
                 servo_topic="motion/herkulex/servo_states",
                 joint_topic="joint_states", overheat_temp=80.0,
                 ignored_servos=None, subscribe=True):
        self._lock = threading.Lock()
        self._battery: Optional[BatteryState] = None
        # name -> latest NOTABLE ServoInfo (off / overheating); healthy servos are absent
        self._servos: Dict[str, ServoInfo] = {}
        self._fault_filter = ServoFaultFilter()
        self._overheat_temp = overheat_temp
        # hardware-disabled servos (single source of truth: /disabled_servos, hardware.yaml):
        # entirely invisible to the LLM state view - a known-dead servo is not news
        self._ignored_servos = set(ignored_servos or ())

        if subscribe:
            rospy.Subscriber(battery_topic, BatteryState, self._on_battery, queue_size=1)
            rospy.Subscriber(joint_topic, JointState, self._on_joint, queue_size=1)
            if _HAS_HERKULEX:
                rospy.Subscriber(servo_topic, HerkulexState, self._on_servo, queue_size=10)
        self._moving = None

    # -- subscribers --------------------------------------------------------------------------

    def _on_battery(self, msg: BatteryState):
        with self._lock:
            self._battery = msg

    def _on_joint(self, msg: JointState):
        # cheap movement heuristic; named-pose detection lives in SOAR and is not duplicated here
        try:
            moving = any(abs(v) > 0.05 for v in (msg.velocity or []))
        except Exception:  # noqa: BLE001
            moving = None
        with self._lock:
            self._moving = moving

    def _on_servo(self, msg):
        name = getattr(msg, "name", None)
        if name in self._ignored_servos:
            return
        key = name or "unknown"
        try:
            # decode is a pure function in the AI core (unit-tested there); this handler only
            # adapts the ROS message fields (note the msg's field is misspelled "respond_sucess")
            info = servo_info(
                key,
                status_error=int(getattr(msg, "status_error", 0) or 0),
                status_detail=int(getattr(msg, "status_detail", 0) or 0),
                respond_success=bool(getattr(msg, "respond_sucess", True)),
                temperature_c=getattr(msg, "temperature", None),
                overheat_temp=self._overheat_temp,
            )
        except Exception:  # noqa: BLE001
            return
        # transient bus comm glitches (a one-frame MOTOR_ON flicker or spurious error bit) are
        # NORMAL on this robot - only a persistent notable state reaches the LLM
        notable = self._fault_filter.observe(key, info is not None)
        with self._lock:
            if notable and info is not None:
                self._servos[key] = info
            else:
                self._servos.pop(key, None)

    # -- RobotStateProvider seam --------------------------------------------------------------

    def snapshot(self) -> RobotState:
        now = datetime.datetime.now()
        with self._lock:
            bat = self._battery
            servos = list(self._servos.values())
            moving = self._moving
        battery_percent = None
        battery_status = None
        if bat is not None:
            if bat.percentage is not None and bat.percentage >= 0:
                battery_percent = float(bat.percentage) * 100.0
            battery_status = _BATTERY_STATUS.get(getattr(bat, "power_supply_status", 0), "unknown")
        return RobotState(
            datetime_iso=now.strftime("%Y-%m-%d %H:%M"),
            weekday=_WEEKDAYS[now.weekday()],
            battery_percent=battery_percent,
            battery_status=battery_status,
            servos=servos,
            moving=moving,
        )
