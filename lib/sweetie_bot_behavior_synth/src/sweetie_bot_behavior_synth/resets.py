"""Inter-test reset primitives.

P23 (established empirically, 2/2): in-process /soar/reconfigure wedges the node's rospy
service layer — the port keeps listening with a full accept queue while handlers are dead, and
topic subscriptions persist, so liveness checks lie. Consequently:
  * the standard SOAR reset is KILL+RESPAWN (launch respawn=true; a fresh process re-reads all
    rosparams and configures cleanly) — ~20-25 s;
  * every soar service interaction is a subprocess `timeout N rosservice call` (wall-clock
    bounded; a wedged service can never hang the harness);
  * reconfigure stays available behind BSYNTH_USE_RECONFIGURE=1 for when devel fixes P23.
"""
from __future__ import annotations

import os
import subprocess
import time
from typing import Callable, Dict, Optional, Tuple

import rospy
from std_srvs.srv import Trigger

_UNSET = object()


def _sh(cmd: str, timeout: float = 15.0) -> Tuple[int, str]:
    try:
        r = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, "timeout"


def _call_srv(name: str, args: str = "", timeout: float = 10.0) -> Tuple[bool, str]:
    code, out = _sh(f"timeout {timeout} rosservice call {name} {args}", timeout + 5)
    return code == 0 and "success: True" in out or (code == 0 and "ERROR" not in out), out


# ------------------------------------------------------------------ agent ------------------------
def agent_reset(ns: str = "/llm_agent") -> bool:
    rospy.wait_for_service(ns + "/reset", timeout=10.0)
    return bool(rospy.ServiceProxy(ns + "/reset", Trigger)().success)


# ------------------------------------------------------------------ params -----------------------
def apply_params(overrides: Dict[str, object], base: str) -> Callable[[], None]:
    priors: Dict[str, object] = {}
    for key, val in overrides.items():
        full = base.rstrip("/") + "/" + key.lstrip("/")
        try:
            priors[full] = rospy.get_param(full)
        except KeyError:
            priors[full] = _UNSET
        rospy.set_param(full, val)

    def undo():
        for full, prior in priors.items():
            if prior is _UNSET:
                try:
                    rospy.delete_param(full)
                except KeyError:
                    pass
            else:
                rospy.set_param(full, prior)
    return undo


# ------------------------------------------------------------------ soar --------------------------
def soar_services_responsive(ns: str = "/soar", timeout: float = 6.0) -> bool:
    """ANY response (even a refusal) proves the service layer is alive; /soar/step refuses
    while running, which is a perfect harmless probe."""
    code, out = _sh(f"timeout {timeout} rosservice call {ns}/step", timeout + 3)
    return code == 0 and "ERROR" not in out


def soar_configured(ns: str = "/soar") -> bool:
    code, out = _sh(f"rosnode info {ns} 2>/dev/null | grep -c generate_reply", 10)
    try:
        return code == 0 and int(out.splitlines()[-1]) >= 1
    except (ValueError, IndexError):
        return False


def _soar_pid() -> Optional[int]:
    code, out = _sh("pgrep -f '[s]oar __name' | head -1", 10)
    try:
        return int(out.splitlines()[-1])
    except (ValueError, IndexError):
        return None


def soar_respawn(ns: str = "/soar", timeout: float = 90.0) -> bool:
    """Kill + wait for the launch respawn. A P23-wedged soar SURVIVES SIGTERM (the SML kernel
    thread blocks it) — escalate to SIGKILL if the old PID is still alive."""
    old_pid = _soar_pid()
    _sh(f"rosnode kill {ns} >/dev/null 2>&1", 15)
    time.sleep(5.0)
    if old_pid is not None and _soar_pid() == old_pid:
        _sh(f"kill {old_pid} 2>/dev/null", 5)
        time.sleep(4.0)
        if _soar_pid() == old_pid:
            _sh(f"kill -9 {old_pid} 2>/dev/null", 5)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pid = _soar_pid()
        if pid is not None and pid != old_pid and soar_configured(ns) \
                and soar_services_responsive(ns):
            return True
        time.sleep(2.5)
    return False


def set_operational(on: bool, ns: str = "/soar", timeout: float = 10.0) -> bool:
    ok, out = _call_srv(f"{ns}/set_operational", f'"data: {str(on).lower()}"', timeout)
    if not ok:
        rospy.logwarn("behavior_synth: set_operational(%s) failed/wedged (%s) - respawning soar",
                      on, out[:120])
        if soar_respawn(ns):
            ok, _ = _call_srv(f"{ns}/set_operational", f'"data: {str(on).lower()}"', timeout)
    return ok


def soar_reconfigure(overrides: Optional[Dict[str, object]] = None,
                     ns: str = "/soar", timeout: float = 10.0) -> Tuple[bool, Callable[[], None]]:
    """Full SOAR state reset + optional param overrides. Standard path = respawn (P23);
    reconfigure only when BSYNTH_USE_RECONFIGURE=1 AND it proves healthy afterwards."""
    undo = apply_params(overrides or {}, ns)
    if os.environ.get("BSYNTH_USE_RECONFIGURE") == "1":
        ok, _ = _call_srv(f"{ns}/reconfigure", "", timeout)
        if ok and soar_configured(ns) and soar_services_responsive(ns):
            return True, undo
        rospy.logwarn("behavior_synth: reconfigure unhealthy (P23) - falling back to respawn")
    return soar_respawn(ns), undo


# ------------------------------------------------------------------ head pose (O.2) --------------
def recenter_head(timeout: float = 8.0) -> bool:
    """Drive the head joints back to nominal (zeros) via /motion/controller/joint_trajectory.

    Sim head pose drifts monotonically across test-worlds (look-at churn + aborted animations);
    once the head parks off-center the synthetic human's `yaw-head` bin leaves `center` and SOAR
    stops binding TALK-EVENT SOURCE from vision — say_and_wait then times out with a perfectly
    healthy llm_agent. Saved head trajectories can't recover on their own: they START at nominal
    with 0.522 rad path tolerance, so a drifted head gets `invalid_pose` rejections. This goal is
    synthesized FROM the current pose, so it is always admissible.
    """
    import actionlib
    from actionlib_msgs.msg import GoalStatus
    from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal,
                                  JointTolerance)
    from trajectory_msgs.msg import JointTrajectoryPoint
    from sensor_msgs.msg import JointState

    names = ["head_joint1", "head_joint2", "head_joint3", "head_joint4"]

    def head_pose():
        js = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
        pos = dict(zip(js.name, js.position))
        return [pos.get(n, 0.0) for n in names]

    client = actionlib.SimpleActionClient("motion/controller/joint_trajectory",
                                          FollowJointTrajectoryAction)
    # 2 attempts: right after a soar respawn the first goal has been seen to come back quickly
    # without moving the head (transient rejection); one retry after a settle covers it
    for attempt in range(2):
        try:
            cur = head_pose()
        except rospy.ROSException:
            rospy.logwarn("behavior_synth: recenter_head: no /joint_states - skipping")
            return False
        if max(abs(c) for c in cur) < 0.1:
            return True                  # already nominal - don't spend a motion on it

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = names
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=cur, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=[0.0] * len(names),
                                 time_from_start=rospy.Duration(2.0)),
        ]
        # mirror the saved-trajectory convention (0.522/0.174), path widened: we deliberately
        # traverse a large arc from an arbitrary drifted pose
        goal.path_tolerance = [JointTolerance(name=n, position=1.0) for n in names]
        goal.goal_tolerance = [JointTolerance(name=n, position=0.174) for n in names]

        if not client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("behavior_synth: recenter_head: joint_trajectory server unavailable")
            return False
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(timeout)):
            state, res = client.get_state(), client.get_result()
            if state != GoalStatus.SUCCEEDED:
                rospy.logwarn("behavior_synth: recenter_head: goal state=%s error=%s %r", state,
                              getattr(res, "error_code", "?"), getattr(res, "error_string", "?"))
        else:
            client.cancel_goal()
            rospy.logwarn("behavior_synth: recenter_head: motion timed out")
        residual = max(abs(p) for p in head_pose())
        if residual <= 0.3:
            return True
        rospy.logwarn("behavior_synth: recenter_head: residual %.2f rad (attempt %d)",
                      residual, attempt + 1)
        rospy.sleep(2.0)
    return False
