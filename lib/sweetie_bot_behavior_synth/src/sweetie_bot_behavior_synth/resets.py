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
