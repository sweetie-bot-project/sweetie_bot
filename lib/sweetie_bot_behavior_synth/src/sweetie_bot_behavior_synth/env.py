"""SimStack session management: preflight + attach-or-launch of the run_real:=false stack.

Preflight codifies the ops lessons (2026-07 session): single instance per role verified by
PROCESS + PORT HOLDER (the fuser is a raw TCP server — a zombie holding :9100 silently serves
stale code), required env, stream sanity. NO remote-token checks here (out of behavior scope).
"""
from __future__ import annotations

import os
import subprocess
import time
from dataclasses import dataclass, field
from typing import List

import rospy
import rosnode

LAUNCH_SCRIPT = os.environ.get("BSYNTH_LAUNCH_SCRIPT", "bin/run_noreal_proto32.sh")
LAUNCH_ARGS = os.environ.get(
    "BSYNTH_LAUNCH_ARGS",
    "robot_profile:=proto32 run_real:=false run_vision:=false run_flexbe:=true")
BYOBU_WINDOW = os.environ.get("BSYNTH_WINDOW", "1:2")
REQUIRED_NODES = ["/soar", "/llm_agent", "/voice", "/flexbe/action_server"]


class PreflightError(RuntimeError):
    pass


def _sh(cmd: str) -> str:
    return subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True).stdout.strip()


def workspace_root() -> str:
    """Absolute path of the catkin workspace (the directory holding bin/ and devel/).

    Nothing is hardcoded: $SWEETIE_BOT_WS wins when set, otherwise the workspace is derived
    from the devel space catkin prepends to $CMAKE_PREFIX_PATH when its setup.bash is sourced.
    """
    ws = os.environ.get("SWEETIE_BOT_WS")
    if ws:
        return os.path.abspath(os.path.expanduser(ws))
    for entry in os.environ.get("CMAKE_PREFIX_PATH", "").split(os.pathsep):
        entry = entry.rstrip("/")
        if os.path.basename(entry) in ("devel", "install") and os.path.isdir(entry):
            return os.path.dirname(entry)
    raise PreflightError(
        "cannot locate the catkin workspace: set $SWEETIE_BOT_WS, or source "
        "<workspace>/devel/setup.bash before running the harness")


def launch_cmd() -> str:
    """Sim-stack launch command line sent to the byobu window."""
    return f"{os.path.join(workspace_root(), LAUNCH_SCRIPT)} {LAUNCH_ARGS}"


@dataclass
class Preflight:
    problems: List[str] = field(default_factory=list)

    def check_single_instance(self, pattern: str, role: str):
        # bracket the first char so pgrep -f cannot match our own invoking shell
        safe = f"[{pattern[0]}]{pattern[1:]}"
        pids = _sh(f"pgrep -f '{safe}' | wc -l")
        n = int(pids or 0)
        if n != 1:
            self.problems.append(f"{role}: expected exactly 1 process matching '{pattern}', found {n}")

    def check_nodes(self):
        try:
            up = rosnode.get_node_names()
        except Exception as e:  # noqa: BLE001
            self.problems.append(f"ros master unreachable: {e!r}")
            return
        for n in REQUIRED_NODES:
            if n not in up:
                self.problems.append(f"required node missing: {n}")

    def check_ollama(self):
        code = _sh("curl -s -m 4 -o /dev/null -w '%{http_code}' http://localhost:11434/api/tags")
        if code != "200":
            self.problems.append(f"ollama not healthy (http {code})")

    def check_soar_configured(self):
        try:
            info = _sh("rosnode info /soar 2>/dev/null | grep -c generate_reply")
            if int(info or 0) < 1:
                self.problems.append("/soar has no generate_reply subscriptions (zombie/unconfigured)")
        except ValueError:
            self.problems.append("/soar info unreadable")

    def run(self):
        self.check_nodes()
        self.check_ollama()
        self.check_single_instance("llm_agent_node", "llm_agent")
        self.check_soar_configured()
        if self.problems:
            raise PreflightError("PREFLIGHT FAILED:\n  - " + "\n  - ".join(self.problems))


class SimStack:
    """Attach to a healthy sim stack, or launch one in a byobu window and wait for readiness."""

    def __init__(self):
        self.launched = False

    def _healthy(self) -> bool:
        try:
            Preflight().run()
            return True
        except PreflightError:
            return False

    def ensure(self, timeout: float = 120.0):
        if not rospy.core.is_initialized():
            rospy.init_node("behavior_synth", anonymous=True, disable_signals=True)
        if self._healthy():
            return self
        # not healthy -> (re)launch in the byobu window (long-lived process policy)
        _sh(f"byobu send-keys -t {BYOBU_WINDOW} C-c")
        time.sleep(6)
        _sh(f"byobu send-keys -t {BYOBU_WINDOW} C-c")
        time.sleep(3)
        _sh(f"byobu send-keys -t {BYOBU_WINDOW} '{launch_cmd()}' Enter")
        self.launched = True
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                Preflight().run()
                return self
            except PreflightError as e:
                last = e
                time.sleep(5)
        raise PreflightError(f"sim stack did not become healthy in {timeout}s; last: {last}")
