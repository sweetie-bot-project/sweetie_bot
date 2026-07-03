"""P21: the local provider WebSocket link must fail LOUDLY and self-heal.

Live incident (2026-07-03 ~00:26): the local ws link died established-but-wedged; the proxy
neither logged nor reconnected — perception went blind for the rest of the session while the
demo stream looked healthy. One `rosnode kill /vision_proxy` (fresh WS) fixed it instantly.
These tests kill/wedge the LOCAL provider server mid-run and demand (a) a loud log within
seconds, (b) detections resume after the server returns, with NO node restart.

CLASSICAL package tests (NOT behavior-synth scenarios — this is transport plumbing, no robot
behavior involved): the exact process chain of the incident (C++ proxy + fresh fuser + stub
provider server) runs on a PRIVATE roscore with dedicated ports, so no sim stack, no SOAR, and
nothing leaks onto a live bench. Needs the raider deployment env (ROS sourced, ~/sbcore-venv,
~/c/sweetie_bot_vision); skipped elsewhere.

    export SWEETIE_BOT_WS=${SWEETIE_BOT_WS:-~/ros/sweetie_bot}   # your catkin workspace
    source /opt/ros/sweetie_bot/setup.bash && source "$SWEETIE_BOT_WS"/devel/setup.bash
    python3 -m pytest lib/sweetie_bot_vision_proxy/test/
"""
import os
import shutil
import signal
import subprocess
import time

import pytest

VISION_REPO = os.path.expanduser("~/c/sweetie_bot_vision")
SBCORE_PY = os.path.expanduser("~/sbcore-venv/bin/python")
MASTER_PORT, STUB_PORT, FUSER_PORT = 11397, 8093, 9103   # disjoint from the behavior harness's
CONST_PONY = "object_stub:drop_start=0;drop_end=0"       # score schedule with no dip -> steady pony
PIPELINE = ("videotestsrc is-live=true pattern=smpte ! "
            "video/x-raw,width=800,height=600,framerate=10/1 ! jpegenc ! jpegparse ! "
            "appsink name=sink emit-signals=true sync=false drop=true max-buffers=1")

pytestmark = pytest.mark.skipif(
    not (os.path.exists(SBCORE_PY) and os.path.isdir(VISION_REPO) and shutil.which("roscore")),
    reason="needs the raider vision deployment (roscore + sbcore-venv + vision repo)")


class LinkCluster:
    """Private-master vision chain owning its Popens, so tests can kill/wedge/respawn members."""

    def __init__(self, tmp):
        self.tmp = tmp
        self.env = dict(os.environ, ROS_MASTER_URI=f"http://127.0.0.1:{MASTER_PORT}")
        self._procs = []
        self._specs = {}

    def logfile(self, name):
        return os.path.join(self.tmp, name + ".log")

    def spawn(self, name, cmd, extra_env=None):
        env = dict(self.env)
        if extra_env:
            env.update(extra_env)
        p = subprocess.Popen(cmd, env=env, stdout=open(self.logfile(name), "a"),
                             stderr=subprocess.STDOUT)
        self._procs.append((name, p))
        self._specs[name] = (cmd, extra_env)
        return p

    def proc(self, name):
        for n, p in reversed(self._procs):
            if n == name:
                return p
        raise KeyError(name)

    def respawn(self, name):
        """Re-launch a killed member with its original command (same ports)."""
        cmd, extra_env = self._specs[name]
        return self.spawn(name, cmd, extra_env)

    def wait_pony(self, timeout):
        """True once a /detections message carrying the stub's pony arrives on the private master.
        Loops `rostopic echo -n1` because the first post-(re)connect arrays can be empty while the
        object tracker confirms the track (2 frames)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            budget = max(1.0, min(6.0, deadline - time.monotonic()))
            try:
                r = subprocess.run(["rostopic", "echo", "-n1", "/detections"], env=self.env,
                                   capture_output=True, text=True, timeout=budget)
            except subprocess.TimeoutExpired:
                continue
            if r.returncode == 0 and "pony" in r.stdout:
                return True
        return False

    def __enter__(self):
        self.spawn("roscore", ["roscore", "-p", str(MASTER_PORT)])
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if subprocess.run(["rostopic", "list"], env=self.env,
                              capture_output=True).returncode == 0:
                break
            time.sleep(0.3)
        else:
            raise RuntimeError("private roscore did not come up")
        py_env = {"PYTHONPATH": os.path.join(VISION_REPO, "src")}
        self.spawn("stub_provider", [SBCORE_PY, "-m", "perfusion.transport.server",
                                     "--providers-only", "--providers", "object_stub",
                                     "--provider-params", CONST_PONY,
                                     "--host", "127.0.0.1", "--port", str(STUB_PORT), "--no-auth"],
                   py_env)
        self.spawn("fuser", [SBCORE_PY, os.path.join(VISION_REPO, "scripts", "vision_proxy_fuser"),
                             "--listen", f"127.0.0.1:{FUSER_PORT}", "--tracker", "human",
                             "--camera", os.path.join(VISION_REPO, "config", "camera_proto3.json")],
                   py_env)
        time.sleep(2.0)
        self.spawn("proxy", ["rosrun", "sweetie_bot_vision_proxy", "vision_proxy_node",
                             "__name:=vision_proxy_linktest",
                             f"_pipeline:={PIPELINE}",
                             f"_provider_port:={STUB_PORT}", "_provider_target:=/ws",
                             f"_fuser_port:={FUSER_PORT}",
                             "_detections_topic:=/detections",
                             "_image_topic:=/linktest_image", "_remote_host:="])
        return self

    def __exit__(self, *exc):
        for name, p in reversed(self._procs):
            if p.poll() is None:
                try:
                    os.kill(p.pid, signal.SIGCONT)   # a SIGSTOPped member ignores SIGTERM
                except ProcessLookupError:
                    pass
                p.terminate()
        for name, p in reversed(self._procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        self._procs = []
        return False


class ProxyLog:
    """Tail the proxy's stdout from anchor (construction) time."""

    def __init__(self, path):
        self.path = path
        self._pos = os.path.getsize(path) if os.path.exists(path) else 0

    def wait_for(self, needle, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if os.path.exists(self.path):
                with open(self.path, errors="replace") as f:
                    f.seek(self._pos)
                    if needle in f.read():
                        return True
            time.sleep(0.2)
        return False


@pytest.fixture()
def cluster(tmp_path):
    with LinkCluster(str(tmp_path)) as c:
        assert c.wait_pony(25.0), "cluster broken: pony never flowed"
        yield c


def test_local_link_death_is_loud_and_recovers(cluster):
    """SIGKILL the local provider server mid-run: loud DOWN log <=5 s; after the server is
    restarted on the same port, detections resume with NO node restart."""
    log = ProxyLog(cluster.logfile("proxy"))
    cluster.proc("stub_provider").kill()                  # server dies; kernel closes the socket
    assert log.wait_for("provider link DOWN", 5.0), \
        "proxy did not LOG the local link death within 5 s (P21 silent degradation)"
    cluster.respawn("stub_provider")
    assert cluster.wait_pony(15.0), \
        "detections did not resume after the provider came back (no reconnect)"
    assert log.wait_for("provider link RECOVERED", 5.0), "recovery transition not logged"


def test_local_link_wedge_is_bounded_and_recovers(cluster):
    """SIGSTOP the server (half-open wedge — the incident mode: socket stays open, writes land
    in kernel buffers, replies never come). Bounded I/O must surface it within the io timeout
    and log; SIGCONT must lead to recovery without any node restart."""
    log = ProxyLog(cluster.logfile("proxy"))
    stub = cluster.proc("stub_provider")
    os.kill(stub.pid, signal.SIGSTOP)                     # wedged, not dead: reads block forever
    try:
        assert log.wait_for("provider link DOWN", 8.0), \
            "proxy did not detect the WEDGED half-open link (unbounded read - the exact incident mode)"
    finally:
        os.kill(stub.pid, signal.SIGCONT)
    assert cluster.wait_pony(20.0), "detections did not resume after the wedge cleared"
