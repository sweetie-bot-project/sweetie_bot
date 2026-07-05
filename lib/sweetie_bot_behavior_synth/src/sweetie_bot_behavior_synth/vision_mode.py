"""Vision-chain mode: a dedicated CPU-only vision cluster for fuser-level scenarios.

    gst videotestsrc (synthetic camera) -> dedicated vision_proxy instance -> fresh fuser
    -> stub provider server (perfusion depth_stub / stub_detector)

For camera-occlusion scenarios use ``covered=True``: the synthetic camera emits a flat, uniform
gray frame, which the fuser's OcclusionMonitor detects the SAME way a real covered lens is
detected — the raw-frame sharpness (Laplacian variance) and luminance spread both collapse. This
exercises the REAL image trigger end-to-end (NOT an injected depth seam: monocular depth predicts
far, not near, on a covered lens, so the old depth_stub:value=0.08 injection no longer fires).
All processes are test-scoped subprocesses (torn down with the context), on dedicated ports so
the ambient sim stack is untouched — EXCEPT the detections topic, which is remapped to the
standard /detections so SOAR + the agent consume the fused output.
"""
from __future__ import annotations

import os
import subprocess
import time

VISION_REPO = os.path.expanduser("~/c/sweetie_bot_vision")
SBCORE_PY = os.path.expanduser("~/sbcore-venv/bin/python")
STUB_PORT = 8091
FUSER_PORT = 9101

# a covered lens = a flat, featureless field of view. Mid-gray solid so the fuser's image trigger
# (low Laplacian variance AND low luminance std) fires exactly as it does on a real covered frame.
COVERED_PATTERN = "pattern=solid-color foreground-color=0xFF808080"
SCENE_PATTERN = "pattern=smpte"


def _pipeline(pattern: str) -> str:
    return ("videotestsrc is-live=true " + pattern + " ! "
            "video/x-raw,width=800,height=600,framerate=10/1 ! jpegenc ! jpegparse ! "
            "appsink name=sink emit-signals=true sync=false drop=true max-buffers=1")


class VisionCluster:
    """Context manager owning the three test-scoped processes."""

    def __init__(self, providers: str = "depth_stub",
                 provider_params: str = "depth_stub:value=0.08",
                 covered: bool = False):
        self.providers = providers
        self.provider_params = provider_params
        # covered -> flat gray camera that drives the real image-based occlusion trigger
        self.pattern = COVERED_PATTERN if covered else SCENE_PATTERN
        self._procs = []

    def _spawn(self, cmd, env_extra=None, name=""):
        env = dict(os.environ)
        if env_extra:
            env.update(env_extra)
        p = subprocess.Popen(cmd, env=env, stdout=open(f"/tmp/bsynth_{name}.log", "w"),
                             stderr=subprocess.STDOUT)
        self._procs.append((name, p))
        return p

    def __enter__(self):
        py_env = {"PYTHONPATH": os.path.join(VISION_REPO, "src")}
        # 1) stub provider server (CPU, no models)
        args = [SBCORE_PY, "-m", "perfusion.transport.server", "--providers-only",
                "--providers", self.providers, "--host", "127.0.0.1",
                "--port", str(STUB_PORT), "--no-auth"]
        if self.provider_params:
            args += ["--provider-params", self.provider_params]
        self._spawn(args, py_env, "stub_provider")
        # 2) fresh fuser on its own port
        self._spawn([SBCORE_PY, os.path.join(VISION_REPO, "scripts", "vision_proxy_fuser"),
                     "--listen", f"127.0.0.1:{FUSER_PORT}", "--tracker", "human",
                     "--camera", os.path.join(VISION_REPO, "config", "camera_proto3.json")],
                    py_env, "fuser")
        time.sleep(3.0)
        # 3) dedicated C++ proxy with the synthetic camera, wired to the test cluster
        self._spawn(["rosrun", "sweetie_bot_vision_proxy", "vision_proxy_node",
                     "__name:=vision_proxy_synth",
                     f"_pipeline:={_pipeline(self.pattern)}",
                     f"_provider_port:={STUB_PORT}", "_provider_target:=/ws",
                     f"_fuser_port:={FUSER_PORT}",
                     "_detections_topic:=detections",
                     "_image_topic:=/bsynth_image", "_remote_host:="],
                    None, "proxy")
        time.sleep(4.0)
        return self

    def __exit__(self, *exc):
        for name, p in reversed(self._procs):
            p.terminate()
        for name, p in reversed(self._procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        self._procs = []
        return False
