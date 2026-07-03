"""Pony lost-once — at the VISION seam (the live bug's real home).

Live 2026-07-02/03: once the pony detection dropped, she never saw it again even when clearly
back in frame. Suspect was the fuser's no-ReID ByteTrack object tracker: does a DEAD track get
re-born once detections recover? object_stub replays exactly that: pony_body at score 0.8, a
0-score stretch of 75 frames (~7.5 s at 10 fps), then 0.8 again — /detections must carry a pony
again after recovery. VERDICT (first run, 2026-07-03): PASSES — the fuser re-births cleanly; the
live bug is DETECTOR RECALL (object_openvino on the handled plushie), not tracker code. This
test stays as the pipeline regression anchor.

CLASSICAL package test (relocated from tests/behavior/ per the test-placement rule: no robot
behavior here, no SOAR/sim — just the proxy+fuser+stub chain on a private roscore).
"""
import time

from test_link_recovery import LinkCluster, pytestmark  # noqa: F401  (same skipif gate)

DIP_PONY = ("object_stub:score_high=0.8;score_low=0;drop_start=80;drop_end=155")


def test_pony_retrack_after_score_dip(tmp_path):
    with LinkCluster(str(tmp_path), provider_params=DIP_PONY) as c:
        # phase 1 (frames 0-79, ~8 s): high score - the pony must be tracked and published
        assert c.wait_pony(25.0), "pony never appeared at high score - stub/cluster broken"
        # phase 2 (frames 80-154, ~7.5 s): score 0 - the track dies. Wait the dip out fully
        # (worst case: pony seen at frame 0 -> dip ends 15.5 s later; margin for jitter).
        time.sleep(18.0)
        # phase 3: score is high again - a NEW track must be born and published
        assert c.wait_pony(15.0), ("pony NOT re-tracked after the score dip recovered - "
                                   "the lost-once bug lives in the fuser object tracker")
