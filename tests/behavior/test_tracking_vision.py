"""Pony lost-once — at the VISION seam (the live bug's real home).

Live 2026-07-02/03: once the pony detection dropped, she never saw it again even when clearly
back in frame. The suspect is the fuser's no-ReID ByteTrack object tracker: existing tracks ride
through low-confidence stretches, but once the track DIES nothing may be re-born from
sub-threshold detections. object_stub replays exactly that: pony_body at score 0.8, a 0.15
stretch (~7.5 s at 10 fps), then 0.8 again — does /detections carry a pony again after recovery?

UNKNOWN group on first run: PASS -> the fuser is fine and the live bug sits in the provider
(detector recall after handling); FAIL -> the tracker re-birth bug is confirmed -> becomes the
must-fail xfail(strict) regression anchor for the fix.
"""
import pytest
import rospy

from sweetie_bot_behavior_synth import behavior_test, person, scene


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
@pytest.mark.unknown
def test_pony_retrack_after_score_dip_vision_seam(world):
    from sweetie_bot_behavior_synth.vision_mode import VisionCluster

    def pony_frames(since):
        return [m for m in world.col["detections"].messages(since)
                if any(d.type == "pony" for d in m.detections)]

    import time
    with VisionCluster(providers="object_stub",
                       provider_params="object_stub:score_high=0.8;score_low=0;"
                                       "drop_start=80;drop_end=155"):
        t0 = time.monotonic()
        # phase 1 (frames 0-79, ~8s): high score - the pony must be tracked and published
        got = world.col["detections"].wait_for(
            lambda m: any(d.type == "pony" for d in m.detections), timeout=25.0)
        assert got is not None, "pony never appeared at high score - stub/cluster broken"
        # phase 2 (frames 80-154, ~7.5s): score dips below track threshold - pony vanishes
        rospy.sleep(10.0)
        t_dip = time.monotonic()
        # phase 3 (recovery): score is high again - the pony MUST come back
        rospy.sleep(12.0)
        recovered = pony_frames(t_dip + 9.0)   # look only well after the dip ended
        assert recovered, ("pony NOT re-tracked after the score dip recovered - "
                           "the lost-once bug lives in the fuser object tracker")
