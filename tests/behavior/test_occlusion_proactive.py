"""Reactive occlusion complaint - the EDGE path (immediate), not the held cadence.

The held complaint rides occluded_after (10s) + min_gap (25s) + occluded_gap (30s): the user
covered the lens live and waited a long time (2026-07-08). The edge path must complain within a
couple of seconds of the cover (2s debounce), ONCE per cover episode, exempt from
min_gap/occluded_gap; the ~30s repeat while the cover holds is unchanged.

Occlusion is injected as a synthetic camera_occluded detection on /detections - the same entity
the fuser's OcclusionMonitor emits. The real image-trigger chain is already pinned by
test_occlusion.py; THIS test targets the agent-side proactive driver, so the cheap seam is right.

Asserts on the fire-ATTEMPT marker '[proactive #' (logged BEFORE generation), never on
'[proactive] say': the silence guard may legitimately drop a voiced text (see test_self_talk).
Production cadence values are pinned explicitly so the RED reasoning is self-contained: old code
requires occluded_for >= occluded_after (10s) before ANY complaint, so an attempt within 8s of
the cover is impossible regardless of clock history.
"""
import re

import rospy

from sweetie_bot_behavior_synth import agent_params, behavior_test, entity, scene

# match the fire attempt for the occlusion cue specifically (the cue text rides in the marker)
_ATTEMPT_RX = r"\[proactive #\d+\] self_talk .*pressed right against your camera"
_SAY_EMO_RX = re.compile(r"\[proactive\] say \[(\w+)\]")


def _attempts(world):
    return len(world.col["agent_log"].grep(_ATTEMPT_RX))


def _wait_attempts(world, n, timeout):
    """Wait until the occlusion fire-attempt count reaches n (grep counts from the test anchor,
    so a plain wait_grep would keep matching the FIRST attempt on later phases)."""
    deadline = rospy.get_time() + timeout
    while rospy.get_time() < deadline:
        if _attempts(world) >= n:
            return True
        rospy.sleep(0.3)
    return False


@behavior_test
@scene()   # empty: alone/lull disabled below; while covered the chooser cannot reach them anyway
@agent_params(**{"proactive/enabled": True,
                 "proactive/min_gap": 25.0,
                 "proactive/occluded_after": 10.0,
                 "proactive/occluded_gap": 30.0,     # pinned LARGE for phase isolation: only
                                                     # the EDGE may fire inside these phases
                 "proactive/occluded_edge_after": 2.0,
                 "proactive/alone_after": 999.0, "proactive/alone_gap": 999.0,
                 "proactive/lull_after": 999.0, "proactive/lull_prob": 0.0})
def test_occlusion_edge_complains_immediately_once_per_episode(world):
    assert _attempts(world) == 0, "complaint attempt before the lens was covered"

    # phase 1 - EDGE: cover the lens; the complaint attempt must land within ~8s.
    # Old code: occluded_for >= 10s required -> impossible -> THE RED assertion.
    world.spawn(entity("camera_occluded", id=901, bearing=0.0, dist=0.1))
    assert _wait_attempts(world, 1, timeout=8.0), \
        "no immediate complaint on the occlusion edge (rode the held cadence?)"

    # phase 2 - ONE per episode: cover held past occluded_after but still inside
    # min_gap/occluded_gap: neither the edge (episode consumed) nor the held repeat may fire
    rospy.sleep(10.0)
    assert _attempts(world) == 1, "edge complaint re-fired within one cover episode"

    # phase 3 - RE-COVER: clearing the lens ends the episode; a fresh cover must complain
    # immediately again even though since_selftalk (~13-18s) is inside min_gap (25s) and
    # occluded_gap (30s) - the in-sim proof of the gap exemption. The 5s clear window lets
    # any in-flight generation release the lock so a tick can observe the falling edge.
    world.vanish(901)
    rospy.sleep(5.0)
    world.spawn(entity("camera_occluded", id=902, bearing=0.0, dist=0.1))
    assert _wait_attempts(world, 2, timeout=10.0), \
        "no immediate complaint on a fresh cover episode (gap exemption missing?)"

    # every voiced aside in this test must carry the forced negative emotion (only occlusion
    # cues can fire here; vacuously true if the silence guard dropped the text)
    for line in world.col["agent_log"].grep(_SAY_EMO_RX.pattern):
        m = _SAY_EMO_RX.search(line)
        assert m and m.group(1) == "anger", \
            "occluded aside voiced without anger: %r" % (line,)


@behavior_test
@scene()
@agent_params(**{"proactive/enabled": True,
                 "proactive/min_gap": 25.0,          # deliberately LARGER than occluded_gap:
                 "proactive/occluded_after": 5.0,    # the repeat landing well inside min_gap
                 "proactive/occluded_gap": 8.0,      # IS the exemption proof
                 "proactive/occluded_edge_after": 2.0,
                 "proactive/alone_after": 999.0, "proactive/alone_gap": 999.0,
                 "proactive/lull_after": 999.0, "proactive/lull_prob": 0.0})
def test_occlusion_repeat_paces_on_occluded_gap_not_min_gap(world):
    """While the cover HOLDS the complaint must repeat at her normal speech cadence
    (occluded_gap), not the idle-aside min_gap (user 2026-07-11: 30s felt far too long).
    With occluded_gap=8 < min_gap=25 the second complaint must land well before min_gap
    could ever allow it. RED on the old chooser (min_gap gated the occlusion branch too)."""
    world.spawn(entity("camera_occluded", id=903, bearing=0.0, dist=0.1))
    assert _wait_attempts(world, 1, timeout=8.0), "no edge complaint on cover"

    # too early for a repeat: _last_selftalk is set after generation, so since_selftalk
    # here is at most ~4s < occluded_gap — the held path must still be silent
    rospy.sleep(4.0)
    assert _attempts(world) == 1, "repeat fired faster than occluded_gap"

    # the held repeat must arrive ~occluded_gap after the last aside — far inside min_gap.
    # Old code needed since_selftalk >= min_gap (25s) too: earliest repeat ~29s after the
    # edge complaint, past this deadline -> RED there, GREEN with the exemption.
    assert _wait_attempts(world, 2, timeout=20.0), \
        "no held repeat at occluded_gap (min_gap still gating the occlusion branch?)"
