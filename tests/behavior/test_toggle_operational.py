"""Operational TOGGLE path — the repeat-train defect (live 2026-07-09, HANDOFF T.4).

The rviz panel fires /soar/toggle_operational as a periodic repeat TRAIN (4 calls at
~0.58s observed live). A toggle is state-relative, so which pulses survive the debounce
decides the FINAL state: the old window-from-last-ACCEPTED semantics let every other
pulse through (state parity == train-length parity, "off->on->off inconsistent"); the
fixed semantics re-arm the window on EVERY call, collapsing any train to its first pulse.

Test geometry (why 7 calls and a 3.0s window): the production 0.7s window cannot be
timed reliably from a test (service round-trip + soar.stop() inside the first accepted
handler eat most of it), so the scenario widens ~toggle_debounce to 3.0s. The train must
then SPAN the window — with a short train that fits inside one window even the OLD
semantics collapse it and the test could never fail. 7 calls x ~0.6s ≈ 3.6s > 3.0s: the
old code re-accepts one mid-train pulse (net state True again), the new code accepts
only the first (net state False). Each 0.6s gap has ~2.4s of latency slack.
"""
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from sweetie_bot_behavior_synth import behavior_test, scene, soar_params

_WINDOW = 3.0


def _latched_operational(timeout=5.0):
    return bool(rospy.wait_for_message("/soar/operational", Bool, timeout=timeout).data)


@behavior_test
@scene()  # no human needed: this exercises the service/state path only
@soar_params(toggle_debounce=_WINDOW)
def test_toggle_repeat_train_collapses_to_first_call(world):
    rospy.wait_for_service("/soar/toggle_operational", timeout=10.0)
    toggle = rospy.ServiceProxy("/soar/toggle_operational", Trigger)
    assert _latched_operational() is True, "World.start should have opened operational"

    # the live repeat-train shape, elongated to span the widened window
    resps = []
    for _ in range(7):
        resps.append(toggle())
        rospy.sleep(0.6)

    assert resps[0].success, "first call of the train must be accepted (it IS the press)"
    assert all("debounced" in r.message for r in resps[1:]), \
        "a mid-train pulse escaped the debounce (parity defect): %r" % \
        [(r.success, r.message) for r in resps]
    assert _latched_operational() is False, \
        "net effect of a repeat train must be exactly ONE toggle (ended operational)"
    hits = world.col["soar_log"].wait_grep(r"toggle_operational accepted", timeout=10.0)
    assert len(hits) == 1, "expected exactly one accepted toggle, got: %r" % (hits,)
    warns = world.col["soar_log"].grep(r"toggle_operational repeat within")
    assert len(warns) == 6, "expected 6 debounce warns, got %d" % len(warns)

    # a deliberate press after the window re-arms works (measured AFTER the last response)
    rospy.sleep(_WINDOW + 0.6)
    assert toggle().success, "deliberate toggle after the window must be accepted"
    assert _latched_operational() is True
