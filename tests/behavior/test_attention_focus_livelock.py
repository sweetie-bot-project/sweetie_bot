"""Attention focus on a phantom SWM belief: focusing-attention must never machine-gun.

Live 2026-07-14 (session fdd28860, soar-21.log): SOAR was stopped 23:33->00:15 while vision
kept publishing. The SWM store swept the tracked human during the stop (detectionCallback
runs __updateSpatialMemory when the SOAR thread is stale) and queued its input WME for
removal; the first update() after the restart DISCARDED the queue (swm.py
_soar_view_remove_list.clear() without deinit()), leaving the WME frozen at `visible now
^perceive-begin now`. The i-supported belief (vision*elaborate*object) then lived forever
and out-scored every live human (perceive-begin now = +100 utility), so focusing-attention
re-initiated against the vanished store key at ~4/s for 5.5 min ("SWM object is missing
(not activated)" x457), starving the answer machinery until a reconfigure.

Contract (attention.soar latch + swm.py view deinit):
  * a focusing-attention process terminating with status failed latches its object
    (attention-focus-failed predicate, 15 s TTL via the native predicate deadline sweep);
    both proposers skip latched objects -> at most one probe per object per TTL;
  * the latch self-clears (DEADLINE seam) -> a transient look-at failure never blacklists
    a live person permanently;
  * gaze recovers after the restart: a look-at activates on a live object (the phantom
    never captures the head);
  * normal operation (companion test) latches at most transiently (a controller abort
    can honestly fail one look-at) and the latch self-clears.

The latch asserts are an IMPLICATION (if a probe failed, it must have latched): once the
swm.py deinit fix prevents the phantom from forming, the staged scenario produces zero
probe failures and the implication is vacuous - the cap and recovery asserts still bind.

Mechanism-first asserts: SOAR log seams only (process/marker names, never tracker ids).
Do NOT stage this via behavior_synth.resets.set_operational - its failure fallback
respawns the soar node, which would destroy the frozen-WME state staged on purpose.
"""
import rospy
from std_srvs.srv import SetBool

from sweetie_bot_behavior_synth import agent_params, behavior_test, person, scene, soar_params

# Shrink ONLY store retention so the phantom forms in seconds instead of 600 s: the store
# entry must age out DURING the operational stop (perceive_end backdates to the first
# ROS-thread sweep ~2 s after vanish; removal at perceive_end + memorize_time). Visibility
# bins keep production values - the frozen `visible now` is the point of the reproduction.
_FAST_MEMORIZE = {"input/swm/memorize_time": 6}

_INIT_RX = r"INITIATE PROCESS: focusing-attention"
_FAILED_RX = r"FINISH PROCESS focusing-attention \S+ WITH STATUS failed"
_MISSING_RX = r"SWM object is missing \(not activated\)"
_LATCH_RX = r"SPECIFIC: ATTENTION-FOCUS-FAILED"
_TTL_RX = r"DEADLINE predicate attention-focus-failed"
# consumer-agnostic gaze health: the look-at module activated the controller on a live
# object (any consumer - attention focus, interlocutor gaze, looking-at-*)
_GAZE_OK_RX = r"lookat output module: set operational"


def _count(world, rx):
    return len(world.col["soar_log"].grep(rx))


def _set_operational(on):
    rospy.wait_for_service("/soar/set_operational", timeout=10.0)
    resp = rospy.ServiceProxy("/soar/set_operational", SetBool)(on)
    assert resp.success, "set_operational(%s) failed: %s" % (on, resp.message)


@behavior_test
@agent_params(**{"proactive/enabled": False})
@soar_params(**_FAST_MEMORIZE)
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_attention_never_machine_guns_a_phantom_after_stop(world):
    # 1. attention engages the visible person
    assert world.col["soar_log"].wait_grep(_INIT_RX, timeout=30.0), \
        "attention never engaged the visible person"

    # 2. freeze window: stop the kernel (update() stops -> input WMEs freeze at `visible
    #    now`), vanish the person, let the ROS-thread sweep age the store entry out
    #    (2 s seen_timeout + 6 s memorize_time + margin - the 8 Hz synth publisher keeps
    #    frames flowing, which is what drives the sweep while SOAR sleeps), then stage a
    #    live low-utility bystander (far -> the phantom's frozen utility must dominate,
    #    exactly like the live wedge).
    _set_operational(False)
    world.vanish(101)
    rospy.sleep(11.0)
    world.spawn(person(id=102, bearing=30.0, dist=3.5))
    rospy.sleep(2.5)

    # 3. restart: pre-fix this unleashed the ~4/s fast-fail machine-gun
    world.col["soar_log"].anchor()
    _set_operational(True)

    # 4. observation window > latch TTL (15 s): the cap must hold across a full TTL cycle
    rospy.sleep(20.0)
    n_missing = _count(world, _MISSING_RX)
    n_failed = _count(world, _FAILED_RX)
    assert n_missing <= 3, \
        "focusing-attention fast-fail livelock: %d 'SWM object is missing' in 20 s" % n_missing
    assert n_failed <= 4, \
        "failed-process churn despite the latch: %d FINISH failed in 20 s" % n_failed

    # 5. any failed probe must have latched its object (vacuous once the swm.py deinit
    #    fix stops the phantom from forming), and the latch must self-clear (TTL, not a
    #    tombstone - a transient failure may only cost 15 s of attention)
    if n_missing:
        assert _count(world, _LATCH_RX) >= 1, \
            "focusing-attention failed but attention-focus-failed never latched"
        assert world.col["soar_log"].wait_grep(_TTL_RX, timeout=20.0), \
            "attention-focus-failed latch never expired (deadline sweep did not clear it)"

    # 6. gaze recovered: a look-at ACTIVATED on a live object after the restart. NOT
    #    keyed on focusing-attention INITIATEs: with a lone newcomer the greeting owns
    #    the head (focusing-on-interlocutor / looking-at-*) and attention's proposals
    #    are legitimately resource-rejected - what matters is that the gaze machinery
    #    re-engaged reality instead of the phantom.
    assert world.col["soar_log"].wait_grep(_GAZE_OK_RX, timeout=10.0), \
        "no look-at activated on a live object after the restart"


@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_attention_latch_transient_only_in_normal_operation(world):
    # healthy visible person: attention cycles over reconsider deadlines. A single
    # transient latch is legitimate (the harness's startup recenter aborts an in-flight
    # look-at -> "LookAt controller failed or vanished (status: 4)" -> one honest failed
    # finish); the contract is that it stays ISOLATED and SELF-CLEARS - a transient
    # failure must never blacklist a live person permanently.
    assert world.col["soar_log"].wait_grep(_INIT_RX, timeout=30.0), \
        "attention never engaged the visible person"
    rospy.sleep(25.0)   # two focus-reconsider periods (10 s each)
    n_latch = _count(world, _LATCH_RX)
    assert n_latch <= 1, "latch churn on a healthy visible person: %d latches" % n_latch
    if n_latch:
        assert world.col["soar_log"].wait_grep(_TTL_RX, timeout=20.0), \
            "transient latch on a live person never expired (permanent blacklist)"
    assert _count(world, _MISSING_RX) == 0, "look-at lost a healthy SWM object"

    # genuine disappearance (store keeps the entry for memorize_time -> no look-at
    # failure; the evaluation frame just retracts) followed by a genuine appearance.
    # Gaze health is consumer-agnostic (_GAZE_OK_RX): the newcomer's greeting owns the
    # head, so a focusing-attention INITIATE is not guaranteed inside the window.
    world.col["soar_log"].anchor()
    world.vanish(101)
    rospy.sleep(6.0)
    world.spawn(person(id=111, bearing=10.0, dist=1.5))
    assert world.col["soar_log"].wait_grep(_GAZE_OK_RX, timeout=20.0), \
        "no look-at activated after a genuine disappearance/appearance episode"
    assert _count(world, _LATCH_RX) == 0, "a genuine disappearance episode latched"
    assert _count(world, _MISSING_RX) == 0, "genuine episode produced a look-at fast-fail"
