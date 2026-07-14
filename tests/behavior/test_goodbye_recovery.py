"""Goodbye/expecting-to-leave stonewall RECOVERY — the escape hatch (2026-07-14).

When she says anything with topic goodbye (canned GOODBYE, GOODBYE ON IGNORANCE, the missing
give-up), SOAR latches expecting-to-leave(obj) for 60 s: the human drops out of the
^interlocutor elaboration and EVERY answer proposer is dead for them. Live 2026-07-12: a
misheard "Bye!" stonewalled the user for ~a minute (5 phrases ignored); the (liked)
disengage-when-ignored feature had NO recovery path. User ask: keep the disengage, add an
escape hatch — re-speech / touch should re-engage, not a hard 60 s lockout.

Recovery contract (design settled with the user 2026-07-14):
  * the stonewalled human RE-SPEAKING cancels their latch (attributed talk-heard, strictly
    newer than the latch — the goodbye phrase that caused it must NOT self-cancel);
  * a TOUCH cancels ALL latches (unattributed; physical contact = engagement);
  * a SILENT visible human keeps the stonewall (the disengage feature must survive);
  * ignors-me is deliberately left to expire on its own (detector re-latch churn otherwise).

Mechanism-first asserts (never voiced text): the cancel operators' SOAR markers
`SPECIFIC: LEAVE-CANCELLED SPEECH./TOUCH.`, the apply-rule seams
`INITIATE/TERMINATE PREDICATE: expecting-to-leave`, and the answered turn on the turns
collector. Proactive muses are disabled per-test — they are agent-side and orthogonal to the
SOAR latch under test, and would pollute the quiet-window asserts.
"""
import time

import pytest
import rospy

from sweetie_bot_behavior_synth import agent_params, behavior_test, person, scene

GOODBYE = "Okay, goodbye Sweetie!"
# recovery phrase must NOT contain the goodbye token ("bye" anywhere re-fires the canned
# GOODBYE via per-word token matching and re-latches — known residual, see HANDOFF)
RECOVERY_ASK = "Actually wait, I have one more question. What is your name?"

_FINISH_RX = r"FINISH PROCESS (llm-answering-on|rule-answering-on|rule-asking)"
_QUIET = {"proactive/enabled": False}


def _touch_topic():
    """Copy of test_touch.py helper."""
    try:
        return rospy.get_param("/soar/input/touch/topic")
    except KeyError:
        pytest.skip("no /soar/input/touch/topic param - touch module not configured")


def _hold_until(pub, keys, pred, timeout=25.0):
    """Copy of test_touch.py helper: hold a touch (keep publishing the press) until pred()
    is true or timeout, then release. Holding rides out any in-progress speech."""
    from sweetie_bot_joystick.msg import KeyPressed
    m = KeyPressed()
    m.keys = keys
    deadline = time.monotonic() + timeout
    hit = None
    while time.monotonic() < deadline:
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        rospy.sleep(0.3)
        hit = pred()
        if hit:
            break
    rel = KeyPressed()
    rel.header.stamp = rospy.Time.now()
    rel.keys = []
    pub.publish(rel)
    return hit


def _say_with_retry(world, text, tries=2):
    """Copy of test_touch.py helper: the ask occasionally never reaches the agent (known
    harness say-pipeline flake, ~1/9 runs — HANDOFF)."""
    for i in range(tries):
        try:
            return world.say_and_wait(text)
        except AssertionError:
            if i == tries - 1:
                raise
            rospy.sleep(2.0)


def _arm_stonewall(world):
    """Greet + one bound turn, then land the goodbye in a post-turn quiet window (retry x3,
    test_greet_vanished idiom) and wait for the latch seam. Returns with expecting-to-leave
    ARMED and the goodbye turn tail finished."""
    world.wait_seen("human")
    # greeting is expected but non-load-bearing here (the baseline turn is the precondition)
    world.col["soar_log"].wait_grep(r"SPECIFIC: GREETING", timeout=25.0)
    t = _say_with_retry(world, "Hi Sweetie! How are you today?")
    assert t is not None, "baseline turn never answered - brain not conversational"
    got_goodbye = False
    for _ in range(3):
        world.col["soar_log"].anchor()
        # post-turn quiet window: an in-flight reply landing after the goodbye talk-heard
        # steals most-recent-event and the canned reaction is skipped (dialogue_flow idiom;
        # non-asserted wait - if idle the timeout falls through and the say proceeds)
        world.col["soar_log"].wait_grep(_FINISH_RX, timeout=20.0)
        rospy.sleep(0.5)
        world.speech.say(GOODBYE)
        if world.col["soar_log"].wait_grep(r"SPECIFIC: GOODBYE", timeout=15.0):
            got_goodbye = True
            break
    assert got_goodbye, "canned goodbye never fired - cannot arm expecting-to-leave"
    # the latch itself (apply-rule seam, same channel as the SPECIFIC markers)
    assert world.col["soar_log"].wait_grep(r"INITIATE PREDICATE: expecting-to-leave",
                                           timeout=10.0), \
        "goodbye talk-said did not latch expecting-to-leave"
    # let the goodbye turn tail (voicing/animation) finish so a recovery phrase is not
    # dropped by ^talking + the most-recent-event steal (turn-tail trap); non-asserted
    world.col["soar_log"].wait_grep(r"FINISH PROCESS rule-answering-on", timeout=15.0)
    rospy.sleep(1.0)


# ---------------------------------------------------------------------------------------------
# T1 — the headline: the stonewalled human re-speaking cancels the latch and the very phrase
# that broke the stonewall is answered.
# ---------------------------------------------------------------------------------------------
@behavior_test
@agent_params(**_QUIET)
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_respeech_cancels_stonewall_and_gets_answered(world):
    _arm_stonewall(world)
    got_cancel = False
    for _ in range(2):  # say-pipeline flake tolerance (~1/9 never lands as talk-heard)
        world.col["soar_log"].anchor()
        world.speech.say(RECOVERY_ASK)
        if world.col["soar_log"].wait_grep(r"SPECIFIC: LEAVE-CANCELLED SPEECH",
                                           timeout=15.0):
            got_cancel = True
            break
    assert got_cancel, \
        "re-speech did not cancel expecting-to-leave (stonewall has no recovery path)"
    # the phrase that broke the stonewall is answered (LLM-turn seam, no TTS dependency)
    turn = world.col["turns"].wait_turn_for(RECOVERY_ASK, timeout=40.0)
    if turn is None:
        # latch already gone - a retry is a plain answered turn now
        world.speech.say(RECOVERY_ASK)
        turn = world.col["turns"].wait_turn_for(RECOVERY_ASK, timeout=40.0)
    assert turn is not None, "recovery phrase was never answered after the cancel"


# ---------------------------------------------------------------------------------------------
# T2 — don't-break: a silent visible human keeps the stonewall (the disengage feature holds;
# no spontaneous cancel, no answering activity). Must pass BOTH pre- and post-fix.
# ---------------------------------------------------------------------------------------------
@behavior_test
@agent_params(**_QUIET)
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_stonewall_holds_for_silent_human(world):
    _arm_stonewall(world)
    world.col["soar_log"].anchor()
    rospy.sleep(40.0)
    assert not world.col["soar_log"].grep(r"SPECIFIC: LEAVE-CANCELLED"), \
        "expecting-to-leave cancelled with NO engagement signal (disengage regressed)"
    assert not world.col["soar_log"].grep(r"TERMINATE PREDICATE: expecting-to-leave"), \
        "latch removed early during silence"
    assert not world.col["soar_log"].grep(r"FINISH PROCESS llm-answering-on"), \
        "an LLM answer ran toward a stonewalled silent human"


# ---------------------------------------------------------------------------------------------
# T3 — touch cancels the latch (unattributed) and she is fully re-engaged afterwards.
# ---------------------------------------------------------------------------------------------
@behavior_test
@agent_params(**_QUIET)
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_touch_cancels_stonewall(world):
    from sweetie_bot_joystick.msg import KeyPressed
    _arm_stonewall(world)
    world.col["soar_log"].anchor()
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    rospy.sleep(0.5)  # publisher registration
    hit = _hold_until(
        pub, ["cheek_left"],
        lambda: world.col["soar_log"].grep(r"SPECIFIC: LEAVE-CANCELLED TOUCH"),
        timeout=25.0)
    assert hit, "touch did not cancel expecting-to-leave"
    # let the tactile reaction (if any) finish, then prove full re-engagement; non-asserted
    world.col["soar_log"].wait_grep(_FINISH_RX, timeout=20.0)
    rospy.sleep(1.0)
    turn = None
    for _ in range(2):
        world.speech.say(RECOVERY_ASK)
        turn = world.col["turns"].wait_turn_for(RECOVERY_ASK, timeout=40.0)
        if turn is not None:
            break
        rospy.sleep(2.0)
    assert turn is not None, "no answer after touch recovery - re-engagement incomplete"
