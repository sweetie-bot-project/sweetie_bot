"""REGRESSION: touch-stress wedge (was a live-bug PROBE; the wedge was fixed in
5b64b83c — verbolize fast-fail loop — and this test has been green since; kept as the
permanent guard that rapid multi-zone touch during conversation never wedges SOAR again.
Probe-era diagnosis below, preserved for archaeology.)

Original probe notes: reenact the user's live sequence that killed touch reactions.

Live report (2026-07-06): "I talked to the robot and touched a LOT of places very quickly,
then tried the boop - and it's not working"; later NO touch zone reacted at all (operational
was ON, sensors fine). Hypothesis: the rapid multi-zone touch stress during conversation wedges
SOAR state - a rule-answering-on-event verbolize process that never completes (its text-action
never reaches text-action-status) keeps `^talking me` elaborated forever
(talk-llm*elaborate*answering-subporcess), blocking every -^talking touch rule; the eyes stay
LATCHED on the last touch emotion (reset-to-normal lives in terminate-verbolize), which reads
as "emotional reactions still happen" while nothing actually fires.

Sequence: converse -> 40s of rapid random multi-zone taps interleaved with speech -> then:
  CHECK 1 (boop):  held nose touch must dispatch voice/play_wav squeak_nose;
  CHECK 2 (wedge): held cheek touch must voice one of the CANNED pleasure phrases
                   (not just any say - a pause-comment could mask a wedge).
Either check failing = the live bug, reproduced in isolation.

Attribution notes (learned from test_boop_race_probe.py): `enjoyment_look` is the ONLY eyes
emotion exclusive to tactile_joy (happy/pleasure_look are shared with reply `joy`); assert on
mechanism markers (voice/syn goals + soar_log TALK/SPECIFIC markers), print emotions for the
postmortem only.
"""
import random
import time

import pytest
import rospy

from sweetie_bot_behavior_synth import agent_params, behavior_test, person, scene

ZONES = ["nose", "forehead", "cheek_left", "cheek_right", "temple_left", "temple_right"]

# canned cheek/temple pleasure phrases from talk_simple.soar (either variant rule)
_CHEEK_PHRASES = ("mmmmm", "yeah", "please touch this spot", "touch me here",
                  "right there", "purrr", "bliss", "do not stop", "feels so good",
                  "i forgive you")


def _touch_topic() -> str:
    try:
        return rospy.get_param("/soar/input/touch/topic")
    except KeyError:
        pytest.skip("no /soar/input/touch/topic param - touch module not configured")


def _collectors():
    from sweetie_bot_text_msgs.msg import TextActionActionGoal, TextCommand
    from sweetie_bot_behavior_synth.collectors import TopicCollector
    return (TopicCollector("voice/syn/goal", TextActionActionGoal),
            TopicCollector("control", TextCommand))


def _press(pub, keys, hold_s):
    from sweetie_bot_joystick.msg import KeyPressed
    m = KeyPressed()
    m.keys = list(keys)
    deadline = time.monotonic() + hold_s
    while time.monotonic() < deadline:
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        rospy.sleep(0.1)


def _release(pub):
    from sweetie_bot_joystick.msg import KeyPressed
    rel = KeyPressed()
    rel.header.stamp = rospy.Time.now()
    rel.keys = []
    pub.publish(rel)


def _hold_until(pub, keys, pred, timeout):
    deadline = time.monotonic() + timeout
    hit = None
    while time.monotonic() < deadline:
        _press(pub, keys, 0.3)
        hit = pred()
        if hit:
            break
    _release(pub)
    return hit


@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_touch_stress_then_boop_and_cheek(world):
    from sweetie_bot_joystick.msg import KeyPressed  # noqa: F401
    rnd = random.Random(42)                     # reproducible stress pattern
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    voice, control = _collectors()
    rospy.sleep(0.5)

    # establish a live conversation (talk context + most-recent-event)
    world.say_and_wait("Hello Sweetie!")

    # ---- STRESS: ~40s of rapid random touches (single and multi-zone) while conversing ------
    lines = ["You are so cute!", "Tell me what you like!", "Such a good little robot!",
             "Do you enjoy this?", "What a lovely pony you are!"]
    t_end = time.monotonic() + 40.0
    next_line = time.monotonic() + 2.0
    taps = 0
    while time.monotonic() < t_end:
        if time.monotonic() >= next_line and lines:
            world.speech.say(lines.pop(0))      # keep the talk machinery churning (no wait)
            next_line = time.monotonic() + 8.0
        n = 1 if rnd.random() < 0.7 else 2      # sometimes a hand covers 2 sensors
        keys = rnd.sample(ZONES, n)
        _press(pub, keys, rnd.uniform(0.3, 0.9))
        _release(pub)
        taps += 1
        rospy.sleep(rnd.uniform(0.15, 0.5))
    print("\n[stress] delivered %d rapid taps in 40s" % taps)
    rospy.sleep(4.0)                            # let any in-flight reactions settle

    # ---- CHECK 1: the boop (held nose -> squeak_nose must dispatch) --------------------------
    since1 = time.monotonic()

    def _boop_cmd():
        for m in voice.messages(since1):
            c = m.goal.command
            if c.type == "voice/play_wav" and "squeak_nose" in c.command:
                return c
        return None

    boop = _hold_until(pub, ["nose"], _boop_cmd, timeout=25.0)
    print("[check1:boop] play_wav after stress: %r" % (boop.command if boop else None))

    # ---- CHECK 2: any-zone reaction (held cheek -> a CANNED pleasure phrase) -----------------
    rospy.sleep(2.0)
    since2 = time.monotonic()

    def _cheek_cmd():
        for m in voice.messages(since2):
            c = m.goal.command
            if c.type.startswith("voice/say/") and \
                    any(p in c.command.lower() for p in _CHEEK_PHRASES):
                return c
        return None

    cheek = _hold_until(pub, ["cheek_left"], _cheek_cmd, timeout=25.0)
    print("[check2:cheek] canned phrase after stress: %r" % (cheek.command if cheek else None))

    # ---- postmortem dump ---------------------------------------------------------------------
    emos = [m.command for m in control.messages(0) if m.type == "eyes/emotion"]
    allv = [(m.goal.command.type, m.goal.command.command[:60]) for m in voice.messages(0)]
    print("[postmortem] eyes/emotion timeline: %s" % emos)
    print("[postmortem] all voice goals: %s" % allv)
    voice.close(); control.close()

    assert boop is not None, \
        "LIVE BUG REPRODUCED (facet A): after the touch stress, a held nose boop never " \
        "dispatched voice/play_wav squeak_nose"
    assert cheek is not None, \
        "LIVE BUG REPRODUCED (facet B/wedge): after the touch stress, a held cheek touch " \
        "voiced no canned pleasure phrase - touch reactions are dead"
