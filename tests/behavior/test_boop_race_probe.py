"""PROBE: reproduce the LIVE boop signature — emotion displays, squeak_nose sound never plays.

Live diagnosis (boop_sound_diagnosis_2026-07-06.md): on the real robot, nose boops showed an
emotional reaction but NO `voice/play_wav squeak_nose` was ever dispatched (zero
`TALK: text action play file` markers, zero `executing (voice/play_wav...)`), while text-zone
touches voiced fine. The held-touch sim test passes, so the chain is healthy under a HOLD.
Hypothesis: momentary TAPS (real boops) race the verbolize process — the emotion operator
(automatic, instant `eyes/emotion` on /control) runs, but the sound text-action (multi-cycle
substate + actionlib round-trip) is abandoned when the press vanishes / collides with speech.

Three scenarios, each a fresh SOAR reset:
  1. short tap while idle
  2. short tap DURING her speech (touch rules are gated on -^talking: expect nothing at all;
     emotion-without-sound here would also match live)
  3. tap train across the end-of-speech boundary (the racy window called out in test_touch.py)

Healthy contract asserted in each: a TOUCH-SOURCED emotion display implies the squeak_nose
sound was also dispatched. Emotion-without-sound = the live bug, reproduced.

ATTRIBUTION (2026-07-06 rework): happy_look/pleasure_look also fire for a reply's `joy`
emotion, so a sighting near one of her say goals proves nothing (first probe version failed
on exactly that false positive — her story reply's own emotion). An emotion event counts as
touch-sourced ONLY if it is enjoyment_look (exclusive to ^emotion tactile_joy), or it lands
more than 4s away from every voice/say goal. Timelines are printed for postmortem.
"""
import time

import pytest
import rospy

from sweetie_bot_behavior_synth import agent_params, behavior_test, person, scene

# eyes emotions reachable from ^emotion tactile_joy (initialization-proto3.soar mapping);
# enjoyment_look is EXCLUSIVE to tactile_joy, happy/pleasure also carry a reply's joy.
_TOUCH_EMOTIONS = ("enjoyment_look", "happy_look", "pleasure_look")


def _touch_topic() -> str:
    try:
        return rospy.get_param("/soar/input/touch/topic")
    except KeyError:
        pytest.skip("no /soar/input/touch/topic param - touch module not configured")


def _collectors():
    from sweetie_bot_text_msgs.msg import TextActionActionGoal, TextCommand
    from sweetie_bot_behavior_synth.collectors import TopicCollector
    voice = TopicCollector("voice/syn/goal", TextActionActionGoal)
    control = TopicCollector("control", TextCommand)
    return voice, control


def _observe_timed(voice, control, since):
    """Return timed event lists seen since `since`:
    (emotion (t, TextCommand)), (squeak plays (t, cmd)), (says (t, cmd))."""
    with control._lock:
        ctl = [(t, m) for t, m in control._buf if t >= since]
    with voice._lock:
        vc = [(t, m) for t, m in voice._buf if t >= since]
    emotions = [(t, m) for t, m in ctl if m.type == "eyes/emotion"]
    plays = [(t, m.goal.command) for t, m in vc
             if m.goal.command.type == "voice/play_wav"
             and "squeak_nose" in m.goal.command.command]
    says = [(t, m.goal.command) for t, m in vc
            if m.goal.command.type.startswith("voice/say/")]
    return emotions, plays, says


def _touch_sourced(emotions, says):
    """Emotion events attributable to a TOUCH reaction, not to a reply's own emotion.

    enjoyment_look is exclusive to tactile_joy -> always touch-sourced. happy_look /
    pleasure_look also fire for a reply's joy: count them only when no voice/say goal
    went out within +-4s (verbolize-llm emits its emotion textcmd around its say dispatch)."""
    touch = []
    for t, m in emotions:
        if m.command not in _TOUCH_EMOTIONS:
            continue
        if m.command == "enjoyment_look" or \
                not any(abs(t - ts) <= 4.0 for ts, _ in says):
            touch.append((t, m))
    return touch


def _tap(pub, press_s=0.4):
    """One momentary tap: press for press_s, then release. Returns (t_start, t_end)."""
    from sweetie_bot_joystick.msg import KeyPressed
    m = KeyPressed()
    m.keys = ["nose"]
    t0 = time.monotonic()
    deadline = t0 + press_s
    while time.monotonic() < deadline:
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        rospy.sleep(0.1)
    rel = KeyPressed()
    rel.header.stamp = rospy.Time.now()
    rel.keys = []
    pub.publish(rel)
    return t0, time.monotonic()


def _report(tag, since, taps, emotions, touch, plays, says):
    print("\n[probe:%s] taps=%s" % (tag, ["%.1f-%.1f" % (a - since, b - since) for a, b in taps]))
    print("[probe:%s] emotions=%s (touch-sourced: %s)"
          % (tag, ["%.1f:%s" % (t - since, m.command) for t, m in emotions],
             ["%.1f:%s" % (t - since, m.command) for t, m in touch]))
    print("[probe:%s] squeaks=%s says=%s"
          % (tag, ["%.1f" % (t - since) for t, _ in plays],
             ["%.1f:%r" % (t - since, c.command[:40]) for t, c in says]))


@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_tap_idle(world):
    """A single momentary tap while she is quiet."""
    from sweetie_bot_joystick.msg import KeyPressed  # noqa: F401
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    voice, control = _collectors()
    rospy.sleep(0.5)
    world.say_and_wait("Hello Sweetie!")          # establish talk context (most-recent-event)
    rospy.sleep(2.0)                              # let the reply's own emotion settle
    since = time.monotonic()
    try:
        taps = [_tap(pub, press_s=0.4)]
        rospy.sleep(8.0)                          # generous window for the sound to dispatch
        emotions, plays, says = _observe_timed(voice, control, since)
        touch = _touch_sourced(emotions, says)
        _report("tap-idle", since, taps, emotions, touch, plays, says)
        assert not (touch and not plays), \
            "LIVE BUG REPRODUCED (tap-idle): touch emotion %s displayed but squeak_nose " \
            "never dispatched" % [m.command for _, m in touch]
    finally:
        voice.close(); control.close()


@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_tap_during_speech(world):
    """A momentary tap in the middle of her voicing a reply (touch rules gated on -^talking)."""
    from sweetie_bot_joystick.msg import KeyPressed  # noqa: F401
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    voice, control = _collectors()
    rospy.sleep(0.5)
    world.say_and_wait("Hello Sweetie!")
    rospy.sleep(2.0)
    since = time.monotonic()
    try:
        world.speech.say("Please tell me a nice long story about your adventures!")
        # wait until her say goal actually goes out, then tap mid-TTS
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if any(m.goal.command.type.startswith("voice/say/") for m in voice.messages(since)):
                break
            rospy.sleep(0.3)
        rospy.sleep(1.0)                          # she is voicing now
        taps = [_tap(pub, press_s=0.4)]
        rospy.sleep(10.0)
        emotions, plays, says = _observe_timed(voice, control, since)
        touch = _touch_sourced(emotions, says)
        _report("tap-during-speech", since, taps, emotions, touch, plays, says)
        assert not (touch and not plays), \
            "LIVE BUG REPRODUCED (tap-during-speech): touch emotion %s displayed but " \
            "squeak_nose never dispatched" % [m.command for _, m in touch]
    finally:
        voice.close(); control.close()


@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_tap_train_across_speech_end(world):
    """Momentary taps every ~1s spanning her speech and its end - hunts the -^talking flip race
    the harness docstring calls out ('a momentary tap right after an utterance is racy')."""
    from sweetie_bot_joystick.msg import KeyPressed
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    voice, control = _collectors()
    rospy.sleep(0.5)
    world.say_and_wait("Hello Sweetie!")
    rospy.sleep(2.0)
    since = time.monotonic()
    try:
        world.speech.say("Please tell me a nice long story about your adventures!")
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if any(m.goal.command.type.startswith("voice/say/") for m in voice.messages(since)):
                break
            rospy.sleep(0.3)
        # tap train: 12 taps x (0.4s press + 0.8s gap) ~= 15s, spanning TTS + its end
        taps = []
        for _ in range(12):
            taps.append(_tap(pub, press_s=0.4))
            rospy.sleep(0.8)
        rospy.sleep(8.0)
        emotions, plays, says = _observe_timed(voice, control, since)
        touch = _touch_sourced(emotions, says)
        _report("tap-train", since, taps, emotions, touch, plays, says)

        # WEDGE CHECK (live Symptom B): a stuck boop verbolize process keeps `^talking me`
        # elaborated (talk-llm*elaborate*answering-subporcess) which blocks EVERY -^talking
        # touch rule -> "no reactions to any touch at all". A held cheek touch must still
        # voice its pleasure phrase; silence here = the wedge, reproduced.
        since2 = time.monotonic()

        def _cheek_phrase():
            for m in voice.messages(since2):
                if m.goal.command.type.startswith("voice/say/"):
                    return m.goal.command
            return None

        deadline2 = time.monotonic() + 25.0
        hit = None
        m2 = KeyPressed()
        m2.keys = ["cheek_left"]
        while time.monotonic() < deadline2:
            m2.header.stamp = rospy.Time.now()
            pub.publish(m2)
            rospy.sleep(0.3)
            hit = _cheek_phrase()
            if hit:
                break
        rel = KeyPressed(); rel.keys = []
        rel.header.stamp = rospy.Time.now()
        pub.publish(rel)
        print("\n[probe:wedge-check] cheek phrase after tap train: %r"
              % (hit.command if hit else None))

        # facet 1 (Symptom A): touch emotion displayed but the squeak was NEVER dispatched
        assert not (touch and not plays), \
            "LIVE BUG REPRODUCED (tap-train): touch emotions %s displayed but squeak_nose " \
            "NEVER dispatched" % [m.command for _, m in touch]
        # facet 2 (Symptom B): touch reactions wedged after the boop train
        assert hit is not None, \
            "LIVE BUG REPRODUCED (wedge): no reaction to a held cheek touch after the boop train " \
            "(stuck rule-answering-on-event process holds ^talking)"
    finally:
        voice.close(); control.close()
