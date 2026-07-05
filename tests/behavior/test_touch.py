"""Touch reactions — per-zone canned SOAR behavior (no LLM needed to fire them).

The touch input module is soar.yaml `touch: {type: joystick}`; a synth touch is a KeyPressed
message with the zone name on the touch decoder's output topic (button->name map in
`proto32/touch_config.yaml`: nose, forehead, cheek_left/right, temple_left/right). The exact
topic is resolved at runtime from rosparam (/soar/input/touch/*).

Touch reactions are gated on `-^talking` (they will not interrupt speech), so a momentary tap
right after an utterance is racy - HOLD the touch until she is free. We assert on the command
SOAR emits on voice/syn (the behavioural decision), not on audio-backend success.

Per-zone reactions (talk_simple.soar):
  * nose            -> SOUND ONLY: emits `voice/play_wav squeak_nose`, no spoken phrase (boop);
  * forehead        -> a canned rejection phrase (unwelcome touch);
  * cheek / temple  -> a canned pleasure phrase (the phrase zones).
"""
import time

import pytest
import rospy


from sweetie_bot_behavior_synth import behavior_test, person, scene


def _touch_topic() -> str:
    try:
        return rospy.get_param("/soar/input/touch/topic")
    except KeyError:
        pytest.skip("no /soar/input/touch/topic param - touch module not configured")


def _voice_collector():
    from sweetie_bot_text_msgs.msg import TextActionActionGoal
    from sweetie_bot_behavior_synth.collectors import TopicCollector
    return TopicCollector("voice/syn/goal", TextActionActionGoal)


def _hold_until(pub, keys, pred, timeout=25.0):
    """Hold a touch (keep publishing the press) until pred() is true or timeout, then release.

    Holding rides out any in-progress speech: the reaction fires the moment she is free."""
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


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_boop_nose_plays_sound_not_speech(world):
    """A boop on the nose issues the `squeak_nose` sound command and speaks NO phrase.

    The nose is a sound-only touch zone (user decision); spoken phrases live on the cheek/temple
    zones. Guards against the nose ever regressing to a spoken reaction.
    """
    from sweetie_bot_joystick.msg import KeyPressed
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    voice = _voice_collector()
    rospy.sleep(0.5)
    world.say_and_wait("Hello Sweetie!")            # establish talk context (most-recent-event)
    since = time.monotonic()

    def _boop_cmd():
        for msg in voice.messages(since):
            c = msg.goal.command
            if c.type == "voice/play_wav" and "squeak_nose" in c.command:
                return c
        return None

    try:
        hit = _hold_until(pub, ["nose"], _boop_cmd, timeout=25.0)
        assert hit is not None, "boop did not issue the squeak_nose sound command"
        # sound-only zone: no spoken phrase must accompany a boop
        says = [m.goal.command.command for m in voice.messages(since)
                if m.goal.command.type.startswith("voice/say/")]
        assert not says, f"boop unexpectedly spoke: {says}"
    finally:
        voice.close()


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_cheek_scratch_speaks_a_phrase(world):
    """A scratch on a cheek voices a canned pleasure phrase (a phrase zone)."""
    from sweetie_bot_joystick.msg import KeyPressed
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    voice = _voice_collector()
    rospy.sleep(0.5)
    world.say_and_wait("Hello Sweetie!")
    since = time.monotonic()

    def _say_cmd():
        for msg in voice.messages(since):
            if msg.goal.command.type.startswith("voice/say/"):
                return msg.goal.command
        return None

    try:
        hit = _hold_until(pub, ["cheek_left"], _say_cmd, timeout=25.0)
        assert hit is not None, "no spoken reaction to a cheek scratch"
    finally:
        voice.close()
