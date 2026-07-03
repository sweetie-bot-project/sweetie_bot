"""Touch reactions — UNKNOWN group (user: unsure touch generates LLM reactions at all today;
the canned SOAR reactions exist, the LLM-dilution rules are drafted but unsourced).

These tests DOCUMENT current behavior; they are report-only until classified into
must-pass/must-fail (see the faithfulness ledger in the harness README).

The touch input module is soar.yaml `touch: {type: joystick}` — synth touch = a Joy message
with the touch button mapping on the touch decoder's output topic. The exact topic/button map
is resolved at runtime from rosparam (/soar/input/touch/*).
"""
import pytest
import rospy

from sweetie_bot_behavior_synth import behavior_test, person, scene


def _touch_topic() -> str:
    try:
        return rospy.get_param("/soar/input/touch/topic")
    except KeyError:
        pytest.skip("no /soar/input/touch/topic param - touch module not configured")


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_touch_produces_any_reaction(world):
    # CLASSIFIED (was unknown): WORKS - the earlier no-repro was the wedged voice node (P24);
    # with a healthy voice the canned touch reaction voices within seconds.
    """Does ANY touch reaction (canned phrase or LLM) reach the voice within 15 s of a touch?"""
    from sweetie_bot_joystick.msg import KeyPressed
    topic = _touch_topic()
    pub = rospy.Publisher(topic, KeyPressed, queue_size=1)
    rospy.sleep(0.5)
    # live, touch reactions fired mid-conversation - establish the talk context first
    world.say_and_wait("Hello Sweetie!")
    n_says = len(world.col["say"].says())
    m = KeyPressed()
    m.keys = ["nose"]                   # boop
    for _ in range(8):
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        rospy.sleep(0.25)
    rel = KeyPressed()
    rel.header.stamp = rospy.Time.now()
    rel.keys = []
    pub.publish(rel)                    # release
    said = world.col["say"].wait_say(n_before=n_says, timeout=20.0)
    assert said is not None, "no vocal reaction to touch within 20s"
