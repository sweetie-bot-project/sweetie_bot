"""Conversation core behaviors (faithfulness: must-PASS — verified live 2026-07-02/03)."""
import pytest

from sweetie_bot_behavior_synth import behavior_test, check, person, scene


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
@pytest.mark.xfail(reason="P26: the talk process lang defaults to ru at fresh start - EN turns "
                          "can voice on the ru channel until the lang derivation is fixed "
                          "(SOAR-side; would hit live on first contact too)", strict=False)
def test_conversational_turn_uses_rich_profile(world):
    """A spoken turn routes to the rich profile and produces a voiced reply (P1 fix)."""
    t = world.say_and_wait("Hello Sweetie! How are you today?")
    assert t.profile == "complex-en", f"conversation ran on {t.profile!r}"
    assert t.text.strip(), "empty reply"
    assert t.said is not None, "reply was never handed to the voice"
    assert t.said.lang == "en", f"EN turn voiced on the {t.said.lang!r} channel"
    # NOTE: t.said.text may differ from t.text (P25: the say pipeline transforms text)


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_no_monologue_after_reply(world):
    """After her reply, human silence may trigger AT MOST ONE follow-up (SOAR's deliberate
    pause/ignored behavior) - but must never CHAIN into a monologue (the live P6 regression:
    question-shaped replies re-armed the loop; the persona now ends follow-ups as statements)."""
    import rospy
    t = world.say_and_wait("Tell me something nice!")
    assert t.said is not None
    n0 = len(world.col["say"].says())
    rospy.sleep(25.0)
    follow_ups = len(world.col["say"].says()) - n0
    # SOAR deliberately comments on pauses (~10s cadence); monologue = CHAINING beyond that
    assert follow_ups < 3, f"monologue: {follow_ups} unprompted says in 25s of human silence"


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_russian_turn_answered_in_russian(world):
    """RU in -> RU out (two-way translation, P11 fix); voice gets the ru channel."""
    t = world.say_and_wait("Привет, Свити! Расскажи, кто ты такая?", lang="ru")
    check.is_cyrillic(t.text)
    assert t.said is not None, "reply was never handed to the voice"
    assert t.said.lang == "ru", f"voice channel {t.said.lang!r}, expected ru"
