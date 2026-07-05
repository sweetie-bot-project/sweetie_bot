"""Standard dialogue-flow regression — the baseline that must keep working as new behaviors land.

Covers, in sim (real path), everything from the dialogue-quality pass:
  * she stays SILENT before the SOAR operational toggle (reply path AND proactive self-talk gate);
  * a human APPEARING while operational -> she greets;
  * a QUESTION -> she answers WITHOUT reciting the scene description verbatim or speaking the
    internal person ids ("interlocutor 4");
  * a human DISAPPEARING -> she notices (missing-notice, or re-greets on return);
  * GOODBYE -> goodbye reaction;
  * repeated identical questions do NOT loop into the same stuck phrase (anti-repetition guard).

Mechanism-first: assert on SOAR `SPECIFIC:` markers (canned reactions get rephrased + the voice
transforms text, so never assert exact voiced words) and on the agent-log `[proactive]` marker.
The pure pieces are unit-tested separately (test_repeat_guard.py; test_proactive.py).
"""
import re

import rospy

from sweetie_bot_behavior_synth import agent_params, behavior_test, person, scene
from sweetie_bot_behavior_synth.dsl import get_spec
from sweetie_bot_behavior_synth.resets import set_operational


def starts_off(func):
    """Begin the scenario NON-operational (SOAR stopped) so we can prove pre-toggle silence."""
    get_spec(func).operational = False
    return func


# ---------------------------------------------------------------------------------------------
# Fix 1: no self-talk (and no reply) while NOT operational; both resume once toggled on.
# Proactive is forced aggressive so it WOULD fire in the window - the gate is what keeps her quiet.
# ---------------------------------------------------------------------------------------------
@behavior_test
@agent_params(**{"proactive/enabled": True, "proactive/period": 2.0, "proactive/min_gap": 1.0,
                 "proactive/alone_after": 2.0, "proactive/alone_gap": 2.0,
                 "proactive/lull_after": 2.0, "proactive/lull_prob": 1.0})
@starts_off
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_no_self_talk_while_not_operational(world):
    world.wait_seen("human")
    # even with proactive forced to fire every couple seconds, she must stay silent while stopped
    world.speech.say("Hello Sweetie!")
    world.expect_quiet(10.0)
    # toggle operational -> proactive self-talk resumes (agent-log marker, distinct from SOAR)
    assert set_operational(True), "set_operational(True) failed"
    assert world.col["agent_log"].wait_grep(r"\[proactive\]", timeout=25.0), \
        "proactive self-talk did not resume after operational was turned on"


# ---------------------------------------------------------------------------------------------
# The standard conversational flow + appeared/disappeared, plus the no-id / no-recite assertions.
# ---------------------------------------------------------------------------------------------
@behavior_test
@starts_off
@scene()
def test_standard_dialogue_flow(world):
    # a human appears while she is still NOT operational
    world.spawn(person(id=101, bearing=0.0, dist=1.5))
    world.wait_seen("human")
    # (1) SILENT before the operational toggle (reply path)
    world.speech.say("Hello Sweetie!")
    world.expect_quiet(8.0)
    # (2) operational on -> she greets the present, un-greeted human
    assert set_operational(True), "set_operational(True) failed"
    assert world.col["soar_log"].wait_grep("SPECIFIC: GREETING", timeout=25.0), \
        "no greeting after operational turned on"
    # (3) a question -> she answers, WITHOUT reciting the scene block or speaking internal ids
    t = world.say_and_wait("Hi Sweetie! What can you see around you right now?")
    assert t.said is not None and t.text.strip(), "no voiced answer to the question"
    low = t.text.lower()
    assert "interlocutor" not in low, f"spoke the internal 'interlocutor' label: {t.text!r}"
    assert "(id" not in low and not re.search(r"\bid\s*\d", low), f"spoke an internal id: {t.text!r}"
    assert "around you right now" not in low, f"recited the scene block verbatim: {t.text!r}"
    # (5a) a SECOND human appears -> greeted too
    world.spawn(person(id=102, bearing=-30.0, dist=1.6))
    assert world.col["soar_log"].wait_grep("SPECIFIC: GREETING", timeout=25.0), \
        "no greeting for the second human who appeared"
    # (5b) that human disappears -> missing-notice, or (robust fallback) re-greet on return
    world.vanish(102)
    rospy.sleep(4.5)   # SWM visibility timeout
    if not world.col["soar_log"].wait_grep("SPECIFIC: MISSING", timeout=8.0):
        world.spawn(person(id=102, bearing=-30.0, dist=1.6))
        assert world.col["soar_log"].wait_grep("SPECIFIC: GREETING", timeout=25.0), \
            "neither a missing-notice nor a re-greet after a human disappeared and returned"
    # (4) human says goodbye -> goodbye reaction
    world.speech.say("Okay, goodbye Sweetie!")
    assert world.col["soar_log"].wait_grep("SPECIFIC: GOODBYE", timeout=25.0), \
        "no goodbye reaction to 'goodbye'"


# ---------------------------------------------------------------------------------------------
# Re-answer loop (live-reported): after the human STOPS talking, SOAR re-emits the last speech
# event on the lull and — without the guard — she answers the SAME question again and again
# (live: "What's wrong with your feet now?" answered 3x, ~25s apart). History is context only:
# an already-answered turn re-poked on a pause must NOT be answered a second time. The agent logs
# a 're-poke' marker when the guard catches the duplicate.
# Proactive self-talk is disabled so the ONLY thing that could re-voice the answer is the SOAR
# re-poke path we are guarding.
# ---------------------------------------------------------------------------------------------
@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_does_not_reanswer_after_you_stop(world):
    # ask a distinctive question and let her answer it once
    t = world.say_and_wait("What's wrong with your feet now?")
    assert t.said is not None and t.text.strip(), "no first answer"
    n0 = len(world.col["say"].says())
    # now STOP. Over the lull SOAR re-poses the same turn; the guard must fire (marker) and she
    # must not re-answer the feet question. Give it long enough for a re-poke (~25-30s live).
    got_repoke = world.col["agent_log"].wait_grep(r"re-poke", timeout=75.0)
    followups = world.col["say"].says()[n0:]
    # she must never re-answer the feet question after you went quiet
    for s in followups:
        low = s.text.lower()
        reanswer = ("stiff" in low or "hoof" in low or "hooves" in low) and \
                   ("leg" in low or "feet" in low or "foot" in low or "joint" in low)
        assert not reanswer, f"re-answered the feet question after you stopped: {s.text!r}"
    # mechanism-first: if SOAR re-poked at all, the guard must have caught it (proves the wiring;
    # if SOAR never re-poked in this window there is nothing to guard and the behavioural check
    # above already held).
    assert got_repoke or not followups, \
        "SOAR re-poked but the re-answer guard never fired"


# ---------------------------------------------------------------------------------------------
# Cadence: she must take a deliberate beat before answering — FAIL if she answers too fast.
# ---------------------------------------------------------------------------------------------
@behavior_test
@agent_params(**{"reply_delay": 3.0})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_reply_is_not_too_fast(world):
    import time
    t0 = time.monotonic()
    t = world.say_and_wait("Hi Sweetie, tell me something nice.")
    elapsed = time.monotonic() - t0
    assert t.said is not None and t.text.strip(), "no voiced reply"
    # reply_delay is 3.0s here; generation adds more. Too-fast (<2.5s) means the pause was skipped.
    assert elapsed >= 2.5, f"answered too fast ({elapsed:.1f}s) — the deliberate pause was skipped"


# ---------------------------------------------------------------------------------------------
# Weaving: when the human goes silent, SOAR feeds a 'human stays silent' cue (+ scene/state notes);
# she must WEAVE these, never recite or answer them literally. Proactive self-talk disabled so the
# only follow-up speech is SOAR's pause-comment path (what we are testing).
# ---------------------------------------------------------------------------------------------
@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_does_not_recite_system_state(world):
    t = world.say_and_wait("Hi Sweetie! How are you today?")
    assert t.text.strip(), "no first reply"
    # go quiet -> SOAR injects the pause/no-answer cue; capture whatever she voices in response
    n0 = len(world.col["say"].says())
    rospy.sleep(28.0)
    followups = world.col["say"].says()[n0:]
    # banned = the internal cue phrasings she was caught reciting/commenting on live. The live
    # silence cue is "The person in front of you has gone quiet ... just standing there without
    # saying anything." — she must WEAVE, never echo it or narrate the person's position.
    banned = ["stays silent", "didn't answer", "says nothing", "human is silent", "no-answer",
              "interlocutor", "(id", "around you right now",
              "gone quiet", "standing there", "without saying", "what to say next",
              "thinking about what to say", "the person in front", "the one in front",
              "in front of me", "gaze_pitch", "gaze_yaw"]
    for s in followups:
        low = s.text.lower()
        for b in banned:
            assert b not in low, f"recited a system-state phrase {b!r} verbatim: {s.text!r}"


# ---------------------------------------------------------------------------------------------
# Recitation (live-reported: "she talks about the human like 'the one in front of you'"): when
# talking WITH one person she must address them directly and must NOT narrate their scene
# position. The virtual human speaks to her; her spoken reply is what we check.
# ---------------------------------------------------------------------------------------------
@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_does_not_narrate_human_position(world):
    world.wait_seen("human")
    # she is invited to refer to the person she is looking at; a natural answer is "you", a
    # reciting answer parrots the scene block's spatial data about the person.
    t = world.say_and_wait("Hi Sweetie! I'm so happy to see you. Who are you looking at right now?")
    assert t.said is not None and t.text.strip(), "no voiced reply"
    low = t.text.lower()
    banned = ["in front of", "the one in front", "someone in front", "a person in front",
              "to my left", "to my right", "to your left", "to your right",
              "interlocutor", "(id"]
    for b in banned:
        assert b not in low, f"narrated the human's scene position ({b!r}): {t.text!r}"
