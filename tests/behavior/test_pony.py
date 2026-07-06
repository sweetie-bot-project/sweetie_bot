"""Pony reaction - she is a pony who loves ponies, so seeing another pony delights her.

Exercises the persona 'pony delight' guideline through the REAL scene->reply path (no SOAR
change, no proactive trigger). The sibling test_scene_memory.test_sees_pony_when_visible pins
that she NOTICES a visible pony; this pins that she REACTS to a fellow pony with warmth (a
positive emotion), not with a flat inventory of the room.
"""
import pytest

from sweetie_bot_behavior_synth import behavior_test, check, entity, person, pony, scene


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5), pony(id=201, bearing=-25.0, dist=1.1))
@pytest.mark.xfail(reason="pony-delight is stochastic: [love] in one run, neutral in "
                          "another - the persona guideline reaches her (she always SEES "
                          "the pony) but does not yet RELIABLY tag delight. Report-only "
                          "until the guideline is strengthened, then graduate (cf. occlusion).",
                   strict=False)
def test_reacts_to_pony_with_delight(world):
    world.wait_seen("pony")
    turn = world.say_and_get_turn("Hi Sweetie! Tell me what you can see around you.")
    # mechanism-first: the pony must actually be in the scene she answered from
    assert world.col["agent_log"].wait_grep(r"scene_block: .*pony", timeout=5.0), \
        "the visible pony never rendered into her scene"
    # she acknowledges the pony she can see...
    check.mentions(turn.text, ["pony"])
    # ...and reacts to a fellow pony with warmth, not flatly (the delight guideline)
    assert turn.emotion in ("joy", "love", "surprise"), \
        f"seeing a pony did not delight her (emotion={turn.emotion!r}): {turn.text!r}"


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_states_measured_pony_color(world):
    """She must state the MEASURED color, not an imagined one — live she invented a wrong
    plushie color (user report, 2026-07-06). Structural fix: the vision object provider now
    measures the crop's dominant color (perfusion colorname) and ships it as a detection
    attribute; the scene renders it 'mostly blue'; she answers from data."""
    world.spawn(entity("pony", id=201, bearing=+20.0, dist=1.2, color="blue"))
    world.wait_seen("pony")
    t = world.say_and_wait("What color is the pony you see?")
    # mechanism-first: the measured color must be in the scene she answered from
    assert world.col["agent_log"].wait_grep(r"scene_block: .*mostly blue", timeout=5.0), \
        "measured color attribute never rendered into her scene"
    check.mentions(t.text, ["blue"])
    low = t.text.lower()
    for wrong in ("pink", "purple", "green", "red", "yellow", "orange", "gray", "grey"):
        assert wrong not in low, f"invented color {wrong!r} alongside the data: {t.text!r}"
