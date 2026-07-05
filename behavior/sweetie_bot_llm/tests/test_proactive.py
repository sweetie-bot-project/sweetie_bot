"""CPU unit tests for the pure proactive-self-talk decision (no ROS)."""
from sweetie_bot_llm.proactive import ProactiveConfig, choose_proactive_cue

CFG = ProactiveConfig(min_gap=25, alone_after=20, alone_gap=45, lull_after=18, lull_prob=0.25)


def test_min_gap_blocks_everything():
    # too soon after the last aside -> nothing, regardless of presence
    assert choose_proactive_cue(False, 999, 10, CFG, 0.0) is None
    assert choose_proactive_cue(True, 999, 10, CFG, 0.0) is None


def test_alone_fires_when_empty_and_quiet():
    assert choose_proactive_cue(False, 30, 60, CFG, 0.99) == CFG.cue_alone


def test_alone_needs_both_activity_and_aside_gaps():
    assert choose_proactive_cue(False, 5, 60, CFG, 0.0) is None    # a turn happened too recently
    assert choose_proactive_cue(False, 30, 30, CFG, 0.0) is None   # aside gap 30 < alone_gap 45


def test_lull_is_probabilistic_and_needs_presence_and_delay():
    assert choose_proactive_cue(True, 30, 60, CFG, 0.10) in CFG.cue_lull_pool  # roll < prob
    assert choose_proactive_cue(True, 30, 60, CFG, 0.50) is None               # roll >= prob
    assert choose_proactive_cue(True, 5, 60, CFG, 0.0) is None                 # no lull yet (5<18)


def test_present_never_emits_the_alone_cue():
    for roll in (0.0, 0.3, 0.9):
        assert choose_proactive_cue(True, 999, 999, CFG, roll) != CFG.cue_alone


def test_lull_draws_varied_self_directed_cues():
    """A lull must prompt HER own thought (never commentary on the silent person, which produced
    'they might be thinking...'), and must VARY across pauses rather than repeat one bland line."""
    pool = ProactiveConfig().cue_lull_pool
    for cue in pool:
        low = cue.lower()
        for bad in ("the person", "gone quiet", "standing there", "without saying", "they ",
                    "not answer"):
            assert bad not in low, f"lull cue points at the person: {cue!r}"
    # different (gated) rolls select different subjects -> variety, not repetition
    seen = {choose_proactive_cue(True, 30, 60, CFG, r) for r in (0.01, 0.08, 0.15, 0.22)}
    assert len(seen) >= 3, f"lull cues barely vary: {seen}"


def test_explicit_cue_lull_override_wins():
    cfg = ProactiveConfig(lull_after=18, lull_prob=0.25, cue_lull="just my one fixed line")
    assert choose_proactive_cue(True, 30, 60, cfg, 0.10) == "just my one fixed line"
