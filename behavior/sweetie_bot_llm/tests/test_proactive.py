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
    assert choose_proactive_cue(True, 30, 60, CFG, 0.10) == CFG.cue_lull  # roll < prob
    assert choose_proactive_cue(True, 30, 60, CFG, 0.50) is None          # roll >= prob
    assert choose_proactive_cue(True, 5, 60, CFG, 0.0) is None            # no lull yet (5<18)


def test_present_never_emits_the_alone_cue():
    for roll in (0.0, 0.3, 0.9):
        assert choose_proactive_cue(True, 999, 999, CFG, roll) in (None, CFG.cue_lull)
