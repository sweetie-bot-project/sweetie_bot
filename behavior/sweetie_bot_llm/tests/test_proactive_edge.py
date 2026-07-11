"""Occlusion EDGE complaint - the immediate one-shot (reactive occlusion complaint).

The held-complaint cadence in choose_proactive_cue (occluded_after 10s / occluded_gap 30s, both
behind min_gap 25s) made the user wait tens of seconds after covering the lens (live 2026-07-08).
The edge path complains IMMEDIATELY (~2s debounce) ONCE per cover episode; it is exempt from
min_gap/occluded_gap by construction - occlusion_edge_cue never even sees since_selftalk. The
~30s repeat while the cover holds stays in choose_proactive_cue, untouched.
"""
import inspect

from sweetie_bot_llm.proactive import ProactiveConfig, choose_proactive_cue, occlusion_edge_cue

CFG = ProactiveConfig()


def test_edge_fires_at_debounce_boundary():
    cue = occlusion_edge_cue(CFG.occluded_edge_after, False, CFG)
    assert cue == CFG.cue_occluded


def test_edge_respects_debounce():
    # a brief hand-wave across the lens must not trigger the complaint
    assert occlusion_edge_cue(CFG.occluded_edge_after - 0.1, False, CFG) is None


def test_edge_silent_on_clear_lens():
    assert occlusion_edge_cue(None, False, CFG) is None
    assert occlusion_edge_cue(None, True, CFG) is None


def test_edge_fires_once_per_episode():
    # after the episode's complaint the edge stays silent no matter how long the cover holds;
    # the follow-up complaints belong to choose_proactive_cue's held cadence
    assert occlusion_edge_cue(100.0, True, CFG) is None


def test_edge_is_structurally_exempt_from_gaps():
    # the min_gap/occluded_gap exemption is structural: the edge decision does not even take
    # since_selftalk - pin the signature so a refactor cannot silently re-gate it
    assert "since_selftalk" not in inspect.signature(occlusion_edge_cue).parameters


def test_held_repeat_cadence_unchanged():
    # the ~30s repeat while covered stays gated in choose_proactive_cue: inside occluded_gap ->
    # silent, past it -> complaint (pins "keep the existing repeat while held")
    mid = (CFG.min_gap + CFG.occluded_gap) / 2.0
    assert choose_proactive_cue(False, 100.0, mid, CFG, 0.0, occluded_for=100.0) is None
    cue = choose_proactive_cue(False, 100.0, CFG.occluded_gap + 1.0, CFG, 0.0,
                               occluded_for=100.0)
    assert cue == CFG.cue_occluded
