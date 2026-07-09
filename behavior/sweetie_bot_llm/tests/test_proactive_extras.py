"""Remaining decision-matrix cells for choose_proactive_cue (complements test_proactive.py).

Covers boundary equalities, degenerate configs (prob 0 / empty pool), the deterministic
roll->pool-index mapping the variety guarantee relies on, and the humans_present grace
predicate (T.1#3: a 1-frame detector dropout must not read as "alone").
"""
from types import SimpleNamespace

from sweetie_bot_llm.proactive import ProactiveConfig, choose_proactive_cue, humans_present

CFG = ProactiveConfig(min_gap=25, alone_after=20, alone_gap=45, lull_after=18, lull_prob=0.25)


# --- boundary equalities (>= vs >) -----------------------------------------------------------------

def test_min_gap_boundary_is_inclusive():
    # since_selftalk == min_gap is NOT "too soon" (guard is <); probe via the lull branch,
    # whose own gap requirement is exactly min_gap (the alone branch needs alone_gap=45 too)
    assert choose_proactive_cue(True, 30, 25.0, CFG, 0.0) in CFG.cue_lull_pool
    assert choose_proactive_cue(True, 30, 24.9, CFG, 0.0) is None


def test_alone_boundaries_are_inclusive():
    assert choose_proactive_cue(False, 20.0, 45.0, CFG, 0.0) == CFG.cue_alone
    assert choose_proactive_cue(False, 19.9, 45.0, CFG, 0.0) is None
    assert choose_proactive_cue(False, 20.0, 44.9, CFG, 0.0) is None


def test_lull_after_boundary_is_inclusive():
    assert choose_proactive_cue(True, 18.0, 60, CFG, 0.0) in CFG.cue_lull_pool
    assert choose_proactive_cue(True, 17.9, 60, CFG, 0.0) is None


def test_roll_equal_to_prob_does_not_fire():
    assert choose_proactive_cue(True, 30, 60, CFG, 0.25) is None      # roll < prob is strict


# --- degenerate configs ------------------------------------------------------------------------------

def test_lull_prob_zero_never_fires():
    cfg = ProactiveConfig(min_gap=0, lull_after=0, lull_prob=0.0)
    for roll in (0.0, 0.5, 0.999):
        assert choose_proactive_cue(True, 999, 999, cfg, roll) is None


def test_empty_pool_without_override_stays_silent():
    cfg = ProactiveConfig(min_gap=0, lull_after=0, lull_prob=1.0, cue_lull="",
                          cue_lull_pool=())
    assert choose_proactive_cue(True, 999, 999, cfg, 0.5) is None


def test_alone_gap_ignored_when_present():
    # a present human means the alone machinery must never trigger, even at huge gaps
    assert choose_proactive_cue(True, 30, 46, CFG, 0.99) is None      # roll blocks lull; no alone


# --- deterministic roll -> pool index mapping ---------------------------------------------------------

def test_pool_index_mapping_spreads_the_gated_roll():
    pool = CFG.cue_lull_pool
    n = len(pool)
    # roll just under the gate maps to the LAST pool entry; roll ~0 to the first
    assert choose_proactive_cue(True, 30, 60, CFG, 0.0) == pool[0]
    almost = 0.25 - 1e-9
    expected = pool[int((almost / 0.25) * n) % n]
    assert choose_proactive_cue(True, 30, 60, CFG, almost) == expected


def test_pool_index_exact_formula():
    pool = CFG.cue_lull_pool
    n = len(pool)
    for roll in (0.01, 0.08, 0.15, 0.22):
        expected = pool[int((roll / 0.25) * n) % n]
        assert choose_proactive_cue(True, 30, 60, CFG, roll) == expected


# --- humans_present grace predicate (T.1#3) --------------------------------------------------------

def _e(type_="human", in_frame=True, last_seen_s=0.0):
    return SimpleNamespace(type=type_, in_frame=in_frame, last_seen_s=last_seen_s)


def test_in_frame_human_present_even_with_zero_grace():
    assert humans_present([_e()], grace_s=0.0)


def test_dropout_within_grace_counts_as_present():
    # the live shape: a far/blurry person dropped for a frame, remembered 0.3s ago
    assert humans_present([_e(in_frame=False, last_seen_s=0.3)], grace_s=10.0)


def test_departed_beyond_grace_is_absent():
    assert not humans_present([_e(in_frame=False, last_seen_s=12.0)], grace_s=10.0)


def test_grace_boundary_is_inclusive():
    assert humans_present([_e(in_frame=False, last_seen_s=10.0)], grace_s=10.0)


def test_non_human_types_do_not_count():
    entities = [_e(type_="chair"), _e(type_="pony_face", in_frame=False, last_seen_s=1.0)]
    assert not humans_present(entities, grace_s=10.0)


def test_empty_scene_is_absent():
    assert not humans_present([], grace_s=10.0)
