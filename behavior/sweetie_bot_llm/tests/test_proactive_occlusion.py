"""Occlusion-aware proactive cue selection (live gap, 2026-07-08 session 12).

While the camera is covered, SOAR sends no reply goals (no visible person to bind a talk
event to) and its own searching behavior owns "where did everyone go" — so the agent's
proactive self-talk is the ONLY path that can voice the occlusion complaint. Before this,
choose_proactive_cue read a covered camera as an empty scene and mused "the space is empty
and quiet" — the wrong message while blindfolded.
"""
from sweetie_bot_llm.proactive import ProactiveConfig, choose_proactive_cue

CFG = ProactiveConfig()


def test_occlusion_complaint_beats_alone_muse():
    # covered long enough + gaps satisfied -> the complaint, never the "empty space" muse
    cue = choose_proactive_cue(False, 100.0, 100.0, CFG, 0.0,
                               occluded_for=CFG.occluded_after + 1.0)
    assert cue == CFG.cue_occluded


def test_occlusion_complaint_fires_even_with_person_present():
    # partial cover can keep a person in retention: occlusion still outranks the lull branch
    cue = choose_proactive_cue(True, 100.0, 100.0, CFG, 0.0,
                               occluded_for=CFG.occluded_after + 1.0)
    assert cue == CFG.cue_occluded


def test_covered_but_not_yet_long_enough_stays_silent():
    # a brief cover: no complaint yet AND no misleading alone-muse either — silence
    cue = choose_proactive_cue(False, 100.0, 100.0, CFG, 0.0, occluded_for=1.0)
    assert cue is None


def test_occluded_gap_rate_limits_complaints():
    # past min_gap but inside occluded_gap -> hold the next complaint
    assert CFG.min_gap < CFG.occluded_gap, "test assumes occluded_gap > min_gap"
    mid = (CFG.min_gap + CFG.occluded_gap) / 2.0
    cue = choose_proactive_cue(False, 100.0, mid, CFG, 0.0, occluded_for=100.0)
    assert cue is None


def test_no_occlusion_keeps_existing_behavior():
    assert choose_proactive_cue(False, 100.0, 100.0, CFG, 0.0) == CFG.cue_alone
    assert choose_proactive_cue(False, 100.0, 100.0, CFG, 0.0,
                                occluded_for=None) == CFG.cue_alone
