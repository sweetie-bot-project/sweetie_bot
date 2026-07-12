"""CPU unit tests for the PTT/speech gate on the proactive self-talk driver.

Restores the pre-refactor "wait for the human" behavior: while a human is speaking or holding the
push-to-talk button she must NOT muse over them. The gate regressed when commit 7e71bad1 moved
proactive muses out of SOAR (bypassing SOAR's -^talking / io.input-link.sound.speech gate); this
mirrors that gate on the agent side using the same SPEECH_DETECTING signal + the raw mic_button.
"""
from sweetie_bot_llm.proactive import (
    ProactiveConfig,
    choose_proactive_cue,
    is_human_speaking,
)

CFG = ProactiveConfig(min_gap=25, alone_after=20, alone_gap=45, lull_after=18, lull_prob=0.25,
                      speech_grace=2.5)


def test_human_speaking_suppresses_lull():
    # a present lull that WOULD fire (roll < prob, past lull_after) is silenced while the human speaks
    assert choose_proactive_cue(True, 30, 60, CFG, 0.10) in CFG.cue_lull_pool
    assert choose_proactive_cue(True, 30, 60, CFG, 0.10, human_speaking=True) is None


def test_human_speaking_suppresses_alone():
    assert choose_proactive_cue(False, 30, 60, CFG, 0.99) == CFG.cue_alone
    assert choose_proactive_cue(False, 30, 60, CFG, 0.99, human_speaking=True) is None


def test_human_speaking_suppresses_occlusion_complaint():
    # even a covered lens must not make her talk over the human mid-question
    assert choose_proactive_cue(True, 30, 60, CFG, 0.0, occluded_for=30) == CFG.cue_occluded
    assert choose_proactive_cue(True, 30, 60, CFG, 0.0, occluded_for=30, human_speaking=True) is None


def test_default_human_speaking_is_false_preserves_behaviour():
    # omitting the kwarg keeps the original decision (back-compat for existing callers/tests)
    assert choose_proactive_cue(True, 30, 60, CFG, 0.10) in CFG.cue_lull_pool


def test_is_human_speaking_ptt_held():
    # a held button counts as speaking regardless of how long since the last speech-detect frame
    assert is_human_speaking(True, 999.0, 2.5) is True


def test_is_human_speaking_within_grace():
    assert is_human_speaking(False, 1.0, 2.5) is True     # spoke 1s ago, within 2.5s grace
    assert is_human_speaking(False, 2.4, 2.5) is True


def test_is_human_speaking_after_grace():
    assert is_human_speaking(False, 2.5, 2.5) is False    # boundary: grace elapsed -> not speaking
    assert is_human_speaking(False, 10.0, 2.5) is False


def test_default_lull_after_is_27():
    # user 2026-07-12: she mused too much in a present lull; raise the delay to ~25-30 s
    assert ProactiveConfig().lull_after == 27.0
