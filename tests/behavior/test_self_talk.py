"""Proactive self-talk - the autonomous trigger wired on the LLM side (agent_node timer),
NOT SOAR. She speaks a brief spontaneous aside in two situations:

  A) nobody visible  -> she muses to herself (the "no one is here" moment);
  B) a human present but silent for ~>= one turn -> sometimes (probabilistic) she speaks up,
     never nagging (a statement, demands no answer).

These drive the REAL path: the llm_agent proactive driver (proactive.py decision -> self_talk
-> voice) running in the sim brain. Each test forces fast, deterministic timings via
@agent_params so it does not wait out the production cadence, and asserts mechanism-first on the
'[proactive]' agent-log marker (proving OUR driver fired, not a SOAR say). The pure decision is
unit-tested in behavior/sweetie_bot_llm/tests/test_proactive.py; this pins the wired behavior.

NOTE the coupling with test_conversation.test_no_monologue_after_reply: trigger B deliberately
speaks during a present lull, so both live in the same silence regime - reconciled because asides
are rate-limited (min_gap) and are statements, so they never CHAIN into a monologue.
"""
import re

from sweetie_bot_behavior_synth import agent_params, behavior_test, person, scene

_SAY_RX = re.compile(r"\[proactive\] say \[\w+\] (.+?)\s*$")
_QUOTES = "\"" + chr(39)   # both quote chars, without embedding a bare ' in source


def _proactive_said(world, timeout):
    """Wait for a VOICED proactive aside; return its (unquoted) text, asserting on timeout."""
    hits = world.col["agent_log"].wait_grep(r"\[proactive\] say \[", timeout=timeout)
    assert hits, "no proactive self-talk aside was voiced"
    m = _SAY_RX.search(hits[-1])
    return (m.group(1).strip().strip(_QUOTES) if m else hits[-1])


@behavior_test
@scene()   # nobody visible
@agent_params(**{"proactive/enabled": True, "proactive/period": 2.0, "proactive/min_gap": 1.0,
                 "proactive/alone_after": 2.0, "proactive/alone_gap": 2.0,
                 "proactive/lull_after": 999.0, "proactive/lull_prob": 0.0})
def test_empty_scene_prompts_self_talk(world):
    """With no one visible, the driver makes her say ONE spontaneous aside unprompted."""
    said = _proactive_said(world, timeout=30.0)
    assert said, "empty-scene aside had no text"
    # a spontaneous musing is a statement, not a question demanding an answer (no-nag)
    assert not said.endswith("?"), f"spontaneous aside nagged (question): {said!r}"


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))   # human present, but the test never speaks
@agent_params(**{"proactive/enabled": True, "proactive/period": 2.0, "proactive/min_gap": 1.0,
                 "proactive/alone_after": 999.0, "proactive/alone_gap": 999.0,
                 "proactive/lull_after": 2.0, "proactive/lull_prob": 1.0})
def test_present_but_silent_prompts_self_talk(world):
    """A human is visible but says nothing; after a one-turn lull she speaks up unprompted
    (lull_prob forced to 1.0 for determinism). alone_* is disabled so ONLY the present-lull
    trigger can fire - proving trigger B specifically, not the empty-scene path."""
    said = _proactive_said(world, timeout=30.0)
    assert said, "present-lull aside had no text"
    assert not said.endswith("?"), f"lull aside nagged (question): {said!r}"
