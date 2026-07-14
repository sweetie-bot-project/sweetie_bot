"""Unit tests for the position-recital guard on the reply path (ROS-free).

Live incident (14B suite 2026-07-14, consistent 3/3 reruns; the 7B flaked the same ~1-in-2,
2026-07-11): "Who are you looking at?" with a LONE person in frame — the scene block renders
them as "you" with NO position (scene.py locate_people), yet the model narrated "You're right
in front of me." from thin air. Nothing to remove at the source (pure inference, not a data
leak), so the backstop is post-decode and DETERMINISTIC, model-agnostic by construction:
strip the offending sentences (keeps the good part of the answer, no GPU); regenerate once
with the missing-datum note only when stripping guts the reply; land on the fixed fallback
if the retry recites again. Position talk stays LEGAL when the prompt itself carried it
(crowds, located objects) — the gate reads the rendered scene_block, the single source of
truth for what she perceived. Scene jargon (interlocutor, raw ids) is banned unconditionally:
the prompt forbids speaking the id even when positions render.
"""
import json

import pytest

from sweetie_bot_ai_core import AgentRequest, SceneConfig, SceneState, classify_zone
from sweetie_bot_ai_core.agent import Agent, _echoes_note
from sweetie_bot_ai_core.client import ChatResult
from sweetie_bot_ai_core.persona import PersonaRegistry
from sweetie_bot_ai_core.schema import SceneEntity

CFG = SceneConfig(front_deg=60, side_deg=120, max_entities=3)

LIVE_14B_REPLY = "I'm looking at you! You're right in front of me."


def _c(text, emo="joy"):
    return json.dumps({"response_text": text, "emotion": emo, "sentence_type": "statement"})


def _person(id=1, bearing=0.0, **kw):
    return SceneEntity(id=id, type="human", bearing_deg=bearing,
                       zone=classify_zone(bearing, CFG), **kw)


class _SeqReg:
    """Duck-typed registry returning queued contents; records every message list."""
    def __init__(self, *contents):
        self.contents = list(contents)
        self.calls = []

    def chat(self, messages, **kw):
        self.calls.append(messages)
        content = self.contents.pop(0) if len(self.contents) > 1 else self.contents[0]
        return ChatResult(content=content), "seq"


class SceneStub:
    def __init__(self, state):
        self.state = state

    def snapshot(self, include_remembered=False):
        return self.state


def _agent(entities, *contents):
    a = Agent(_SeqReg(*contents), personas=PersonaRegistry(),
              scene_provider=SceneStub(SceneState(entities=list(entities))),
              scene_config=CFG)
    # pure reply path: the default ToolRegistry offers tools, and the tool loop would consume
    # queued contents before the final structured reply — call counts must stay deterministic
    a.tools._tools.clear()
    return a


ASK = "Who are you looking at right now?"


# --- recital regex: live phrasings vs innocent uses of the same words --------------------------

RECITES = [
    LIVE_14B_REPLY,                                # 14B, consistent (test logs 2026-07-14)
    "The one in front of me, of course!",
    "I'm watching someone directly in front of me.",
    "You're standing to my right.",                # 7B flake shape ("to my right", 2026-07-11)
    "I see someone on your left.",
    "There's a person just to the left of you.",
    "You are at my left side.",
]

INNOCENT = [
    "I'm looking at you!",
    "You're right about that!",
    "All right, let's play!",
    "That's right!",
    "You left so quickly yesterday.",
    "I write with my right hoof.",
    "Right you are, my friend.",
]


@pytest.mark.parametrize("text", RECITES)
def test_position_recital_detected(text):
    from sweetie_bot_ai_core.agent import _recites_scene
    assert _recites_scene(text, scene_has_position=False)


@pytest.mark.parametrize("text", INNOCENT)
def test_innocent_wording_passes(text):
    from sweetie_bot_ai_core.agent import _recites_scene
    assert not _recites_scene(text, scene_has_position=False)


@pytest.mark.parametrize("text", RECITES)
def test_position_talk_legal_when_scene_carried_it(text):
    # recall of rendered data is not recital — the gate disarms the position tier entirely
    from sweetie_bot_ai_core.agent import _recites_scene
    assert not _recites_scene(text, scene_has_position=True)


@pytest.mark.parametrize("text", ["You mean my interlocutor?", "The person (id 2) is smiling."])
def test_scene_jargon_banned_even_with_positions_rendered(text):
    from sweetie_bot_ai_core.agent import _recites_scene
    assert _recites_scene(text, scene_has_position=True)


# --- deterministic strip ------------------------------------------------------------------------

def test_strip_keeps_the_good_sentence():
    from sweetie_bot_ai_core.agent import _strip_recital
    assert _strip_recital(LIVE_14B_REPLY, scene_has_position=False) == "I'm looking at you!"


def test_strip_of_pure_recital_leaves_nothing():
    from sweetie_bot_ai_core.agent import _strip_recital
    assert _strip_recital("You're right in front of me.", scene_has_position=False) == ""


def test_strip_touches_nothing_when_scene_carried_position():
    from sweetie_bot_ai_core.agent import _strip_recital
    kept = _strip_recital("The one in front of me, of course!", scene_has_position=True)
    assert kept == "The one in front of me, of course!"


# --- reply path, lone person (scene block carries no position) ----------------------------------

def test_lone_person_recital_is_stripped_not_regenerated():
    a = _agent([_person()], _c(LIVE_14B_REPLY))
    reply = a.handle(AgentRequest(text=ASK, profile="complex-en"))
    assert reply.response_text == "I'm looking at you!"
    assert len(a.registry.calls) == 1          # strip is free: no second LLM call


def test_pure_recital_regenerates_once_with_note():
    from sweetie_bot_ai_core.agent import _NO_POSITION_NOTE
    a = _agent([_person()], _c("You're right in front of me."), _c("I'm looking at you, silly!"))
    reply = a.handle(AgentRequest(text=ASK, profile="complex-en"))
    assert reply.response_text == "I'm looking at you, silly!"
    assert len(a.registry.calls) == 2
    assert any(m.get("content") == _NO_POSITION_NOTE for m in a.registry.calls[-1])


def test_reciting_retry_lands_on_fallback():
    from sweetie_bot_ai_core.agent import _DEGENERATE_FALLBACK
    a = _agent([_person()], _c("You're right in front of me."),
               _c("Still, you are right in front of me!"))
    reply = a.handle(AgentRequest(text=ASK, profile="complex-en"))
    assert reply.response_text == _DEGENERATE_FALLBACK


def test_note_parroting_retry_lands_on_fallback():
    from sweetie_bot_ai_core.agent import _DEGENERATE_FALLBACK, _NO_POSITION_NOTE
    a = _agent([_person()], _c("You're right in front of me."), _c(_NO_POSITION_NOTE))
    reply = a.handle(AgentRequest(text=ASK, profile="complex-en"))
    assert reply.response_text == _DEGENERATE_FALLBACK


def test_clean_reply_is_untouched():
    a = _agent([_person()], _c("I'm looking at you, my friend!"))
    reply = a.handle(AgentRequest(text=ASK, profile="complex-en"))
    assert reply.response_text == "I'm looking at you, my friend!"
    assert len(a.registry.calls) == 1


# --- reply path, crowd (scene block legitimately carries positions) -----------------------------

def test_crowd_position_answer_passes_untouched():
    a = _agent([_person(1, bearing=10.0), _person(2, bearing=-30.0)],
               _c("The one in front of me, of course!"))
    reply = a.handle(AgentRequest(text=ASK, profile="complex-en"))
    assert reply.response_text == "The one in front of me, of course!"
    assert len(a.registry.calls) == 1


def test_crowd_id_leak_still_guarded():
    a = _agent([_person(1, bearing=10.0), _person(2, bearing=-30.0)],
               _c("The person (id 2), to my left."), _c("My friend over there!"))
    reply = a.handle(AgentRequest(text=ASK, profile="complex-en"))
    assert reply.response_text == "My friend over there!"
    assert len(a.registry.calls) == 2


# --- wording contracts ---------------------------------------------------------------------------

def test_echo_guard_covers_the_position_note():
    from sweetie_bot_ai_core.agent import _NO_POSITION_NOTE
    assert _echoes_note(_NO_POSITION_NOTE)


def test_note_does_not_trip_the_guard_itself():
    # the regenerate-leg note must never match the very regexes that trigger it, or the
    # verify step could re-fire on an echo-free retry that merely resembles the note
    from sweetie_bot_ai_core.agent import (_NO_POSITION_NOTE, _POSITION_RECITAL_RX,
                                           _SCENE_JARGON_RX)
    assert not _POSITION_RECITAL_RX.search(_NO_POSITION_NOTE)
    assert not _SCENE_JARGON_RX.search(_NO_POSITION_NOTE)


def test_scene_gate_matches_renderer_vocabulary():
    # the renderer's exact position phrasings (scene.py _pos_phrase) must all disarm the gate;
    # if this drifts the guard starts blocking legitimate crowd answers
    from sweetie_bot_ai_core.agent import _POSITION_RECITAL_RX
    for rendered in ("directly in front of you",
                     "in front of you, to your right",
                     "in front of you, to your left"):
        assert _POSITION_RECITAL_RX.search(rendered)
