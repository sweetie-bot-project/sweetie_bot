"""Unit tests for the unseen-hand guard on the reply path (ROS-free).

Live incident (HANDOFF, 2026-07-08 23:06): "What's in my hand?" with NO 'holding' attribute
in the scene — the 7B confabulated "a small book". Root cause is absence-of-signal: 'holding'
is present-or-absent by design (vision attaches it only when confident), absence renders
nothing about hands, and the model fills the void. Fix pinned here: a question-gated system
note delivers the missing datum as CONTEXT (the lull/re-poke note doctrine) — she answers
from what she actually sees instead of inventing.
"""
import json

import pytest

from sweetie_bot_ai_core import AgentRequest, SceneConfig, SceneState, classify_zone
from sweetie_bot_ai_core.agent import (_HAND_UNSEEN_NOTE, _NO_REPEAT_NOTE, Agent,
                                       _asks_whats_in_hand, _echoes_note)
from sweetie_bot_ai_core.client import ChatResult
from sweetie_bot_ai_core.persona import PersonaRegistry
from sweetie_bot_ai_core.schema import SceneEntity

CFG = SceneConfig(front_deg=60, side_deg=120, max_entities=3)


def _c(text, emo="joy"):
    return json.dumps({"response_text": text, "emotion": emo, "sentence_type": "statement"})


def _person(id=1, bearing=0.0, **kw):
    return SceneEntity(id=id, type="human", bearing_deg=bearing,
                       zone=classify_zone(bearing, CFG), **kw)


def _thing(id=9, bearing=10.0, type="pony", **kw):
    return SceneEntity(id=id, type=type, bearing_deg=bearing,
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


def _has_note(messages):
    return any(m.get("content") == _HAND_UNSEEN_NOTE for m in messages)


# --- trigger regex ----------------------------------------------------------------------------

FIRES = [
    "What's in my hand?",
    "What is in my hands?",
    "What am I holding?",
    "what am i holding",
    "Do you see what I'm holding?",
    "What do I have in my hand?",          # LibreTranslate pivot shape for the RU ask
    "Guess what's in my hand!",
    "Can you tell what's in my hand?",
    "Что у меня в руке?",
    "что у меня в руках?",
    "Что я держу?",
    "Угадай, что я держу!",
    "Видишь, что у меня в руке?",
    "Что в руке у меня?",
]

NOT_FIRES = [
    "Shake my hand!",
    "Can you shake my hand?",
    "Hold my hand",
    "Give me your hoof!",
    "What happened to my hand?",
    "What's in my handbag?",
    "I am holding a ball",                 # statement, not a question — no lead word
    "Пожми мне руку",
    "Возьми меня за руку",
    "Дай копыто",
]


@pytest.mark.parametrize("text", FIRES)
def test_hand_question_fires(text):
    assert _asks_whats_in_hand(text)


@pytest.mark.parametrize("text", NOT_FIRES)
def test_non_hand_text_does_not_fire(text):
    assert not _asks_whats_in_hand(text)


def test_any_of_several_texts_fires():
    # the reply path probes both request.text (raw, maybe RU) and user_text (EN pivot)
    assert _asks_whats_in_hand("Что у меня в руке?", "What do I have in my hand?")
    assert not _asks_whats_in_hand(None, "")


# --- gating: when the note is injected --------------------------------------------------------

def test_hand_question_without_holding_injects_note():
    a = _agent([_person()], _c("I don't see anything in your hands right now."))
    a.handle(AgentRequest(text="What's in my hand?", profile="complex-en"))
    assert _has_note(a.registry.calls[-1])


def test_russian_hand_question_injects_note():
    a = _agent([_person()], _c("I don't see anything in your hands right now."))
    a.handle(AgentRequest(text="Что у меня в руке?", profile="complex-en"))
    assert _has_note(a.registry.calls[-1])


def test_visible_holding_suppresses_note():
    a = _agent([_person(attributes={"holding": "a ball"})], _c("A ball!"))
    a.handle(AgentRequest(text="What's in my hand?", profile="complex-en"))
    assert not _has_note(a.registry.calls[-1])


def test_visible_held_by_object_suppresses_note():
    a = _agent([_person(), _thing(attributes={"held_by": "1"})], _c("A pony plushie!"))
    a.handle(AgentRequest(text="What's in my hand?", profile="complex-en"))
    assert not _has_note(a.registry.calls[-1])


def test_no_person_in_frame_still_injects():
    # tracker briefly lost the asker: "you see nothing held" is true a fortiori
    a = _agent([], _c("I don't see anything in your hands right now."))
    a.handle(AgentRequest(text="What's in my hand?", profile="complex-en"))
    assert _has_note(a.registry.calls[-1])


def test_remembered_holder_does_not_suppress_note():
    # only IN-FRAME percepts count: a remembered (out-of-frame) holding is not current vision
    a = _agent([_person(),
                _person(id=2, bearing=20.0, in_frame=False, last_seen_s=30.0,
                        attributes={"holding": "a cup"})],
               _c("I don't see anything in your hands right now."))
    a.handle(AgentRequest(text="What's in my hand?", profile="complex-en"))
    assert _has_note(a.registry.calls[-1])


def test_unrelated_question_gets_no_note():
    a = _agent([_person()], _c("I love tag!"))
    a.handle(AgentRequest(text="What games do you like?", profile="complex-en"))
    assert not _has_note(a.registry.calls[-1])


def test_reply_still_flows_normally_with_note():
    a = _agent([_person()], _c("I don't see anything in your hands right now."))
    reply = a.handle(AgentRequest(text="What's in my hand?", profile="complex-en"))
    assert reply.response_text == "I don't see anything in your hands right now."
    assert reply.error_code.value == 0


# --- echo guard: the note itself must never be voiced ------------------------------------------

def test_echo_guard_covers_the_hand_note():
    assert _echoes_note(_HAND_UNSEEN_NOTE)


def test_echo_guard_still_covers_the_no_repeat_note():
    assert _echoes_note(_NO_REPEAT_NOTE)


def test_honest_reply_is_not_a_note_echo():
    # the natural desired answer shares no 5-word shingle with the note (wording contract:
    # keep it that way when editing _HAND_UNSEEN_NOTE, or good retries get discarded)
    assert not _echoes_note("I don't see anything in your hands right now.")
