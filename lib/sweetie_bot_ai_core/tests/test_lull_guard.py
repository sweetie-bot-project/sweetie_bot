"""Unit tests for the empty-text lull-goal guard on the reply path (ROS-free).

Live defect (HANDOFF M.3, 2026-07-08 00:08): SOAR pokes ``generate_reply`` with ``text=''``
on a conversational lull; ``build_messages`` then appends NO user turn, the model continues
after its own last line with an undefined task and occasionally decodes label junk
(``response_text='joy'``, ``'you'``) — which got voiced («радость»). Two layers pinned here:
the lull goal gets a defined task (a system note), and decode junk is regenerated once and
never returned (the reply path must never return empty — lang.py lesson).
"""
import json

from sweetie_bot_ai_core import AgentRequest
from sweetie_bot_ai_core.agent import (_DEGENERATE_FALLBACK, _LULL_NOTE, _RETRY_SENTENCE_NOTE,
                                       Agent)
from sweetie_bot_ai_core.client import ChatResult
from sweetie_bot_ai_core.persona import PersonaRegistry


def _c(text, emo="joy"):
    return json.dumps({"response_text": text, "emotion": emo, "sentence_type": "statement"})


class _SeqReg:
    """Duck-typed registry returning queued contents; records every message list."""
    def __init__(self, *contents):
        self.contents = list(contents)
        self.calls = []

    def chat(self, messages, **kw):
        self.calls.append(messages)
        content = self.contents.pop(0) if len(self.contents) > 1 else self.contents[0]
        return ChatResult(content=content), "seq"


def _agent(*contents):
    a = Agent(_SeqReg(*contents), personas=PersonaRegistry())
    # pure reply path: the default ToolRegistry offers tools, and the tool loop would consume
    # queued contents before the final structured reply — call counts must stay deterministic
    a.tools._tools.clear()
    return a


def _has_lull_note(messages):
    return any(m.get("content") == _LULL_NOTE for m in messages)


def _retried(reg):
    return any(any(m.get("content") == _RETRY_SENTENCE_NOTE for m in msgs)
               for msgs in reg.calls)


def test_lull_goal_gets_a_defined_task_note():
    a = _agent(_c("Shall we make up a little story together?"))
    a.handle(AgentRequest(text="", profile="complex-en"))
    assert _has_lull_note(a.registry.calls[-1])


def test_real_turn_gets_no_lull_note():
    a = _agent(_c("I love tag!"))
    a.handle(AgentRequest(text="What games do you like?", profile="complex-en"))
    assert not _has_lull_note(a.registry.calls[-1])


def test_label_junk_on_lull_is_regenerated():
    a = _agent(_c("joy"), _c("The evening light is so pretty today!"))
    reply = a.handle(AgentRequest(text="", profile="complex-en"))
    assert reply.response_text == "The evening light is so pretty today!"
    assert len(a.registry.calls) == 2 and _retried(a.registry)


def test_single_word_junk_on_lull_is_regenerated():
    a = _agent(_c("you"), _c("Do you want to hear a little rhyme?"))
    reply = a.handle(AgentRequest(text="", profile="complex-en"))
    assert reply.response_text == "Do you want to hear a little rhyme?"
    assert len(a.registry.calls) == 2 and _retried(a.registry)


def test_persistent_junk_falls_back_to_a_real_sentence():
    a = _agent(_c("joy"))                       # every call returns the same junk
    reply = a.handle(AgentRequest(text="", profile="complex-en"))
    assert reply.response_text == _DEGENERATE_FALLBACK
    assert len(reply.response_text.split()) >= 2
    assert reply.error_code.value == 0


def test_single_word_answer_to_a_real_turn_is_legitimate():
    a = _agent(_c("Yes!"))
    reply = a.handle(AgentRequest(text="Do you like tea?", profile="complex-en"))
    assert reply.response_text == "Yes!"
    assert not _retried(a.registry)             # no retry burned on a valid short answer


def test_echo_of_own_history_line_is_a_repeat_even_after_restart():
    # after an agent restart/reset the in-memory _recent_replies window is empty, but the
    # SOAR-delivered history still carries what she actually said — echoing her own last
    # line on a lull is a repeat, not a fresh remark (seen 4/5 in the sim probe)
    from sweetie_bot_ai_core.schema import TalkTurn
    echo = "I love tag! Chasing games are my favourite."
    a = _agent(_c(echo), _c("Shall we make up a new game together?"))
    reply = a.handle(AgentRequest(
        text="", profile="complex-en",
        history=[TalkTurn(speaker="human", text="What games do you like to play?"),
                 TalkTurn(speaker="sweetie", text=echo, emotion="joy")]))
    assert reply.response_text == "Shall we make up a new game together?"


def test_insistent_echo_never_escapes():
    # live 02:12 run: the no-repeat retry RE-ECHOED her own line 3/5 times (a small model
    # latches onto the copy source in the prompt; instruction alone loses). The retry is
    # verified too — a still-repeating retry falls back rather than voicing a loop.
    from sweetie_bot_ai_core.schema import TalkTurn
    echo = "I love tag! Chasing games are my favourite."
    a = _agent(_c(echo))                        # every call, retry included, echoes
    reply = a.handle(AgentRequest(
        text="", profile="complex-en",
        history=[TalkTurn(speaker="human", text="What games do you like to play?"),
                 TalkTurn(speaker="sweetie", text=echo, emotion="joy")]))
    assert reply.response_text == _DEGENERATE_FALLBACK
    assert reply.error_code.value == 0


def test_empty_reply_on_a_real_turn_never_escapes():
    a = _agent(_c(""))                          # model returns empty text on every call
    reply = a.handle(AgentRequest(text="Tell me something nice.", profile="complex-en"))
    assert reply.response_text.strip()
    assert reply.error_code.value == 0
