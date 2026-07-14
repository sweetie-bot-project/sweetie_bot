"""TurnAborted: a superseded turn stops at the next LLM call boundary (the single _chat
choke point) instead of finishing its remaining calls.

Calibrated against the 2026-07-13 live diagnosis (proto3 llm_agent + sbllm GIN logs): on the
single-flight GPU a superseded rephrase goal ran all its calls to completion, head-of-line
blocking the goal that replaced it past the caller's ~3s literal-text fallback ('Do not stop
please.' 23:31:03 -> voiced verbatim 23:31:07).
"""
import pytest

from sweetie_bot_ai_core import Agent, AgentRequest
from sweetie_bot_ai_core.agent import TurnAborted
from sweetie_bot_ai_core.client import ChatResult
from sweetie_bot_ai_core.schema import RequestType


def _reply_json(text):
    import json
    return json.dumps({"response_text": text, "emotion": "joy", "sentence_type": "statement"})


class CountingRegistry:
    def __init__(self, contents):
        self.contents = list(contents)
        self.calls = 0

    def chat(self, messages, *, tools=None, response_schema=None, **kw):
        self.calls += 1
        content = self.contents.pop(0) if len(self.contents) > 1 else self.contents[0]
        return ChatResult(content=content), "scripted"


class AbortAfter:
    """abort_check that returns False for the first ``allow`` polls, then True forever
    (= a newer goal arrived while call #``allow`` was in flight)."""

    def __init__(self, allow):
        self.allow = allow

    def __call__(self):
        if self.allow <= 0:
            return True
        self.allow -= 1
        return False


REPEAT_LINE = "You're as kind as a cloud on a sunny day!"
FRESH_LINE = "Oh, I spotted a little pony over there!"


def test_abort_before_first_call_raises_without_any_llm_call():
    reg = CountingRegistry([_reply_json(FRESH_LINE)])
    agent = Agent(reg)
    with pytest.raises(TurnAborted):
        agent.handle(AgentRequest(text="Hello there!", profile="simple-en"),
                     abort_check=lambda: True)
    assert reg.calls == 0


def test_abort_between_calls_stops_the_anti_repeat_retry():
    # first call returns a repeat -> the anti-repeat retry would be call #2; the abort lands
    # between them and must propagate OUT of _regenerate_if_repeat (whose generic never-raises
    # catch must not swallow a superseded turn into "keep the first reply")
    reg = CountingRegistry([_reply_json(REPEAT_LINE), _reply_json(FRESH_LINE)])
    agent = Agent(reg)
    agent._recent_replies.append(REPEAT_LINE)
    with pytest.raises(TurnAborted):
        agent.handle(AgentRequest(text="Tell me something nice!", profile="simple-en"),
                     abort_check=AbortAfter(1))
    assert reg.calls == 1


def test_self_talk_aborts_when_real_turn_arrives():
    reg = CountingRegistry([_reply_json(FRESH_LINE)])
    agent = Agent(reg)
    with pytest.raises(TurnAborted):
        agent.handle(AgentRequest(request_type=RequestType.self_talk,
                                  text="a warm thought crosses your mind",
                                  profile="simple-en"),
                     abort_check=lambda: True)
    assert reg.calls == 0


def test_abort_state_clears_after_the_turn():
    # the same agent must serve the NEXT goal normally after an aborted one (handle()'s
    # finally resets the per-turn check; single-flight callers reuse one Agent instance)
    reg = CountingRegistry([_reply_json(FRESH_LINE)])
    agent = Agent(reg)
    with pytest.raises(TurnAborted):
        agent.handle(AgentRequest(text="Hello!", profile="simple-en"),
                     abort_check=lambda: True)
    reply = agent.handle(AgentRequest(text="Hello again!", profile="simple-en"))
    assert reply.response_text == FRESH_LINE
    assert reply.error_code.value == 0
    assert reg.calls == 1


def test_no_abort_check_is_the_default_and_changes_nothing():
    reg = CountingRegistry([_reply_json(FRESH_LINE)])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(text="Hi!", profile="simple-en"))
    assert reply.response_text == FRESH_LINE
    assert reg.calls == 1
