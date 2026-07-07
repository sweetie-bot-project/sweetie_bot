"""Agent orchestration paths not covered by test_core: classify, in-handler anti-repeat
regenerate, context_facts injection, and the re-poke (already-answered) note path.

Calibrated against real live traffic (proto3 llm_agent logs 2026-07-03): real question texts
and the exact re-answer-loop scenario ("What's wrong with your feet now?" re-asked on a lull).
"""
from sweetie_bot_ai_core import Agent, AgentRequest
from sweetie_bot_ai_core.client import ChatResult
from sweetie_bot_ai_core.schema import RequestType


def _reply_json(text):
    import json
    return json.dumps({"response_text": text, "emotion": "joy", "sentence_type": "statement"})


class RecordingRegistry:
    def __init__(self, contents):
        self.contents = list(contents)
        self.calls = []

    def chat(self, messages, *, tools=None, response_schema=None, **kw):
        self.calls.append({"messages": [dict(m) for m in messages], "tools": bool(tools),
                           "schema": response_schema is not None, "opts": dict(kw),
                           "schema_obj": response_schema})
        content = self.contents.pop(0) if len(self.contents) > 1 else self.contents[0]
        return ChatResult(content=content), "scripted"


# --- classify path -----------------------------------------------------------------------------

def test_classify_returns_label_at_temperature_zero():
    reg = RecordingRegistry(['{"label":"positive"}'])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(request_type=RequestType.classify,
                                      text="I love sunny days!",
                                      labels=["positive", "negative", "neutral"]))
    assert reply.response_text == "positive"
    assert reg.calls[0]["opts"]["temperature"] == 0.0
    # the constrained schema restricts the label to the allowed set
    assert reg.calls[0]["schema_obj"]["properties"]["label"]["enum"] == \
        ["positive", "negative", "neutral"]


def test_classify_bad_decode_degrades_to_last_label():
    reg = RecordingRegistry(["not json at all"])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(request_type=RequestType.classify,
                                      text="whatever", labels=["a", "b", "fallback"]))
    assert reply.response_text == "fallback"


# --- anti-repeat regenerate INSIDE _handle_reply -------------------------------------------------

REPEAT_LINE = "You're as kind as a cloud on a sunny day!"   # real live reply 2026-07-03
FRESH_LINE = "Oh, I spotted a little pony over there!"


def test_reply_regenerates_once_on_repeat_with_bumped_temperature():
    reg = RecordingRegistry([_reply_json(REPEAT_LINE), _reply_json(FRESH_LINE)])
    agent = Agent(reg)
    agent._recent_replies.append(REPEAT_LINE)
    reply = agent.handle(AgentRequest(text="Tell me something nice!", profile="simple-en"))
    # exactly one retry happened and it won
    assert len(reg.calls) == 2
    assert reply.response_text == FRESH_LINE
    # the retry carries the no-repeat steer note and temperature +0.3 over the profile base
    retry = reg.calls[1]
    assert any("Do NOT repeat yourself" in m["content"]
               for m in retry["messages"] if m["role"] == "system")
    assert retry["opts"]["temperature"] == 0.7 + 0.3     # simple-en base 0.7


def test_reply_regenerate_temperature_caps_at_1_2():
    from sweetie_bot_ai_core.agent import ProfileConfig
    hot = {"hot": ProfileConfig(allow_tools=False,
                                options={"temperature": 1.0, "max_tokens": 64})}
    reg = RecordingRegistry([_reply_json(REPEAT_LINE), _reply_json(FRESH_LINE)])
    agent = Agent(reg, profiles=hot)
    agent._recent_replies.append(REPEAT_LINE)
    agent.handle(AgentRequest(text="Tell me something nice!", profile="hot"))
    assert reg.calls[1]["opts"]["temperature"] == 1.2    # min(1.2, 1.0 + 0.3)


def test_reply_keeps_first_answer_when_retry_fails():
    class FlakyRegistry(RecordingRegistry):
        def chat(self, messages, **kw):
            if len(self.calls) >= 1:
                self.calls.append({})
                raise RuntimeError("retry boom")
            return super().chat(messages, **kw)

    reg = FlakyRegistry([_reply_json(REPEAT_LINE)])
    agent = Agent(reg)
    agent._recent_replies.append(REPEAT_LINE)
    reply = agent.handle(AgentRequest(text="Tell me something nice!", profile="simple-en"))
    # never empty/error on the reply path — the first (repeated) reply is still returned
    assert reply.response_text == REPEAT_LINE
    assert reply.error_code == 0


def test_short_affirmations_never_count_as_repeats():
    reg = RecordingRegistry([_reply_json("Yes!")])
    agent = Agent(reg)
    agent._recent_replies.append("Yes!")
    reply = agent.handle(AgentRequest(text="Do you like ponies?", profile="simple-en"))
    assert len(reg.calls) == 1                       # no regenerate for a short line
    assert reply.response_text == "Yes!"


# --- context_facts injection ---------------------------------------------------------------------

def test_context_facts_reach_the_system_prompt():
    reg = RecordingRegistry([_reply_json(FRESH_LINE)])
    agent = Agent(reg)
    agent.handle(AgentRequest(text="What did I tell you before?", profile="simple-en",
                              context_facts=["the human's favorite color is teal",
                                             "a pony plushie sits on the shelf"]))
    sys_msg = next(m["content"] for m in reg.calls[0]["messages"] if m["role"] == "system")
    assert "Relevant things you remember right now:" in sys_msg
    assert "- the human's favorite color is teal" in sys_msg
    assert "- a pony plushie sits on the shelf" in sys_msg


# --- re-poke (already-answered) guard --------------------------------------------------------------

REPOKE_TEXT = "What's wrong with your feet now?"     # the live re-answer-loop question


def test_repoke_of_answered_turn_gets_move_on_note():
    reg = RecordingRegistry([_reply_json("My legs feel a bit stiff today."),
                             _reply_json("Anyway — lovely weather, right?")])
    logs = []
    agent = Agent(reg, logger=logs.append)
    agent.handle(AgentRequest(text=REPOKE_TEXT, profile="simple-en"))
    # SOAR re-emits the same speech event on a lull -> the guard must steer, not re-answer
    agent.handle(AgentRequest(text=REPOKE_TEXT, profile="simple-en"))
    first, second = reg.calls[0], reg.calls[1]
    note = "ALREADY responded"
    assert not any(note in m["content"] for m in first["messages"] if m["role"] == "system")
    assert any(note in m["content"] for m in second["messages"] if m["role"] == "system")
    assert any("re-poke" in l for l in logs)


def test_short_turns_are_allowed_to_recur_without_note():
    reg = RecordingRegistry([_reply_json("Okay!")])
    agent = Agent(reg)
    agent.handle(AgentRequest(text="ok", profile="simple-en"))
    agent.handle(AgentRequest(text="ok", profile="simple-en"))
    note = "ALREADY responded"
    assert not any(note in m["content"]
                   for c in reg.calls for m in c["messages"] if m["role"] == "system")
