"""Profile-path coverage through ``Agent.handle`` (simple / failsafe / unknown-fallback).

Pins the CURRENT semantics before the profiles-to-YAML refactor:
* simple-en / failsafe-en never offer tools and pass their configured sampling options;
* an UNKNOWN profile silently falls back to the complex profile (per-handler fallback) —
  this fallback behavior is pinned as-is (diagnostics become log-only later, C3).
"""
from sweetie_bot_ai_core import Agent, AgentRequest, Emotion
from sweetie_bot_ai_core.client import ChatResult


REPLY_JSON = '{"response_text":"Hi there!","emotion":"joy","sentence_type":"statement"}'


class RecordingRegistry:
    """Replays scripted content and records every chat call (messages + kwargs)."""

    def __init__(self, contents):
        self.contents = list(contents)
        self.calls = []

    def chat(self, messages, *, tools=None, response_schema=None, **kw):
        self.calls.append({"messages": [dict(m) for m in messages], "tools": bool(tools),
                           "schema": response_schema is not None, "opts": dict(kw)})
        content = self.contents.pop(0) if len(self.contents) > 1 else self.contents[0]
        return ChatResult(content=content), "scripted"


def test_simple_profile_no_tools_and_its_options():
    reg = RecordingRegistry([REPLY_JSON])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(text="Hello Sweetie! How are you today?",
                                      profile="simple-en"))
    assert reply.response_text == "Hi there!"
    assert reply.emotion == Emotion.joy
    assert len(reg.calls) == 1 and not reg.calls[0]["tools"] and reg.calls[0]["schema"]
    # simple-en config: temperature 0.7, max_tokens 160 (DEFAULT_PROFILES)
    assert reg.calls[0]["opts"]["temperature"] == 0.7
    assert reg.calls[0]["opts"]["max_tokens"] == 160


def test_failsafe_profile_no_tools_and_its_options():
    reg = RecordingRegistry([REPLY_JSON])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(text="Hello Sweetie!", profile="failsafe-en"))
    assert reply.response_text == "Hi there!"
    assert len(reg.calls) == 1 and not reg.calls[0]["tools"]
    assert reg.calls[0]["opts"]["temperature"] == 0.6
    assert reg.calls[0]["opts"]["max_tokens"] == 120


def test_unknown_profile_falls_back_to_complex():
    """An unknown profile name must degrade to the complex profile, not crash (pinned as-is).
    With the default ToolRegistry the complex profile offers tools -> first call has tools."""
    reg = RecordingRegistry(["(no tool calls)", REPLY_JSON])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(text="Hello Sweetie! How are you today?",
                                      profile="totally-unknown-profile"))
    assert reply.response_text == "Hi there!"
    assert reply.error_code == 0
    # complex fallback offered tools on the first call, then did the structured call
    assert reg.calls[0]["tools"] is True
    assert reg.calls[-1]["schema"] is True
    assert reg.calls[-1]["opts"]["temperature"] == 0.8


def test_unknown_profile_rephrase_falls_back_to_rephrase_profile():
    from sweetie_bot_ai_core.schema import RequestType
    reg = RecordingRegistry([REPLY_JSON])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(request_type=RequestType.rephrase,
                                      profile="unknown-rephrase-profile",
                                      text="Yes. Please touch this spot."))
    assert reply.response_text == "Hi there!"
    # rephrase-en fallback options (temperature 1.05, max_tokens 96)
    assert reg.calls[0]["opts"]["temperature"] == 1.05
    assert reg.calls[0]["opts"]["max_tokens"] == 96


def test_unknown_profile_self_talk_falls_back_to_self_talk_profile():
    from sweetie_bot_ai_core.schema import RequestType
    reg = RecordingRegistry([REPLY_JSON])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(request_type=RequestType.self_talk,
                                      profile="unknown-self-talk-profile",
                                      text="You notice another pony toy nearby."))
    assert reply.response_text == "Hi there!"
    assert reg.calls[0]["opts"]["temperature"] == 0.8
    assert reg.calls[0]["opts"]["max_tokens"] == 80
