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


def test_tool_phase_runs_on_lean_prompt_and_grafts_results_into_full_prompt():
    """The tool-DECISION call must use the lean tool-phase prompt (identity + tool note only:
    a 7B stops emitting native tool calls when the schema competes with the full persona
    style guidance), while the FINAL constrained call runs on the full reply prompt with the
    assistant tool-call turn + tool result grafted in."""
    from sweetie_bot_ai_core.client import ChatResult
    from sweetie_bot_ai_core.prompt import build_tool_phase_prompt
    from sweetie_bot_ai_core.schema import ToolCall

    class ToolCallingRegistry(RecordingRegistry):
        def chat(self, messages, *, tools=None, response_schema=None, **kw):
            self.calls.append({"messages": [dict(m) for m in messages], "tools": bool(tools),
                               "schema": response_schema is not None, "opts": dict(kw)})
            scripted = self.contents.pop(0)
            if isinstance(scripted, ChatResult):
                return scripted, "scripted"
            return ChatResult(content=scripted), "scripted"

    reg = ToolCallingRegistry([
        ChatResult(content="", tool_calls=[ToolCall(name="get_robot_state", arguments={})]),
        ChatResult(content=""),          # tool loop iter 1: no further calls
        REPLY_JSON,                      # final constrained reply
    ])
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(text="Sweetie, how full is your battery?",
                                      profile="complex-en"))
    assert reply.response_text == "Hi there!"
    tool_call, final_call = reg.calls[0], reg.calls[-1]
    # tool phase: lean system prompt, tools offered, no grammar
    assert tool_call["tools"] and not tool_call["schema"]
    assert tool_call["messages"][0]["content"] == build_tool_phase_prompt(
        agent.personas.active)
    # final phase: FULL prompt (differs from the lean one), grammar on, tools not re-offered
    assert final_call["schema"] and not final_call["tools"]
    full_system = final_call["messages"][0]["content"]
    assert full_system != tool_call["messages"][0]["content"]
    assert "Keep your reply to at most" in full_system
    # the tool exchange is grafted into the final messages (assistant tool-call + tool result)
    roles = [m["role"] for m in final_call["messages"]]
    assert "tool" in roles, f"tool result missing from final messages: {roles}"
    assert any(m["role"] == "assistant" and m.get("tool_calls")
               for m in final_call["messages"]), "assistant tool-call turn missing"
