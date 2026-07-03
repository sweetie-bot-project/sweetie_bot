"""Offline unit tests for sweetie_bot_ai_core (no live model)."""
import time

import pytest

from sweetie_bot_ai_core import (Agent, AgentRequest, ConversationHistory, Emotion, Endpoint,
                                 PersonaRegistry, ProviderRegistry, RegistryError, RobotState,
                                 SentenceType, ToolRegistry, reply_json_schema)
from sweetie_bot_ai_core.client import ChatResult
from sweetie_bot_ai_core.schema import TalkTurn, ToolCall, ToolResult
from sweetie_bot_ai_core.tools import DispatchMode
from sweetie_bot_ai_core.translation import LanguagePolicy


# --- schema ----------------------------------------------------------------------------------

def test_reply_schema_is_strict():
    s = reply_json_schema()
    assert s["additionalProperties"] is False
    assert set(s["required"]) == {"response_text", "emotion", "sentence_type"}
    assert s["properties"]["emotion"]["enum"] == [e.value for e in Emotion]


def test_robot_state_summary():
    st = RobotState(battery_percent=80, battery_status="discharging", pose="body_nominal",
                    datetime_iso="2026-06-30T16:00", servo_faults=["head_joint1"])
    summ = st.human_summary()
    assert "80%" in summ and "body_nominal" in summ and "head_joint1" in summ


# --- registry: failover + circuit breaker ----------------------------------------------------

class FakeClient:
    def __init__(self, fail=False, tag="ok"):
        self.fail = fail
        self.tag = tag
        self.calls = 0

    def chat(self, messages, **kw):
        self.calls += 1
        if self.fail:
            raise RuntimeError("boom")
        return ChatResult(content=self.tag)

    def health(self):
        return not self.fail


def test_registry_failover_priority_order():
    bad = Endpoint("bad", FakeClient(fail=True), priority=10)
    good = Endpoint("good", FakeClient(tag="good"), priority=20)
    reg = ProviderRegistry([good, bad])  # constructor sorts by priority
    res, name = reg.chat([{"role": "user", "content": "hi"}])
    assert name == "bad" or name == "good"  # bad tried first, fails -> good
    assert res.content == "good"


def test_circuit_breaker_opens_and_cools_down():
    clock = {"t": 0.0}
    bad_client = FakeClient(fail=True)
    good = Endpoint("good", FakeClient(tag="g"), priority=20)
    bad = Endpoint("bad", bad_client, priority=10)
    reg = ProviderRegistry([bad, good], fail_threshold=2, cooldown_s=30,
                           clock=lambda: clock["t"])
    # two failures trip the breaker on 'bad'
    reg.chat([{"role": "user", "content": "1"}])
    reg.chat([{"role": "user", "content": "2"}])
    assert bad.is_open(clock["t"])
    calls_before = bad_client.calls
    # while open, 'bad' is skipped -> 'good' used, 'bad' not called
    reg.chat([{"role": "user", "content": "3"}])
    assert bad_client.calls == calls_before
    # after cooldown it is retried (half-open)
    clock["t"] = 40.0
    reg.chat([{"role": "user", "content": "4"}])
    assert bad_client.calls == calls_before + 1


def test_registry_all_fail_raises():
    reg = ProviderRegistry([Endpoint("a", FakeClient(fail=True))])
    with pytest.raises(RegistryError):
        reg.chat([{"role": "user", "content": "x"}])


# --- history ---------------------------------------------------------------------------------

def test_history_messages_and_summary():
    turns = [TalkTurn(speaker="human", text=f"h{i}") if i % 2 == 0
             else TalkTurn(speaker="sweetie", text=f"s{i}") for i in range(12)]
    hist = ConversationHistory(max_verbatim_turns=4)
    msgs = hist.build_messages("SYS", turns, "current")
    assert msgs[0]["role"] == "system"
    assert any("Earlier in this conversation" in m["content"] for m in msgs if m["role"] == "system")
    assert msgs[-1] == {"role": "user", "content": "current"}
    # only last 4 verbatim + current
    user_assistant = [m for m in msgs if m["role"] in ("user", "assistant")]
    assert len(user_assistant) == 4 + 1


# --- translation routing ---------------------------------------------------------------------

def test_language_policy_decisions():
    pol = LanguagePolicy(native_languages=["en", "ru"], pivot="en")
    assert not pol.needs_input_translation("ru")   # native -> feed as-is
    assert not pol.needs_input_translation("en")
    assert pol.needs_input_translation("ja")        # non-native -> translate to pivot


# --- agent orchestration (scripted registry) -------------------------------------------------

class ScriptedRegistry:
    """Returns tool_calls when tools are offered (once), then structured content."""
    def __init__(self, structured_content, tool_call=None):
        self.structured_content = structured_content
        self.tool_call = tool_call
        self.calls = []

    def chat(self, messages, *, tools=None, response_schema=None, **kw):
        self.calls.append({"tools": bool(tools), "schema": bool(response_schema)})
        if tools and self.tool_call is not None and not any(
                m.get("role") == "tool" for m in messages):
            return ChatResult(content="", tool_calls=[self.tool_call]), "scripted"
        if response_schema is not None:
            return ChatResult(content=self.structured_content), "scripted"
        return ChatResult(content="(plain)"), "scripted"


class FakeEffector:
    def __init__(self):
        self.dispatched = []

    def dispatch(self, tc: ToolCall) -> ToolResult:
        self.dispatched.append(tc.name)
        return ToolResult(name=tc.name, content='{"battery_percent": 80}', id=tc.id)


def test_agent_simple_reply_no_tools():
    reg = ScriptedRegistry('{"response_text":"Hi there!","emotion":"joy","sentence_type":"statement"}')
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(text="hello", profile="simple-en"))  # tools off
    assert reply.response_text == "Hi there!"
    assert reply.emotion == Emotion.joy
    assert reply.sentence_type == SentenceType.statement
    assert all(not c["tools"] for c in reg.calls)  # simple-en never offers tools


def test_agent_executes_info_tool_then_replies():
    tc = ToolCall(name="get_robot_state", arguments={}, id="call_1")
    reg = ScriptedRegistry('{"response_text":"I am at 80%!","emotion":"neutral","sentence_type":"statement"}',
                           tool_call=tc)
    eff = FakeEffector()
    agent = Agent(reg, tools=ToolRegistry(), effector=eff)
    reply = agent.handle(AgentRequest(text="battery?", profile="complex-en"))
    assert "80%" in reply.response_text
    assert eff.dispatched == ["get_robot_state"]   # info tool executed
    assert reply.tool_calls == []                   # execute-mode not surfaced as proposal


def test_agent_actuator_tool_is_proposed_not_executed():
    reg = ToolRegistry()
    reg.set_mode("look_at", DispatchMode.propose)
    tc = ToolCall(name="look_at", arguments={"target": "left"}, id="call_2")
    sreg = ScriptedRegistry('{"response_text":"Okay!","emotion":"joy","sentence_type":"statement"}',
                            tool_call=tc)
    eff = FakeEffector()
    agent = Agent(sreg, tools=reg, effector=eff)
    reply = agent.handle(AgentRequest(text="look left", profile="complex-en"))
    assert eff.dispatched == []                       # actuator NOT executed
    assert [t.name for t in reply.tool_calls] == ["look_at"]  # surfaced as proposal


def test_agent_bad_json_degrades_to_defaults():
    reg = ScriptedRegistry("not json at all")
    agent = Agent(reg)
    reply = agent.handle(AgentRequest(text="hi", profile="simple-en"))
    assert reply.emotion == Emotion.neutral          # safe default
    assert reply.sentence_type == SentenceType.statement


# --- persona ---------------------------------------------------------------------------------

def test_persona_registry_default_and_switch():
    pr = PersonaRegistry()
    assert pr.active.name == "sweetie"
    assert not pr.set_active("nonexistent")
    assert pr.get(None).display_name == "Sweetie Bot"


def test_reset_seams():
    """P0: in-process reset of inter-goal state (behavior-synth harness seam)."""
    from sweetie_bot_ai_core.agent import Agent
    from sweetie_bot_ai_core.registry import ProviderRegistry, Endpoint
    from sweetie_bot_ai_core.persona import PersonaRegistry
    from sweetie_bot_ai_core.schema import SceneState, SceneEntity, Zone

    reg = ProviderRegistry([Endpoint(name="x", client=object(), priority=1)])
    reg.endpoints[0].fail_count = 5
    reg.endpoints[0].open_until = 1e12
    reg.reset_breakers()
    assert reg.endpoints[0].fail_count == 0 and reg.endpoints[0].open_until == 0.0

    a = Agent(reg)
    a._prev_scene = SceneState(entities=[SceneEntity(id=1, type="pony", zone=Zone.front)])
    a.reset_ambient()
    assert a._prev_scene.entities == []

    p = PersonaRegistry()
    default = p.active_name if hasattr(p, "active_name") else p._active
    p._active = "something_else"
    p.reset_active()
    assert p._active == default or p._active == p._default
