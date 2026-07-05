"""Unit tests for the anti-repetition (loop) guard on the Agent (Fix 3, ROS-free)."""
from sweetie_bot_ai_core.agent import Agent
from sweetie_bot_ai_core.registry import ProviderRegistry, Endpoint
from sweetie_bot_ai_core.persona import PersonaRegistry


def _bare_agent():
    reg = ProviderRegistry([Endpoint(name="x", client=object(), priority=1)])
    return Agent(reg, personas=PersonaRegistry())


def test_is_repeat_near_duplicate():
    a = _bare_agent()
    a._recent_replies.append("That warm light makes everything look so cozy.")
    assert a._is_repeat("That warm light makes everything look so cozy!")       # punctuation only
    assert a._is_repeat("That warm light makes everything look really cozy.")   # >0.9 ratio
    assert a._is_repeat("THAT WARM LIGHT makes everything look so cozy.")       # case-insensitive


def test_is_repeat_allows_different_and_short():
    a = _bare_agent()
    a._recent_replies.append("That warm light makes everything look so cozy.")
    assert not a._is_repeat("Let's go for a walk in the park this afternoon!")  # clearly different
    assert not a._is_repeat("Yes")            # short affirmations may recur
    assert not a._is_repeat("Okay, sure!")    # short
    assert not a._is_repeat("")               # empty


def test_reset_ambient_clears_the_window():
    a = _bare_agent()
    a._recent_replies.append("some longer remembered sentence here for the window")
    a.reset_ambient()
    assert len(a._recent_replies) == 0


# --- re-poke guard: an already-answered human turn re-emitted on a lull must NOT be re-answered
from sweetie_bot_ai_core import AgentRequest        # noqa: E402
from sweetie_bot_ai_core.client import ChatResult   # noqa: E402


class _CapReg:
    """Duck-typed registry that records every message list passed to chat()."""
    def __init__(self):
        self.calls = []
        self.content = '{"response_text":"Sure thing!","emotion":"joy","sentence_type":"statement"}'

    def chat(self, messages, **kw):
        self.calls.append(messages)
        return ChatResult(content=self.content), "cap"


def _guarded(text):
    return any("ALREADY responded" in m.get("content", "") for m in text)


def test_reanswer_guard_moves_on_when_same_turn_repoked():
    reg = _CapReg()
    a = Agent(reg, personas=PersonaRegistry())
    q = "What's wrong with your feet now?"
    a.handle(AgentRequest(text=q, profile="complex-en"))
    assert not _guarded(reg.calls[-1])          # first time: a normal answer
    a.handle(AgentRequest(text=q, profile="complex-en"))
    assert _guarded(reg.calls[-1])              # re-poke: steered to move on


def test_reanswer_guard_allows_a_genuinely_new_turn():
    reg = _CapReg()
    a = Agent(reg, personas=PersonaRegistry())
    a.handle(AgentRequest(text="Tell me a little about yourself please.", profile="complex-en"))
    a.handle(AgentRequest(text="Can you make up a short cheerful story?", profile="complex-en"))
    assert not _guarded(reg.calls[-1])


def test_reanswer_guard_reset_clears_memory():
    reg = _CapReg()
    a = Agent(reg, personas=PersonaRegistry())
    q = "What's wrong with your feet now?"
    a.handle(AgentRequest(text=q, profile="complex-en"))
    a.reset_ambient()
    a.handle(AgentRequest(text=q, profile="complex-en"))
    assert not _guarded(reg.calls[-1])          # memory cleared -> answered fresh
