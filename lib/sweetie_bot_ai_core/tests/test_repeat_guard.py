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
