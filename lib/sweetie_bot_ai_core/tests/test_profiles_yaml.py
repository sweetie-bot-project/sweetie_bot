"""profiles.py: language-neutral canonical names, alias resolution, YAML loading, and the
repo-yaml == code-fallback introduction pin."""
import os

import pytest
import yaml

from sweetie_bot_ai_core import Agent, DEFAULT_PROFILES, ProfileConfig, load_profiles
from sweetie_bot_ai_core.client import ChatResult

PROFILES_YAML = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..",
    "behavior", "sweetie_bot_llm", "config", "profiles.yaml"))


class RecordingRegistry:
    def __init__(self):
        self.calls = []

    def chat(self, messages, *, tools=None, response_schema=None, **kw):
        self.calls.append({"opts": dict(kw), "tools": bool(tools)})
        return ChatResult(
            content='{"response_text":"Hi!","emotion":"joy","sentence_type":"statement"}'), "r"


# --- canonical names are language-neutral ----------------------------------------------------------

def test_default_profiles_have_neutral_names():
    assert set(DEFAULT_PROFILES) == {"complex", "simple", "failsafe", "rephrase", "self-talk"}


# --- alias resolution: exact -> strip -<lang> -> fallback -------------------------------------------

def test_profile_alias_strips_language_suffix():
    agent = Agent(RecordingRegistry())
    # SOAR request names keep working against neutral canonical names
    assert agent._profile("complex-en", "complex") is DEFAULT_PROFILES["complex"]
    assert agent._profile("rephrase-en", "rephrase") is DEFAULT_PROFILES["rephrase"]
    assert agent._profile("self-talk-en", "self-talk") is DEFAULT_PROFILES["self-talk"]
    assert agent._profile("simple-ru", "complex") is DEFAULT_PROFILES["simple"]


def test_profile_exact_match_wins_over_strip():
    profs = dict(DEFAULT_PROFILES)
    profs["special-en"] = ProfileConfig(allow_tools=False, options={"temperature": 0.42})
    profs["special"] = ProfileConfig(allow_tools=True, options={"temperature": 0.99})
    agent = Agent(RecordingRegistry(), profiles=profs)
    assert agent._profile("special-en", "complex").options["temperature"] == 0.42


def test_profile_unknown_falls_back_with_log_only():
    logs = []
    agent = Agent(RecordingRegistry(), logger=logs.append)
    p = agent._profile("no-such-profile", "complex")
    assert p is DEFAULT_PROFILES["complex"]
    assert any("unknown profile" in l for l in logs)


def test_all_three_handlers_resolve_soar_request_names():
    """The three per-handler lookups all go through the alias helper (was: three raw dict
    lookups whose fallbacks silently absorbed every SOAR request name)."""
    from sweetie_bot_ai_core import AgentRequest
    from sweetie_bot_ai_core.schema import RequestType
    reg = RecordingRegistry()
    agent = Agent(reg)
    agent.handle(AgentRequest(text="hello there friend", profile="simple-en"))
    assert reg.calls[-1]["opts"]["max_tokens"] == 160          # simple, via alias
    agent.handle(AgentRequest(request_type=RequestType.rephrase, profile="rephrase-en",
                              text="Yes. Please touch this spot."))
    assert reg.calls[-1]["opts"]["max_tokens"] == 96           # rephrase, via alias
    agent.handle(AgentRequest(request_type=RequestType.self_talk, profile="self-talk-en",
                              text="You notice another pony toy nearby."))
    assert reg.calls[-1]["opts"]["max_tokens"] == 80           # self-talk, via alias


# --- load_profiles ----------------------------------------------------------------------------------

def test_load_profiles_roundtrip_and_defaults():
    cfg = {"profiles": {
        "complex": {"allow_tools": True, "max_verbatim_turns": 8, "max_tool_iters": 3,
                    "options": {"temperature": 0.8, "max_tokens": 512}},
        "tiny": {"allow_tools": False}}}
    out = load_profiles(cfg)
    assert out["complex"] == DEFAULT_PROFILES["complex"]
    assert out["tiny"].allow_tools is False
    assert out["tiny"].max_tool_iters == 3                     # dataclass default fills gaps


def test_load_profiles_empty_returns_defaults_copy():
    out = load_profiles(None)
    assert out == DEFAULT_PROFILES and out is not DEFAULT_PROFILES
    assert load_profiles({}) == DEFAULT_PROFILES


# --- introduction pin: repo yaml == code fallback ----------------------------------------------------

@pytest.mark.skipif(not os.path.exists(PROFILES_YAML), reason="profiles.yaml not present")
def test_repo_profiles_yaml_matches_code_fallback():
    """At introduction time the deployed YAML and the shrunk code fallback are identical; a
    later deliberate divergence should update this test with the reason."""
    with open(PROFILES_YAML, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    assert load_profiles(cfg) == DEFAULT_PROFILES
