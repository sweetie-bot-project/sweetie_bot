"""Config-driven construction against the REAL shipped YAMLs (tools.yaml / providers.yaml)
+ ConversationHistory custom-summarizer seam.

These pin the config->object wiring the ROS node relies on at startup, without any network:
build_llm_registry only CONSTRUCTS clients, it never connects.
"""
import os

import pytest
import yaml

from sweetie_bot_ai_core import ConversationHistory, ToolRegistry, build_llm_registry
from sweetie_bot_ai_core.schema import TalkTurn
from sweetie_bot_ai_core.tools import DispatchMode

LLM_CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "..",
                              "behavior", "sweetie_bot_llm", "config")


def _load(name):
    path = os.path.join(LLM_CONFIG_DIR, name)
    if not os.path.exists(path):
        pytest.skip(f"{name} not present (deployment layout differs)")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# --- ToolRegistry.from_config with the real tools.yaml -------------------------------------------

def test_tool_registry_from_real_tools_yaml():
    cfg = _load("tools.yaml")
    reg = ToolRegistry.from_config(cfg.get("tools"))
    # arbitration contract: info tools execute, actuator tools stay disabled
    assert reg.mode("get_robot_state") == DispatchMode.execute
    assert reg.mode("look_at") == DispatchMode.disabled
    assert reg.mode("play_animation") == DispatchMode.disabled
    # get_scene has no yaml override -> keeps its code default (execute, read-only perception)
    assert reg.mode("get_scene") == DispatchMode.execute
    offered = {t.name for t in reg.offered()}
    assert offered == {"get_robot_state", "get_scene"}


def test_tool_registry_unknown_names_in_config_are_ignored():
    reg = ToolRegistry.from_config({"no_such_tool": {"dispatch_mode": "execute"}})
    assert reg.get("no_such_tool") is None


# --- build_llm_registry with the real providers.yaml ----------------------------------------------

def test_build_llm_registry_from_real_providers_yaml():
    cfg = _load("providers.yaml")
    reg = build_llm_registry(cfg["llm"], default_options=cfg.get("default_options"))
    names = [ep.name for ep in reg.endpoints]
    # priority order: local container first, cloud fallback second
    assert names == ["local", "remote"]
    local = reg.endpoints[0]
    assert local.client.base_url == "http://localhost:11434/v1"
    assert local.client.model.startswith("qwen2.5")
    assert reg.fail_threshold == cfg["llm"]["fail_threshold"]
    assert reg.cooldown_s == cfg["llm"]["cooldown_s"]
    # default sampling options flow into every client
    assert local.client.default_options.get("temperature") == 0.8


def test_build_llm_registry_empty_config_raises():
    from sweetie_bot_ai_core import RegistryError
    with pytest.raises(RegistryError):
        build_llm_registry({"providers": {}})


def test_build_llm_registry_skips_providers_without_url():
    reg = build_llm_registry({"providers": {
        "broken": {"model": "x"},
        "ok": {"url": "http://localhost:1234/v1", "model": "m", "priority": 5}}})
    assert [ep.name for ep in reg.endpoints] == ["ok"]


# --- ConversationHistory custom summarize ---------------------------------------------------------

def test_history_custom_summarizer_is_used():
    seen = {}

    def summarize(turns):
        seen["n"] = len(turns)
        return "CUSTOM-SUMMARY"

    turns = [TalkTurn(speaker="human" if i % 2 == 0 else "sweetie", text=f"t{i}")
             for i in range(10)]
    hist = ConversationHistory(max_verbatim_turns=4, summarize=summarize)
    msgs = hist.build_messages("SYS", turns, "now")
    assert seen["n"] == 6                                 # the older, out-of-window turns
    assert any(m["role"] == "system" and "CUSTOM-SUMMARY" in m["content"] for m in msgs)


def test_history_empty_summary_adds_no_message():
    hist = ConversationHistory(max_verbatim_turns=2, summarize=lambda turns: "")
    turns = [TalkTurn(speaker="human", text=f"t{i}") for i in range(5)]
    msgs = hist.build_messages("SYS", turns, "now")
    assert not any("Earlier in this conversation" in m["content"] for m in msgs)
