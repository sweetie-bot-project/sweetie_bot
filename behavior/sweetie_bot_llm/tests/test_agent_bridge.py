"""Tests for the ROS-free action<->agent bridge (no rospy needed)."""
import json
import os
import sys
from types import SimpleNamespace

# make both the ai_core lib and this package importable from source
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, os.path.abspath(os.path.join(
    HERE, "..", "..", "..", "lib", "sweetie_bot_ai_core", "src")))

from sweetie_bot_llm.agent_bridge import (fill_result, goal_to_request,  # noqa: E402
                                          parse_history, reply_to_result_dict)
from sweetie_bot_ai_core.schema import (AgentReply, Emotion, RequestType,  # noqa: E402
                                        SentenceType, ToolCall)


def test_parse_history_roundtrip():
    hj = json.dumps([{"speaker": "human", "text": "hi"},
                     {"speaker": "sweetie", "text": "hello!", "emotion": "joy"}])
    turns = parse_history(hj)
    assert len(turns) == 2
    assert turns[0].speaker == "human"
    assert turns[1].emotion == Emotion.joy


def test_parse_history_bad_json_safe():
    assert parse_history("not json") == []
    assert parse_history("") == []


def test_goal_to_request_full():
    goal = SimpleNamespace(
        request_type="reply", profile="complex-en", text="battery?",
        history_json=json.dumps([{"speaker": "human", "text": "earlier"}]),
        text_language="ru", reply_language="ru", persona="sweetie",
        labels_json="", image_b64="")
    req = goal_to_request(goal)
    assert req.request_type == RequestType.reply
    assert req.text == "battery?"
    assert req.text_language == "ru"
    assert req.persona == "sweetie"
    assert len(req.history) == 1


def test_goal_to_request_defaults_and_bad_enum():
    goal = SimpleNamespace(request_type="bogus", text="hi")  # missing most fields
    req = goal_to_request(goal)
    assert req.request_type == RequestType.reply       # bad enum -> reply
    assert req.profile == "complex-en"                 # default
    assert req.persona is None
    assert req.history == []


def test_reply_to_result_dict():
    reply = AgentReply(response_text="Hi!", emotion=Emotion.joy,
                       sentence_type=SentenceType.statement,
                       tool_calls=[ToolCall(name="look_at", arguments={"target": "left"})])
    d = reply_to_result_dict(reply)
    assert d["emotion"] == "joy"
    assert d["sentence_type"] == "statement"
    assert json.loads(d["tool_calls_json"])[0]["name"] == "look_at"
    assert d["error_code"] == 0


def test_fill_result_inplace():
    res = SimpleNamespace(response_text="", emotion="", sentence_type="",
                          tool_calls_json="", error_code=0, error_desc="")
    reply = AgentReply(response_text="Yo", emotion=Emotion.surprise)
    fill_result(res, reply)
    assert res.response_text == "Yo"
    assert res.emotion == "surprise"


def test_goal_to_request_labels_and_image():
    goal = SimpleNamespace(request_type="classify", text="I love sunny days!",
                           labels_json=json.dumps(["positive", "negative", "neutral"]),
                           image_b64="aGVsbG8=")
    req = goal_to_request(goal)
    assert req.request_type == RequestType.classify
    assert req.labels == ["positive", "negative", "neutral"]
    assert req.image_b64 == "aGVsbG8="


def test_goal_to_request_malformed_json_fields_degrade_empty():
    goal = SimpleNamespace(request_type="reply", text="hi",
                           labels_json="{not json", context_json="[unterminated",
                           history_json="also not json")
    req = goal_to_request(goal)
    assert req.labels == []
    assert req.context_facts == []
    assert req.history == []


def test_goal_to_request_context_facts_stringified():
    goal = SimpleNamespace(text="hi", context_json=json.dumps(
        ["the human's favorite color is teal", 42, {"k": "v"}]))
    req = goal_to_request(goal)
    # every entry is coerced to str (SOAR predicates arrive as arbitrary json values)
    assert req.context_facts[0] == "the human's favorite color is teal"
    assert req.context_facts[1] == "42"
    assert len(req.context_facts) == 3


def test_reply_with_real_tool_call_args_roundtrips():
    # real live tool calls (proto3 llm_agent log 2026-07-03): get_scene + get_robot_state
    reply = AgentReply(
        response_text="Let me look around!", emotion=Emotion.joy,
        tool_calls=[ToolCall(name="get_scene", arguments={"include_remembered": True}),
                    ToolCall(name="get_robot_state",
                             arguments={"fields": ["battery_level", "charge_status"]})])
    d = reply_to_result_dict(reply)
    calls = json.loads(d["tool_calls_json"])
    assert calls[0] == {"name": "get_scene", "arguments": {"include_remembered": True}}
    assert calls[1]["arguments"]["fields"] == ["battery_level", "charge_status"]


def test_parse_history_skips_malformed_turns_keeps_good():
    hj = json.dumps([{"speaker": "human", "text": "hi"},
                     {"no_speaker_field": "x"},
                     {"speaker": "sweetie", "text": "hello!"}])
    turns = parse_history(hj)
    assert [t.speaker for t in turns] == ["human", "sweetie"]


def test_servo_fault_filter_debounces_comm_noise():
    """Bus comm errors are normal noise; only persistent error states are faults."""
    import pytest
    pytest.importorskip("rospy")   # state_collector imports rospy; runs under the ROS env
    from sweetie_bot_llm.state_collector import ServoFaultFilter
    t = [0.0]
    f = ServoFaultFilter(min_reports=4, min_span_s=3.0, clock=lambda: t[0])
    # transient blip: two errors then clean -> never faulted
    assert f.observe("leg1", True) is False
    t[0] = 1.0
    assert f.observe("leg1", True) is False
    t[0] = 1.5
    assert f.observe("leg1", False) is False
    # persistent: 4 reports over >=3s -> faulted; clean report clears
    for i, ts in enumerate([2.0, 3.0, 4.0, 5.5]):
        t[0] = ts
        r = f.observe("head2", True)
    assert r is True
    t[0] = 6.0
    assert f.observe("head2", False) is False


def test_state_collector_ignores_disabled_servos():
    """Servos listed in /disabled_servos are invisible to the LLM state view - no fault,
    no overheat - while other servos still report (the ignore must not mask real faults)."""
    import pytest
    pytest.importorskip("rospy")
    from types import SimpleNamespace

    from sweetie_bot_llm.state_collector import ServoFaultFilter, StateCollector

    c = StateCollector(subscribe=False, ignored_servos={"head_joint1"})
    t = [0.0]
    c._fault_filter = ServoFaultFilter(min_reports=4, min_span_s=3.0, clock=lambda: t[0])
    for ts in (0.0, 1.5, 3.0, 4.5):
        t[0] = ts
        for name in ("head_joint1", "leg1_joint2"):
            c._on_servo(SimpleNamespace(name=name, status_error=1, temperature=95.0))
    s = c.snapshot()
    assert "head_joint1" not in s.servo_faults
    assert "head_joint1" not in s.overheated_servos
    assert "leg1_joint2" in s.servo_faults
    assert "leg1_joint2" in s.overheated_servos
