"""Pure (ROS-free) bridge between the GenerateReply action and sweetie_bot_ai_core.

Kept import-clean of rospy so it is unit-testable without a ROS master. ``agent_node`` (rospy)
imports these helpers; they also work with any duck-typed goal/result object (real ROS message
or a test stub) because fields are read/written by attribute name.
"""
from __future__ import annotations

import json
from typing import Any, List

from sweetie_bot_ai_core.schema import (AgentReply, AgentRequest, ErrorCode, RequestType, TalkTurn,
                                        ToolCall)


def _get(obj: Any, name: str, default: str = "") -> str:
    val = getattr(obj, name, default)
    return val if val is not None else default


def parse_history(history_json: str) -> List[TalkTurn]:
    if not history_json:
        return []
    try:
        data = json.loads(history_json)
    except (json.JSONDecodeError, TypeError):
        return []
    turns: List[TalkTurn] = []
    for item in data if isinstance(data, list) else []:
        try:
            turns.append(TalkTurn(**item))
        except Exception:  # noqa: BLE001 - skip malformed turns
            continue
    return turns


def goal_to_request(goal: Any) -> AgentRequest:
    """Build an AgentRequest from a GenerateReply goal (or a stub with the same attributes)."""
    rtype = _get(goal, "request_type", "reply") or "reply"
    try:
        request_type = RequestType(rtype)
    except ValueError:
        request_type = RequestType.reply
    labels: List[str] = []
    labels_json = _get(goal, "labels_json")
    if labels_json:
        try:
            labels = list(json.loads(labels_json))
        except (json.JSONDecodeError, TypeError):
            labels = []
    context_facts: List[str] = []
    context_json = _get(goal, "context_json")
    if context_json:
        try:
            context_facts = [str(x) for x in json.loads(context_json)]
        except (json.JSONDecodeError, TypeError):
            context_facts = []
    return AgentRequest(
        request_type=request_type,
        profile=_get(goal, "profile", "complex-en") or "complex-en",
        text=_get(goal, "text"),
        history=parse_history(_get(goal, "history_json")),
        context_facts=context_facts,
        text_language=_get(goal, "text_language", "en") or "en",
        reply_language=_get(goal, "reply_language", "en") or "en",
        persona=_get(goal, "persona") or None,
        labels=labels,
        image_b64=_get(goal, "image_b64") or None,
    )


def reply_to_result_dict(reply: AgentReply) -> dict:
    """Flatten an AgentReply into the GenerateReply result fields."""
    return {
        "response_text": reply.response_text,
        "emotion": reply.emotion.value,
        "sentence_type": reply.sentence_type.value,
        "tool_calls_json": json.dumps(
            [{"name": tc.name, "arguments": tc.arguments} for tc in reply.tool_calls]),
        "error_code": int(reply.error_code),
        "error_desc": reply.error_desc,
    }


def fill_result(result: Any, reply: AgentReply) -> Any:
    """Write reply fields onto a GenerateReply result message in-place; returns it."""
    for k, v in reply_to_result_dict(reply).items():
        setattr(result, k, v)
    return result


# Convenience for the (rewritten) SOAR output module side: serialize talk events to history_json.
def events_to_history_json(events: List[dict]) -> str:
    """events: list of {speaker, text, emotion?} dicts (already verbolized by the caller)."""
    return json.dumps(events)
