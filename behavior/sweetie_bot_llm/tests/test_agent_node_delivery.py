"""Delivery contract of agent_node._execute: a REPLY goal is never aborted nor dropped.

lang.py (SOAR side) maps ANY non-SUCCEEDED terminal state to "error", which kills the answer
and can wedge the SOAR say pipeline (standing rule: never return empty/error on the reply
path). So a superseded REPLY runs to completion and is DELIVERED late — a stale client handle
ignores it — while droppable goals (rephrase/self_talk) keep the preempt drop and the
TurnAborted call-boundary abort (their caller text_action.py handles PREEMPTED with a
verbatim-line fallback: slow, not silent).

Needs rospy + the devel-workspace messages importable (run via the ROS-sourced llm suite
recipe, sbllm-venv); skips cleanly elsewhere.
"""
import os
import sys
import threading
from types import SimpleNamespace

import pytest

rospy = pytest.importorskip("rospy")
pytest.importorskip("actionlib")
pytest.importorskip("sweetie_bot_text_msgs.msg")

# make both the ai_core lib and this package importable from source (test_agent_bridge pattern)
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, os.path.abspath(os.path.join(
    HERE, "..", "..", "..", "lib", "sweetie_bot_ai_core", "src")))

from sweetie_bot_llm.agent_node import LLMAgentNode  # noqa: E402
from sweetie_bot_ai_core.agent import TurnAborted  # noqa: E402
from sweetie_bot_ai_core.schema import AgentReply  # noqa: E402

THE_ANSWER = "The answer, delivered."


class FakeServer:
    """SimpleActionServer stand-in: preempt flag + terminal-state recording."""

    def __init__(self, preempt=False):
        self.preempt = preempt
        self.result = None
        self.preempted = False

    def is_preempt_requested(self):
        return self.preempt

    def set_preempted(self):
        self.preempted = True

    def set_succeeded(self, result):
        self.result = result


class FakeAgent:
    """Canned-reply agent that can simulate a newer goal landing mid-generation (flips the
    server's preempt flag) and honours abort_check the way the real _chat choke point does."""

    def __init__(self, server, preempt_mid_generation=False):
        self.server = server
        self.reply = AgentReply(response_text=THE_ANSWER)
        self.preempt_mid = preempt_mid_generation
        self.calls = 0
        self.seen_abort_check = "unset"

    def handle(self, request, abort_check=None):
        self.calls += 1
        self.seen_abort_check = abort_check
        if self.preempt_mid:
            self.server.preempt = True
        if abort_check is not None and abort_check():
            raise TurnAborted()
        return self.reply


def make_node(server, agent):
    node = LLMAgentNode.__new__(LLMAgentNode)  # skip __init__ (no ROS master in unit tests)
    node._server = server
    node._agent = agent
    node._agent_lock = threading.Lock()
    node._last_activity = 0.0
    return node


def goal(rtype, text="What is in my hoof?"):
    return SimpleNamespace(request_type=rtype, text=text, profile="complex-en")


@pytest.fixture(autouse=True)
def quiet_rospy(monkeypatch):
    monkeypatch.setattr(rospy, "get_time", lambda: 1000.0)
    monkeypatch.setattr(rospy, "get_param",
                        lambda name, default=None: 0.0 if "reply_delay" in name else default)
    monkeypatch.setattr(rospy, "sleep", lambda *_a, **_k: None)
    monkeypatch.setattr(rospy, "is_shutdown", lambda: False)
    monkeypatch.setattr(rospy, "loginfo", lambda *_a, **_k: None)
    monkeypatch.setattr(rospy, "logwarn", lambda *_a, **_k: None)


def test_superseded_reply_still_delivers_result():
    # preempt already requested when the goal starts (a canned-reaction rephrase landed):
    # the reply must still be generated and delivered SUCCEEDED
    server = FakeServer(preempt=True)
    agent = FakeAgent(server)
    make_node(server, agent)._execute(goal("reply"))
    assert agent.calls == 1
    assert server.preempted is False
    assert server.result is not None and server.result.response_text == THE_ANSWER


def test_reply_gets_no_abort_check_and_survives_mid_generation_preempt():
    server = FakeServer(preempt=False)
    agent = FakeAgent(server, preempt_mid_generation=True)
    make_node(server, agent)._execute(goal("reply"))
    assert agent.seen_abort_check is None   # a reply is never aborted at a call boundary
    assert server.preempted is False
    assert server.result is not None and server.result.response_text == THE_ANSWER


def test_rephrase_superseded_before_start_drops_without_llm_calls():
    server = FakeServer(preempt=True)
    agent = FakeAgent(server)
    make_node(server, agent)._execute(goal("rephrase"))
    assert agent.calls == 0                 # doomed goal spends no GPU
    assert server.preempted is True
    assert server.result is None


def test_rephrase_aborts_at_call_boundary_mid_generation():
    server = FakeServer(preempt=False)
    agent = FakeAgent(server, preempt_mid_generation=True)
    make_node(server, agent)._execute(goal("rephrase"))
    assert agent.calls == 1
    assert callable(agent.seen_abort_check)  # droppable goals keep the boundary abort
    assert server.preempted is True
    assert server.result is None


def test_unpreempted_reply_flows_normally():
    server = FakeServer(preempt=False)
    agent = FakeAgent(server)
    make_node(server, agent)._execute(goal("reply"))
    assert server.preempted is False
    assert server.result is not None and server.result.response_text == THE_ANSWER
