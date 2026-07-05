"""LLM rephrase interceptor for canned speech (approved design; replaces sibling-rule dilution).

Mechanism tests drive the REAL text-action output module through a real SML kernel (the
test_soar_adapter pattern) against the live agent + voice pipeline — no SOAR rules involved, so
the spec holds regardless of which reactions the sim's rules can trigger:
  * p=1.0 -> a canned line is voiced REPHRASED (never verbatim), in character;
  * recent-reply guard: text identical to a fresh generate_reply result passes VERBATIM;
  * p=0.0 -> canned passes verbatim (instant path preserved).
"""
import os
import sys
import time

import pytest
import rospy

pytestmark = pytest.mark.skipif(
    not os.path.isdir("/opt/soar"), reason="needs the SML kernel (/opt/soar)")

sys.path.insert(0, "/opt/soar/lib")
sys.path.insert(0, "/opt/soar")

CANNED_LINE = "Yes. Please touch this spot."


def _mk_module(probability: float):
    import Python_sml_ClientInterface as sml
    from sweetie_bot_soar.output_modules.text_action import TextAction

    kernel = sml.Kernel.CreateKernelInNewThread()
    agent = kernel.CreateAgent("bsynth_rephrase")
    mod = TextAction("text-action", {
        "type": "text-action",
        "action_ns": "voice/syn",
        "llm_rephrase": {"probability": probability, "timeout": 12.0,
                         "action_ns": "generate_reply"},
    })
    return kernel, agent, mod


def _wait_connected(mod, timeout=8.0):
    import rospy as _r
    assert mod._reph_client is None or mod._reph_client.wait_for_server(_r.Duration(timeout)), \
        "generate_reply server not reachable"


def _say_via_module(mod, agent, text: str, timeout: float = 25.0):
    il = agent.GetInputLink()          # any identifier tree works for building the cmd WME
    cmd = il.CreateIdWME("cmd")
    cmd.CreateStringWME("type", "voice/say/en")
    cmd.CreateStringWME("command", text)
    agent.Commit()
    r = mod.startHook(cmd)
    assert r is None, f"startHook refused: {r}"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        r = mod.updateHook(cmd, None)
        if r is not None:
            return r
        rospy.sleep(0.2)
    return "timeout"


@pytest.fixture()
def voice_says(sim):
    from sweetie_bot_behavior_synth.collectors import VoiceScraper
    v = VoiceScraper()
    v.anchor()
    return v


def test_canned_rephrased_at_p1(sim, voice_says):
    kernel, agent, mod = _mk_module(probability=1.0)
    _wait_connected(mod)
    try:
        assert _say_via_module(mod, agent, CANNED_LINE) == "succeed"
        said = voice_says.wait_say(timeout=10.0)
        assert said is not None, "nothing reached the voice"
        assert said.text.strip() != CANNED_LINE, "canned line voiced VERBATIM despite p=1.0"
        assert len(said.text.split()) >= 2, f"degenerate rephrase: {said.text!r}"
        # strictly in-context: the rephrase must still be ABOUT the original line ("touch this
        # spot"), not drift onto unrelated things. The constrained rephrase path forbids adding
        # new topics, so an on-topic content anchor must survive the rewording.
        anchors = ("touch", "spot", "here", "pet", "scratch", "place", "this")
        low = said.text.lower()
        assert any(a in low for a in anchors), \
            f"rephrase drifted off the original line (no on-topic anchor): {said.text!r}"
    finally:
        kernel.Shutdown()


def test_recent_llm_reply_passes_verbatim(sim, voice_says):
    import actionlib
    from sweetie_bot_text_msgs.msg import GenerateReplyAction, GenerateReplyGoal
    c = actionlib.SimpleActionClient("generate_reply", GenerateReplyAction)
    assert c.wait_for_server(rospy.Duration(10.0))
    # module FIRST: its recent-reply subscriber must see the upcoming result
    kernel, agent, mod = _mk_module(probability=1.0)
    rospy.sleep(0.5)
    c.send_goal(GenerateReplyGoal(request_type="reply", profile="complex-en",
                                  text="Say one short kind sentence.", history_json="[]",
                                  context_json="[]", text_language="en", reply_language="en"))
    assert c.wait_for_result(rospy.Duration(60.0))
    reply = c.get_result().response_text
    rospy.sleep(0.5)
    try:
        assert _say_via_module(mod, agent, reply) == "succeed"
        said = voice_says.wait_say(timeout=10.0)
        assert said is not None
        assert said.text == reply, "an LLM reply was re-rephrased (guard failed)"
    finally:
        kernel.Shutdown()


def test_canned_verbatim_at_p0(sim, voice_says):
    kernel, agent, mod = _mk_module(probability=0.0)
    try:
        assert _say_via_module(mod, agent, CANNED_LINE) == "succeed"
        said = voice_says.wait_say(timeout=10.0)
        assert said is not None
        assert said.text.strip() == CANNED_LINE, f"expected verbatim canned, got {said.text!r}"
    finally:
        kernel.Shutdown()
