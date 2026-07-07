"""Empty-text lull goals must never produce voiceable junk (regression, live-calibrated).

Live defect (2026-07-08 00:08, llm_agent log): SOAR pokes ``generate_reply`` with ``text=''``
on a conversational lull (no talk-heard event); the agent occasionally decoded label junk —
``reply [joy] 'joy'`` — which was voiced as «радость». Guarded agent-side (ai_core
``_LULL_NOTE`` + degenerate-reply regenerate; unit-pinned in
``lib/sweetie_bot_ai_core/tests/test_lull_guard.py``). This test replays the exact live goal
shape against the real agent + LLM several times: every reply must be a real utterance.
"""
import json
import re

import pytest
import rospy

SAMPLES = 5   # replies are stochastic; the guard must hold on every one

HISTORY = [
    {"speaker": "human", "text": "What games do you like to play?"},
    {"speaker": "sweetie", "text": "I love tag! Chasing games are my favourite.",
     "emotion": "joy"},
]
CONTEXT = ["The person you are talking with has gone quiet and is not answering right now."]


def test_lull_goal_always_yields_a_real_utterance(sim):
    import actionlib
    from sweetie_bot_text_msgs.msg import GenerateReplyAction, GenerateReplyGoal
    c = actionlib.SimpleActionClient("generate_reply", GenerateReplyAction)
    assert c.wait_for_server(rospy.Duration(10.0))
    echoes = 0
    for i in range(SAMPLES):
        c.send_goal(GenerateReplyGoal(request_type="reply", profile="complex-en", text="",
                                      history_json=json.dumps(HISTORY),
                                      context_json=json.dumps(CONTEXT),
                                      text_language="en", reply_language="en"))
        assert c.wait_for_result(rospy.Duration(90.0)), f"sample {i}: no result"
        r = c.get_result()
        assert r.error_code == 0, f"sample {i}: error {r.error_code} {r.error_desc!r}"
        words = re.findall(r"\w+", r.response_text or "")
        assert len(words) >= 2, f"sample {i}: degenerate lull reply {r.response_text!r}"
        echoes += r.response_text.strip() == HISTORY[-1]["text"]
    # echoing her OWN last history line is a repeat, not a fresh remark (the repeat guard
    # sees the history's sweetie turns even right after an agent restart); one stochastic
    # near-miss is tolerated, a majority echo is the pre-guard failure mode (was 4/5)
    assert echoes <= 1, f"{echoes}/{SAMPLES} lull replies verbatim-echo her own last line"
