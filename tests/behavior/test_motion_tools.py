"""Motion tool calls — "dance for me" / "give me your hoof" must MOVE the robot (HANDOFF #3).

The LLM agent gets play_animation as an execute-mode tool: on a motion request it runs the
saved trajectory through flexbe ExecuteJointTrajectory SYNCHRONOUSLY inside the reply turn
(SOAR is idle-waiting on the GenerateReply result then, so the tool cannot race SOAR-proposed
behaviors; no .soar change). SOAR's own emotion-driven animations also play dance_stamp /
greeting2, so these tests anchor on the AGENT-side causal chain (tool-call log -> execution
marker), never on a bare flexbe goal appearing.
"""
from sweetie_bot_behavior_synth import behavior_test, person, scene


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_dance_request_plays_dance_stamp(world):
    turn = world.say_and_get_turn("Sweetie, dance for me, please!")
    # the model chose the tool...
    assert world.col["agent_log"].wait_grep(r"tool call play_animation", timeout=5.0), \
        "she never called play_animation on a direct dance request"
    # ...and the animation actually ran to completion through flexbe (not just got proposed)
    assert world.col["agent_log"].wait_grep(
        r"play_animation.*dance_stamp.*completed", timeout=15.0), \
        "dance_stamp never executed/completed via the agent path"
    # she still answers like a conversation partner, not a silent actuator
    assert turn.text.strip(), "no verbal reply accompanied the dance"


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_hoof_request_plays_greeting2(world):
    turn = world.say_and_get_turn("Sweetie, give me your hoof!")
    assert world.col["agent_log"].wait_grep(r"tool call play_animation", timeout=5.0), \
        "she never called play_animation on a hoof-shake request"
    assert world.col["agent_log"].wait_grep(
        r"play_animation.*greeting2.*completed", timeout=15.0), \
        "greeting2 never executed/completed via the agent path"
    assert turn.text.strip(), "no verbal reply accompanied the hoof-shake"


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_plain_question_does_not_trigger_animation(world):
    """Tool precision: an ordinary question must not fire body animations (a 7B happily
    over-calls tools when the description invites it — this pins the boundary)."""
    world.say_and_get_turn("What is your name?")
    hits = world.col["agent_log"].grep(r"tool call play_animation")
    assert not hits, f"play_animation fired on a plain question: {hits[:2]}"


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_time_ask_still_calls_get_robot_state(world):
    """Info-tool regression through the SAME tool loop: the phase-split moved ALL tool
    decisions onto the lean prompt (motion-tools work) — get_robot_state must keep firing
    there (it was live-verified on the old single-prompt path, M.3)."""
    t = world.say_and_get_turn("Sweetie, what time is it now?")
    assert world.col["agent_log"].wait_grep(r"tool call get_robot_state", timeout=5.0), \
        "get_robot_state no longer fires through the lean tool phase"
    assert t.text.strip(), "no verbal reply accompanied the state lookup"
