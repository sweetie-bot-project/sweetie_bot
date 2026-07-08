"""ROS EffectorPort: execute LLM tool calls that are safe to run from the agent.

Arbitration policy (see tools.yaml): SOAR is the sole arbiter of shared actuators and already
drives expression from the returned ``emotion``. Info/perception tools (get_robot_state,
get_scene) are side-effect-free and always safe. Actuator tools are safe here ONLY when run
SYNCHRONOUSLY inside the reply turn: while SOAR awaits the GenerateReply result it proposes no
talk-driven behaviors, so a blocking animation cannot race them (and at 4-6 s per animation the
turn stays well under SOAR's 30 s lang-module timeout). play_animation follows that contract —
it returns only after flexbe reports the trajectory done. Anything without a synchronous story
(continuous gaze) stays refused.
"""
from __future__ import annotations

import rospy

from sweetie_bot_ai_core.schema import ToolCall, ToolResult


class ToolAdapters:
    def __init__(self, state_collector, scene_collector=None, animations=None):
        self._state = state_collector
        self._scene = scene_collector
        # friendly animation name -> saved joint trajectory (tools.yaml play_animation.animations)
        self._animations = dict(animations or {})
        self._flexbe = None

    def dispatch(self, tool_call: ToolCall) -> ToolResult:
        name = tool_call.name
        rospy.loginfo("llm_agent: tool call %s(%s)", name, tool_call.arguments)
        try:
            if name == "get_robot_state":
                state = self._state.snapshot()
                return ToolResult(name=name, content=state.model_dump_json(), id=tool_call.id)
            if name == "get_scene":
                if self._scene is None:
                    return ToolResult(name=name, content="(no scene available)", ok=False,
                                      id=tool_call.id)
                inc = bool(tool_call.arguments.get("include_remembered", False))
                scene = self._scene.snapshot(include_remembered=inc)
                return ToolResult(name=name, content=scene.model_dump_json(), id=tool_call.id)
            if name == "play_animation" and self._animations:
                return self._play_animation(tool_call)
            # Any other actuator tool has no synchronous-execution story yet — refuse.
            rospy.logwarn("ToolAdapters: refusing to execute actuator tool '%s' (SOAR-arbitrated)",
                          name)
            return ToolResult(name=name, content="(refused: actuator is SOAR-arbitrated)",
                              ok=False, id=tool_call.id)
        except Exception as e:  # noqa: BLE001 - never crash the agent on a tool error
            rospy.logerr("ToolAdapters: error dispatching '%s': %r", name, e)
            return ToolResult(name=name, content=f"(error: {e})", ok=False, id=tool_call.id)

    # -- play_animation ----------------------------------------------------------------------

    def resolve_animation(self, args: dict):
        """Friendly name -> trajectory name, or None (strict allowlist: the model only ever
        reaches trajectories the deploy config exposes — structural control, not instruction)."""
        return self._animations.get(str(args.get("name", "")).strip())

    def _play_animation(self, tool_call: ToolCall) -> ToolResult:
        import actionlib
        from actionlib_msgs.msg import GoalStatus
        from flexbe_msgs.msg import BehaviorExecutionAction, BehaviorExecutionGoal

        friendly = str(tool_call.arguments.get("name", "")).strip()
        trajectory = self.resolve_animation(tool_call.arguments)
        if trajectory is None:
            return ToolResult(
                name=tool_call.name, ok=False, id=tool_call.id,
                content=f"(unknown animation {friendly!r}; available: "
                        f"{', '.join(sorted(self._animations))})")

        if self._flexbe is None:
            self._flexbe = actionlib.SimpleActionClient("flexbe/flexbe/execute_behavior",
                                                        BehaviorExecutionAction)
        if not self._flexbe.wait_for_server(rospy.Duration(3.0)):
            rospy.logwarn("ToolAdapters: play_animation %r: flexbe server unavailable", friendly)
            return ToolResult(name=tool_call.name, ok=False, id=tool_call.id,
                              content="(animation failed: motion system unavailable)")

        goal = BehaviorExecutionGoal()
        goal.behavior_name = "ExecuteJointTrajectory"
        goal.arg_keys = ["/joint_trajectory"]
        goal.arg_values = [trajectory]
        self._flexbe.send_goal(goal)
        # synchronous by contract (see module docstring); 15 s bounds a stuck behavior while
        # keeping the whole turn under SOAR's 30 s timeout
        if not self._flexbe.wait_for_result(rospy.Duration(15.0)):
            self._flexbe.cancel_goal()
            rospy.logwarn("llm_agent: play_animation %r -> ExecuteJointTrajectory(%s): timed out",
                          friendly, trajectory)
            return ToolResult(name=tool_call.name, ok=False, id=tool_call.id,
                              content=f"(animation '{friendly}' timed out)")
        state = self._flexbe.get_state()
        outcome = getattr(self._flexbe.get_result(), "outcome", "")
        if state == GoalStatus.SUCCEEDED and outcome not in ("failed", "failure"):
            rospy.loginfo("llm_agent: play_animation %r -> ExecuteJointTrajectory(%s): completed",
                          friendly, trajectory)
            return ToolResult(name=tool_call.name, id=tool_call.id,
                              content=f"(you performed the '{friendly}' animation)")
        rospy.logwarn("llm_agent: play_animation %r -> ExecuteJointTrajectory(%s): state=%s "
                      "outcome=%r", friendly, trajectory, state, outcome)
        return ToolResult(name=tool_call.name, ok=False, id=tool_call.id,
                          content=f"(animation '{friendly}' did not complete)")
