"""ROS EffectorPort: execute LLM tool calls that are safe to run from the agent.

Arbitration policy (see tools.yaml): SOAR is the sole arbiter of shared actuators
(eyes/animations/gaze) and already drives expression from the returned ``emotion``. So this
adapter only *executes* side-effect-free info/perception tools (e.g. get_robot_state, get_scene).
Actuator tools are 'disabled'/'propose' in config and never reach an execute path here; if one ever
does, we refuse it rather than race SOAR.
"""
from __future__ import annotations

import rospy

from sweetie_bot_ai_core.schema import ToolCall, ToolResult


class ToolAdapters:
    def __init__(self, state_collector, scene_collector=None):
        self._state = state_collector
        self._scene = scene_collector

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
            # Actuator tools must not be executed from here (SOAR arbitrates them).
            rospy.logwarn("ToolAdapters: refusing to execute actuator tool '%s' (SOAR-arbitrated)",
                          name)
            return ToolResult(name=name, content="(refused: actuator is SOAR-arbitrated)",
                              ok=False, id=tool_call.id)
        except Exception as e:  # noqa: BLE001 - never crash the agent on a tool error
            rospy.logerr("ToolAdapters: error dispatching '%s': %r", name, e)
            return ToolResult(name=name, content=f"(error: {e})", ok=False, id=tool_call.id)
