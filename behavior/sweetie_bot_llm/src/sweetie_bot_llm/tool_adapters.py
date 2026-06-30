"""ROS EffectorPort: execute LLM tool calls that are safe to run from the agent.

Arbitration policy (see tools.yaml): SOAR is the sole arbiter of shared actuators
(eyes/animations/gaze) and already drives expression from the returned ``emotion``. So this
adapter only *executes* side-effect-free info tools (e.g. get_robot_state). Actuator tools are
'disabled'/'propose' in config and never reach an execute path here; if one ever does, we refuse
it rather than race SOAR.
"""
from __future__ import annotations

import rospy

from sweetie_bot_ai_core.schema import ToolCall, ToolResult


class ToolAdapters:
    def __init__(self, state_collector):
        self._state = state_collector

    def dispatch(self, tool_call: ToolCall) -> ToolResult:
        name = tool_call.name
        try:
            if name == "get_robot_state":
                state = self._state.snapshot()
                return ToolResult(name=name, content=state.model_dump_json(), id=tool_call.id)
            # Actuator tools must not be executed from here (SOAR arbitrates them).
            rospy.logwarn("ToolAdapters: refusing to execute actuator tool '%s' (SOAR-arbitrated)",
                          name)
            return ToolResult(name=name, content="(refused: actuator is SOAR-arbitrated)",
                              ok=False, id=tool_call.id)
        except Exception as e:  # noqa: BLE001 - never crash the agent on a tool error
            rospy.logerr("ToolAdapters: error dispatching '%s': %r", name, e)
            return ToolResult(name=name, content=f"(error: {e})", ok=False, id=tool_call.id)
