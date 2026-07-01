"""Tool registry + dispatch policy.

Tools are *declared* here (ROS-free); execution happens through an injected ``EffectorPort`` in
the agent. Each tool carries a ``dispatch_mode``:

* ``execute``  — the agent may run it now (only safe for tools that do NOT touch SOAR-arbitrated
                 actuators: info/query tools, future web/smart-home).
* ``propose``  — returned to the caller as a proposal, not executed (for when, in the future,
                 SOAR rules opt in to consuming tool proposals).
* ``disabled`` — never offered to the model.

Rationale: SOAR is the sole arbiter of shared actuators (eyes/animations/gaze) and already drives
expression from the ``emotion`` field. The LLM must not race it, so actuator tools default to
``disabled``/``propose`` until rules opt in.
"""
from __future__ import annotations

from enum import Enum
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field


class DispatchMode(str, Enum):
    execute = "execute"
    propose = "propose"
    disabled = "disabled"


class ToolSpec(BaseModel):
    name: str
    description: str
    parameters: Dict[str, Any] = Field(
        default_factory=lambda: {"type": "object", "properties": {}})
    dispatch_mode: DispatchMode = DispatchMode.disabled

    def to_openai(self) -> Dict[str, Any]:
        return {"type": "function",
                "function": {"name": self.name, "description": self.description,
                             "parameters": self.parameters}}


class ToolRegistry:
    def __init__(self, tools: Optional[List[ToolSpec]] = None):
        self._tools: Dict[str, ToolSpec] = {}
        for t in tools or DEFAULT_TOOLS:
            self._tools[t.name] = t

    def get(self, name: str) -> Optional[ToolSpec]:
        return self._tools.get(name)

    def offered(self) -> List[ToolSpec]:
        """Tools to offer the model: anything not disabled."""
        return [t for t in self._tools.values() if t.dispatch_mode != DispatchMode.disabled]

    def to_openai_tools(self) -> List[Dict[str, Any]]:
        return [t.to_openai() for t in self.offered()]

    def mode(self, name: str) -> DispatchMode:
        t = self._tools.get(name)
        return t.dispatch_mode if t else DispatchMode.disabled

    def set_mode(self, name: str, mode: DispatchMode) -> None:
        if name in self._tools:
            self._tools[name] = self._tools[name].model_copy(update={"dispatch_mode": mode})

    @classmethod
    def from_config(cls, config: Optional[dict]) -> "ToolRegistry":
        """Build from a config mapping tool-name -> {dispatch_mode: ...} overriding defaults."""
        reg = cls()
        for name, override in (config or {}).items():
            if name in reg._tools and "dispatch_mode" in override:
                reg.set_mode(name, DispatchMode(override["dispatch_mode"]))
        return reg


# Built-in tools. Actuator tools default to 'disabled' (arbitration-safe). The single info tool
# is 'execute' because it only reads state and does not touch any actuator.
DEFAULT_TOOLS: List[ToolSpec] = [
    ToolSpec(
        name="get_robot_state",
        description=("Get Sweetie's current live state: date and time, battery level and charge "
                     "status, body pose, and any servo faults. Call this when the human asks "
                     "about how you feel physically, your battery, the time/date, or your status."),
        parameters={"type": "object", "properties": {
            "fields": {"type": "array", "items": {"type": "string"},
                       "description": "Optional subset of state fields to fetch."}},
            "required": []},
        dispatch_mode=DispatchMode.execute,
    ),
    ToolSpec(
        name="get_scene",
        description=("Look at what's around you in more detail: the people and objects you can see, "
                     "where they are, and what they're doing. Set include_remembered=true to also "
                     "recall things that recently went out of your view and where they were, so you "
                     "can decide to turn toward them."),
        parameters={"type": "object", "properties": {
            "include_remembered": {"type": "boolean",
                                   "description": "Also return recently-seen things now out of view."}},
            "required": []},
        dispatch_mode=DispatchMode.execute,   # read-only perception
    ),
    ToolSpec(
        name="look_at",
        description="Turn Sweetie's head and gaze toward a named direction or object.",
        parameters={"type": "object", "properties": {
            "target": {"type": "string", "description": "Direction or object to look at."}},
            "required": ["target"]},
        dispatch_mode=DispatchMode.disabled,   # actuator: SOAR-arbitrated
    ),
    ToolSpec(
        name="play_animation",
        description="Play a named body animation (e.g. brohoof, nod, shake_head).",
        parameters={"type": "object", "properties": {
            "name": {"type": "string"}}, "required": ["name"]},
        dispatch_mode=DispatchMode.disabled,   # actuator: SOAR-arbitrated
    ),
]
