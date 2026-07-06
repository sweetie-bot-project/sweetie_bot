"""Scenario DSL: decorators mark test functions, register them (with their scenario spec) in the
harness REGISTRY, and at test time the harness builds the synth world and injects it as the
function's argument (see tests/behavior/conftest.py and runner.py).

Decorator order in source (outermost first):
    @behavior_test
    @scene(person(id=101, bearing=0, dist=1.5), pony(id=201, bearing=+30, dist=1.2))
    def test_hidden_pony_recall(world): ...
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

REGISTRY: Dict[str, Callable] = {}   # test name -> function (spec on func.__scenario__)

_SPEC_ATTR = "__scenario__"


# ---------------------------------------------------------------- entity builders ---------------
@dataclass
class SynthEntity:
    """Ground truth of one synthetic object; converted to Detection msgs by streams.py.

    bearing: degrees, + = to HER RIGHT (matches the calibrated scene convention);
    elevation: degrees, + = above her horizon; dist: metres (horizontal).
    Poses are emitted in the base_link frame (REP-103: x fwd, y left, z up) with the published
    y sign chosen so the collector+scene chain reproduces `bearing` after its own calibration.
    """
    id: int
    type: str
    bearing: float = 0.0
    elevation: float = 0.0
    dist: float = 1.5
    label: str = ""
    attributes: Dict[str, str] = field(default_factory=dict)
    score: float = 0.9

    def xyz(self) -> Tuple[float, float, float]:
        b = math.radians(self.bearing)
        e = math.radians(self.elevation)
        x = self.dist * math.cos(b)
        # scene chain: bearing = bearing_sign * atan2(-y, x), bearing_sign = -1.0 (calibrated)
        # => +bearing(right) needs atan2(-y,x) = -b  =>  y = dist*sin(b)
        y = self.dist * math.sin(b)
        z = self.dist * math.tan(e)
        return x, y, z

    def zone(self, front_deg: float = 60.0, side_deg: float = 120.0) -> str:
        m = abs(self.bearing)
        return "front" if m <= front_deg else ("side" if m <= side_deg else "rear")


def entity(type_: str, id: int, bearing: float = 0.0, dist: float = 1.5,
           elevation: float = 0.0, label: str = "", score: float = 0.9,
           **attributes: str) -> SynthEntity:
    return SynthEntity(id=id, type=type_, bearing=bearing, elevation=elevation, dist=dist,
                       label=label or f"{type_}_{id}",
                       attributes={k: str(v) for k, v in attributes.items()}, score=score)


def person(id: int = 101, bearing: float = 0.0, dist: float = 1.5, **kw) -> SynthEntity:
    """A human as the post-federation wire carries it (type 'human' — the proxy's type_map)."""
    return entity("human", id=id, bearing=bearing, dist=dist, **kw)


def pony(id: int = 201, bearing: float = 0.0, dist: float = 1.2, **kw) -> SynthEntity:
    return entity("pony", id=id, bearing=bearing, dist=dist, **kw)


# ---------------------------------------------------------------- scenario spec ------------------
@dataclass
class ScenarioSpec:
    entities: List[SynthEntity] = field(default_factory=list)
    lang: str = "en"
    soar_params: Dict[str, object] = field(default_factory=dict)   # relative to /soar (e.g. "input/swm/visibility_timeout")
    agent_params: Dict[str, object] = field(default_factory=dict)  # relative to /llm_agent (set on start, undone on stop)
    operational: bool = True            # open the SOAR operational window for the test
    detections_rate: float = 8.0


def _spec(func) -> ScenarioSpec:
    if not hasattr(func, _SPEC_ATTR):
        setattr(func, _SPEC_ATTR, ScenarioSpec())
    return getattr(func, _SPEC_ATTR)


def get_spec(func) -> Optional[ScenarioSpec]:
    return getattr(func, _SPEC_ATTR, None)


# ---------------------------------------------------------------- decorators ---------------------
def behavior_test(func):
    """Mark + register a behavior test. The harness injects `world` at test time."""
    _spec(func)
    REGISTRY[func.__name__] = func
    return func


def scene(*entities: SynthEntity):
    def deco(func):
        _spec(func).entities.extend(entities)
        return func
    return deco


def speech_lang(lang: str):
    def deco(func):
        _spec(func).lang = lang
        return func
    return deco


def soar_params(**params):
    """Per-test /soar/* rosparam overrides (applied + /soar/reconfigure; restored after).
    Keys use '/' for nesting, relative to /soar, e.g. soar_params(**{"input/swm/visibility_timeout": 2})."""
    def deco(func):
        _spec(func).soar_params.update(params)
        return func
    return deco


def agent_params(**params):
    """Per-test /llm_agent/* overrides, e.g. agent_params(**{"proactive/enabled": False}).
    Applied as rosparams by World.start() + restored by stop(); NO node restart — the node
    live-re-reads ~proactive each tick and ~reply_delay per reply. Params the node reads only
    at construction (~llm, ~persona_dir, ...) are NOT supported this way."""
    def deco(func):
        _spec(func).agent_params.update(params)
        return func
    return deco
