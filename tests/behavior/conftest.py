"""Behavior-synth pytest glue: session sim fixture + per-test World injection.

Tests declare their scenario via the DSL decorators (@behavior_test/@scene/...); this conftest
reads the spec off the collected test function, builds the World, injects it as the `world`
argument, and resets the system afterwards. Skips cleanly when no ROS master is reachable so
CPU-only environments stay green.

Markers:
  unknown  - behavior whose live status was never established (report-only; classify later)
"""
from __future__ import annotations

import os

import pytest


def _ros_available() -> bool:
    try:
        import rosgraph
        return rosgraph.is_master_online()
    except Exception:  # noqa: BLE001
        return False


ROS_UP = _ros_available()


def pytest_configure(config):
    config.addinivalue_line("markers", "unknown: behavior with unestablished live status (report-only)")


@pytest.fixture(scope="session")
def sim():
    if not ROS_UP:
        pytest.skip("no ROS master - behavior tests need the sim stack")
    from sweetie_bot_behavior_synth.env import SimStack
    return SimStack().ensure()


@pytest.fixture()
def world(request, sim):
    from sweetie_bot_behavior_synth.dsl import get_spec
    from sweetie_bot_behavior_synth.world import World
    spec = get_spec(request.function)
    if spec is None:
        pytest.fail(f"{request.function.__name__} has no @behavior_test scenario spec")
    w = World(spec).start()
    try:
        yield w
    finally:
        w.stop()
