"""Standalone single-scenario runner (registry-driven) for manual debugging.

    ~/sbllm-venv/bin/python -m sweetie_bot_behavior_synth run test_hidden_pony_recall

Imports the tests/behavior tree (which registers scenarios via @behavior_test), builds the
World from the spec, runs the one test with live output, and resets afterwards.
"""
from __future__ import annotations

import importlib.util
import os
import sys
import glob


def _load_scenarios(tests_dir: str):
    for path in sorted(glob.glob(os.path.join(tests_dir, "test_*.py"))):
        name = "bsynth_scenarios_" + os.path.basename(path)[:-3]
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mod)
        except Exception as e:  # noqa: BLE001 - a broken scenario file must not hide the rest
            print(f"[runner] skipping {path}: {e!r}")


def default_tests_dir() -> str:
    """`tests/behavior` of this checkout, located relative to this file - never hardcoded.

    $BSYNTH_TESTS_DIR overrides. realpath() first: the package is reached through the catkin
    workspace's src/ symlink, and __file__ keeps the symlinked path.
    """
    override = os.environ.get("BSYNTH_TESTS_DIR")
    if override:
        return os.path.abspath(os.path.expanduser(override))
    here = os.path.dirname(os.path.realpath(__file__))
    # <checkout>/lib/<package>/src/<module>/ -> <checkout>
    root = os.path.abspath(os.path.join(here, *([os.pardir] * 4)))
    return os.path.join(root, "tests", "behavior")


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if len(argv) < 2 or argv[0] != "run":
        print(__doc__)
        return 2
    test_name = argv[1]
    tests_dir = argv[2] if len(argv) > 2 else default_tests_dir()

    from .dsl import REGISTRY, get_spec
    from .env import SimStack
    from .world import World

    _load_scenarios(tests_dir)
    func = REGISTRY.get(test_name)
    if func is None:
        print(f"unknown scenario {test_name!r}; registered: {sorted(REGISTRY)}")
        return 2

    SimStack().ensure()
    world = World(get_spec(func)).start()
    print(f"[runner] {test_name}: world up ({len(world.spec.entities)} entities, "
          f"lang={world.spec.lang})")
    try:
        func(world)
        print(f"[runner] {test_name}: PASS")
        return 0
    except AssertionError as e:
        print(f"[runner] {test_name}: FAIL - {e}")
        return 1
    finally:
        world.stop()


if __name__ == "__main__":
    raise SystemExit(main())
