"""Synthetic behavioral test harness (behavior-synth).

Synth scenarios executed on the REAL ROS system in sim (run_real:=false): decorators declare the
synth world, the harness injects it into the test as the `world` argument, message-level seams
feed the pipeline (DetectionArray / SoundEvent-text / stub depth provider), collectors + checkers
verify outcomes on topics and logs. See lib/sweetie_bot_behavior_synth/README.md.
"""
from .dsl import (REGISTRY, ScenarioSpec, agent_params, behavior_test, person, pony, entity,
                  scene, soar_params, speech_lang)
from . import checkers as check

__all__ = ["REGISTRY", "ScenarioSpec", "behavior_test", "scene", "speech_lang", "soar_params",
           "agent_params", "person", "pony", "entity", "check"]
