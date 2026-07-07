"""PersonaRegistry.from_dir round-trip against the REAL shipped persona YAML + drift guard.

The launch loads config/persona/ at startup, so the YAML — not DEFAULT_PERSONA — is what runs
on the robot. The drift guard pins that fact: whenever the persona dir is present, the loaded
persona must be the (richer) YAML one, never silently the code fallback (C9).
"""
import os

import pytest

from sweetie_bot_ai_core import DEFAULT_PERSONA, PersonaRegistry

PERSONA_DIR = os.path.join(os.path.dirname(__file__), "..", "config", "persona")

pytestmark = pytest.mark.skipif(not os.path.isdir(PERSONA_DIR),
                                reason="shipped persona dir not present")


def test_from_dir_loads_real_sweetie_yaml():
    reg = PersonaRegistry.from_dir(PERSONA_DIR)
    p = reg.get("sweetie")
    assert p.name == "sweetie"
    assert p.display_name == "Sweetie Bot"
    assert "unicorn pony robot" in p.description
    assert p.max_reply_words == 150
    assert p.allow_tools is True
    assert p.voice_hints.get("ru") == "female"
    assert len(p.guidelines) >= 10


def test_loaded_persona_is_the_yaml_not_the_code_fallback():
    """Drift guard (C9): the YAML persona carries guidance the code fallback does not.
    If this fails, the registry silently served DEFAULT_PERSONA despite the dir existing."""
    reg = PersonaRegistry.from_dir(PERSONA_DIR)
    p = reg.get("sweetie")
    # YAML-only guidance: the differentiated per-spot touch map (forehead dislikes touch)
    assert any("forehead" in g and "NOT" in g for g in p.guidelines), \
        "the loaded persona lacks the YAML touch-map guideline — code fallback served instead?"
    # and the pony-kin + self-talk guidelines that never existed in code
    assert any("plushie or figurine" in g for g in p.guidelines)
    assert any("think out loud" in g for g in p.guidelines)


def test_active_default_and_unknown_name_fall_back():
    reg = PersonaRegistry.from_dir(PERSONA_DIR)
    assert reg.active.name == "sweetie"
    assert reg.get("no-such-persona").name == "sweetie"
    assert reg.get(None).name == "sweetie"


def test_from_dir_missing_path_degrades_to_default():
    reg = PersonaRegistry.from_dir("/nonexistent/persona/dir")
    assert reg.active.name == DEFAULT_PERSONA.name
