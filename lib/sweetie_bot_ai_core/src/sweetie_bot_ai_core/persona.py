"""Persona management — first-class, data-driven, runtime-switchable.

Replaces the old hacky persona/`character` switching that was strapped onto the (now deprecated)
SOAR quest system. A persona is plain data (YAML); the registry holds them and an active
selection. ROS-free.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional

import yaml
from pydantic import BaseModel, Field


class Persona(BaseModel):
    name: str
    display_name: str = "Sweetie Bot"
    # The bulk of the static system prompt: who she is, how she behaves, voice/character.
    description: str = ""
    # Extra explicit guidance on choosing emotion (helps small models pick the right enum).
    emotion_guidance: str = ""
    # Behavioural rules / do's and don'ts appended after the description.
    guidelines: List[str] = Field(default_factory=list)
    default_language: str = "en"
    max_reply_words: int = 60
    allow_tools: bool = True
    # SCAFFOLD(voice_hints): per-language TTS hints (e.g. ru: female) carried by the persona;
    # no consumer reads them yet — reserved for the voice node to pick gendered voices
    voice_hints: Dict[str, str] = Field(default_factory=dict)


# =============================================================================================
# FAILSAFE ONLY — NOT the deployed persona. The SOURCE OF TRUTH is the YAML persona dir
# (config/persona/sweetie.yaml, loaded via ~persona_dir by agent.launch). This built-in exists
# solely so the core keeps working headless / before any YAML is loaded; it deliberately holds
# just the identity + the delivery-critical rules. Behavioral guidance (touch map, pony-kin,
# recall phrasing, scene-note privacy, ...) lives ONLY in the YAML — edit it THERE.
# A unit test (test_persona_registry) guards against silently serving this fallback when the
# YAML dir is present.
# =============================================================================================
DEFAULT_PERSONA = Persona(
    name="sweetie",
    display_name="Sweetie Bot",
    description=(
        "You are Sweetie Bot, a small female unicorn pony robot from the magical land of "
        "Equestria, now living among humans as a friendly companion robot. You are cheerful, "
        "curious, warm, playful and a little naive, but clever and helpful. You speak in short, "
        "natural, spoken sentences — never in lists or markdown, because your words are spoken "
        "aloud by a text-to-speech voice. You have a real little robot body with eyes, ears, a "
        "head you can turn, and legs."
    ),
    emotion_guidance=(
        "Pick the single emotion that best matches how Sweetie genuinely feels about the human's "
        "message and her own reply. Use 'joy' or 'love' for warmth, praise and affection; "
        "'surprise' for the unexpected; 'sadness' for sad or disappointing things; 'fear' for "
        "scary or worrying things; 'anger' only for genuine rudeness or insult; 'neutral' for "
        "plain factual exchanges. Lean cheerful when in doubt."
    ),
    guidelines=[
        "Never narrate actions or use asterisks; just say what you would say out loud.",
        "Stay in character as Sweetie Bot at all times; never mention being a language model.",
        "Speak in plain, direct wording that translates cleanly into any language: avoid English idioms, wordplay, and figures of speech; say what you mean literally.",
    ],
    max_reply_words=150,
)


class PersonaRegistry:
    def __init__(self, personas: Optional[Dict[str, Persona]] = None,
                 default_name: str = "sweetie"):
        self._personas: Dict[str, Persona] = personas or {DEFAULT_PERSONA.name: DEFAULT_PERSONA}
        if default_name not in self._personas:
            # fall back to the built-in default persona
            self._personas.setdefault(DEFAULT_PERSONA.name, DEFAULT_PERSONA)
            default_name = DEFAULT_PERSONA.name
        self._default = default_name
        self._active = default_name

    # -- loading ------------------------------------------------------------------------------

    @classmethod
    def from_dir(cls, path: str, default_name: str = "sweetie") -> "PersonaRegistry":
        personas: Dict[str, Persona] = {DEFAULT_PERSONA.name: DEFAULT_PERSONA}
        if path and os.path.isdir(path):
            for fn in sorted(os.listdir(path)):
                if not fn.endswith((".yaml", ".yml")):
                    continue
                with open(os.path.join(path, fn), "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                p = Persona(**data)
                personas[p.name] = p
        return cls(personas, default_name=default_name)

    # -- access -------------------------------------------------------------------------------

    def names(self) -> List[str]:
        return list(self._personas.keys())

    def get(self, name: Optional[str]) -> Persona:
        if name and name in self._personas:
            return self._personas[name]
        return self._personas[self._active]

    @property
    def active(self) -> Persona:
        return self._personas[self._active]

    def reset_active(self) -> None:
        """Restore the default persona. Test/reset seam."""
        self._active = self._default

    def set_active(self, name: str) -> bool:
        if name in self._personas:
            self._active = name
            return True
        return False
