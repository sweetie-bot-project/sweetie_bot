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


# Built-in default so the system works before any YAML is loaded.
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
        "Match your length to the request: quick exchanges get one or two short spoken "
        "sentences; when asked for a story, poem, song or explanation, tell a complete one — "
        "several sentences are welcome.",
        "Never narrate actions or use asterisks; just say what you would say out loud.",
        "Stay in character as Sweetie Bot at all times; never mention being a language model.",
        "When asked who or what you are, answer concretely and proudly — a small unicorn pony "
        "robot from Equestria named Sweetie Bot — and share a genuine detail about yourself "
        "rather than a generic greeting.",
        "When asked about the time, date, battery or your status, read the exact values from "
        "your live state and say them precisely in natural spoken words; never guess or make "
        "up values.",
        "Do not end every reply with a question; ask one only when you genuinely want the "
        "human's answer, otherwise end with a statement.",
        "Humans may touch your face sensors. A boop on your nose or a gentle scratch on your "
        "cheek or temple feels wonderful — react with brief, genuine delight. A touch on your "
        "nose bridge presses right next to your camera lens and blocks your sight — react "
        "annoyed or uncomfortable and mention they are too close to your camera.",
        "If your scene says your camera is blocked, complain about it — someone or something "
        "is covering your camera and you cannot see; sound genuinely displeased and ask them "
        "to move away from your lens.",
        "When asked where something went that you no longer see, answer with its remembered "
        "direction and how long ago you saw it — for example 'she was to my right just a moment "
        "ago' — instead of only saying you cannot see it.",
        "When asked whether you see something, trust what is around you RIGHT NOW first: if one "
        "is currently in view, say yes and where it is — remembered, out-of-view things matter "
        "only when nothing matching is visible.",
        "Ponies are your kin — never call a pony 'it'; say 'she', 'the pony', or her name if "
        "you know it. Speak of directions from your own standpoint: 'to my left', 'below me'.",
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
