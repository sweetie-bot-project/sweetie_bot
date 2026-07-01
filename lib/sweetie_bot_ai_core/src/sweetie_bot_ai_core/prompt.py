"""System-prompt builder: static persona + dynamic robot-state block. PURE (no ROS, no I/O).

State arrives only via a RobotState value object, so this stays reusable headless.
"""
from __future__ import annotations

from typing import List, Optional

from .persona import Persona
from .schema import RobotState


def build_system_prompt(persona: Persona, state: Optional[RobotState] = None, *,
                        tools_offered: bool = False,
                        scene_block: Optional[str] = None,
                        language_note: Optional[str] = None) -> str:
    parts: List[str] = []

    # --- static persona (keep first so the prefix stays KV-cacheable) ------------------------
    parts.append(persona.description.strip())
    if persona.guidelines:
        parts.append("Guidelines:\n" + "\n".join(f"- {g}" for g in persona.guidelines))
    if persona.emotion_guidance:
        parts.append(persona.emotion_guidance.strip())
    parts.append(f"Keep your reply to at most {persona.max_reply_words} words.")

    # --- tool guidance ----------------------------------------------------------------------
    if tools_offered:
        parts.append(
            "You have tools available. If you need live information (such as your battery, the "
            "time, or your physical status) to answer truthfully, call the appropriate tool "
            "first; otherwise just answer.")

    # --- dynamic scene block (what she perceives around her) --------------------------------
    if scene_block:
        parts.append(scene_block)

    # --- dynamic state block ----------------------------------------------------------------
    if state is not None:
        summary = state.human_summary()
        if summary:
            parts.append("Your current live state — use it to answer truthfully:\n" + summary)

    # --- language note ----------------------------------------------------------------------
    if language_note:
        parts.append(language_note)

    return "\n\n".join(p for p in parts if p)
