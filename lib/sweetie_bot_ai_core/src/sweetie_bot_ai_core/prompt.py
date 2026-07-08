"""System-prompt builder: static persona + dynamic robot-state block. PURE (no ROS, no I/O).

State arrives only via a RobotState value object, so this stays reusable headless.
"""
from __future__ import annotations

from typing import List, Optional

from .persona import Persona
from .schema import RobotState

# Shared tool guidance. Keep ACTION tools first-class: an earlier info-only phrasing ("if you
# need live information ... otherwise just answer") steered the model away from ever calling
# actuator tools on direct "dance for me" asks (probed on qwen2.5:7b, 2026-07-08).
_TOOL_NOTE = (
    "You have tools available. If you need live information (such as your battery, the "
    "time, or your physical status) to answer truthfully, call the appropriate tool "
    "first. If the human asks you to perform a physical action that one of your tools "
    "provides, call that tool so you actually do it, then answer. Otherwise just answer.")


def build_tool_phase_prompt(persona: Persona) -> str:
    """Minimal system prompt for the TOOL-DECISION call (identity + tool guidance only).

    The tool loop must not share the full reply prompt: on a 7B the ~14 persona style
    guidelines dilute the tool schema until native tool-calling collapses (measured on
    qwen2.5:7b: "give me your hoof" triggered play_animation 0/5 under the full prompt —
    sometimes emitting the call as junk TEXT — vs 5/5 under this lean prompt; scene/state
    blocks alone cost 3-5/5). Structurally the decision call needs nothing else: the scene
    and robot state are exactly what the info tools FETCH, and the style guidelines govern
    the voiced reply, which is composed by the later constrained call under the full prompt.
    """
    return persona.description.strip() + "\n\n" + _TOOL_NOTE


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
    parts.append(
        "Using your live awareness: everything the system tells you about your situation — "
        "what you see around you, how your body feels, and whether the human has gone quiet, "
        "is not answering or has stepped away — is your OWN private awareness, never text to "
        "read back. Do NOT quote, list, restate or directly answer these notes; let them "
        "naturally shape what you say, the way you would simply mention your leg feeling hot "
        "rather than reading out a fault line. In particular, if you are told the human is "
        "silent or has not answered, never reply to that literally or announce it — just stay "
        "warm and natural, make a light passing remark or gently carry on, and never nag them "
        "to respond."
    )

    # --- tool guidance ----------------------------------------------------------------------
    if tools_offered:
        parts.append(_TOOL_NOTE)

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
