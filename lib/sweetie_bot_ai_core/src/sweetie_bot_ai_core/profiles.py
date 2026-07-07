"""LLM profile presets: named sampling/behavior configs, loadable from YAML config.

Canonical profile names are LANGUAGE-NEUTRAL (``complex``, ``simple``, ``failsafe``,
``rephrase``, ``self-talk``): the agent is canonical-English regardless of the conversation
language (the voice node localizes), so a language suffix in the profile name was noise.
SOAR request names like ``complex-en`` keep working unchanged — ``Agent._profile`` strips a
trailing ``-<2-letter-lang>`` suffix when resolving (no .soar edits needed).

``DEFAULT_PROFILES`` is the code FALLBACK; the deployed source of truth is
``sweetie_bot_llm/config/profiles.yaml`` (loaded via the ``~profiles`` rosparam, mirrored by
``load_profiles``). A unit test pins yaml == fallback at introduction time.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class ProfileConfig:
    allow_tools: bool = True
    max_verbatim_turns: int = 8
    max_tool_iters: int = 3
    options: Dict[str, object] = field(default_factory=lambda: {"temperature": 0.8})


DEFAULT_PROFILES: Dict[str, ProfileConfig] = {
    # primary path: full context + dynamic state + tools
    "complex": ProfileConfig(allow_tools=True, max_verbatim_turns=8, max_tool_iters=3,
                             options={"temperature": 0.8, "max_tokens": 512}),
    # fast lightweight path: reduced context, no tools
    "simple": ProfileConfig(allow_tools=False, max_verbatim_turns=4, max_tool_iters=0,
                            options={"temperature": 0.7, "max_tokens": 160}),
    # minimal degraded path
    "failsafe": ProfileConfig(allow_tools=False, max_verbatim_turns=2, max_tool_iters=0,
                              options={"temperature": 0.6, "max_tokens": 120}),
    # canned-speech rephrase hop (text-action interceptor): fast, tool-free. Runs WARM for real
    # variety - the isolated prompt (no scene/ambient) keeps it on-topic, so the old 1.0->0.8 trim
    # (which fought full-reply-path drift) isn't needed here; 0.8 read as near-verbatim on-robot.
    "rephrase": ProfileConfig(allow_tools=False, max_verbatim_turns=0, max_tool_iters=0,
                              options={"temperature": 1.05, "max_tokens": 96}),
    # spontaneous self-talk hop (proactive seam): scene-aware, tool-free, brief
    "self-talk": ProfileConfig(allow_tools=False, max_verbatim_turns=0, max_tool_iters=0,
                               options={"temperature": 0.8, "max_tokens": 80}),
}


def load_profiles(cfg: dict) -> Dict[str, ProfileConfig]:
    """Build a profile map from a plain config dict (ROS-free; mirrors ToolRegistry.from_config).

    Expected shape (the ``~profiles`` rosparam / profiles.yaml ``profiles:`` block)::

        profiles:
          complex: {allow_tools: true, max_verbatim_turns: 8, max_tool_iters: 3,
                    options: {temperature: 0.8, max_tokens: 512}}

    Accepts either the wrapping ``{"profiles": {...}}`` dict or the inner mapping directly.
    Unknown keys inside a profile entry are ignored; missing keys take the dataclass defaults.
    An empty/None config returns a copy of ``DEFAULT_PROFILES``.
    """
    inner = (cfg or {}).get("profiles", cfg) or {}
    if not inner:
        return dict(DEFAULT_PROFILES)
    out: Dict[str, ProfileConfig] = {}
    fields = ("allow_tools", "max_verbatim_turns", "max_tool_iters", "options")
    for name, entry in inner.items():
        kwargs = {k: entry[k] for k in fields if isinstance(entry, dict) and k in entry}
        out[str(name)] = ProfileConfig(**kwargs)
    return out
