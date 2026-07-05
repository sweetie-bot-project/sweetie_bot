"""Autonomous proactive self-talk trigger logic (PURE; no ROS).

Decides whether the LLM agent should emit a spontaneous self-talk aside and which cue, given the
current presence/activity timing. Kept ROS-free so it is unit-testable on CPU. agent_node owns the
clock, the live scene snapshot, the RNG and the voicing; this module only makes the decision.

Two triggers (option: driven on the LLM side, bypassing SOAR):
  A) empty scene  - no human visible and things have been quiet: she muses to herself.
  B) present lull - a human is in front of her but has not answered for ~>= one turn: sometimes
                    (probabilistic) she speaks up unprompted, never nagging (self-talk stays a
                    statement, demands no answer).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ProactiveConfig:
    enabled: bool = True
    period: float = 5.0         # driver tick period (s)
    min_gap: float = 25.0       # hard global minimum between ANY two asides (s)
    alone_after: float = 20.0   # empty scene: quiet at least this long before musing (s)
    alone_gap: float = 45.0     # empty scene: minimum gap between successive "alone" asides (s)
    lull_after: float = 18.0    # human present but no turn for ~this long == a one-turn lull (s)
    lull_prob: float = 0.25     # per-tick chance to speak up during a present lull
    profile: str = "self-talk-en"
    persona: str = ""
    cue_alone: str = ("You look around and realise there is no one here with you right now - "
                      "the space is empty and quiet.")
    cue_lull: str = ("The person in front of you has gone quiet for a little while now, just "
                     "standing there without saying anything.")


def choose_proactive_cue(present, since_activity, since_selftalk, cfg, roll):
    """Return the cue string to speak, or None. Pure decision, no I/O.

    present        - is a human currently visible (bool)
    since_activity - seconds since the last real conversational turn
    since_selftalk - seconds since the last proactive aside
    cfg            - ProactiveConfig
    roll           - a random draw in [0, 1) for the probabilistic lull trigger
    """
    if since_selftalk < cfg.min_gap:
        return None
    if not present:
        if since_selftalk >= cfg.alone_gap and since_activity >= cfg.alone_after:
            return cfg.cue_alone
        return None
    if since_activity >= cfg.lull_after and roll < cfg.lull_prob:
        return cfg.cue_lull
    return None
