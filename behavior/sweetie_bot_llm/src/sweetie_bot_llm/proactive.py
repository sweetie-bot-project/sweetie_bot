"""Autonomous proactive self-talk trigger logic (PURE; no ROS).

Decides whether the LLM agent should emit a spontaneous self-talk aside and which cue, given the
current presence/activity timing. Kept ROS-free so it is unit-testable on CPU. agent_node owns the
clock, the live scene snapshot, the RNG and the voicing; this module only makes the decision.

Two triggers (option: driven on the LLM side, bypassing SOAR):
  A) nobody in frame - no human visible and no conversation for a while: she muses to herself
                    (about wanting company - NEVER "it's empty/quiet here": out of frame does
                    not mean absent, and conventions are loud).
  B) present lull - a human is in front of her but has not answered for ~>= one turn: sometimes
                    (probabilistic) she speaks up unprompted, never nagging (self-talk stays a
                    statement, demands no answer).
  C) covered camera - something is pressed against the lens. This OUTRANKS A and B: while
                    blind she must never muse "the space is empty" (it only LOOKS empty), and
                    self-talk is the ONLY speech path left — SOAR sends no reply goals without
                    a visible person to bind a talk event to (live finding, 2026-07-08).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple


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
    # "alone" = nobody visible IN FRAME and nobody talking with her. She must NOT conclude the
    # place is empty or silent from that (user rule 2026-07-08: she usually works conventions —
    # loud and crowded, people just out of frame). The cue is about HER social wish, never a
    # claim about the environment ("no one here" / "so quiet" are forbidden inferences).
    cue_alone: str = ("Nobody is talking with you right now and you cannot see anyone in front "
                      "of you - maybe everyone is just somewhere around. You feel a little wish "
                      "for company: you would love to chat with somebody or make new friends.")
    # a lull cue must point her at her OWN inner world, NOT at the person's silence (that made her
    # narrate them: "they might be thinking a lot about something"). To keep her from repeating one
    # bland line every pause, draw from a POOL of concrete, in-character musing seeds for variety.
    # cue_lull is an optional fixed override (rosparam); empty -> pick from the pool by the roll.
    cue_lull: str = ""
    cue_lull_pool: Tuple[str, ...] = field(default_factory=lambda: (
        "a happy little memory from Equestria drifts up",
        "you feel curious about some small wonder in the world around you",
        "a cheerful tune starts humming quietly in your head",
        "you feel a small wish to make even more new friends today",
        "a warm, fond thought about one of your pony friends crosses your mind",
        "a playful little what-if pops into your head",
        "you feel a small urge to hum, rhyme, or share a tiny cheerful thought",
    ))
    # C) covered camera: complain, do not muse. The cue names the situation explicitly (the
    # scene render adds its WARNING banner too); the anger override in _handle_self_talk
    # drives the eyes.
    occluded_after: float = 10.0  # covered at least this long before complaining (s)
    occluded_gap: float = 30.0    # minimum gap between successive complaints (s)
    cue_occluded: str = ("Something is pressed right against your camera - your view is "
                         "blocked and you can barely see anything. It is annoying and rude: "
                         "complain out loud that your view is covered and ask whoever did it "
                         "to take their hand (or whatever it is) off your face.")


def choose_proactive_cue(present, since_activity, since_selftalk, cfg, roll, occluded_for=None):
    """Return the cue string to speak, or None. Pure decision, no I/O.

    present        - is a human currently visible (bool)
    since_activity - seconds since the last real conversational turn
    since_selftalk - seconds since the last proactive aside
    cfg            - ProactiveConfig
    roll           - a random draw in [0, 1) for the probabilistic lull trigger
    occluded_for   - seconds the camera has been continuously covered; None when clear
    """
    if since_selftalk < cfg.min_gap:
        return None
    if occluded_for is not None:
        # covered RIGHT NOW: complain (C) or stay silent — never fall through to the
        # misleading "empty space" muse and never narrate a lull she cannot actually see
        if occluded_for >= cfg.occluded_after and since_selftalk >= cfg.occluded_gap:
            return cfg.cue_occluded
        return None
    if not present:
        if since_selftalk >= cfg.alone_gap and since_activity >= cfg.alone_after:
            return cfg.cue_alone
        return None
    if since_activity >= cfg.lull_after and roll < cfg.lull_prob:
        if cfg.cue_lull:                       # explicit fixed override
            return cfg.cue_lull
        pool = cfg.cue_lull_pool
        if not pool:
            return None
        # spread the (already gated) roll across the pool so successive lulls vary the subject
        idx = int((roll / cfg.lull_prob) * len(pool)) % len(pool) if cfg.lull_prob > 0 else 0
        return pool[idx]
    return None
