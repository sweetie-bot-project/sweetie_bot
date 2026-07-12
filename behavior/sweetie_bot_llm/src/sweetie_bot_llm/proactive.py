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
                    Two complaint paths: the EDGE (occlusion_edge_cue - immediate one-shot per
                    cover episode, ~2s debounce, exempt from min_gap/occluded_gap) and the
                    repeat-while-held (here in choose_proactive_cue, occluded_after/occluded_gap
                    cadence).
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
    presence_grace: float = 10.0  # a human seen within this many seconds still counts as present -
                                  # bridges 1-frame detector dropouts of far/blurry people (T.1#3)
    lull_after: float = 27.0    # human present but no turn for ~this long == a one-turn lull (s).
                                # Raised 18->27 (user 2026-07-12: she filled every pause / mused
                                # too much when nobody was speaking to her; keep in the 25-30 band,
                                # above min_gap so a lull aside never beats the global idle floor).
    lull_prob: float = 0.25     # per-tick chance to speak up during a present lull
    speech_grace: float = 2.5   # a human counts as "still speaking" for this long after the last
                                # SPEECH_DETECTING frame / button release (mirrors SOAR's
                                # soar.yaml speech_timeout); bridges 1-frame VAD dropouts so a
                                # muse cannot slip through the gap between words (see is_human_speaking)
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
    occluded_after: float = 10.0  # covered at least this long before a held-repeat
                                  # complaint (s; the episode's FIRST complaint is the
                                  # edge path below)
    occluded_gap: float = 10.0    # gap between repeat complaints while covered (s) — her
                                  # normal speech cadence; deliberately BELOW and exempt
                                  # from min_gap (an ongoing cover is an active grievance)
    occluded_edge_after: float = 2.0  # rising-edge debounce before the IMMEDIATE first
                                      # complaint of a cover episode; that complaint is
                                      # exempt from min_gap/occluded_gap by design
    cue_occluded: str = ("Something is pressed right against your camera - your view is "
                         "blocked and you can barely see anything. It is annoying and rude: "
                         "complain out loud that your view is covered and ask whoever did it "
                         "to take their hand (or whatever it is) off your face.")


def choose_proactive_cue(present, since_activity, since_selftalk, cfg, roll, occluded_for=None,
                         human_speaking=False):
    """Return the cue string to speak, or None. Pure decision, no I/O.

    present        - is a human currently visible (bool)
    since_activity - seconds since the last real conversational turn
    since_selftalk - seconds since the last proactive aside
    cfg            - ProactiveConfig
    roll           - a random draw in [0, 1) for the probabilistic lull trigger
    occluded_for   - seconds the camera has been continuously covered; None when clear
    human_speaking - a human is speaking or holding the push-to-talk button RIGHT NOW: never
                     talk over them (restores the pre-refactor SOAR -^talking wait that this
                     SOAR-bypassing driver dropped, regression commit 7e71bad1). OUTRANKS even
                     the covered-camera complaint: waiting for the question beats complaining.
    """
    if human_speaking:
        return None
    if occluded_for is not None:
        # covered RIGHT NOW: complain (C) or stay silent — never fall through to the
        # misleading "empty space" muse and never narrate a lull she cannot actually see.
        # Checked BEFORE min_gap: while the cover holds she repeats the complaint at her
        # normal speech cadence (occluded_gap), not the idle-aside min_gap — an ongoing
        # cover is an active grievance, not musing (user, 2026-07-11). The episode's
        # FIRST complaint is faster still: the edge path (occlusion_edge_cue).
        if occluded_for >= cfg.occluded_after and since_selftalk >= cfg.occluded_gap:
            return cfg.cue_occluded
        return None
    if since_selftalk < cfg.min_gap:
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


def is_human_speaking(ptt_pressed, since_speech, speech_grace):
    """Is a human speaking or about to speak right now? Pure predicate, no I/O.

    Mirrors SOAR's `io.input-link.sound.speech` gate on the agent side (see choose_proactive_cue's
    human_speaking). True while the push-to-talk button is held OR the last SPEECH_DETECTING frame
    was within `speech_grace` seconds. The button is the earliest, most decisive signal (it fires
    before any transcription and survives the mic-mute-while-she-talks); the graced speech flag
    covers ambient/non-PTT speech and bridges 1-frame VAD dropouts between words.

    ptt_pressed  - is the mic button currently held (bool)
    since_speech - seconds since the last SPEECH_DETECTING frame (or button release)
    speech_grace - how long a human still counts as speaking after that (s)
    """
    return bool(ptt_pressed or since_speech < speech_grace)


def occlusion_edge_cue(occluded_for, complained, cfg):
    """The IMMEDIATE complaint on the occlusion rising edge. Pure decision, no I/O.

    Fires ONCE per cover episode as soon as the lens has been covered for
    cfg.occluded_edge_after (a short debounce against a hand merely waved across the camera).
    Deliberately takes NO since_selftalk: the first complaint of an episode is exempt from
    min_gap/occluded_gap by construction (the user covered the lens live and had to wait out
    the full held cadence before hearing anything, 2026-07-08). Repeat-while-held complaints
    stay behind choose_proactive_cue's occluded_after/occluded_gap cadence.

    occluded_for - seconds the camera has been continuously covered; None when clear
    complained   - has this cover episode already had its complaint (edge OR held)
    """
    if occluded_for is None or complained:
        return None
    return cfg.cue_occluded if occluded_for >= cfg.occluded_edge_after else None

# entity types that count as a human for the presence gate (ponies are pony/pony_face)
_HUMAN_TYPES = frozenset({"person", "human", "face", "body"})


def humans_present(entities, grace_s, types=_HUMAN_TYPES):
    """Is a human present, tolerating brief detector dropouts?

    An entity counts while it is in frame OR was last seen no more than ``grace_s`` ago
    (remembered scene entities carry ``last_seen_s``; in-frame ones have it at 0.0).
    A 1-frame dropout of a far/blurry person must not read as "alone" - that fired the
    alone-cue with a person standing right there (live 2026-07-08, T.1#3).
    """
    return any((getattr(e, "type", "") or "") in types
               and (getattr(e, "in_frame", True) or getattr(e, "last_seen_s", 0.0) <= grace_s)
               for e in entities)
