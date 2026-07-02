"""Scene salience, inter-turn diff, and prompt rendering — PURE (no ROS, no I/O).

Turns a raw ``SceneState`` (from a SceneProvider) into (a) a compact, front-weighted natural-language
block for the dynamic prompt, and (b) an inter-turn "what changed" delta. All geometry arrives as
coarse fields on ``SceneEntity``; this module only filters, orders and phrases.

Design rules (settled with the user):
* People are referred to by spatial position + ``(id N)`` — never names, and she must not speak the id.
* Front cone = full salience; sides = only to fill spare slots; rear = dropped (esp. sound).
* Out-of-frame "remembered" entities are NOT put in the always-on block; they surface only via the
  ``get_scene`` tool (``include_remembered``).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from .schema import SceneEntity, SceneEvent, SceneState, SoundCue, Zone


@dataclass
class SceneConfig:
    front_deg: float = 60.0        # |bearing| <= front_deg  -> front
    side_deg: float = 120.0        # front_deg < |bearing| <= side_deg -> side; else rear
    max_entities: int = 6          # cap on entities in the ambient block
    max_remembered: int = 4
    max_sounds: int = 2
    # pretty rendering for known attribute keys; value is a format string using {v}
    attr_pretty: Dict[str, str] = field(default_factory=lambda: {
        "gaze_at_robot": "looking at you",
        "looking_at_robot": "looking at you",
        "smiling": "smiling",
        "emotion": "looks {v}",
        "holding": "holding {v}",
        "waving": "waving",
    })


# --- zones -----------------------------------------------------------------------------------

def classify_zone(bearing_deg: float, cfg: SceneConfig) -> Zone:
    b = abs(bearing_deg)
    if b <= cfg.front_deg:
        return Zone.front
    if b <= cfg.side_deg:
        return Zone.side
    return Zone.rear


# --- salience --------------------------------------------------------------------------------

def _entity_sort_key(e: SceneEntity):
    # interlocutor first, then front before side, then closest-to-center, then speaking
    zone_rank = {Zone.front: 0, Zone.side: 1, Zone.rear: 2}[e.zone]
    return (0 if e.is_interlocutor else 1, zone_rank, abs(e.bearing_deg), 0 if e.is_speaking else 1)


def select_salient(state: SceneState, cfg: SceneConfig) -> SceneState:
    """Front-weighted selection for the ambient block: in-frame only, drop rear, sides only to fill.

    Keeps every front entity, then fills up to ``max_entities`` with side entities. Rear is dropped.
    """
    visible = [e for e in state.entities if e.in_frame and e.zone != Zone.rear]
    front = [e for e in visible if e.zone == Zone.front]
    side = [e for e in visible if e.zone == Zone.side]
    front.sort(key=_entity_sort_key)
    side.sort(key=_entity_sort_key)
    chosen = front[: cfg.max_entities]
    if len(chosen) < cfg.max_entities:
        chosen += side[: cfg.max_entities - len(chosen)]
    chosen.sort(key=_entity_sort_key)
    # remembered (out-of-frame) entities pass through regardless of zone - their bearing is
    # recomputed against her CURRENT forward, so "it was to your right" stays true after she
    # turns; render_scene shows them in the "Recently seen" section
    remembered = [e for e in state.entities if not e.in_frame]
    remembered.sort(key=lambda e: e.last_seen_s)
    chosen += remembered[: cfg.max_remembered]

    sounds = [s for s in state.sounds if s.zone != Zone.rear]
    sounds.sort(key=lambda s: (0 if s.zone == Zone.front else 1, abs(s.bearing_deg)))
    return SceneState(entities=chosen, sounds=sounds[: cfg.max_sounds])


# --- inter-turn diff -------------------------------------------------------------------------

def diff(prev: SceneState, curr: SceneState, cfg: SceneConfig) -> List[SceneEvent]:
    """Diff two *salient* SceneStates by id → arrived / left / changed events."""
    prev_map = {e.id: e for e in prev.entities}
    curr_map = {e.id: e for e in curr.entities}
    events: List[SceneEvent] = []
    for eid, e in curr_map.items():
        if eid not in prev_map:
            events.append(SceneEvent(kind="arrived", entity_id=eid,
                                     detail=f"{_who(e)} appeared {bearing_words(e.bearing_deg, cfg)}"))
        else:
            changed = _attr_changes(prev_map[eid].attributes, e.attributes, cfg)
            if changed:
                events.append(SceneEvent(kind="changed", entity_id=eid,
                                         detail=f"{_who(e)} is now {changed}"))
    for eid, e in prev_map.items():
        if eid not in curr_map:
            events.append(SceneEvent(kind="left", entity_id=eid, detail=f"{_who(e)} left"))
    return events


def _attr_changes(old: Dict[str, str], new: Dict[str, str], cfg: SceneConfig) -> str:
    bits = [render_attr(k, v, cfg) for k, v in new.items() if old.get(k) != v]
    return ", ".join(b for b in bits if b)


# --- phrasing --------------------------------------------------------------------------------

def bearing_words(bearing_deg: float, cfg: SceneConfig) -> str:
    """+bearing = to her right, -bearing = to her left, 0 = ahead."""
    b = bearing_deg
    side = "right" if b > 0 else "left"
    m = abs(b)
    if m <= 15:
        return "directly in front of you"
    if m <= cfg.front_deg:
        return f"in front of you, to your {side}"
    return f"to your {side}"


def render_attr(key: str, value: str, cfg: SceneConfig) -> str:
    fmt = cfg.attr_pretty.get(key)
    if fmt is None:
        return f"{key}: {value}" if value not in ("", "1", "true", "True") else key
    # boolean-ish attrs: skip when explicitly negative
    if str(value).lower() in ("0", "false", "no", "none"):
        return f"not {fmt}" if "{v}" not in fmt else ""
    return fmt.format(v=value)


def _who(e: SceneEntity) -> str:
    if e.type in ("person", "human", "face"):
        return "someone"
    return f"a {e.type.replace('_', ' ')}"


def elevation_words(elevation_deg: float) -> str:
    """Vertical placement vs her horizon; empty when roughly level."""
    if elevation_deg >= 45:
        return "well above you"
    if elevation_deg >= 15:
        return "above you"
    if elevation_deg <= -45:
        return "far below you"
    if elevation_deg <= -15:
        return "below you"
    return ""


def _describe(e: SceneEntity, cfg: SceneConfig) -> str:
    if e.type in ("person", "human", "face"):
        subj = f"a person (id {e.id})"
    else:
        subj = f"a {e.type.replace('_', ' ')} (id {e.id})"
    where = bearing_words(e.bearing_deg, cfg)
    vert = elevation_words(e.elevation_deg)
    if vert:
        where = f"{where}, {vert}"
    parts = [subj, where]
    tags = []
    if e.is_interlocutor:
        tags.append("the one talking with you")
    elif e.is_speaking:
        tags.append("speaking")
    for k, v in e.attributes.items():
        r = render_attr(k, v, cfg)
        if r:
            tags.append(r)
    if e.distance:
        tags.append(e.distance)
    if tags:
        return f"{parts[0]} {parts[1]} — {', '.join(tags)}"
    return f"{parts[0]} {parts[1]}"


# --- prompt block ----------------------------------------------------------------------------

_ID_NOTE = ("(Refer to people naturally by where they are or what they look like — never say the id "
            "number out loud. Mention what you notice only if it's relevant.)")


def render_scene(state: SceneState, events: List[SceneEvent], cfg: SceneConfig) -> str:
    """The always-on ambient block: 'Around you right now' + 'Since you last replied'."""
    lines: List[str] = []
    occluded = [e for e in state.entities if e.type == "camera_occluded" and e.in_frame]
    normal = [e for e in state.entities if e.type != "camera_occluded"]
    if occluded:
        lines.append("WARNING: something is pressed right against your camera - your view is "
                     "blocked and you can barely see anything right now.")
    if normal:
        lines.append("Around you right now:")
        for e in normal:
            lines.append(f"- {_describe(e, cfg)}")
    for s in state.sounds:
        kind = "a voice" if s.kind == "speech" else "a sound"
        lines.append(f"- You hear {kind} {bearing_words(s.bearing_deg, cfg)}.")
    if events:
        lines.append("Since you last replied:")
        for ev in events:
            lines.append(f"- {ev.detail}.")
    # recently-departed objects: keep them in her ambient awareness for the retention window so
    # "where did the X go" works without requiring a tool call (7b models are tool-shy)
    remembered = [e for e in state.entities
                  if not e.in_frame and e.type != "camera_occluded"]
    if remembered:
        lines.append("Recently seen (now OUT of view - when asked where one of these went, "
                     "tell the remembered direction and how long ago):")
        for e in sorted(remembered, key=lambda e: e.last_seen_s):
            ago = int(round(e.last_seen_s))
            lines.append(f"- {_describe(e, cfg)} (last seen ~{ago}s ago)")
    if not lines:
        return ""
    return "\n".join(lines) + "\n" + _ID_NOTE


def render_remembered(entities: List[SceneEntity], cfg: SceneConfig) -> str:
    """Natural-language rendering of out-of-frame remembered objects, for the get_scene tool."""
    out = [e for e in entities if not e.in_frame]
    if not out:
        return "Nothing notable out of view is still remembered."
    lines = ["Remembered (out of view) — you could turn to look:"]
    for e in sorted(out, key=lambda e: e.last_seen_s):
        subj = "a person" if e.type in ("person", "human", "face") else f"a {e.type.replace('_', ' ')}"
        lines.append(f"- {subj} (id {e.id}) was {bearing_words(e.bearing_deg, cfg)} "
                     f"about {int(e.last_seen_s)}s ago")
    return "\n".join(lines)
