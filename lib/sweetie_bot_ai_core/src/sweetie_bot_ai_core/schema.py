"""Pydantic-v2 data model for the Sweetie Bot LLM agent.

This module is the single source of truth for the agent's wire/validation/structured-output
schema. It is **ROS-free** so the same agent can run headless (sim, TUI, messenger).

The emotion vocabulary is fixed by the downstream SOAR/animation system: SOAR reuses the
returned ``emotion`` as an ``animation-tag``, so the enum must stay exactly these seven values.
"""
from __future__ import annotations

from enum import Enum
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field


# --- fixed vocabularies (must match the SOAR/animation contract) -----------------------------

class Emotion(str, Enum):
    """THE emotion vocabulary — single source of truth (R4).

    Everything else points here: GenerateReply.action's emotion comment, the soar.yaml
    emotion maps (their keys must be a subset — pinned by test_agent_bridge), and the SOAR
    rules that reuse the value as an animation-tag. Change it HERE first; the seven values
    are frozen by the downstream animation set.
    """
    love = "love"
    joy = "joy"
    surprise = "surprise"
    neutral = "neutral"
    sadness = "sadness"
    fear = "fear"
    anger = "anger"


class SentenceType(str, Enum):
    question = "question"
    statement = "statement"


class RequestType(str, Enum):
    reply = "reply"            # conversational reply (implemented now)
    rephrase = "rephrase"      # constrained rewording of a scripted line (text-action hop)
    self_talk = "self_talk"    # spontaneous in-character remark from a cue (proactive seam)
    classify = "classify"      # reserved: classify text into provided labels
    assess_scene = "assess_scene"  # reserved: describe/assess an image (future VLM)


# --- servo/body naming -----------------------------------------------------------------------

# Her four legs are named leg1..leg4 in hardware. She cannot map those tokens to a body location
# on her own and confuses front vs hind (observed live: same fault voiced as "front" then "hind"),
# so we annotate any servo/joint fault name with an explicit location. Layout (user-specified):
#   leg1 = front-left   leg2 = front-right   leg3 = hind-left   leg4 = hind-right
_LEG_LOCATION = {
    "leg1": "front-left leg",
    "leg2": "front-right leg",
    "leg3": "hind-left leg",
    "leg4": "hind-right leg",
}


def friendly_servo(name: str) -> str:
    """Annotate a raw servo/joint name with its body location so she never guesses front vs hind.
    Names containing leg1..leg4 get the mapped location appended; anything else passes through."""
    low = name.lower()
    for tok, loc in _LEG_LOCATION.items():
        if tok in low:
            return f"{name} ({loc})"
    return name


# --- conversation + state --------------------------------------------------------------------

class TalkTurn(BaseModel):
    """One dialogue turn. ``speaker`` is 'human' or 'sweetie'."""
    speaker: str
    text: str
    emotion: Optional[Emotion] = None


class RobotState(BaseModel):
    """Live robot state used to build the dynamic system-prompt block and answer state tools.

    All fields optional so a stub provider (sim/TUI) can omit what it does not know.
    """
    datetime_iso: Optional[str] = None     # current local date/time, ISO-8601
    weekday: Optional[str] = None
    battery_percent: Optional[float] = None
    battery_status: Optional[str] = None   # charging|discharging|full|unknown
    servo_faults: List[str] = Field(default_factory=list)   # names of failed/overheated servos
    overheated_servos: List[str] = Field(default_factory=list)
    pose: Optional[str] = None             # named body pose, e.g. body_nominal
    moving: Optional[bool] = None
    mood: Optional[str] = None             # current robot mood if known
    extra: Dict[str, Any] = Field(default_factory=dict)

    def human_summary(self) -> str:
        """Compact one-block rendering for the system prompt."""
        parts: List[str] = []
        if self.datetime_iso:
            wd = f" ({self.weekday})" if self.weekday else ""
            parts.append(f"Current time: {self.datetime_iso}{wd}.")
        if self.battery_percent is not None:
            st = f", {self.battery_status}" if self.battery_status else ""
            parts.append(f"Battery: {self.battery_percent:.0f}%{st}.")
        if self.pose:
            mv = " (moving)" if self.moving else ""
            parts.append(f"Body pose: {self.pose}{mv}.")
        if self.servo_faults:
            parts.append(f"Servo faults: {', '.join(friendly_servo(s) for s in self.servo_faults)}.")
        if self.overheated_servos:
            parts.append("Overheated servos: "
                         f"{', '.join(friendly_servo(s) for s in self.overheated_servos)}.")
        if self.mood:
            parts.append(f"Mood: {self.mood}.")
        return " ".join(parts)


# --- scene / environmental awareness ---------------------------------------------------------

class Zone(str, Enum):
    front = "front"
    side = "side"
    rear = "rear"


class SceneEntity(BaseModel):
    """One perceived thing around the robot. People are described by ``id`` only (no names /
    gallery / persistence). Geometry is coarse and word-friendly for the LLM."""
    id: int
    type: str = "person"                   # detector/object type, e.g. person, pony_face
    zone: Zone = Zone.front
    bearing_deg: float = 0.0               # +/- azimuth vs her current forward (right = +)
    elevation_deg: float = 0.0             # +/- pitch vs her horizon (up = +)
    distance: Optional[str] = None         # coarse: near|mid|far
    attributes: Dict[str, str] = Field(default_factory=dict)  # generic, from vision attribute[]/value[]
    is_speaking: bool = False
    is_interlocutor: bool = False
    in_frame: bool = True                  # False -> remembered (out of frame, from retention buffer)
    last_seen_s: float = 0.0               # seconds since last seen (0 while visible)


class SoundCue(BaseModel):
    zone: Zone = Zone.front
    bearing_deg: float = 0.0
    kind: str = "sound"                    # speech|sound
    # SCAFFOLD(intensity): populated by the collector but rendered nowhere yet — reserved for
    # loudness-aware phrasing ("a LOUD noise to your left")
    intensity: Optional[str] = None        # coarse: quiet|normal|loud


class SceneState(BaseModel):
    """A snapshot of what's around her. Rendering + salience live in scene.py (kept ROS-free)."""
    entities: List[SceneEntity] = Field(default_factory=list)
    sounds: List[SoundCue] = Field(default_factory=list)


class SceneEvent(BaseModel):
    """An inter-turn change, computed by diffing consecutive SceneStates."""
    kind: str                              # arrived|left|changed
    entity_id: int
    detail: str = ""


# --- tool calling ----------------------------------------------------------------------------

class ToolCall(BaseModel):
    name: str
    arguments: Dict[str, Any] = Field(default_factory=dict)
    id: Optional[str] = None


class ToolResult(BaseModel):
    name: str
    content: str
    ok: bool = True
    id: Optional[str] = None


# --- request / reply -------------------------------------------------------------------------

class AgentRequest(BaseModel):
    """Everything the agent needs to produce one reply. Plain data — no ROS types.

    ``text_language`` is the language the ``text``/``history`` are actually in (STT may already
    have translated to English). ``reply_language`` is the language SOAR/TTS wants back. The
    agent's *canonical* output language is English; TTS does final localization (see translation).
    """
    request_type: RequestType = RequestType.reply
    profile: str = "complex-en"            # maps to an agent profile (complex/simple/failsafe)
    text: str = ""                         # current human input
    history: List[TalkTurn] = Field(default_factory=list)
    context_facts: List[str] = Field(default_factory=list)  # SOAR predicates / remembered facts
    text_language: str = "en"
    reply_language: str = "en"
    persona: Optional[str] = None          # active persona name; None -> registry default
    labels: List[str] = Field(default_factory=list)   # for request_type=classify
    image_b64: Optional[str] = None        # for request_type=assess_scene (future VLM)


class ReplyContent(BaseModel):
    """The structured-output target for a conversational reply (constrained-decoded as JSON).

    Field order matters: ``response_text`` is first so future streaming can forward spoken text
    while the trailing metadata resolves at completion.
    """
    response_text: str
    emotion: Emotion = Emotion.neutral
    sentence_type: SentenceType = SentenceType.statement


class ErrorCode(int, Enum):
    SUCCESS = 0
    # UNKNOWN_PROFILE / PARSE_ERROR are RESERVED: never emitted on the SOAR reply path — those
    # conditions degrade silently with a log-only warning (an error reply wedges the SOAR say
    # pipeline). Kept for wire-compat and for future non-SOAR callers that can handle errors.
    UNKNOWN_PROFILE = 1
    SERVER_UNREACHABLE = 2
    PARSE_ERROR = 3
    INTERNAL = 4


class AgentReply(BaseModel):
    response_text: str = ""
    emotion: Emotion = Emotion.neutral
    sentence_type: SentenceType = SentenceType.statement
    tool_calls: List[ToolCall] = Field(default_factory=list)
    language: str = "en"                   # language of response_text as returned
    error_code: ErrorCode = ErrorCode.SUCCESS
    error_desc: str = ""
    raw: Optional[str] = None              # raw model content, for observability


# --- structured-output JSON schema helpers ---------------------------------------------------

def _deref(schema: Dict[str, Any]) -> Dict[str, Any]:
    """Inline all ``$ref`` against ``$defs`` and drop ``$defs``.

    Grammar backends (llama.cpp/ollama json-schema-to-grammar) are happiest with a fully
    inlined, self-contained schema. Pydantic v2 emits enums as ``$ref`` into ``$defs``; we
    resolve those (and flatten single-entry ``allOf`` wrappers Pydantic adds for defaults).
    """
    import copy
    defs = schema.get("$defs", {})

    def resolve(node: Any) -> Any:
        if isinstance(node, dict):
            if "$ref" in node:
                name = node["$ref"].split("/")[-1]
                target = resolve(copy.deepcopy(defs.get(name, {})))
                # merge sibling keys (e.g. default/description) onto the resolved target
                for k, v in node.items():
                    if k != "$ref":
                        target[k] = v
                return target
            if "allOf" in node and len(node["allOf"]) == 1 and "type" not in node:
                merged = resolve(copy.deepcopy(node["allOf"][0]))
                for k, v in node.items():
                    if k != "allOf":
                        merged[k] = v
                return merged
            return {k: resolve(v) for k, v in node.items() if k != "$defs"}
        if isinstance(node, list):
            return [resolve(v) for v in node]
        return node

    out = resolve(schema)
    out.pop("$defs", None)
    return out


def _strict(schema: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively force additionalProperties=false and required=all-keys on object schemas.

    Makes the schema strict enough for grammar-constrained decoding (llama.cpp / ollama).
    """
    if schema.get("type") == "object" and "properties" in schema:
        schema["additionalProperties"] = False
        schema["required"] = list(schema["properties"].keys())
        for sub in schema["properties"].values():
            _strict(sub)
    for key in ("items", "$defs", "definitions"):
        node = schema.get(key)
        if isinstance(node, dict):
            if key in ("$defs", "definitions"):
                for v in node.values():
                    _strict(v)
            else:
                _strict(node)
    for comb in ("anyOf", "oneOf", "allOf"):
        if comb in schema:
            for v in schema[comb]:
                _strict(v)
    return schema


def reply_json_schema() -> Dict[str, Any]:
    """JSON schema for ReplyContent, inlined + strict, ready for response_format=json_schema."""
    return _strict(_deref(ReplyContent.model_json_schema()))


def classify_json_schema(labels: List[str]) -> Dict[str, Any]:
    """Strict schema for a classification reply: a single label from the allowed set."""
    return {
        "type": "object",
        "properties": {"label": {"type": "string", "enum": list(labels)}},
        "required": ["label"],
        "additionalProperties": False,
    }
