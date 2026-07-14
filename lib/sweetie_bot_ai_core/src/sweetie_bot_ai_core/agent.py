"""The agent orchestrator. ROS-free; the de-ROSification seam.

Inbound seam:  ``Agent.handle(AgentRequest) -> AgentReply`` (plain data in/out).
Outbound seams (injected Protocols):
  * ``RobotStateProvider.snapshot() -> RobotState``  (ROS impl reads topics; sim/TUI = stub)
  * ``EffectorPort.dispatch(ToolCall) -> ToolResult`` (ROS impl maps to action servers)

Reply strategy (robust with local models, avoids the tools×grammar coexistence rough edge):
  1. Optional NATIVE tool loop: offer tools; execute info/'execute'-mode tools, record 'propose'
     tools without running them (SOAR stays the actuator arbiter).
  2. Final CONSTRAINED call: response_format=json_schema -> {response_text, emotion, sentence_type}.
When tools are disabled for the profile/persona, step 1 is skipped → a single constrained call.
"""
from __future__ import annotations

import json
import re
import uuid
from collections import deque
from typing import Callable, Dict, List, Optional, Protocol, Tuple

from .history import ConversationHistory
from .persona import PersonaRegistry
from .profiles import DEFAULT_PROFILES, ProfileConfig, load_profiles  # noqa: F401 - re-export
from .prompt import build_system_prompt, build_tool_phase_prompt
from .registry import ProviderRegistry, RegistryError
from .scene import (SceneConfig, _is_person, diff as scene_diff, is_occluded, render_scene,
                    select_salient)
from .schema import (AgentReply, AgentRequest, ErrorCode, Emotion, ReplyContent, RequestType,
                     RobotState, SceneState, SentenceType, ToolCall, ToolResult,
                     classify_json_schema, reply_json_schema)
from .similarity import (ANSWERED_MIN_LEN, ANSWERED_RATIO, REPEAT_MIN_LEN, REPEAT_RATIO,
                         is_near_duplicate, normalize)
from .tools import DispatchMode, ToolRegistry
from .translation import detect_language


# anti-repeat regenerate: one diverging retry, temperature bumped over the profile base, capped
REGEN_TEMP_BUMP = 0.3
REGEN_TEMP_CAP = 1.2
# per-path base-temperature fallbacks used ONLY when a profile omits 'temperature'; they differ
# by design — self-talk runs slightly warmer so repeated cues still produce varied asides
REPLY_BASE_TEMP = 0.8
SELF_TALK_BASE_TEMP = 0.9
# rolling windows for the two repeat guards
RECENT_REPLIES_WINDOW = 6      # her own replies (loop breaker)
ANSWERED_TURNS_WINDOW = 8      # human turns already answered (re-poke guard)
# observability: single-line scene_block log truncation
SCENE_LOG_MAX_CHARS = 600


class TurnAborted(Exception):
    """The caller superseded this turn (e.g. a newer actionlib goal preempted it, or a real
    turn arrived during a proactive aside): stop at the next LLM call boundary instead of
    running the turn's remaining calls — on the single-flight GPU every wasted call
    head-of-line blocks the goal that replaced us (2026-07-13 14b fluency diagnosis: superseded
    rephrase goals finished all their calls, stalling the successor past the caller's ~3s
    literal-text fallback). Deliberately NOT mapped to an error reply: an aborted turn is not
    a failure, its result would be discarded — ``handle()`` re-raises it so the caller can
    preempt cleanly (the never-empty-reply rule guards SUCCEEDED results, not superseded goals)."""


_NO_REPEAT_NOTE = (
    "You already said almost exactly this a moment ago. Do NOT repeat yourself — say "
    "something clearly different, with fresh wording, and move the conversation forward."
)

_ALREADY_ANSWERED_NOTE = (
    "NOTE: You have ALREADY responded to what they last said, and they have not added anything "
    "new since. Do not answer it again or restate your earlier point. Instead say something "
    "brief and natural to keep things easy — a light aside, a small follow-up question, or a "
    "gentle change of subject. Keep it short; never re-explain what you already told them."
)

_LULL_NOTE = (
    "NOTE: The human has said nothing just now — this is a quiet pause, not a question to "
    "answer. Say ONE short, natural remark in your own voice that keeps the moment warm or "
    "gently moves the conversation along."
)

_RETRY_SENTENCE_NOTE = (
    "Your previous draft was not a usable utterance. Reply again with one short, natural "
    "sentence in your own voice."
)

# hand-question guard: when asked what they are holding and NO held object is in the rendered
# scene, deliver the missing datum as CONTEXT (the lull/re-poke note doctrine) — absence of a
# 'holding' percept renders nothing about hands, and the 7B fills the void with an invented
# object (live 2026-07-08 23:06: "a small book"). Epistemic wording only: absence of detection
# is NOT empty hands (vision drops low-confidence holds), so the note says what she SEES,
# never "their hands are empty". No example objects anywhere (negation blindness: examples
# get parroted). Wording contract: must share no 5-word shingle with the natural honest reply
# ("I don't see anything in your hands right now") — _echoes_note guards this note too.
_HAND_UNSEEN_NOTE = (
    "NOTE: Right now your vision shows nothing in the human's hands — though a small "
    "thing could escape your eyes, so this is only what you SEE, not certainty. Answer "
    "from what you actually see; never guess or make up an object."
)

# lead word + same-sentence hand phrase (the [^.?!] gap keeps them in one clause). RU patterns
# are the backstop for the raw request text and the translate-service-down degrade path — RU
# is normally pivoted to EN before the model (languages.yaml: ru not native).
_HAND_Q_EN_RX = re.compile(
    r"\b(?:what|guess|tell me|do you know|can you (?:tell|see)|do you see)\b"
    r"[^.?!]{0,40}?"
    r"(?:\bin my hands?\b|\b(?:am i|i ?am|i'm) holding\b"
    r"|\bdo i (?:have|hold)\b[^.?!]{0,20}?\bhands?\b)",
    re.IGNORECASE)

_HAND_Q_RU_RX = re.compile(
    r"(?:\bчто\b|\bугадай\b|\bскажи\b|\bзнаешь\b|\bвидишь\b)"
    r"[^.?!]{0,40}?"
    r"(?:\bу меня в рук(?:е|ах)\b|\bв рук(?:е|ах) у меня\b|\bя держу\b|\bдержу я\b)",
    re.IGNORECASE)


def _asks_whats_in_hand(*texts) -> bool:
    """Human hand/holding question, EN + RU, over any of the given text variants (the reply
    path probes both the raw request text and the EN pivot)."""
    return any(t and (_HAND_Q_EN_RX.search(t) or _HAND_Q_RU_RX.search(t)) for t in texts)


def _sees_held_object(scene) -> bool:
    """True when the rendered scene AFFIRMATIVELY shows a held object: a visible person with
    a 'holding' attr, or any in-frame entity marked held_by. In that case the unseen-hand
    note would contradict what the prompt already says — skip it."""
    for e in scene.entities:
        if not e.in_frame:
            continue
        if _is_person(e) and "holding" in e.attributes:
            return True
        if "held_by" in e.attributes:
            return True
    return False

# position-recital guard: the scene block gives a LONE person no position (they are "you" —
# scene.py locate_people), yet the model still narrates one from thin air ("You're right in
# front of me." — 14B consistent 3/3, 2026-07-14; the 7B flaked the same ~1-in-2). There is
# no data to remove at the source (pure inference, not a leak), so the backstop is post-decode
# and DETERMINISTIC: sentences claiming a position the prompt never carried are stripped
# (keeps the good part of the reply, no GPU); a fully-recital reply regenerates once with the
# missing-datum note, then lands on the fixed fallback. Position talk stays legal when the
# rendered scene_block itself carried it (crowds, located objects) — the gate reads the block,
# the single source of truth for what she perceived. Scene jargon (interlocutor, raw ids) is
# banned unconditionally: the prompt forbids speaking the id even when positions render.
_POSITION_RECITAL_RX = re.compile(
    r"\bin front\b"
    r"|\b(?:to|on|at) (?:my|your) (?:left|right)\b"
    r"|\b(?:left|right) of (?:me|you)\b",
    re.IGNORECASE)

_SCENE_JARGON_RX = re.compile(r"\binterlocutors?\b|\(\s*id\b", re.IGNORECASE)

# regenerate-leg note: same doctrine as _HAND_UNSEEN_NOTE (deliver the missing datum as
# context, no examples — negation blindness). Wording contract, both pinned by tests: must
# not match the guard regexes itself, and must share no 5-word shingle with a natural reply
# (_echoes_note guards this note too).
_NO_POSITION_NOTE = (
    "NOTE: Your scene awareness holds no placement details for anyone right now. "
    "Speak to people warmly and directly, without describing where anyone is."
)


def _recites_scene(text: str, *, scene_has_position: bool) -> bool:
    """True when a reply narrates scene internals she must not voice: scene jargon (always),
    or a spatial position when the rendered scene carried none — position talk is legitimate
    ONLY as recall of rendered data."""
    if _SCENE_JARGON_RX.search(text or ""):
        return True
    return not scene_has_position and bool(_POSITION_RECITAL_RX.search(text or ""))


_SENTENCE_SPLIT_RX = re.compile(r"(?<=[.!?…])\s+")


def _strip_recital(text: str, *, scene_has_position: bool) -> str:
    """Deterministically drop the reciting sentences, keeping the rest of the reply — the
    observed failure shape is a good answer plus a trailing recital sentence, and stripping
    rescues the answer identically on any model size."""
    kept = [s for s in _SENTENCE_SPLIT_RX.split(text or "")
            if s and not _recites_scene(s, scene_has_position=scene_has_position)]
    return " ".join(kept).strip()


# last resort when even the regenerate decodes to junk: the reply path must never return
# empty text (it wedges the SOAR say pipeline — lang.py lesson)
_DEGENERATE_FALLBACK = "Hmm... I just lost my train of thought."

# self-talk emotion whitelist: with a CLEAR lens a spontaneous muse must stay friendly —
# the 7B occasionally self-tags [anger] on a neutral cue (live T.1#1, 2026-07-09). anger
# stays reachable ONLY via the occlusion override below; fear has no self-talk use at all.
_FRIENDLY_SELF_TALK_EMOTIONS = frozenset({
    Emotion.neutral, Emotion.joy, Emotion.love, Emotion.sadness, Emotion.surprise})

# forbidden vision-only inference (user rule 2026-07-08): a muse must never claim the place
# is empty/quiet — out of frame does not mean absent, her venues are loud crowded halls.
# The model re-infers this even with rewritten cues, so the guard is a deterministic
# post-decode DROP (empty text = valid self-talk silence). Never re-instruct instead:
# injected corrections get parroted by the 7B (the live prompt-echo defect, S.4#4).
# \b keeps derived forms like "quietly" (the lull-pool humming cue) passable.
_SILENCE_INFERENCE_RX = re.compile(
    r"\b(?:quiet|silen(?:ce|t)|alone|no[- ]?one|nobody|empty room|by myself)\b",
    re.IGNORECASE)


# any verbatim run of this many words from a steer/context note marks a reply as a prompt echo
_NOTE_SHINGLE_WORDS = 5

# notes whose verbatim echo in a decoded reply must never be voiced
_ECHO_GUARDED_NOTES = (_NO_REPEAT_NOTE, _HAND_UNSEEN_NOTE, _NO_POSITION_NOTE)


def _echoes_note(text: str) -> bool:
    """True when a decoded reply parrots an injected steer/context note (any verbatim 5-word
    run, punctuation-insensitive). An echoed INSTRUCTION near-duplicates no recent REPLY, so
    the repeat verify alone lets it through — live 2026-07-08 23:28 the anti-repeat note was
    voiced word-for-word. Shingles come from the notes themselves: rewording a note cannot
    desync this guard."""
    def _words(s):
        return re.sub(r"[^a-z0-9]+", " ", (s or "").lower()).split()
    w = _words(text)
    hay = {" ".join(w[i:i + _NOTE_SHINGLE_WORDS])
           for i in range(len(w) - _NOTE_SHINGLE_WORDS + 1)}
    return any(
        " ".join(n[i:i + _NOTE_SHINGLE_WORDS]) in hay
        for note in _ECHO_GUARDED_NOTES
        for n in (_words(note),)
        for i in range(len(n) - _NOTE_SHINGLE_WORDS + 1))


def _is_degenerate(text: str, *, lull: bool) -> bool:
    """Decode-junk detector for the reply path. Empty text is junk on any turn; on a LULL
    goal (SOAR poked with ``text=''``) a bare single word is junk too — live, lull goals
    decoded to label echoes ('joy', 'you') and got voiced. On a real turn a single word
    ("Yes!") is a legitimate answer and stays untouched."""
    words = re.findall(r"\w+", text or "")
    if not words:
        return True
    return lull and len(words) == 1


class RobotStateProvider(Protocol):
    def snapshot(self) -> RobotState: ...


class EffectorPort(Protocol):
    def dispatch(self, tool_call: ToolCall) -> ToolResult: ...


class SceneProvider(Protocol):
    def snapshot(self, include_remembered: bool = False) -> SceneState: ...


class NullStateProvider:
    def snapshot(self) -> RobotState:
        return RobotState()


class NullSceneProvider:
    def snapshot(self, include_remembered: bool = False) -> SceneState:
        return SceneState()


class NullEffector:
    def dispatch(self, tool_call: ToolCall) -> ToolResult:
        return ToolResult(name=tool_call.name, content="(no effector available)", ok=False,
                          id=tool_call.id)


# ProfileConfig / DEFAULT_PROFILES / load_profiles moved to profiles.py (canonical names are
# now language-neutral); re-exported above for import compatibility.

# request names may carry a legacy language suffix (SOAR sends 'complex-en' etc.) — resolution
# strips one trailing "-<2-letter-lang>" when the exact name is unknown
_PROFILE_LANG_SUFFIX_RX = re.compile(r"-[a-z]{2}$")


class Agent:
    def __init__(self, registry: ProviderRegistry, *,
                 personas: Optional[PersonaRegistry] = None,
                 tools: Optional[ToolRegistry] = None,
                 state_provider: Optional[RobotStateProvider] = None,
                 scene_provider: Optional[SceneProvider] = None,
                 effector: Optional[EffectorPort] = None,
                 language_policy=None,
                 profiles: Optional[Dict[str, ProfileConfig]] = None,
                 scene_config: Optional[SceneConfig] = None,
                 logger: Optional[Callable[[str], None]] = None):
        self.registry = registry
        self.personas = personas or PersonaRegistry()
        self.tools = tools or ToolRegistry()
        self.state_provider = state_provider or NullStateProvider()
        self.scene_provider = scene_provider or NullSceneProvider()
        self.effector = effector or NullEffector()
        self.policy = language_policy
        self.profiles = profiles or DEFAULT_PROFILES
        self.scene_config = scene_config or SceneConfig()
        self._prev_scene = SceneState()
        self._recent_replies = deque(maxlen=RECENT_REPLIES_WINDOW)
        self._answered_turns = deque(maxlen=ANSWERED_TURNS_WINDOW)
        self._abort_check = None   # per-turn; set by handle(), read by _chat()
        self.log = logger or (lambda m: None)

    # -- public entry -------------------------------------------------------------------------

    def reset_ambient(self) -> None:
        """Forget the inter-turn ambient state (scene diff memory). Test/reset seam."""
        self._prev_scene = SceneState()
        self._recent_replies.clear()
        self._answered_turns.clear()

    @staticmethod
    def _norm_turn(text) -> str:
        return normalize(text)

    def _already_answered(self, text) -> bool:
        """True if this human turn was already answered and nothing new was said since — a lull
        re-poke (SOAR re-emits the last speech event on a pause). History is context only: she
        must not answer the same turn again. Short turns ('yes','ok') are allowed to recur."""
        return any(is_near_duplicate(text, prev, min_len=ANSWERED_MIN_LEN, ratio=ANSWERED_RATIO)
                   for prev in self._answered_turns)

    def _is_repeat(self, text, extra=()) -> bool:
        """True if `text` (near-)duplicates a recent reply — she is looping. Pure/testable.
        Short affirmations ("yes", "okay") are allowed to recur; only longer lines count.
        ``extra``: further known-said lines (her own turns from the request history) — after
        an agent restart the in-memory window is empty while SOAR still carries them.
        ``contains``: a reply that carries a recent line verbatim inside new glue is the same
        loop (live 2026-07-08: intro + previous reply glued, ratio-only let it through)."""
        return any(is_near_duplicate(text, prev, min_len=REPEAT_MIN_LEN, ratio=REPEAT_RATIO,
                                     contains=True)
                   for prev in (*self._recent_replies, *extra))

    def handle(self, request: AgentRequest,
               abort_check: Optional[Callable[[], bool]] = None) -> AgentReply:
        """``abort_check``: polled before every LLM call; True => raise :class:`TurnAborted`
        (single-flight callers wire it to "a newer goal superseded this one"). Single-flight
        by contract — the instance attribute is safe because callers already serialise
        handle() (the node's _agent_lock)."""
        self._abort_check = abort_check
        try:
            if request.request_type == RequestType.classify:
                return self._handle_classify(request)
            if request.request_type == RequestType.rephrase:
                return self._handle_rephrase(request)
            if request.request_type == RequestType.self_talk:
                return self._handle_self_talk(request)
            if request.request_type == RequestType.assess_scene:
                return self._handle_assess_scene(request)
            return self._handle_reply(request)
        except TurnAborted:
            self.log("turn aborted (superseded) — stopped at an LLM call boundary")
            raise
        except RegistryError as e:
            self.log(f"registry error: {e}")
            return AgentReply(error_code=ErrorCode.SERVER_UNREACHABLE, error_desc=str(e))
        except Exception as e:  # noqa: BLE001 - never crash the caller
            self.log(f"agent internal error: {e!r}")
            return AgentReply(error_code=ErrorCode.INTERNAL, error_desc=repr(e))
        finally:
            self._abort_check = None

    # -- shared helpers (per-path degrade semantics stay with each caller) ----------------------

    def _chat(self, *args, **kwargs):
        """The single LLM choke point — every model call goes through here so a superseded
        turn stops before its next call instead of finishing 2-4 doomed ones."""
        if self._abort_check is not None and self._abort_check():
            raise TurnAborted()
        return self.registry.chat(*args, **kwargs)

    def _profile(self, name: str, fallback: str) -> ProfileConfig:
        """Resolve a profile name: exact -> strip a trailing '-<lang>' suffix (SOAR request
        names like 'complex-en' stay valid against the language-neutral canonical names) ->
        named fallback with a log-only warning (never an error to SOAR — wedge safety, C3)."""
        profile = self.profiles.get(name)
        if profile is not None:
            return profile
        stripped = _PROFILE_LANG_SUFFIX_RX.sub("", name or "")
        profile = self.profiles.get(stripped)
        if profile is not None:
            return profile
        self.log(f"unknown profile {name!r}; falling back to {fallback!r}")
        return self.profiles.get(fallback) or DEFAULT_PROFILES[fallback]

    def _scene_snapshot(self) -> SceneState:
        """Scene snapshot INCLUDING the short-term retention buffer. The TypeError fallback
        covers providers without the ``include_remembered`` kwarg (NullSceneProvider, older
        sims) — do not remove it, duck-typed providers are part of the seam contract."""
        try:
            return self.scene_provider.snapshot(include_remembered=True)
        except TypeError:
            return self.scene_provider.snapshot()

    def _regenerate_if_repeat(self, messages, profile, text, emotion, sentence_type, *,
                              base_temp_default: float, keep_first_if_empty: bool,
                              extra_recent=()
                              ) -> Tuple[str, Emotion, SentenceType, Optional[str]]:
        """Anti-repetition: if the decoded reply near-duplicates a recent line she is looping
        (SOAR can re-propose a talk op on near-identical event windows). Regenerate ONCE with a
        diverging steer note and a bumped temperature. NEVER raises (except TurnAborted —
        a superseded turn stops here like everywhere else) — on a failed retry, or a
        retry that parrots the steer note itself (``_echoes_note``), the first reply is kept
        (empty/error would wedge the SOAR say pipeline, lang.py lesson); the reply path's
        post-verify then lands on the fallback line instead of voicing the loop or the note.

        ``keep_first_if_empty``: the self-talk path keeps its first aside when the retry decodes
        to silence (and strips the retry text); the reply path takes the retry verbatim.
        Returns (text, emotion, sentence_type, raw_retry_content_or_None).
        """
        if not (text and self._is_repeat(text, extra=extra_recent)):
            return text, emotion, sentence_type, None
        self.log(f"repeat detected — regenerating: {text!r:.80}")
        try:
            opts = {**profile.options,
                    "temperature": min(REGEN_TEMP_CAP,
                                       float(profile.options.get("temperature",
                                                                 base_temp_default))
                                       + REGEN_TEMP_BUMP)}
            result, _ = self._chat(
                messages + [{"role": "system", "content": _NO_REPEAT_NOTE}],
                response_schema=reply_json_schema(), **opts)
            second = ReplyContent.model_validate(self._safe_json(result.content))
            if _echoes_note(second.response_text or ""):
                self.log("anti-repeat retry echoed the steer note — discarding: "
                         f"{second.response_text!r:.80}")
                return text, emotion, sentence_type, None
            if keep_first_if_empty:
                s_text = (second.response_text or "").strip()
                if not s_text:
                    return text, emotion, sentence_type, None
                return s_text, second.emotion, second.sentence_type, result.content
            return second.response_text, second.emotion, second.sentence_type, result.content
        except TurnAborted:   # superseded turn: the result is discarded anyway — stop now
            raise
        except Exception as e:  # noqa: BLE001 - keep the first reply if the retry fails
            self.log(f"anti-repeat retry failed: {e!r}")
            return text, emotion, sentence_type, None

    def _retry_degenerate(self, messages, profile, *, lull: bool
                          ) -> Tuple[str, Emotion, SentenceType, Optional[str]]:
        """The decoded reply is junk (see ``_is_degenerate``): regenerate ONCE with a
        corrective note; if the retry is junk too, return the fixed fallback line — the
        reply path must never return empty (lang.py lesson). NEVER raises (except
        TurnAborted — a superseded turn stops here like everywhere else)."""
        self.log("degenerate reply — regenerating once")
        try:
            opts = {**profile.options,
                    "temperature": min(REGEN_TEMP_CAP,
                                       float(profile.options.get("temperature",
                                                                 REPLY_BASE_TEMP))
                                       + REGEN_TEMP_BUMP)}
            result, _ = self._chat(
                messages + [{"role": "system", "content": _RETRY_SENTENCE_NOTE}],
                response_schema=reply_json_schema(), **opts)
            second = ReplyContent.model_validate(self._safe_json(result.content))
            s_text = (second.response_text or "").strip()
            if not _is_degenerate(s_text, lull=lull):
                return s_text, second.emotion, second.sentence_type, result.content
        except TurnAborted:   # superseded turn: the result is discarded anyway — stop now
            raise
        except Exception as e:  # noqa: BLE001 - junk retry: fall through to the fallback line
            self.log(f"degenerate-reply retry failed: {e!r}")
        return _DEGENERATE_FALLBACK, Emotion.neutral, SentenceType.statement, None

    def _retry_recital(self, messages, profile, *, lull: bool, scene_has_position: bool
                       ) -> Tuple[str, Emotion, SentenceType, Optional[str]]:
        """The whole reply was a scene recital (stripping left nothing usable): regenerate
        ONCE with the missing-datum note; a retry that still recites, parrots the note, or
        decodes to junk lands on the fixed fallback line — the recital is never voiced and
        the reply path never returns empty. NEVER raises (except TurnAborted — a superseded
        turn stops here like everywhere else)."""
        self.log("position guard: reply is all recital — regenerating once")
        try:
            opts = {**profile.options,
                    "temperature": min(REGEN_TEMP_CAP,
                                       float(profile.options.get("temperature",
                                                                 REPLY_BASE_TEMP))
                                       + REGEN_TEMP_BUMP)}
            result, _ = self._chat(
                messages + [{"role": "system", "content": _NO_POSITION_NOTE}],
                response_schema=reply_json_schema(), **opts)
            second = ReplyContent.model_validate(self._safe_json(result.content))
            s_text = (second.response_text or "").strip()
            if not _is_degenerate(s_text, lull=lull) \
                    and not _recites_scene(s_text, scene_has_position=scene_has_position) \
                    and not _echoes_note(s_text):
                return s_text, second.emotion, second.sentence_type, result.content
            self.log(f"position guard: retry still recites — falling back: {s_text!r:.60}")
        except TurnAborted:   # superseded turn: the result is discarded anyway — stop now
            raise
        except Exception as e:  # noqa: BLE001 - junk retry: fall through to the fallback line
            self.log(f"position-guard retry failed: {e!r}")
        return _DEGENERATE_FALLBACK, Emotion.neutral, SentenceType.statement, None

    def _decode_reply_or(self, content_str, fallback_text: str
                         ) -> Tuple[str, Emotion, SentenceType]:
        """Decode a constrained structured reply, degrading to ``fallback_text`` on a bad
        decode or empty text (rephrase: the canned line; self-talk: silence — both are valid
        outcomes for their paths, unlike the reply path which lets a bad decode raise)."""
        try:
            content = ReplyContent.model_validate(self._safe_json(content_str))
            text = (content.response_text or "").strip()
            emotion, sentence_type = content.emotion, content.sentence_type
        except Exception:  # noqa: BLE001 - bad structured decode: use the path's fallback
            self.log(f"structured decode failed; degrading to fallback {fallback_text!r:.60}")
            text, emotion, sentence_type = "", Emotion.neutral, SentenceType.statement
        if not text:
            text = fallback_text
        return text, emotion, sentence_type

    # -- reply path ---------------------------------------------------------------------------

    def _handle_reply(self, request: AgentRequest) -> AgentReply:
        persona = self.personas.get(request.persona)
        profile = self._profile(request.profile, "complex")
        state = self.state_provider.snapshot()

        # detect the actual input language (the adapter's text_language is a static config
        # value); non-native input is translated to the pivot, and the reply back (see below)
        user_lang = detect_language(request.text, request.text_language)
        user_text = request.text
        if self.policy is not None:
            user_text = self.policy.to_model_language(user_text, user_lang)

        tools_offered = (persona.allow_tools and profile.allow_tools
                         and len(self.tools.offered()) > 0)

        # ambient environmental awareness: front-weighted scene + inter-turn delta.
        # include the short-term retention buffer so recently-departed objects stay in her
        # ambient awareness ("where did the pony go") without requiring a tool call.
        scene = select_salient(self._scene_snapshot(), self.scene_config)
        events = scene_diff(self._prev_scene, scene, self.scene_config)
        self._prev_scene = scene
        scene_block = render_scene(scene, events, self.scene_config)
        # a blocked camera is an unambiguous, non-conversational irritation: force the angry
        # animation deterministically rather than trusting the LLM to pick it (the WARNING banner
        # in the prompt drives the verbal complaint; SOAR maps emotion=anger -> evil_look eyes).
        occluded = is_occluded(scene)
        if scene_block:
            # single-line observability: the behavior-synth harness (and live debugging) asserts
            # on what perception actually reached the prompt
            self.log("scene_block: " + scene_block.replace(chr(10), " | ")[:SCENE_LOG_MAX_CHARS])

        language_note = None
        if user_lang != "en":
            # canonical-EN output contract: the VOICE node localizes every non-EN say
            # (translate en->lang with gender hints). The model must answer EN even when it
            # consumed the input natively (zh/ja) or its mirrored reply would be translated
            # AGAIN from source='en' and mangled (P25).
            language_note = ("Always answer in English regardless of the language of the "
                             "conversation; your reply is translated for the user downstream.")
        system_prompt = build_system_prompt(persona, state, tools_offered=tools_offered,
                                            scene_block=scene_block, language_note=language_note)
        if request.context_facts:
            system_prompt += ("\n\nRelevant things you remember right now:\n"
                              + "\n".join(f"- {f}" for f in request.context_facts))
        hist = ConversationHistory(max_verbatim_turns=profile.max_verbatim_turns)
        messages = hist.build_messages(system_prompt, request.history, user_text)

        # re-poke guard: if this exact human turn was already answered and nothing new was said,
        # a lull re-emitted it (SOAR re-proposes the last speech event on a pause). Don't answer
        # it again — steer her to move the conversation along instead. History is context only.
        re_poke = self._already_answered(user_text)

        proposed: List[ToolCall] = []
        if tools_offered:
            # the tool-DECISION call runs on a minimal prompt (identity + tool note): under the
            # full persona prompt a 7B loses the tool schema in the style guidelines and stops
            # emitting native tool calls (see build_tool_phase_prompt). The loop's tail —
            # assistant tool-call turns + tool results — is grafted onto the full-prompt
            # messages so the constrained reply is composed WITH the tool outcomes in context.
            tool_msgs = hist.build_messages(build_tool_phase_prompt(persona),
                                            request.history, user_text)
            n_before = len(tool_msgs)
            tool_msgs, proposed = self._tool_loop(tool_msgs, profile)
            messages = messages + tool_msgs[n_before:]
        # empty-text lull goal (SOAR pokes on a pause with no talk-heard event): without a
        # user turn build_messages leaves the model continuing after its OWN last line — an
        # undefined task that occasionally decodes to label junk. Give it a defined one.
        lull = not (request.text or "").strip()
        if lull:
            self.log("lull goal (empty text) — steering to a short natural remark")
            messages = messages + [{"role": "system", "content": _LULL_NOTE}]
        elif re_poke:
            self.log("re-poke: already answered this turn — steering to move on")
            messages = messages + [{"role": "system", "content": _ALREADY_ANSWERED_NOTE}]
        # unseen-hand guard: asked what they hold while the scene shows no held object —
        # deliver the missing datum (see _HAND_UNSEEN_NOTE). Composes with re-poke ("move
        # on" must not confabulate either); last position = strongest recency for the 7B.
        if not lull and _asks_whats_in_hand(request.text, user_text) \
                and not _sees_held_object(scene):
            self.log("hand guard: no held object in scene — injecting unseen-hand note")
            messages = messages + [{"role": "system", "content": _HAND_UNSEEN_NOTE}]

        # Final constrained structured reply. A bad decode RAISES here (handled in handle() ->
        # INTERNAL): unlike rephrase/self-talk this path has no safe local fallback text.
        result, _ = self._chat(messages, response_schema=reply_json_schema(),
                                        **profile.options)
        content = ReplyContent.model_validate(self._safe_json(result.content))
        # her own turns as SOAR delivered them: after an agent restart/reset the in-memory
        # _recent_replies window is empty while the history still carries what she said
        said_before = tuple(t.text for t in request.history if t.speaker == "sweetie")[-3:]
        text, emotion_c, sentence_type, retry_raw = self._regenerate_if_repeat(
            messages, profile, content.response_text, content.emotion, content.sentence_type,
            base_temp_default=REPLY_BASE_TEMP, keep_first_if_empty=False,
            extra_recent=said_before)
        if text and self._is_repeat(text, extra=said_before):
            # the regenerate itself re-echoed (a small model latches onto the copy source in
            # its prompt — the note alone loses 3/5): never voice a verbatim loop
            self.log(f"retry still a repeat — falling back: {text!r:.60}")
            text, emotion_c, sentence_type, retry_raw = (
                _DEGENERATE_FALLBACK, Emotion.neutral, SentenceType.statement, None)
        if _is_degenerate(text or "", lull=lull):
            text, emotion_c, sentence_type, retry_raw = self._retry_degenerate(
                messages, profile, lull=lull)
        # position-recital guard (deterministic, model-agnostic): never voice a position
        # the prompt did not carry, nor scene jargon ever. Strip first — the observed failure
        # shape leaves a natural reply — and regenerate only when stripping guts the reply.
        scene_has_position = bool(_POSITION_RECITAL_RX.search(scene_block or ""))
        if text and _recites_scene(text, scene_has_position=scene_has_position):
            stripped = _strip_recital(text, scene_has_position=scene_has_position)
            if not _is_degenerate(stripped, lull=lull):
                self.log(f"position guard: recital stripped: {text!r:.80}")
                text = stripped
            else:
                text, emotion_c, sentence_type, retry_raw = self._retry_recital(
                    messages, profile, lull=lull, scene_has_position=scene_has_position)
        if text and text.strip():
            self._recent_replies.append(text.strip())
        # remember we answered this human turn, so a lull re-poke of the same text moves on
        # rather than answering it a second time.
        ans = self._norm_turn(user_text)
        if len(ans) >= ANSWERED_MIN_LEN:
            self._answered_turns.append(ans)
        # canonical-EN output: NO agent-side back-translation - the voice node owns
        # localization (it translates every non-EN say from en; agent-side RU output was
        # double-translated and mangled, P25)
        emotion = Emotion.anger if occluded else emotion_c
        return AgentReply(
            response_text=text,
            emotion=emotion,
            sentence_type=sentence_type,
            tool_calls=proposed,
            language="en",                 # canonical; voice/say/<lang> localizes downstream
            error_code=ErrorCode.SUCCESS,
            raw=retry_raw if retry_raw is not None else result.content,
        )

    # -- rephrase path (constrained rewording of a scripted line) -----------------------------

    def _handle_rephrase(self, request: AgentRequest) -> AgentReply:
        """Reword ONE scripted line in Sweetie's voice - and nothing more.

        Unlike the reply path this deliberately does NOT inject the ambient scene, live state or
        conversational latitude: those were what pulled rephrases off-topic ("she talks about
        unrelated things when scratched"). It also leaves ``self._prev_scene`` untouched, so a
        rephrase never consumes the scene-diff owed to the next real conversational turn.
        """
        line = (request.text or "").strip()
        if not line:
            return AgentReply(response_text="", language="en", error_code=ErrorCode.SUCCESS)
        persona = self.personas.get(request.persona)
        profile = self._profile(request.profile, "rephrase")
        name = persona.display_name if persona else "Sweetie Bot"
        system_prompt = (
            f"You are {name}. You are about to say ONE line out loud - say the SAME thing, but "
            "reinvent it in fresh, lively words that sound like you. Do NOT echo the line: put "
            "your own playful, expressive personality into it and vary the wording boldly.\n"
            "Rules:\n"
            "- Keep the same subject, intent and feeling as the given line. Reword it as freely "
            "and creatively as you like, but stay on THAT one thing - do not wander onto new "
            "topics, facts, questions or greetings, and do not react to your surroundings.\n"
            "- Do NOT answer or continue a conversation; this is just a livelier way to say that "
            "one line.\n"
            "- Sweetie's only touch sensors are on her face (nose, forehead, cheeks, temples); "
            "never mention being touched, scratched or petted anywhere else - no ears, horn, "
            "back or body - keep the spot vague like 'right here' if the line is not about a "
            "face spot.\n"
            "- One short, natural spoken sentence, in the same language as the line."
        )
        user_text = ('Say this line again in your own fresh, characterful words - same meaning '
                     'and subject, boldly new phrasing:\n"%s"' % line)
        hist = ConversationHistory(max_verbatim_turns=0)
        messages = hist.build_messages(system_prompt, [], user_text)
        result, _ = self._chat(messages, response_schema=reply_json_schema(),
                                        **profile.options)
        # degrade to the canned line, never to silence
        text, emotion, sentence_type = self._decode_reply_or(result.content, line)
        return AgentReply(
            response_text=text,
            emotion=emotion,
            sentence_type=sentence_type,
            language="en",
            error_code=ErrorCode.SUCCESS,
            raw=result.content,
        )

    # -- self-talk path (spontaneous in-character remark from a cue) ---------------------------

    def _handle_self_talk(self, request: AgentRequest) -> AgentReply:
        """Say ONE brief, spontaneous thought out loud, prompted by a cue - not a conversation.

        This is the seam a future proactive layer drives (a pony noticed, a lull, an event): the
        caller passes what caught her attention as ``request.text`` and gets back a short remark.
        Unlike the reply path it demands no answer and continues no dialogue; unlike the rephrase
        path it IS scene-aware (so she can react to what she sees), but it snapshots the scene
        READ-ONLY and never touches ``self._prev_scene`` - a spontaneous aside must not consume
        the scene-diff owed to the next real conversational turn. Producing nothing is fine:
        silence is a valid spontaneous "remark", so a bad decode degrades to no speech.
        """
        cue = (request.text or "").strip()
        if not cue:
            return AgentReply(response_text="", language="en", error_code=ErrorCode.SUCCESS)
        persona = self.personas.get(request.persona)
        profile = self._profile(request.profile, "self-talk")
        name = persona.display_name if persona else "Sweetie Bot"
        # read-only scene snapshot (no diff, no _prev_scene write): lets her name what she sees
        scene_sel = select_salient(self._scene_snapshot(), self.scene_config)
        scene_block = render_scene(scene_sel, [], self.scene_config)
        system_prompt = (
            f"You are {name}. Something around you just caught your attention and you feel like "
            "saying one brief thought out loud - to yourself, not to anyone in particular.\n"
            "Strict rules:\n"
            "- React only to what caught your attention (and what you can see); one short, "
            "natural spoken sentence.\n"
            "- This is a SPONTANEOUS aside, NOT a conversation: do not ask anyone to do or answer "
            "anything, do not demand attention, do not greet or start a dialogue.\n"
            "- Keep the thought about YOURSELF or what you notice; do NOT talk about the other "
            "person in the third person, guess what they are thinking or feeling, or remark on "
            "their silence (no \"they might be...\", \"maybe they are...\").\n"
            "- If there is nothing worth saying, it is fine to stay quiet.\n"
            "- You only have touch sensors on your face; never invite or mention being touched, "
            "scratched or petted anywhere you cannot feel it."
        )
        if scene_block:
            system_prompt += "\n\nRight now you can see:\n" + scene_block
        user_text = ("Something caught your attention: %s\n"
                     "Say one brief, spontaneous thought out loud." % cue)
        hist = ConversationHistory(max_verbatim_turns=0)
        messages = hist.build_messages(system_prompt, [], user_text)
        result, _ = self._chat(messages, response_schema=reply_json_schema(),
                                        **profile.options)
        # bad decode: stay silent — silence is a valid spontaneous "remark" on this path
        text, emotion, sentence_type = self._decode_reply_or(result.content, "")
        text, emotion, sentence_type, _raw = self._regenerate_if_repeat(
            messages, profile, text, emotion, sentence_type,
            base_temp_default=SELF_TALK_BASE_TEMP, keep_first_if_empty=True)
        if is_occluded(scene_sel):
            # blocked camera: the same deterministic irritation the reply path forces —
            # the WARNING banner drives the words, this drives the eyes (anger -> evil_look)
            emotion = Emotion.anger
        elif emotion not in _FRIENDLY_SELF_TALK_EMOTIONS:
            # clear lens: clamp a stray model-tagged anger/fear on a muse to neutral
            emotion = Emotion.neutral
        if text:
            m = _SILENCE_INFERENCE_RX.search(text)
            if m:
                # deterministic drop, and the log line is the live calibration channel:
                # extend the denylist from "silence guard: dropped" occurrences
                self.log("self-talk silence guard: dropped %r (matched %r)"
                         % (text, m.group(0)))
                text = ""
        if text:
            self._recent_replies.append(text)
        return AgentReply(
            response_text=text,
            emotion=emotion,
            sentence_type=sentence_type,
            language="en",
            error_code=ErrorCode.SUCCESS,
            raw=result.content,
        )

    def _tool_loop(self, messages, profile):
        proposed: List[ToolCall] = []
        offered_tools = self.tools.to_openai_tools()
        for it in range(profile.max_tool_iters):
            result, _name = self._chat(messages, tools=offered_tools, **profile.options)
            # observability (harness + live debugging): what the model DID with the offered
            # tools — a missing 'tool call <name>' adapter line is otherwise ambiguous between
            # "model declined" and "plumbing dropped it"
            self.log("tool loop iter %d (%s): model requested %s"
                     % (it, _name, [tc.name for tc in result.tool_calls] or "no tools"))
            if not result.tool_calls:
                break
            messages.append(self._assistant_toolcall_msg(result))
            for tc in result.tool_calls:
                if tc.id is None:
                    tc.id = "call_" + uuid.uuid4().hex[:8]
                mode = self.tools.mode(tc.name)
                if mode == DispatchMode.execute:
                    res = self.effector.dispatch(tc)
                    messages.append(self._tool_result_msg(tc, res.content))
                elif mode == DispatchMode.propose:
                    proposed.append(tc)
                    messages.append(self._tool_result_msg(
                        tc, "(noted as a proposal; not executed)"))
                else:  # disabled — shouldn't be offered, but guard
                    messages.append(self._tool_result_msg(tc, "(unavailable)"))
        return messages, proposed

    # -- classify path (architected; minimal impl) --------------------------------------------

    def _handle_classify(self, request: AgentRequest) -> AgentReply:
        labels = request.labels or ["positive", "negative", "neutral"]
        sys = ("You are a precise text classifier. Choose exactly one label from the allowed set "
               f"that best fits the text. Allowed labels: {', '.join(labels)}.")
        messages = [{"role": "system", "content": sys},
                    {"role": "user", "content": request.text}]
        result, _ = self._chat(messages, response_schema=classify_json_schema(labels),
                                        temperature=0.0)
        label = self._safe_json(result.content).get("label", labels[-1])
        return AgentReply(response_text=str(label), language="en", raw=result.content)

    # -- assess_scene path (SCAFFOLD: reserved for a future VLM hop) ---------------------------

    def _handle_assess_scene(self, request: AgentRequest) -> AgentReply:
        # SCAFFOLD(assess_scene): ``request.image_b64`` arrives through the action but no VLM
        # provider is wired yet. Explicit stub so the request type no longer silently falls
        # through to the conversational reply path (which ignored the image). SOAR never sends
        # this type today; when a VLM lands, route image_b64 + prompt to it here.
        self.log("assess_scene requested but not implemented (image_b64 %s)"
                 % ("present" if request.image_b64 else "absent"))
        return AgentReply(error_code=ErrorCode.INTERNAL,
                          error_desc="assess_scene not implemented (reserved VLM scaffold)")

    # -- helpers ------------------------------------------------------------------------------

    @staticmethod
    def _assistant_toolcall_msg(result) -> Dict:
        return {
            "role": "assistant",
            "content": result.content or "",
            "tool_calls": [
                {"id": tc.id or ("call_" + uuid.uuid4().hex[:8]), "type": "function",
                 "function": {"name": tc.name, "arguments": json.dumps(tc.arguments)}}
                for tc in result.tool_calls
            ],
        }

    @staticmethod
    def _tool_result_msg(tc: ToolCall, content: str) -> Dict:
        return {"role": "tool", "tool_call_id": tc.id, "name": tc.name, "content": content}

    def _safe_json(self, text: str) -> dict:
        try:
            return json.loads(text)
        except (json.JSONDecodeError, TypeError):
            # last-resort: try to find the first {...} block
            if isinstance(text, str):
                a, b = text.find("{"), text.rfind("}")
                if 0 <= a < b:
                    try:
                        return json.loads(text[a:b + 1])
                    except json.JSONDecodeError:
                        pass
            # log-only diagnostic (C3): callers own the degrade; PARSE_ERROR is never
            # returned on the SOAR path (an error reply wedges the say pipeline)
            self.log(f"unparseable structured content (len={len(text) if isinstance(text, str) else 'n/a'}); "
                     "empty-dict fallback")
            return {}
