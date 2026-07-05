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
import uuid
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Protocol

from .history import ConversationHistory
from .persona import PersonaRegistry
from .prompt import build_system_prompt
from .registry import ProviderRegistry, RegistryError
from .scene import SceneConfig, diff as scene_diff, is_occluded, render_scene, select_salient
from .schema import (AgentReply, AgentRequest, ErrorCode, Emotion, ReplyContent, RequestType,
                     RobotState, SceneState, SentenceType, ToolCall, ToolResult,
                     classify_json_schema, reply_json_schema)
from .tools import DispatchMode, ToolRegistry
from .translation import detect_language


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


@dataclass
class ProfileConfig:
    allow_tools: bool = True
    max_verbatim_turns: int = 8
    max_tool_iters: int = 3
    options: Dict[str, object] = field(default_factory=lambda: {"temperature": 0.8})


DEFAULT_PROFILES: Dict[str, ProfileConfig] = {
    # primary path: full context + dynamic state + tools
    "complex-en": ProfileConfig(allow_tools=True, max_verbatim_turns=8, max_tool_iters=3,
                                options={"temperature": 0.8, "max_tokens": 512}),
    # fast lightweight path: reduced context, no tools
    "simple-en": ProfileConfig(allow_tools=False, max_verbatim_turns=4, max_tool_iters=0,
                               options={"temperature": 0.7, "max_tokens": 160}),
    # minimal degraded path
    "failsafe-en": ProfileConfig(allow_tools=False, max_verbatim_turns=2, max_tool_iters=0,
                                 options={"temperature": 0.6, "max_tokens": 120}),
    # canned-speech rephrase hop (text-action interceptor): fast, tool-free, constrained
    # rewording (temp trimmed from 1.0 -> 0.8: raw variety was pulling rephrases off-topic)
    "rephrase-en": ProfileConfig(allow_tools=False, max_verbatim_turns=0, max_tool_iters=0,
                                 options={"temperature": 0.8, "max_tokens": 80}),
    # spontaneous self-talk hop (proactive seam): scene-aware, tool-free, brief
    "self-talk-en": ProfileConfig(allow_tools=False, max_verbatim_turns=0, max_tool_iters=0,
                                  options={"temperature": 0.8, "max_tokens": 80}),
}


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
        self.log = logger or (lambda m: None)

    # -- public entry -------------------------------------------------------------------------

    def reset_ambient(self) -> None:
        """Forget the inter-turn ambient state (scene diff memory). Test/reset seam."""
        self._prev_scene = SceneState()

    def handle(self, request: AgentRequest) -> AgentReply:
        try:
            if request.request_type == RequestType.classify:
                return self._handle_classify(request)
            if request.request_type == RequestType.rephrase:
                return self._handle_rephrase(request)
            if request.request_type == RequestType.self_talk:
                return self._handle_self_talk(request)
            return self._handle_reply(request)
        except RegistryError as e:
            self.log(f"registry error: {e}")
            return AgentReply(error_code=ErrorCode.SERVER_UNREACHABLE, error_desc=str(e))
        except Exception as e:  # noqa: BLE001 - never crash the caller
            self.log(f"agent internal error: {e!r}")
            return AgentReply(error_code=ErrorCode.INTERNAL, error_desc=repr(e))

    # -- reply path ---------------------------------------------------------------------------

    def _handle_reply(self, request: AgentRequest) -> AgentReply:
        persona = self.personas.get(request.persona)
        profile = self.profiles.get(request.profile) or DEFAULT_PROFILES["complex-en"]
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
        try:
            raw_scene = self.scene_provider.snapshot(include_remembered=True)
        except TypeError:   # provider without retention support (e.g. NullSceneProvider)
            raw_scene = self.scene_provider.snapshot()
        scene = select_salient(raw_scene, self.scene_config)
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
            self.log("scene_block: " + scene_block.replace(chr(10), " | ")[:600])

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

        proposed: List[ToolCall] = []
        if tools_offered:
            messages, proposed = self._tool_loop(messages, profile)

        # Final constrained structured reply.
        result, _ = self.registry.chat(messages, response_schema=reply_json_schema(),
                                        **profile.options)
        content = ReplyContent.model_validate(self._safe_json(result.content))
        # canonical-EN output: NO agent-side back-translation - the voice node owns
        # localization (it translates every non-EN say from en; agent-side RU output was
        # double-translated and mangled, P25)
        emotion = Emotion.anger if occluded else content.emotion
        return AgentReply(
            response_text=content.response_text,
            emotion=emotion,
            sentence_type=content.sentence_type,
            tool_calls=proposed,
            language="en",                 # canonical; voice/say/<lang> localizes downstream
            error_code=ErrorCode.SUCCESS,
            raw=result.content,
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
        profile = self.profiles.get(request.profile) or DEFAULT_PROFILES["rephrase-en"]
        name = persona.display_name if persona else "Sweetie Bot"
        system_prompt = (
            f"You are {name}. You are given ONE line that you are about to say out loud. Rewrite "
            "it in fresh words, in your own voice.\n"
            "Strict rules:\n"
            "- Say the SAME thing as the given line: keep its exact meaning, intent and subject.\n"
            "- Do NOT add new topics, facts, questions or greetings, and do NOT react to your "
            "surroundings - the rewrite must be about the same thing and nothing else.\n"
            "- Do NOT answer or continue a conversation; only restate the given line.\n"
            "- Sweetie's only touch sensors are on her face (nose, forehead, cheeks, temples); "
            "never mention being touched, scratched or petted anywhere else - no ears, horn, "
            "back or body - keep the spot vague like 'right here' if the line is not about a "
            "face spot.\n"
            "- Keep it to one short, natural spoken sentence in the same language as the line."
        )
        user_text = 'Rewrite this line, keeping exactly the same meaning:\n"%s"' % line
        hist = ConversationHistory(max_verbatim_turns=0)
        messages = hist.build_messages(system_prompt, [], user_text)
        result, _ = self.registry.chat(messages, response_schema=reply_json_schema(),
                                        **profile.options)
        try:
            content = ReplyContent.model_validate(self._safe_json(result.content))
            text = (content.response_text or "").strip()
            emotion, sentence_type = content.emotion, content.sentence_type
        except Exception:  # noqa: BLE001 - bad structured decode: fall back to the canned line
            text, emotion, sentence_type = "", Emotion.neutral, SentenceType.statement
        if not text:
            text = line                        # degrade to the canned line, never to silence
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
        profile = self.profiles.get(request.profile) or DEFAULT_PROFILES["self-talk-en"]
        name = persona.display_name if persona else "Sweetie Bot"
        # read-only scene snapshot (no diff, no _prev_scene write): lets her name what she sees
        try:
            raw_scene = self.scene_provider.snapshot(include_remembered=True)
        except TypeError:
            raw_scene = self.scene_provider.snapshot()
        scene_block = render_scene(select_salient(raw_scene, self.scene_config), [],
                                   self.scene_config)
        system_prompt = (
            f"You are {name}. Something around you just caught your attention and you feel like "
            "saying one brief thought out loud - to yourself, not to anyone in particular.\n"
            "Strict rules:\n"
            "- React only to what caught your attention (and what you can see); one short, "
            "natural spoken sentence.\n"
            "- This is a SPONTANEOUS aside, NOT a conversation: do not ask anyone to do or answer "
            "anything, do not demand attention, do not greet or start a dialogue.\n"
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
        result, _ = self.registry.chat(messages, response_schema=reply_json_schema(),
                                        **profile.options)
        try:
            content = ReplyContent.model_validate(self._safe_json(result.content))
            text = (content.response_text or "").strip()
            emotion, sentence_type = content.emotion, content.sentence_type
        except Exception:  # noqa: BLE001 - bad decode: stay silent, this is an optional aside
            text, emotion, sentence_type = "", Emotion.neutral, SentenceType.statement
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
        for _ in range(profile.max_tool_iters):
            result, _name = self.registry.chat(messages, tools=offered_tools, **profile.options)
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
        result, _ = self.registry.chat(messages, response_schema=classify_json_schema(labels),
                                        temperature=0.0)
        label = self._safe_json(result.content).get("label", labels[-1])
        return AgentReply(response_text=str(label), language="en", raw=result.content)

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

    @staticmethod
    def _safe_json(text: str) -> dict:
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
            return {}
