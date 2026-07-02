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
from .scene import SceneConfig, diff as scene_diff, render_scene, select_salient
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
                                options={"temperature": 0.8}),
    # fast lightweight path: reduced context, no tools
    "simple-en": ProfileConfig(allow_tools=False, max_verbatim_turns=4, max_tool_iters=0,
                               options={"temperature": 0.7, "max_tokens": 160}),
    # minimal degraded path
    "failsafe-en": ProfileConfig(allow_tools=False, max_verbatim_turns=2, max_tool_iters=0,
                                 options={"temperature": 0.6, "max_tokens": 120}),
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

    def handle(self, request: AgentRequest) -> AgentReply:
        try:
            if request.request_type == RequestType.classify:
                return self._handle_classify(request)
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

        # ambient environmental awareness: front-weighted scene + inter-turn delta
        scene = select_salient(self.scene_provider.snapshot(), self.scene_config)
        events = scene_diff(self._prev_scene, scene, self.scene_config)
        self._prev_scene = scene
        scene_block = render_scene(scene, events, self.scene_config)

        language_note = None
        if self.policy is not None and self.policy.needs_input_translation(user_lang):
            # earlier turns in `history` may still be in the user's language — anchor the model
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
        response_text = content.response_text
        if self.policy is not None:
            response_text = self.policy.to_user_language(response_text, user_lang)
        return AgentReply(
            response_text=response_text,
            emotion=content.emotion,
            sentence_type=content.sentence_type,
            tool_calls=proposed,
            language=user_lang,            # mirror the speaker (en stays en)
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
