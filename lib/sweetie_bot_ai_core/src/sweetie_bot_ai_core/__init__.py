"""sweetie_bot_ai_core — ROS-free core for the Sweetie Bot LLM agent.

Reusable headless (sim / TUI / messenger). The ROS node in ``sweetie_bot_llm`` is a thin adapter
that injects a RobotStateProvider + EffectorPort and exposes the agent over ROS.
"""
from .agent import (Agent, EffectorPort, NullEffector, NullStateProvider, ProfileConfig,
                    RobotStateProvider)
from .client import ChatResult, LLMClientError, OpenAIChatClient
from .history import ConversationHistory
from .persona import DEFAULT_PERSONA, Persona, PersonaRegistry
from .prompt import build_system_prompt
from .registry import Endpoint, ProviderRegistry, RegistryError, build_llm_registry
from .schema import (AgentReply, AgentRequest, Emotion, ErrorCode, ReplyContent, RequestType,
                     RobotState, SentenceType, TalkTurn, ToolCall, ToolResult,
                     reply_json_schema)
from .tools import DispatchMode, ToolRegistry, ToolSpec
from .translation import (LanguagePolicy, LibreTranslateProvider, NullTranslationProvider,
                          TranslationProvider)

__all__ = [
    "Agent", "AgentRequest", "AgentReply", "ProfileConfig",
    "RobotStateProvider", "EffectorPort", "NullStateProvider", "NullEffector",
    "OpenAIChatClient", "ChatResult", "LLMClientError",
    "ProviderRegistry", "Endpoint", "RegistryError", "build_llm_registry",
    "PersonaRegistry", "Persona", "DEFAULT_PERSONA",
    "ToolRegistry", "ToolSpec", "DispatchMode",
    "LanguagePolicy", "TranslationProvider", "LibreTranslateProvider", "NullTranslationProvider",
    "ConversationHistory", "build_system_prompt",
    "RobotState", "TalkTurn", "ToolCall", "ToolResult",
    "Emotion", "SentenceType", "RequestType", "ReplyContent", "ErrorCode", "reply_json_schema",
]
