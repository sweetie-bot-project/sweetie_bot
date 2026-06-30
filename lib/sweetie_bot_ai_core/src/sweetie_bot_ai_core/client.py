"""Low-level OpenAI-compatible chat client (single endpoint).

ROS-free. Uses ``requests`` (blocking) with a streaming generator method so the rest of the
stack is streaming-ready without committing to asyncio yet. A future async/httpx implementation
can replace this class behind the same interface.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional

import requests

from .schema import ToolCall


class LLMClientError(RuntimeError):
    def __init__(self, msg: str, details: str = ""):
        super().__init__(msg)
        self.details = details


@dataclass
class ChatResult:
    content: str = ""
    tool_calls: List[ToolCall] = field(default_factory=list)
    finish_reason: Optional[str] = None
    usage: Dict[str, Any] = field(default_factory=dict)
    raw: Optional[Dict[str, Any]] = None


def _parse_tool_calls(message: Dict[str, Any]) -> List[ToolCall]:
    out: List[ToolCall] = []
    for tc in message.get("tool_calls") or []:
        fn = tc.get("function", {})
        args = fn.get("arguments", {})
        if isinstance(args, str):
            try:
                args = json.loads(args) if args.strip() else {}
            except json.JSONDecodeError:
                args = {"_raw": args}
        out.append(ToolCall(name=fn.get("name", ""), arguments=args or {}, id=tc.get("id")))
    return out


class OpenAIChatClient:
    """Talks to one OpenAI-compatible ``/v1/chat/completions`` endpoint."""

    def __init__(self, base_url: str, model: str, api_key: Optional[str] = None,
                 timeout: float = 120.0, default_options: Optional[Dict[str, Any]] = None):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.api_key = api_key
        self.timeout = timeout
        self.default_options = default_options or {}

    # -- payload assembly ---------------------------------------------------------------------

    def _headers(self) -> Dict[str, str]:
        h = {"Content-Type": "application/json"}
        if self.api_key:
            h["Authorization"] = f"Bearer {self.api_key}"
        return h

    def _payload(self, messages: List[Dict[str, Any]], *, tools=None, response_schema=None,
                 stream=False, **options) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"model": self.model, "messages": messages, "stream": stream}
        payload.update(self.default_options)
        payload.update({k: v for k, v in options.items() if v is not None})
        if tools:
            payload["tools"] = tools
        if response_schema is not None:
            payload["response_format"] = {
                "type": "json_schema",
                "json_schema": {"name": "reply", "schema": response_schema, "strict": True},
            }
        return payload

    # -- blocking chat ------------------------------------------------------------------------

    def chat(self, messages: List[Dict[str, Any]], *, tools=None, response_schema=None,
             **options) -> ChatResult:
        payload = self._payload(messages, tools=tools, response_schema=response_schema,
                                stream=False, **options)
        try:
            r = requests.post(self.base_url + "/chat/completions", json=payload,
                              headers=self._headers(), timeout=self.timeout)
        except requests.RequestException as e:
            raise LLMClientError("connection error", str(e))
        if r.status_code != 200:
            raise LLMClientError(f"http {r.status_code}", r.text[:500])
        try:
            data = r.json()
        except ValueError as e:
            raise LLMClientError("json decode error", str(e))
        choice = (data.get("choices") or [{}])[0]
        msg = choice.get("message", {})
        return ChatResult(
            content=msg.get("content") or "",
            tool_calls=_parse_tool_calls(msg),
            finish_reason=choice.get("finish_reason"),
            usage=data.get("usage", {}),
            raw=data,
        )

    # -- streaming chat (content tokens; groundwork for barge-in) ------------------------------

    def stream_chat(self, messages: List[Dict[str, Any]], *, response_schema=None,
                    **options) -> Iterator[str]:
        """Yield content token deltas. Tools are intentionally not streamed here (resolved in
        the blocking path); this method exists so the TTS path can consume tokens later."""
        payload = self._payload(messages, response_schema=response_schema, stream=True, **options)
        try:
            r = requests.post(self.base_url + "/chat/completions", json=payload,
                              headers=self._headers(), timeout=self.timeout, stream=True)
        except requests.RequestException as e:
            raise LLMClientError("connection error", str(e))
        if r.status_code != 200:
            raise LLMClientError(f"http {r.status_code}", r.text[:500])
        for line in r.iter_lines():
            if not line:
                continue
            line = line.decode("utf-8").strip()
            if not line.startswith("data:"):
                continue
            body = line[5:].strip()
            if body == "[DONE]":
                break
            try:
                chunk = json.loads(body)
            except json.JSONDecodeError:
                continue
            delta = (chunk.get("choices") or [{}])[0].get("delta", {})
            tok = delta.get("content")
            if tok:
                yield tok

    def health(self) -> bool:
        """Lightweight liveness probe against the OpenAI models endpoint."""
        try:
            r = requests.get(self.base_url + "/models", headers=self._headers(), timeout=5)
            return r.status_code == 200
        except requests.RequestException:
            return False
