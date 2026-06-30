"""Provider registry: priority failover + circuit-breaker over a set of endpoints.

This is the evolution of the old ``sweetie_bot_load_balancer.Balancer``. It is **provider-type
agnostic** — it does failover/health over a list of endpoints, each holding a client object, and
runs an arbitrary *operation* against the first healthy endpoint. The LLM uses it with
``OpenAIChatClient`` clients; STT/TTS providers can reuse the same registry with their own client
classes. That is why it lives in the shared ``sweetie_bot_ai_core`` lib, not in the LLM package.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional

from .client import OpenAIChatClient, LLMClientError


class RegistryError(RuntimeError):
    pass


@dataclass
class Endpoint:
    name: str
    client: Any                       # e.g. OpenAIChatClient (must support .health())
    priority: int = 100               # lower = tried first
    enabled: bool = True
    # circuit-breaker state
    fail_count: int = 0
    open_until: float = 0.0           # monotonic time until which this endpoint is skipped

    def is_open(self, now: float) -> bool:
        return self.open_until > now


class ProviderRegistry:
    """Try endpoints in priority order; trip a circuit-breaker on repeated failures."""

    def __init__(self, endpoints: List[Endpoint], *, fail_threshold: int = 2,
                 cooldown_s: float = 30.0, logger: Optional[Callable[[str], None]] = None,
                 clock: Callable[[], float] = time.monotonic):
        self.endpoints = sorted(endpoints, key=lambda e: e.priority)
        self.fail_threshold = fail_threshold
        self.cooldown_s = cooldown_s
        self.log = logger or (lambda m: None)
        self.clock = clock

    # -- breaker bookkeeping ------------------------------------------------------------------

    def _record_success(self, ep: Endpoint) -> None:
        ep.fail_count = 0
        ep.open_until = 0.0

    def _record_failure(self, ep: Endpoint, err: str) -> None:
        ep.fail_count += 1
        self.log(f"provider '{ep.name}' failed ({ep.fail_count}/{self.fail_threshold}): {err}")
        if ep.fail_count >= self.fail_threshold:
            ep.open_until = self.clock() + self.cooldown_s
            self.log(f"provider '{ep.name}' circuit OPEN for {self.cooldown_s:.0f}s")

    # -- core failover ------------------------------------------------------------------------

    def execute(self, operation: Callable[[Any], Any]) -> Any:
        """Run ``operation(client)`` against the first healthy endpoint; failover on error.

        Returns ``(result, endpoint_name)``. Raises ``RegistryError`` if all endpoints fail.
        """
        now = self.clock()
        last_err = "no endpoints"
        tried_any = False
        # first pass: closed circuits in priority order; second pass: half-open (expired) ones
        ordered = [e for e in self.endpoints if e.enabled and not e.is_open(now)] + \
                  [e for e in self.endpoints if e.enabled and e.is_open(now)]
        for ep in ordered:
            tried_any = True
            try:
                result = operation(ep.client)
            except Exception as e:  # noqa: BLE001 - registry must survive any client error
                last_err = f"{type(e).__name__}: {e}"
                self._record_failure(ep, last_err)
                continue
            self._record_success(ep)
            return result, ep.name
        if not tried_any:
            last_err = "all endpoints disabled"
        raise RegistryError(f"all providers failed: {last_err}")

    # -- LLM convenience ----------------------------------------------------------------------

    def chat(self, messages, **kwargs):
        return self.execute(lambda c: c.chat(messages, **kwargs))

    def health(self) -> dict:
        return {ep.name: (ep.enabled and not ep.is_open(self.clock())) for ep in self.endpoints}


# -- construction from config -----------------------------------------------------------------

def build_llm_registry(config: dict, *, logger: Optional[Callable[[str], None]] = None,
                       default_options: Optional[dict] = None) -> ProviderRegistry:
    """Build a registry of OpenAI-compatible LLM endpoints from a config dict.

    Expected config shape (compatible-ish with the old ai.yaml balancer_config)::

        providers:
          local:  { url: http://localhost:11434/v1, model: qwen2.5:14b, priority: 10 }
          remote: { url: https://ai.sweetie.bot/llm/v1, model: ..., priority: 20,
                    api_key_env: SWEETIE_LLM_KEY }
    """
    providers = config.get("providers") or config.get("server_choices") or {}
    eps: List[Endpoint] = []
    for name, desc in providers.items():
        import os
        url = desc.get("url")
        if not url:
            continue
        api_key = None
        if desc.get("api_key_env"):
            api_key = os.environ.get(desc["api_key_env"])
        client = OpenAIChatClient(
            base_url=url,
            model=desc.get("model", config.get("model", "")),
            api_key=api_key or desc.get("api_key"),
            timeout=desc.get("timeout", 120.0),
            default_options=default_options or {},
        )
        eps.append(Endpoint(name=name, client=client, priority=desc.get("priority", 100)))
    if not eps:
        raise RegistryError("no LLM providers configured")
    return ProviderRegistry(eps, fail_threshold=config.get("fail_threshold", 2),
                            cooldown_s=config.get("cooldown_s", 30.0), logger=logger)
