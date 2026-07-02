"""Per-language translation routing.

Policy: some languages are fed to the model natively; others are translated to a pivot (English)
on the way in AND the reply is translated back on the way out (both directions, symmetric).
Native languages pass through untouched in both directions, preventing double-translation.

Execution is behind a ``TranslationProvider`` interface (default: a LibreTranslate HTTP client),
so translation is itself a swappable provider (a second 'AI provider container').
"""
from __future__ import annotations

from typing import List, Optional, Protocol

import requests


def detect_language(text: str, declared: Optional[str] = None, pivot: str = "en") -> str:
    """Cheap script-based language detection for STT output.

    The SOAR adapter sends a *static* configured ``text_language``, so the declared value cannot
    be trusted for a mixed-language audience — detect from the script instead. Only tells apart
    the scripts we route on (Cyrillic -> ru, kana -> ja, CJK -> zh); Latin-script text falls back
    to the declared language (or the pivot).
    """
    if not text:
        return (declared or pivot).lower()
    letters = [c for c in text if c.isalpha()]
    if not letters:
        return (declared or pivot).lower()
    n = len(letters)
    cyr = sum('\u0400' <= c <= '\u04ff' for c in letters)
    kana = sum('\u3040' <= c <= '\u30ff' for c in letters)
    cjk = sum('\u4e00' <= c <= '\u9fff' for c in letters)
    if cyr / n > 0.3:
        return 'ru'
    if kana / n > 0.1:
        return 'ja'
    if cjk / n > 0.3:
        return 'zh'
    return (declared or pivot).lower()


class TranslationProvider(Protocol):
    def translate(self, text: str, source: str, target: str) -> str: ...


class NullTranslationProvider:
    """No-op provider (used when translation is disabled or for tests)."""
    def translate(self, text: str, source: str, target: str) -> str:
        return text


class LibreTranslateProvider:
    def __init__(self, url: str, timeout: float = 15.0, api_key: Optional[str] = None):
        self.url = url.rstrip("/")
        self.timeout = timeout
        self.api_key = api_key

    def translate(self, text: str, source: str, target: str) -> str:
        if not text or source == target:
            return text
        payload = {"q": text, "source": source, "target": target, "format": "text"}
        if self.api_key:
            payload["api_key"] = self.api_key
        r = requests.post(self.url, json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json().get("translatedText", text)


class LanguagePolicy:
    """Decide whether input needs translating to the pivot before the model.

    ``native_languages`` are languages the chosen model handles well enough to consume directly.
    Everything else is translated to ``pivot``.
    """

    def __init__(self, native_languages: Optional[List[str]] = None, pivot: str = "en",
                 provider: Optional[TranslationProvider] = None):
        self.native = {l.lower() for l in (native_languages or ["en"])}
        self.pivot = pivot.lower()
        self.provider = provider or NullTranslationProvider()

    def needs_input_translation(self, text_language: str) -> bool:
        lang = (text_language or self.pivot).lower()
        return lang not in self.native and lang != self.pivot

    def to_model_language(self, text: str, text_language: str) -> str:
        """Translate input to the pivot if the model can't consume it natively."""
        if not text or not self.needs_input_translation(text_language):
            return text
        try:
            return self.provider.translate(text, source=text_language, target=self.pivot)
        except Exception:
            # degrade gracefully: feed the original text rather than failing the turn
            return text

    def to_user_language(self, text: str, target_language: str) -> str:
        """Translate the canonical (pivot) reply into the user's language — the output leg.

        Symmetric with :meth:`to_model_language`: only languages that were translated *in* get
        translated *out*. Native languages pass through untouched (the model consumed them
        directly and typically mirrors them), preventing double translation.
        """
        target = (target_language or self.pivot).lower()
        if not text or target == self.pivot or target in self.native:
            return text
        try:
            return self.provider.translate(text, source=self.pivot, target=target)
        except Exception:
            # degrade gracefully: an English reply is better than a failed turn
            return text
