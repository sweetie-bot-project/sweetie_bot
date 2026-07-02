"""Unit tests for the two-way translation policy (P11: RU both directions)."""
import pytest

from sweetie_bot_ai_core.translation import (LanguagePolicy, NullTranslationProvider,
                                             detect_language)


class FakeProvider:
    """Records calls; returns a tagged string so direction is assertable."""
    def __init__(self):
        self.calls = []

    def translate(self, text, source, target):
        self.calls.append((source, target))
        return f"[{source}->{target}]{text}"


class FailingProvider:
    def translate(self, text, source, target):
        raise RuntimeError("translate service down")


# --- detect_language --------------------------------------------------------------------------

def test_detect_cyrillic_is_ru():
    assert detect_language("Привет, как дела?") == "ru"


def test_detect_latin_falls_back_to_declared():
    assert detect_language("Hello there", "en") == "en"
    assert detect_language("Guten Tag", "de") == "de"   # script can't tell; trust declared


def test_detect_empty_uses_declared_or_pivot():
    assert detect_language("", "ru") == "ru"
    assert detect_language("", None) == "en"
    assert detect_language("123 !?") == "en"            # no letters at all


def test_detect_kana_and_cjk():
    assert detect_language("こんにちは") == "ja"
    assert detect_language("你好世界") == "zh"


# --- LanguagePolicy two-way routing -----------------------------------------------------------

def test_non_native_translated_both_ways():
    p = LanguagePolicy(native_languages=["en"], provider=FakeProvider())
    assert p.to_model_language("привет", "ru") == "[ru->en]привет"
    assert p.to_user_language("hello", "ru") == "[en->ru]hello"


def test_pivot_untouched_both_ways():
    prov = FakeProvider()
    p = LanguagePolicy(native_languages=["en"], provider=prov)
    assert p.to_model_language("hello", "en") == "hello"
    assert p.to_user_language("hello", "en") == "hello"
    assert prov.calls == []


def test_native_language_untouched_both_ways():
    """Native languages are consumed directly and NOT translated back (no double translation)."""
    prov = FakeProvider()
    p = LanguagePolicy(native_languages=["en", "zh"], provider=prov)
    assert p.to_model_language("你好", "zh") == "你好"
    assert p.to_user_language("你好", "zh") == "你好"
    assert prov.calls == []


def test_output_translation_degrades_gracefully():
    p = LanguagePolicy(native_languages=["en"], provider=FailingProvider())
    assert p.to_user_language("hello", "ru") == "hello"   # EN reply better than a failed turn


def test_default_native_is_en_only():
    p = LanguagePolicy()
    assert p.native == {"en"}
    assert p.needs_input_translation("ru")
