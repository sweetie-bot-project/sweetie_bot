"""Language detection + LanguagePolicy wiring through the reply path.

Samples are REAL live traffic: the SOAR adapter always declares lang=en (a static config
value), so detection must come from the script — 'Привет, Свити! Расскажи, кто ты такая?'
arrived live with lang=en and must be treated as ru. The zh sample is the live-verified
LibreTranslate output ("你好 小马"). Canonical-EN contract: the agent always answers in en;
the voice node owns localization.
"""
from sweetie_bot_ai_core import Agent, AgentRequest, LanguagePolicy
from sweetie_bot_ai_core.client import ChatResult
from sweetie_bot_ai_core.translation import detect_language

RU_LIVE = "Привет, Свити! Расскажи, кто ты такая?"
ZH_LIVE = "你好 小马"
EN_LIVE = "Hello Sweetie! How are you today?"
REPLY_JSON = '{"response_text":"Hi!","emotion":"joy","sentence_type":"statement"}'


# --- detect_language on real samples ------------------------------------------------------------

def test_detect_language_real_samples():
    assert detect_language(RU_LIVE, "en") == "ru"      # declared en (static) must not win
    assert detect_language(ZH_LIVE, "en") == "zh"
    assert detect_language(EN_LIVE, "en") == "en"
    assert detect_language("こんにちは、スイーティー", "en") == "ja"


def test_detect_language_empty_and_symbols_fall_back_to_declared():
    assert detect_language("", "ru") == "ru"
    assert detect_language("!!! ...", "en") == "en"
    assert detect_language("", None) == "en"           # pivot fallback


# --- policy wiring through _handle_reply ---------------------------------------------------------

class RecordingProvider:
    def __init__(self):
        self.calls = []

    def translate(self, text, source, target):
        self.calls.append((text, source, target))
        return f"[{source}->{target}] {text}"


class CapturingRegistry:
    def __init__(self, content=REPLY_JSON):
        self.content = content
        self.all_messages = []

    def chat(self, messages, *, tools=None, response_schema=None, **kw):
        self.all_messages.append([dict(m) for m in messages])
        return ChatResult(content=self.content), "scripted"


def test_non_native_input_is_pivoted_and_language_note_added():
    prov = RecordingProvider()
    pol = LanguagePolicy(native_languages=["en", "zh", "ja"], pivot="en", provider=prov)
    reg = CapturingRegistry()
    agent = Agent(reg, language_policy=pol)
    reply = agent.handle(AgentRequest(text=RU_LIVE, text_language="en", profile="simple-en"))
    # ru is NOT native (P11: qwen2.5 ru output poor) -> translated in to the pivot
    assert prov.calls == [(RU_LIVE, "ru", "en")]
    user_msgs = [m["content"] for m in reg.all_messages[0] if m["role"] == "user"]
    assert any("[ru->en]" in c for c in user_msgs)
    # canonical-EN output contract: reply language en + the language note present
    assert reply.language == "en"
    sys_msg = next(m["content"] for m in reg.all_messages[0] if m["role"] == "system")
    assert "Always answer in English" in sys_msg


def test_native_zh_input_passes_through_but_still_gets_language_note():
    prov = RecordingProvider()
    pol = LanguagePolicy(native_languages=["en", "zh", "ja"], pivot="en", provider=prov)
    reg = CapturingRegistry()
    agent = Agent(reg, language_policy=pol)
    reply = agent.handle(AgentRequest(text=ZH_LIVE, text_language="en", profile="simple-en"))
    # zh is native: consumed directly, NO translate hop in
    assert prov.calls == []
    user_msgs = [m["content"] for m in reg.all_messages[0] if m["role"] == "user"]
    assert any(ZH_LIVE in c for c in user_msgs)
    # ...but the model must still answer EN (or the voice node would double-translate, P25)
    sys_msg = next(m["content"] for m in reg.all_messages[0] if m["role"] == "system")
    assert "Always answer in English" in sys_msg
    assert reply.language == "en"


def test_en_input_gets_no_language_note_and_no_translation():
    prov = RecordingProvider()
    pol = LanguagePolicy(native_languages=["en", "zh", "ja"], pivot="en", provider=prov)
    reg = CapturingRegistry()
    agent = Agent(reg, language_policy=pol)
    agent.handle(AgentRequest(text=EN_LIVE, text_language="en", profile="simple-en"))
    assert prov.calls == []
    sys_msg = next(m["content"] for m in reg.all_messages[0] if m["role"] == "system")
    assert "Always answer in English" not in sys_msg


def test_translation_failure_degrades_to_original_text():
    class BoomProvider:
        def translate(self, text, source, target):
            raise RuntimeError("translate down")

    pol = LanguagePolicy(native_languages=["en"], pivot="en", provider=BoomProvider())
    reg = CapturingRegistry()
    agent = Agent(reg, language_policy=pol)
    reply = agent.handle(AgentRequest(text=RU_LIVE, text_language="en", profile="simple-en"))
    # graceful degrade: the original text is fed rather than failing the turn
    user_msgs = [m["content"] for m in reg.all_messages[0] if m["role"] == "user"]
    assert any(RU_LIVE in c for c in user_msgs)
    assert reply.error_code == 0


# --- to_user_language (output-leg SCAFFOLD; unused on the live path) -----------------------------

def test_to_user_language_scaffold_symmetry():
    """SCAFFOLD guard: the output leg exists but the live path never calls it (canonical-EN;
    the voice node localizes). Pin its symmetry so enabling it later cannot double-translate."""
    prov = RecordingProvider()
    pol = LanguagePolicy(native_languages=["en", "zh", "ja"], pivot="en", provider=prov)
    assert pol.to_user_language("Hello!", "en") == "Hello!"       # pivot passthrough
    assert pol.to_user_language("你好!", "zh") == "你好!"           # native passthrough (no hop)
    assert prov.calls == []
    out = pol.to_user_language("Hello!", "ru")                     # non-native -> translate out
    assert prov.calls == [("Hello!", "en", "ru")]
    assert out.startswith("[en->ru]")
