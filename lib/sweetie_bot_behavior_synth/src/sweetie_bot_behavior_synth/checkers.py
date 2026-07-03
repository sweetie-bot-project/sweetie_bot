"""Result checkers: richer than plain asserts (user requirement).

- spaCy text checks for stochastic LLM prose (lemma matching with negation awareness);
- spatial checks: direction words in a reply vs the scenario's ground truth (canonical
  vocabulary shared with the production renderer: sweetie_bot_ai_core.scene);
- mechanism checks live in the tests themselves (profile, say-issued, counts).

Checkers raise AssertionError with a readable message, so they compose with pytest naturally.
"""
from __future__ import annotations

import re
from typing import Iterable, Optional

_NLP = None


def _nlp():
    global _NLP
    if _NLP is None:
        import spacy
        _NLP = spacy.load("en_core_web_sm")
    return _NLP


def _lemmas(text: str):
    return [t.lemma_.lower() for t in _nlp()(text)]


def _negated_heads(text: str):
    """Set of lemma heads that are syntactically negated ('I can't see the pony' -> see)."""
    doc = _nlp()(text)
    out = set()
    for t in doc:
        if t.dep_ == "neg":
            out.add(t.head.lemma_.lower())
    return out


# --------------------------------------------------------------- text checks ---------------------
def mentions(text: str, any_lemmas: Iterable[str], forbid_negated: Optional[str] = None):
    """Assert the reply mentions at least one of the lemmas.

    forbid_negated: a lemma that must NOT appear negated (e.g. 'see' — 'I can't see her'
    would fail a positive-sighting check even though 'pony' is mentioned)."""
    lm = set(_lemmas(text))
    want = {w.lower() for w in any_lemmas}
    assert lm & want, f"reply mentions none of {sorted(want)}: {text!r}"
    if forbid_negated is not None and forbid_negated.lower() in _negated_heads(text):
        raise AssertionError(f"'{forbid_negated}' is negated in reply: {text!r}")


def not_mentions(text: str, lemmas: Iterable[str]):
    lm = set(_lemmas(text))
    bad = lm & {w.lower() for w in lemmas}
    assert not bad, f"reply unexpectedly mentions {sorted(bad)}: {text!r}"


def is_cyrillic(text: str, min_frac: float = 0.5):
    letters = [c for c in text if c.isalpha()]
    assert letters, f"no letters in reply: {text!r}"
    frac = sum('Ѐ' <= c <= 'ӿ' for c in letters) / len(letters)
    assert frac >= min_frac, f"reply not Cyrillic ({frac:.0%}): {text!r}"


# --------------------------------------------------------------- spatial checks ------------------
_SIDE_WORDS = {
    "right": re.compile(r"\b(?:my|your|the)?\s*right\b", re.I),
    "left": re.compile(r"\b(?:my|your|the)?\s*left\b", re.I),
    "front": re.compile(r"\bfront\b|\bahead\b|\bbefore me\b", re.I),
    "behind": re.compile(r"\bbehind\b", re.I),
    "above": re.compile(r"\babove\b|\bup high\b|\boverhead\b", re.I),
    "below": re.compile(r"\bbelow\b|\bdown low\b|\bbeneath\b", re.I),
}


def _expected_side(bearing_deg: float, front_tol: float = 15.0) -> str:
    if abs(bearing_deg) <= front_tol:
        return "front"
    return "right" if bearing_deg > 0 else "left"


def direction(text: str, bearing: float, elevation: float = 0.0,
              front_tol: float = 15.0, elev_tol: float = 15.0):
    """Assert the reply's direction words agree with ground truth (ego standpoint,
    + bearing = HER right; + elevation = above). The wrong side is a hard failure;
    a missing side word is also a failure (she must state the direction)."""
    side = _expected_side(bearing, front_tol)
    wrong = {"right": "left", "left": "right"}.get(side)
    assert _SIDE_WORDS[side].search(text), \
        f"expected direction '{side}' (bearing {bearing:+.0f}°) not stated: {text!r}"
    if wrong is not None:
        assert not _SIDE_WORDS[wrong].search(text), \
            f"reply states the WRONG side ('{wrong}' vs truth '{side}'): {text!r}"
    if elevation >= elev_tol:
        assert _SIDE_WORDS["above"].search(text), f"expected 'above' (elev {elevation:+.0f}°): {text!r}"
    elif elevation <= -elev_tol:
        assert _SIDE_WORDS["below"].search(text), f"expected 'below' (elev {elevation:+.0f}°): {text!r}"
