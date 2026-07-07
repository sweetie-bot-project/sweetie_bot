"""Text near-duplicate detection — the ONE similarity routine behind every repeat guard.

Both guards (anti-repeat on her own replies, already-answered on human turns) share the same
mechanics — normalize, skip short lines, exact-or-ratio match — but keep their independently
tuned thresholds (values unchanged from the inline originals; changing one must not silently
change the other):

* REPEAT_*   — her own recent replies (``Agent._is_repeat``): is she looping?
* ANSWERED_* — human turns already answered (``Agent._already_answered``): a lull re-poke?

Short lines below ``min_len`` never count as duplicates: brief affirmations ("yes", "okay")
legitimately recur in conversation.
"""
from __future__ import annotations

import difflib

# her-own-reply loop guard (was inline in _is_repeat)
REPEAT_MIN_LEN = 12
REPEAT_RATIO = 0.9
# answered-human-turn re-poke guard (was inline in _already_answered)
ANSWERED_MIN_LEN = 8
ANSWERED_RATIO = 0.92


def normalize(text) -> str:
    """Whitespace-collapsed lowercase form used for all similarity comparisons."""
    return " ".join((text or "").lower().split())


def is_near_duplicate(a, b, *, min_len: int, ratio: float) -> bool:
    """True when ``a`` and ``b`` are the same line up to small wording drift.

    ``a`` shorter than ``min_len`` (after normalization) never matches. Comparison is exact
    equality first, then difflib ratio > ``ratio``.
    """
    na, nb = normalize(a), normalize(b)
    if len(na) < min_len:
        return False
    return na == nb or difflib.SequenceMatcher(None, na, nb).ratio() > ratio
