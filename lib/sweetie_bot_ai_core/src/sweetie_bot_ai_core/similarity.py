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


def is_near_duplicate(a, b, *, min_len: int, ratio: float, contains: bool = False) -> bool:
    """True when ``a`` and ``b`` are the same line up to small wording drift.

    ``a`` shorter than ``min_len`` (after normalization) never matches. Comparison is exact
    equality first, then difflib ratio > ``ratio``.

    ``contains``: also match when one normalized line appears verbatim inside the other and
    the contained line is >= ``min_len``. A glued superstring (intro + previous reply) dilutes
    the ratio below the threshold while still voicing the old line word-for-word (live
    2026-07-08: 0.886 vs 0.9). OFF by default: the already-answered guard must NOT treat a
    longer new question that contains an answered one as already answered.
    """
    na, nb = normalize(a), normalize(b)
    if len(na) < min_len:
        return False
    if na == nb or difflib.SequenceMatcher(None, na, nb).ratio() > ratio:
        return True
    return contains and len(nb) >= min_len and (nb in na or na in nb)
