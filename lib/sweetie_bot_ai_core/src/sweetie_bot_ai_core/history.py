"""Conversation history -> chat messages.

Keeps the last N turns verbatim and compresses older turns into a short rolling summary block
("previously..."). The verbatim turns normally come from the request (SOAR's event window); the
rolling summary covers context beyond that window and is kept in the canonical language (English).
Summarization is delegated to an optional callback so this module stays pure/ROS-free; without
one it falls back to a cheap heuristic (concatenate + truncate).
"""
from __future__ import annotations

from typing import Callable, Dict, List, Optional

from .schema import TalkTurn


SummarizeFn = Callable[[List[TalkTurn]], str]


def _role(turn: TalkTurn) -> str:
    return "assistant" if turn.speaker.lower() in ("sweetie", "assistant", "bot") else "user"


def _heuristic_summary(turns: List[TalkTurn]) -> str:
    bits = []
    for t in turns:
        who = "Sweetie" if _role(t) == "assistant" else "Human"
        bits.append(f"{who}: {t.text}")
    text = " ".join(bits)
    return (text[:600] + "…") if len(text) > 600 else text


class ConversationHistory:
    def __init__(self, max_verbatim_turns: int = 8, summarize: Optional[SummarizeFn] = None):
        self.max_verbatim_turns = max_verbatim_turns
        self.summarize = summarize or _heuristic_summary

    def build_messages(self, system_prompt: str, history: List[TalkTurn],
                       user_text: str) -> List[Dict[str, str]]:
        messages: List[Dict[str, str]] = [{"role": "system", "content": system_prompt}]

        older = history[:-self.max_verbatim_turns] if len(history) > self.max_verbatim_turns else []
        recent = history[-self.max_verbatim_turns:] if history else []

        if older:
            summary = self.summarize(older)
            if summary:
                messages.append({"role": "system",
                                 "content": "Earlier in this conversation: " + summary})
        for turn in recent:
            messages.append({"role": _role(turn), "content": turn.text})
        if user_text:
            messages.append({"role": "user", "content": user_text})
        return messages
