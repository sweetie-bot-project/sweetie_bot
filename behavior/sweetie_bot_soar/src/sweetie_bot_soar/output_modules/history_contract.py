"""The SOAR-events -> history_json contract — STDLIB-ONLY (two-python wall).

This module defines the ONE serialization of SOAR talk events into the ``history_json`` field
of a GenerateReply goal. It is imported by the SOAR bridge (system python3 / pydantic 1) and
its fixtures are replayed against ``sweetie_bot_llm.agent_bridge.parse_history`` on the venv
side (python + pydantic 2) — so it must import NOTHING beyond the standard library.

Contract (consumed by ``agent_bridge.parse_history`` -> ``ai_core.schema.TalkTurn``):
    history_json = JSON list of turn dicts, chronological (oldest first), each:
        {"speaker": "human" | "sweetie",   # any other speaker is treated as human downstream
         "text": str,                      # what was said (verbatim)
         "emotion": str | null}            # optional; only 'sweetie' turns carry it

Event mapping (legacy-faithful):
    talk-heard      -> human turn (verbatim text)
    talk-said       -> sweetie turn (+ emotion when present)
    talk-ignored / talk-no-answer
                    -> NO turn: silence is not human speech; it reaches the agent as CONTEXT
                       (history._role maps any non-sweetie speaker to role 'user', so a silence
                       "turn" would be answered as if the human had SAID it — the live
                       "human is silent" re-answer bug)
    talk-illegible  -> human turn "(says something unclear)"

Cross-python pinning: behavior/sweetie_bot_soar/tests/test_history_contract.py (system py)
and behavior/sweetie_bot_llm/tests/test_agent_bridge.py (venv py) feed the SAME literal
fixtures through build_history / parse_history respectively.
"""

CONTRACT_FIELDS = ("speaker", "text", "emotion")

ILLEGIBLE_TEXT = "(says something unclear)"


def build_history(events, max_events=None):
    """Map SOAR talk events (objects with .type/.stamp/.text[/.emotion]) to contract turns.

    ``max_events``: keep only the NEWEST N events, applied to the (chronologically sorted)
    event list BEFORE turn-mapping — legacy ``events[-max_events:]`` semantics, where 0 means
    "no event history at all" and None means unbounded.
    """
    evs = sorted(events, key=lambda e: e.stamp)
    if max_events is not None:
        evs = evs[-max_events:] if max_events > 0 else []
    turns = []
    for ev in evs:
        if ev.type == 'talk-heard':
            turns.append({"speaker": "human", "text": ev.text})
        elif ev.type == 'talk-said':
            turns.append({"speaker": "sweetie", "text": ev.text,
                          "emotion": getattr(ev, 'emotion', None)})
        elif ev.type in ('talk-ignored', 'talk-no-answer'):
            pass  # silence is NOT human speech: conveyed as system context, not a turn
        elif ev.type == 'talk-illegible':
            turns.append({"speaker": "human", "text": ILLEGIBLE_TEXT})
    return turns


def drop_duplicate_tail(history, text):
    """Drop the last turn when it IS the current utterance (SOAR delivers the newest
    talk-heard both as an event and as ^text — sending both would duplicate it)."""
    if text is not None and history and history[-1].get("text") == text:
        return history[:-1]
    return history
