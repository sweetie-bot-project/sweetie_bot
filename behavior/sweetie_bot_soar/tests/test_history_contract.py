"""System-python side of the two-python history-contract pin (no rospy, no venv, no pydantic).

The SAME literal fixture (CONTRACT_TURNS) is replayed on the venv side by
behavior/sweetie_bot_llm/tests/test_agent_bridge.py::test_history_contract_fixture_parses
through agent_bridge.parse_history — if the serialization drifts, ONE of the two suites goes
red. Keep the literals byte-identical in both files.
"""
import importlib.util
import os
from types import SimpleNamespace

# load the module FILE directly: importing the sweetie_bot_soar package would pull rospy via
# its __init__, defeating the no-ROS system-python contract of this suite
_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src",
                                     "sweetie_bot_soar", "output_modules",
                                     "history_contract.py"))
_spec = importlib.util.spec_from_file_location("history_contract", _PATH)
history_contract = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(history_contract)
ILLEGIBLE_TEXT = history_contract.ILLEGIBLE_TEXT
build_history = history_contract.build_history
drop_duplicate_tail = history_contract.drop_duplicate_tail


def ev(type, stamp, text=None, emotion=None):
    e = SimpleNamespace(type=type, stamp=stamp, text=text)
    if emotion is not None:
        e.emotion = emotion
    return e


# ---- THE shared fixture (mirrored literally in test_agent_bridge.py) ---------------------------
FIXTURE_EVENTS = [
    ev("talk-heard", 1.0, "Hello Sweetie! How are you today?"),
    ev("talk-said", 2.0, "I'm feeling chipper today!", emotion="joy"),
    ev("talk-no-answer", 3.0),
    ev("talk-illegible", 4.0),
]
CONTRACT_TURNS = [
    {"speaker": "human", "text": "Hello Sweetie! How are you today?"},
    {"speaker": "sweetie", "text": "I'm feeling chipper today!", "emotion": "joy"},
    {"speaker": "human", "text": "(says something unclear)"},
]


def test_contract_fixture():
    assert build_history(FIXTURE_EVENTS) == CONTRACT_TURNS


def test_silence_types_never_become_turns():
    """history._role maps ANY non-sweetie speaker to 'user' downstream, so a silence turn would
    be ANSWERED as if the human had said it (the live 'human is silent' bug) — must stay out."""
    turns = build_history([ev("talk-no-answer", 1.0), ev("talk-ignored", 2.0)])
    assert turns == []


def test_illegible_maps_to_placeholder_human_turn():
    assert build_history([ev("talk-illegible", 1.0)]) == \
        [{"speaker": "human", "text": ILLEGIBLE_TEXT}]


def test_stamp_ordering_not_arrival_ordering():
    turns = build_history([ev("talk-said", 5.0, "second", emotion="neutral"),
                           ev("talk-heard", 1.0, "first")])
    assert [t["text"] for t in turns] == ["first", "second"]


def test_missing_emotion_serializes_as_none():
    turns = build_history([ev("talk-said", 1.0, "hi")])
    assert turns == [{"speaker": "sweetie", "text": "hi", "emotion": None}]


# ---- max_events: legacy newest-N semantics (R5) --------------------------------------------------

def _n_events(n):
    return [ev("talk-heard", float(i), "t%d" % i) for i in range(n)]


def test_max_events_keeps_newest_in_chronological_order():
    turns = build_history(_n_events(6), max_events=2)
    assert [t["text"] for t in turns] == ["t4", "t5"]


def test_max_events_zero_means_no_history():
    assert build_history(_n_events(3), max_events=0) == []


def test_max_events_none_means_unbounded():
    assert len(build_history(_n_events(9), max_events=None)) == 9


def test_max_events_applies_to_events_before_turn_mapping():
    """Legacy semantics: the cap slices EVENTS, so capped-away silence events still consume
    slots (they were part of the legacy prompt window)."""
    events = [ev("talk-heard", 1.0, "old"),
              ev("talk-no-answer", 2.0),
              ev("talk-heard", 3.0, "new")]
    turns = build_history(events, max_events=2)
    # the newest 2 EVENTS are (no-answer, new) -> only one visible turn
    assert [t["text"] for t in turns] == ["new"]


# ---- tail-dedup (current utterance delivered both as event and ^text) -----------------------------

def test_drop_duplicate_tail():
    hist = [{"speaker": "human", "text": "hello"}]
    assert drop_duplicate_tail(hist, "hello") == []
    assert drop_duplicate_tail(hist, "different") == hist
    assert drop_duplicate_tail(hist, None) == hist
    assert drop_duplicate_tail([], "hello") == []
