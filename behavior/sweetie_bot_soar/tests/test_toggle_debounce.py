"""ToggleDebounce (operational-toggle refractory) — pure, no ROS/SML.

Pins two live defects:
- one physical press double-fired /soar/toggle_operational (~0.4s apart), inverting the
  user's intended OFF back to ON (2026-07-08);
- the panel fires periodic repeat TRAINS (~0.58s period); the original accepted-only
  window anchor re-accepted every other pulse, so the final state flipped with
  train-length parity (2026-07-09, HANDOFF T.4). Fixed semantics: EVERY call re-arms
  the window — a train collapses to exactly its first call."""
import importlib.util
import os

# load the module FILE directly: importing the sweetie_bot_soar package would pull rospy/SML
# via its __init__ chain, defeating the no-ROS system-python contract of this suite
_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src",
                                     "sweetie_bot_soar", "toggle_debounce.py"))
_spec = importlib.util.spec_from_file_location("toggle_debounce", _PATH)
toggle_debounce = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(toggle_debounce)
ToggleDebounce = toggle_debounce.ToggleDebounce


def test_first_toggle_accepted():
    db = ToggleDebounce(window_s=0.7)
    assert db.accept(10.0)


def test_double_fire_within_window_rejected():
    # the 2026-07-08 live shape: second call ~0.4s after the first must be absorbed
    db = ToggleDebounce(window_s=0.7)
    assert db.accept(10.0)
    assert not db.accept(10.4)


def test_deliberate_toggle_after_window_accepted():
    db = ToggleDebounce(window_s=0.7)
    assert db.accept(10.0)
    assert db.accept(11.0)


def test_rejected_repeat_extends_window():
    # a reject RE-ARMS the window: 10.75 is only 0.35s after the rejected 10.4 call.
    # (The original semantics accepted 10.75 — that was the train-parity defect.)
    db = ToggleDebounce(window_s=0.7)
    assert db.accept(10.0)
    assert not db.accept(10.4)
    assert not db.accept(10.75)
    assert not db.accept(11.44)   # 0.69s after 10.75: still inside
    assert db.accept(12.14)       # exactly window_s after 11.44: inclusive boundary


def test_live_train_collapses_to_first_call():
    # the observed rviz train: 4 calls at ~0.58s period -> exactly one accept ...
    db = ToggleDebounce(window_s=0.7)
    assert [db.accept(t) for t in (100.0, 100.58, 101.16, 101.74)] == \
        [True, False, False, False]
    # ... and a deliberate press 0.86s after the train's LAST call works
    assert db.accept(102.60)


def test_two_deliberate_presses_one_second_apart():
    db = ToggleDebounce(window_s=0.7)
    assert db.accept(5.0)
    assert db.accept(6.0)
