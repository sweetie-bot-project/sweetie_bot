"""ToggleDebounce (operational-toggle refractory) — pure, no ROS/SML.

Pins the live defect: one physical press double-fired /soar/toggle_operational (~0.4s
apart), inverting the user's intended OFF back to ON."""
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
    # the live shape: second call ~0.4s after the first must be absorbed
    db = ToggleDebounce(window_s=0.7)
    assert db.accept(10.0)
    assert not db.accept(10.4)


def test_deliberate_toggle_after_window_accepted():
    db = ToggleDebounce(window_s=0.7)
    assert db.accept(10.0)
    assert db.accept(11.0)


def test_rejected_repeat_does_not_extend_window():
    # burst: 10.0 ok, 10.4 rejected, 10.75 accepted (0.75s after the ACCEPTED one,
    # even though only 0.35s after the rejected repeat)
    db = ToggleDebounce(window_s=0.7)
    assert db.accept(10.0)
    assert not db.accept(10.4)
    assert db.accept(10.75)
