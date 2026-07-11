"""PulseLatch (touch boop-on-TAP edge/cooldown state machine) — pure, no ROS/SML.

Ticks are simulated by calling exposed() at increasing times; messages by feed(). The live
defect this pins: a tap whose press+release both land between two ticks was invisible to
the per-tick level mirror (behavior test test_touch.py::test_quick_nose_taps_squeak).
Holds are level-truth: a physically-down zone is ALWAYS exposed; the refractory gates only
tap LATCHES (the live defect the redesign pins: the old refractory hid even a physical
hold for cooldown_s after any exposure ended, punishing the retry after a missed boop)."""
import importlib.util
import os

# load the module FILE directly: importing the sweetie_bot_soar package would pull rospy/SML
# via its __init__ chain, defeating the no-ROS system-python contract of this suite
_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src",
                                     "sweetie_bot_soar", "input_modules", "pulse_latch.py"))
_spec = importlib.util.spec_from_file_location("pulse_latch", _PATH)
pulse_latch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pulse_latch)
PulseLatch = pulse_latch.PulseLatch


def test_sub_tick_tap_is_latched_across_the_tick():
    pl = PulseLatch(hold_s=0.6, cooldown_s=1.5)
    pl.feed(["nose"], 0.00)         # press...
    pl.feed([], 0.03)               # ...and release, both before the next tick
    assert pl.exposed(0.10) == ["nose"]     # the tick still sees the tap
    assert pl.exposed(0.30) == ["nose"]     # held visible for the whole latch window
    assert pl.exposed(0.70) == []           # latch expired


def test_physical_hold_outlives_the_latch():
    pl = PulseLatch(hold_s=0.6, cooldown_s=1.5)
    pl.feed(["nose"], 0.0)
    assert pl.exposed(0.1) == ["nose"]
    assert pl.exposed(2.0) == ["nose"]      # still held -> still visible (held-touch intact)
    pl.feed([], 2.1)
    assert pl.exposed(2.2) == []


def test_refractory_absorbs_a_quick_retap():
    pl = PulseLatch(hold_s=0.6, cooldown_s=1.5)
    pl.feed(["nose"], 0.00)
    pl.feed([], 0.03)
    assert pl.exposed(0.10) == ["nose"]
    assert pl.exposed(0.70) == []           # exposure ends here -> refractory until 2.2
    pl.feed(["nose"], 0.80)                 # re-tap inside the refractory
    pl.feed([], 0.83)
    assert pl.exposed(0.90) == []           # absorbed: no machine-gun squeak
    assert pl.exposed(1.50) == []


def test_burst_of_taps_merges_into_one_exposure():
    pl = PulseLatch(hold_s=0.6, cooldown_s=1.5)
    t = 0.0
    for _ in range(4):                      # taps 0.15 s apart, all within the latch
        pl.feed(["nose"], t)
        pl.feed([], t + 0.03)
        assert pl.exposed(t + 0.05) == ["nose"]   # continuously visible, never a gap
        t += 0.15
    assert pl.exposed(t + 0.40) == ["nose"]       # last latch (t-0.15 + 0.6) still holding
    assert pl.exposed(t + 0.55) == []             # one exposure ended -> one reaction total


def test_hold_during_refractory_is_exposed_immediately():
    pl = PulseLatch(hold_s=0.6, cooldown_s=1.5)
    pl.feed(["nose"], 0.00)
    pl.feed([], 0.03)
    assert pl.exposed(0.10) == ["nose"]
    assert pl.exposed(0.70) == []           # refractory until 2.2
    pl.feed(["nose"], 1.00)                 # press AND HOLD during the refractory
    assert pl.exposed(1.10) == ["nose"]     # finger on sensor = truth: no hiding
    assert pl.exposed(2.00) == ["nose"]     # stays visible for the whole hold
    pl.feed([], 2.05)
    assert pl.exposed(2.10) == []


def test_retry_hold_after_missed_boop_is_seen_immediately():
    # She was talking during the first tap's exposure (SOAR ignored it — the latch cannot
    # know); the human retries with a press-and-hold: the refractory the unconsumed
    # exposure armed must not punish it.
    pl = PulseLatch(hold_s=0.6, cooldown_s=1.5)
    pl.feed(["nose"], 0.00)                 # the missed boop
    pl.feed([], 0.03)
    assert pl.exposed(0.10) == ["nose"]
    assert pl.exposed(0.70) == []           # unconsumed exposure ends -> refractory armed
    pl.feed(["nose"], 0.80)                 # retry: press AND HOLD 0.1 s later
    assert pl.exposed(0.90) == ["nose"]     # seen on the very next tick
    assert pl.exposed(1.50) == ["nose"]     # and stays exposed while held


def test_press_down_at_tick_during_refractory_is_exposed():
    # Level-truth contract: whenever the finger is physically down at a tick the zone is
    # exposed — the refractory suppresses only the tap LATCH (the 0.6 s stretch), so a
    # sub-tick retap inside the refractory stays absorbed (test above) but a press
    # spanning a tick is real contact and is reported.
    pl = PulseLatch(hold_s=0.6, cooldown_s=1.5)
    pl.feed(["nose"], 0.00)
    pl.feed([], 0.03)
    assert pl.exposed(0.10) == ["nose"]
    assert pl.exposed(0.70) == []           # refractory until 2.2
    pl.feed(["nose"], 0.85)                 # press...
    assert pl.exposed(0.90) == ["nose"]     # ...down at the tick -> exposed (level truth)
    pl.feed([], 0.95)                       # ...release: no latch was armed
    assert pl.exposed(1.10) == []           # exposure ended with the release


def test_zones_are_independent():
    pl = PulseLatch(hold_s=0.6, cooldown_s=1.5)
    pl.feed(["cheek_left"], 0.0)            # held cheek
    pl.feed(["cheek_left", "nose"], 0.2)    # nose tap on top
    pl.feed(["cheek_left"], 0.25)
    assert pl.exposed(0.3) == ["cheek_left", "nose"]
    assert pl.exposed(0.9) == ["cheek_left"]          # nose latch gone, cheek still held
