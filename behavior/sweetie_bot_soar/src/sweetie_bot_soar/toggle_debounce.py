"""Refractory for the operational TOGGLE service — PURE (stdlib-only, no ROS/SML).

Live defect this pins (2026-07-08 session 12): a single press of the rviz-panel toggle
button fired /soar/toggle_operational TWICE ~0.4s apart (double-click on a laggy UI /
key repeat), so "toggle off" landed back ON and proactive self-talk kept speaking through
what the user believed was an operational=false gate. A TOGGLE is state-relative, so
accidental repeats invert the intent — absorb them at the SERVER (one choke point for
every caller: rviz panel, killswitch, scripts).

Kept in its own file so tests/test_toggle_debounce.py can load it directly under plain
system python (the package __init__ chain pulls rospy + SML).
"""


class ToggleDebounce:
    def __init__(self, window_s: float = 0.7):
        self.window_s = float(window_s)
        self._last_accepted = None

    def accept(self, now: float) -> bool:
        """True if a toggle at time ``now`` should be honoured; repeats inside the window
        after the last ACCEPTED toggle are rejected (and do NOT extend the window)."""
        if self._last_accepted is not None and (now - self._last_accepted) < self.window_s:
            return False
        self._last_accepted = now
        return True
