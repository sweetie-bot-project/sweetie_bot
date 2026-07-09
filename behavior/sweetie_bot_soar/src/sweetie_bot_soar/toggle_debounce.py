"""Refractory for the operational TOGGLE service — PURE (stdlib-only, no ROS/SML).

Live defects this pins:
- 2026-07-08 (session 12): a single press of the rviz-panel toggle button fired
  /soar/toggle_operational TWICE ~0.4s apart, so "toggle off" landed back ON.
- 2026-07-09 (session 13, HANDOFF T.4): the panel fires periodic repeat TRAINS (4 calls
  at ~0.58s observed). The original debounce anchored its window to the last ACCEPTED
  call only, so every other pulse of a train landed outside the window and was accepted
  — the final state flipped with train-length PARITY ("off->on->off inconsistent"; the
  robot moved while the user believed it toggled off).

A TOGGLE is state-relative, so accidental repeats invert the intent — absorb them at the
SERVER (one choke point for every caller: rviz panel, killswitch, scripts). EVERY call,
accepted or rejected, re-arms the window: a whole repeat train therefore collapses to
exactly its FIRST call, however long it runs. Known limit: an isolated duplicate arriving
later than the window is indistinguishable from a real press by time alone — the complete
fix is panel-side (send absolute SetBool instead of Trigger-toggle).

Kept in its own file so tests/test_toggle_debounce.py can load it directly under plain
system python (the package __init__ chain pulls rospy + SML).
"""


class ToggleDebounce:
    def __init__(self, window_s: float = 0.7):
        self.window_s = float(window_s)
        self._last_call = None

    def accept(self, now: float) -> bool:
        """True iff no call (accepted OR rejected) happened within ``window_s`` before
        ``now``. Records ``now`` as the last call either way — rejected repeats EXTEND
        the refractory window, so a periodic repeat train can never re-toggle mid-train."""
        ok = self._last_call is None or (now - self._last_call) >= self.window_s
        self._last_call = now
        return ok
