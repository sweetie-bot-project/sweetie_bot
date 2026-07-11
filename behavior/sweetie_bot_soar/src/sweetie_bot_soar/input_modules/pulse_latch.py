"""Edge-latch + refractory state machine for touch zones — PURE (stdlib-only, no ROS/SML).

Kept separate from the TouchPulse input module so tests/test_pulse_latch.py can load this
FILE directly under plain system python (the package __init__ chain pulls rospy + SML).
"""


class PulseLatch:
    """Boop-on-TAP semantics (HANDOFF M.4b#4).

    - A rising edge LATCHES its zone visible for ``hold_s`` seconds, so a tap whose press
      and release both arrive between two SOAR input ticks is still seen (the plain level
      mirror lost it: the release overwrote the press in the message buffer).
    - Zones physically held are ALWAYS visible — finger on sensor = truth. The refractory
      never hides real contact (a hold is a deliberate touch, not a bounce; the old
      hold-hiding refractory punished the retry hold after a boop she ignored while talking).
    - When a zone's exposure ends, a ``cooldown_s`` refractory absorbs new tap LATCHES for
      it: rapid tapping merges into one exposure instead of machine-gunning the reaction,
      and release chatter after a hold cannot re-latch.
    """

    def __init__(self, hold_s: float = 0.6, cooldown_s: float = 0.8):
        self.hold_s = float(hold_s)
        self.cooldown_s = float(cooldown_s)
        self._held = frozenset()        # zones down per the LAST message
        self._latch_until = {}          # zone -> time its edge latch expires
        self._quiet_until = {}          # zone -> time its refractory expires
        self._exposed = frozenset()     # zones visible on the previous tick

    def feed(self, keys, now: float):
        """Process one incoming message (edge detection at MESSAGE rate, not tick rate)."""
        keys = frozenset(keys)
        for zone in keys - self._held:
            if now >= self._quiet_until.get(zone, 0.0):
                self._latch_until[zone] = now + self.hold_s
        self._held = keys

    def exposed(self, now: float):
        """The zone set to expose as ``^pressed`` this tick (sorted, stable for WME diffing).

        Exposure = active latches + ALL physically-held zones (level truth: the refractory
        gates only latch creation in feed(), never held visibility). A zone leaving
        exposure arms its refractory — including a hold release, so release chatter
        cannot re-latch."""
        latched = {z for z, t in self._latch_until.items() if now < t}
        visible = latched | self._held
        for zone in self._exposed - visible:
            self._quiet_until[zone] = now + self.cooldown_s
        self._latch_until = {z: t for z, t in self._latch_until.items() if now < t}
        self._exposed = frozenset(visible)
        return sorted(visible)
