"""Edge-latch + refractory state machine for touch zones — PURE (stdlib-only, no ROS/SML).

Kept separate from the TouchPulse input module so tests/test_pulse_latch.py can load this
FILE directly under plain system python (the package __init__ chain pulls rospy + SML).
"""


class PulseLatch:
    """Boop-on-TAP semantics (HANDOFF M.4b#4).

    - A rising edge LATCHES its zone visible for ``hold_s`` seconds, so a tap whose press
      and release both arrive between two SOAR input ticks is still seen (the plain level
      mirror lost it: the release overwrote the press in the message buffer).
    - Zones physically held stay visible beyond the latch — held-touch behaviors unchanged.
    - When a zone's exposure ends, a ``cooldown_s`` refractory absorbs new edges for it:
      rapid tapping merges into one exposure instead of machine-gunning the reaction.
    """

    def __init__(self, hold_s: float = 0.6, cooldown_s: float = 1.5):
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

        Exposure = active latches + physically-held zones not in refractory. A zone leaving
        exposure enters refractory; a hold that outlives its refractory becomes visible
        again (a sustained hold is a deliberate touch, not a bounce)."""
        latched = {z for z, t in self._latch_until.items() if now < t}
        visible = latched | {z for z in self._held
                             if now >= self._quiet_until.get(z, 0.0)}
        for zone in self._exposed - visible:
            self._quiet_until[zone] = now + self.cooldown_s
        self._latch_until = {z: t for z, t in self._latch_until.items() if now < t}
        self._exposed = frozenset(visible)
        return sorted(visible)
