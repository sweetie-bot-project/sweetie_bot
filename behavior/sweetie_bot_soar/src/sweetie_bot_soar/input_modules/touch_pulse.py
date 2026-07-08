"""Touch input as edge-latched pulses (boop on TAP + cooldown, HANDOFF M.4b#4).

The plain ``joystick`` module mirrors the LATEST buffered message once per SOAR input tick:
a tap whose press AND release both arrive between two ticks is never seen at all (the
release overwrites the press in the buffer) — quick nose boops made no squeak on the live
robot. This module exposes the SAME ``^pressed <zone>`` / ``last-activity`` WMEs, so the
talk_simple touch rules work unmodified (no .soar change); only the soar.yaml ``type``
switches. Edge/cooldown logic lives in the pure ``PulseLatch`` (unit-tested separately).
"""
from threading import Lock

import rospy
from sweetie_bot_joystick.msg import KeyPressed

from .bins import BinsMap
from .input_module import InputModule, InputModulesLoader
from .pulse_latch import PulseLatch
from .wme_proxy import SetWMEProxy


class TouchPulse(InputModule):
    def _init(self, name, config, agent):
        # configuration (topic + bins map exactly as the joystick module it replaces)
        topic = self.getConfigParameter(config, "topic", allowed_types=str)
        try:
            self._last_activity_bins_map = BinsMap(config['last_activity_bins_map'])
        except KeyError as e:
            raise ValueError('TouchPulse input module: "last_activity_bins_map" parameter '
                             'must present.') from e
        self._latch = PulseLatch(hold_s=float(config.get('pulse_hold', 0.6)),
                                 cooldown_s=float(config.get('cooldown', 1.5)))

        self._lock = Lock()
        self._last_activity_timestamp = 0.0

        # wmes (same shape as joystick: ^pressed <zone> set + last-activity bin)
        self._last_activity_wme = self._sensor_id.CreateStringWME(
            'last-activity', self._last_activity_bins_map(rospy.Time.now().to_sec()))
        self._pressed_wmes = SetWMEProxy(self._sensor_id, 'pressed')

        self._sub = self.createSubscriber(topic, KeyPressed, self.touchCallback)

    def touchCallback(self, msg):
        # edge detection must run at MESSAGE rate: this is exactly what the per-tick
        # level mirror lost
        with self._lock:
            self._latch.feed(msg.keys, rospy.Time.now().to_sec())

    def update(self):
        now = rospy.Time.now().to_sec()
        with self._lock:
            exposed = self._latch.exposed(now)

        changed = self._pressed_wmes.assign(exposed)
        if changed:
            self._last_activity_timestamp = now

        value = self._last_activity_bins_map(now - self._last_activity_timestamp)
        if self._last_activity_wme.GetValue() != value:
            self._last_activity_wme.Update(value)


InputModulesLoader.register("touch_pulse", TouchPulse)
