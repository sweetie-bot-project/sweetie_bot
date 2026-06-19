from . import input_module
from .input_module import InputModule, InputModulesLoader
from .wme_proxy import SetWMEProxy

from copy import copy
from threading import Lock

import rospy
from sweetie_bot_joystick.msg import KeyPressed

from .bins import BinsMap

class Joystick(InputModule):
    def _init(self, name, config, agent):
        
        # configuration
        joy_topic = self.getConfigParameter(config, "topic", allowed_types=str)
        try:
            self._last_activity_bins_map = BinsMap( config['last_activity_bins_map'] )
        except KeyError:
            raise RuntimeError('Joystick input module: "last_activity_bins_map" parameters must present.')

        # buffers
        self._lock = Lock()
        self._pressed_keys = []
        self._pressed_keys_new_value = False
        # last activity timestamp
        self._last_activity_timestamp = 0.0
        
        # wmes
        self._last_activity_wme = self._sensor_id.CreateStringWME('last-activity', self._last_activity_bins_map(rospy.Time.now().to_sec()))
        self._pressed_wmes = SetWMEProxy(self._sensor_id, 'pressed')

        # add joystick subscriber
        self._joy_sub = self.createSubscriber(joy_topic, KeyPressed, self.joyCallback)

    def joyCallback(self, msg):
        with self._lock:
            # buffer pressed key list
            self._pressed_keys = msg.keys
            self._pressed_keys_new_value = True

    def update(self):
        with self._lock:
            # check if input was updated
            if not self._pressed_keys_new_value:
                return
            self._pressed_keys_new_value = False
            # copy pressed keys list
            pressed_keys = copy(self._pressed_keys)

        # update pressed WMEs
        changed = self._pressed_wmes.assign(pressed_keys)

        # update activity timestamp
        time_now =  rospy.Time.now().to_sec()
        if changed:
            self._last_activity_timestamp = time_now

        # update activity wme if necessary
        value = self._last_activity_bins_map(time_now - self._last_activity_timestamp)
        if self._last_activity_wme.GetValue() != value:
            self._last_activity_wme.Update(value)


InputModulesLoader.register("joystick", Joystick)
