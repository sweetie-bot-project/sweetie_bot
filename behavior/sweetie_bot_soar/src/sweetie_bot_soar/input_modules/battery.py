from . import input_module
from .input_module import InputModule, InputModulesLoader

from threading import Lock

import rospy
from sensor_msgs.msg import BatteryState

from .bins import BinsMap

class Battery(InputModule):
    def _init(self, name, config, agent):

        # get configuration from parameters
        battery_state_topic = self.getConfigParameter(config, "topic", allowed_types = str)
        try:
            self._battery_levels_bins_map = BinsMap( config['level_bins_map'] )
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError('Battery input module: "level_bins_map" parameter is not present or invalid: ') from e

        # message buffers
        self._lock = Lock()
        self._battery_state_msg = None

        # WME ids cache
        self._level_wme_id = self._sensor_id.CreateStringWME("level", self._battery_levels_bins_map(100))

        # subscriber    
        self._battery_state_sub = self.createSubscriber(battery_state_topic, BatteryState, self.newBatteryStateCallback)

    def newBatteryStateCallback(self, msg):
        # buffer msg
        with self._lock:
            self._battery_state_msg = msg

    def update(self):
        with self._lock:
            # check if input was updated
            if self._battery_state_msg == None:
                return
            # update battery level
            level = self._battery_levels_bins_map( battery_state_msg.percentage )

        if level != self._level_wme_id.GetValue():
            self._level_wme_id.Update(level)

InputModulesLoader.register("battery", Battery)
