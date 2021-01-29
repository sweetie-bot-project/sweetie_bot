from . import input_module

import rospy
from sensor_msgs.msg import BatteryState

from .bins import BinsMap

class Battery:
    def __init__(self, name, config, agent):
        self._battery_state_sub = None

        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)

        # get configuration from parameters
        battery_state_topic = config.get("topic")
        if not battery_state_topic or not isinstance(battery_state_topic, str):
            raise RuntimeError("Battery input module: 'topic' parameter is not defined or is not string.")
        try:
            self._battery_levels_bins_map = BinsMap( config['level_bins_map'] )
        except KeyError:
            raise RuntimeError('Battery input module: "level_bins_map" parameter must present.')

        # subscriber    
        self._battery_state_sub = rospy.Subscriber(battery_state_topic, BatteryState, self.newBatteryStateCallback)

        # message buffers
        self._battery_state_msg = None
        # WME ids cache
        self._level_wme_id = self._sensor_id.CreateStringWME("level", self._battery_levels_bins_map(100))

    def newBatteryStateCallback(self, msg):
        # buffer msg
        self._battery_state_msg = msg

    def update(self):
        # check if input was updated
        if self._battery_state_msg == None:
            return

        # update battery level
        level = self._battery_levels_bins_map( self._battery_state_msg.percentage )
        if level != self._level_wme_id.GetValue():
            self._level_wme_id.Update(level)

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        if self._battery_state_sub:
            self._battery_state_sub.unregister()

input_module.register("battery", Battery)
