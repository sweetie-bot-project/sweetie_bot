from . import input_module

from copy import copy
import rospy
from sweetie_bot_joystick.msg import KeyPressed
from .bins import BinsMap

class Joystick:
    def __init__(self, name, config, agent):
        self._joy_sub = None

        input_link_id = agent.GetInputLink()
        # add sensor element  
        self._sensor_id = input_link_id.CreateIdWME(name)
        # configuration
        joy_topic = config.get("topic")
        if not joy_topic:
            raise RuntimeError("Joystick input module: topic parameter is not defined.")
        # add joystick subscriber and buffer
        self._joy_sub = rospy.Subscriber(joy_topic, KeyPressed, self.joyCallback)
        # buffers
        self._pressed_keys = []
        self._pressed_keys_new_value = False
        # last activity timestamp
        self._last_activity_timestamp = 0.0
        try:
            self._last_activity_bins_map = BinsMap( config['last_activity_bins_map'] )
        except KeyError:
            raise RuntimeError('Joystick input module: "last_activity_bins_map" parameters must present.')
        self._last_activity_id = self._sensor_id.CreateStringWME('last-activity', self._last_activity_bins_map(rospy.Time.now().to_sec()))

    def joyCallback(self, msg):
        # buffer pressed key list
        self._pressed_keys = msg.keys
        self._pressed_keys_new_value = True

    def update(self):
        # check if input was updated
        if not self._pressed_keys_new_value:
            return
        self._pressed_keys_new_value = False
        # form list of WMEs for addition and deletion
        add_key_list = copy(self._pressed_keys)
        del_wme_list = []
        # iterate over child wme
        for i in range(self._sensor_id.GetNumberChildren()):
            child_id = self._sensor_id.GetChild(i)
            # check all WME's which symbolize pressed keys
            # form add and del list but do not touch already added elements
            if child_id.GetAttribute() == "pressed":
                value = child_id.GetValueAsString()
                if value in add_key_list:
                    add_key_list.remove(value)
                else:
                    del_wme_list.append(child_id) 
        # del not pressed keys
        for wme_id in del_wme_list:
            wme_id.DestroyWME()
        # add new elements
        for pressed_key in add_key_list:
            self._sensor_id.CreateStringWME("pressed", pressed_key)
        # update activity timestamp
        time_now =  rospy.Time.now().to_sec()
        if add_key_list or del_wme_list:
            self._last_activity_timestamp = time_now
        # update activity wme if necessary
        value = self._last_activity_bins_map(time_now - self._last_activity_timestamp)
        if self._last_activity_id.GetValue() != value:
            self._last_activity_id.Update(value)

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        if self._joy_sub != None:
            self._joy_sub.unregister()

input_module.register("joystick", Joystick)
