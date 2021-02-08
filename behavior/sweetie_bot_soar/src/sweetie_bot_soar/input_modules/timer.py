from . import input_module

import rospy

class Clock(input_module.InputModule):

    def __init__(self, name, config, agent):
        super(Clock, self).__init__(name)
        # common sensor initialization
        input_link_id = agent.GetInputLink()
        self._sensor_wme_id = agent.CreateIdWME(input_link_id, name)
        # get parameters
        self._recently_delay = self.getConfigParameter(config, 'recently_delay', allowed_types = (int, float), check_func = lambda v: v > 0.0)
        self._long_time_delay = self.getConfigParameter(config, 'long_time_delay', allowed_types = (int, float), check_func = lambda v: v > 0.0)
        # create clocks
        time_now = rospy.get_rostime().to_sec()
        self._now_id = agent.CreateFloatWME(self._sensor_wme_id, "now", time_now)
        self._recently_id = agent.CreateFloatWME(self._sensor_wme_id, "recently", time_now - self._recently_delay)
        self._long_time_id = agent.CreateFloatWME(self._sensor_wme_id, "long-time", time_now - self._long_time_delay)
        # cycle counter
        self._cycle = 0
        self._cycle_id = agent.CreateIntWME(self._sensor_wme_id, "cycle", self._cycle)

    def update(self):
        # update cycle counter
        self._cycle += 1;
        self._cycle_id.Update(self._cycle)
        # update time
        time_now = rospy.get_rostime().to_sec()
        self._now_id.Update(time_now)
        self._recently_id.Update(time_now - self._recently_delay)
        self._long_time_id.Update(time_now - self._long_time_delay)

    def __del__(self):
        # remove sensor wme
        print('Destory clock')
        self._sensor_wme_id.DestroyWME()

input_module.register("clock", Clock)
