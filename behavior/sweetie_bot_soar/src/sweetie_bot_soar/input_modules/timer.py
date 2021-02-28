from . import input_module

import rospy

class Clock(input_module.InputModule):

    def __init__(self, name, config, agent):
        super(Clock, self).__init__(name)
        # common sensor initialization
        input_link_id = agent.GetInputLink()
        self._sensor_wme_id = agent.CreateIdWME(input_link_id, name)
        # get parameters
        self._simulation_timestep = self.getConfigParameter(config, 'simulation_timestep', allowed_types = (type(None), int, float), check_func = lambda v: v > 0.0 if v != None else True)
        self._just_delay = self.getConfigParameter(config, 'just_delay', allowed_types = (int, float), check_func = lambda v: v > 0.0)
        self._recently_delay = self.getConfigParameter(config, 'recently_delay', allowed_types = (int, float), check_func = lambda v: v > 0.0)
        self._long_time_delay = self.getConfigParameter(config, 'long_time_delay', allowed_types = (int, float), check_func = lambda v: v > 0.0)
        # create clocks
        if self._simulation_timestep == None:
            time_now = rospy.get_rostime().to_sec()
        else:
            print("SOAR clock input module: timestep debug mode.")
            time_now = 0.0
        # create WMES
        self._now_id = agent.CreateFloatWME(self._sensor_wme_id, "now", time_now)
        self._just_id = agent.CreateFloatWME(self._sensor_wme_id, "just", time_now - self._just_delay)
        self._recently_id = agent.CreateFloatWME(self._sensor_wme_id, "recently", time_now - self._recently_delay)
        self._long_time_id = agent.CreateFloatWME(self._sensor_wme_id, "long-time", time_now - self._long_time_delay)
        # cycle counter
        self._cycle = 0
        self._cycle_id = agent.CreateIntWME(self._sensor_wme_id, "cycle", self._cycle)

    def update(self):
        # check if time is simulated
        self._cycle += 1
        if self._simulation_timestep == None:
            time_now = rospy.get_rostime().to_sec()
        else:
            time_now = self._cycle * self._simulation_timestep
        # update time
        self._now_id.Update(time_now)
        self._just_id.Update(time_now - self._just_delay)
        self._recently_id.Update(time_now - self._recently_delay)
        self._long_time_id.Update(time_now - self._long_time_delay)
        # update cycle counter
        self._cycle_id.Update(self._cycle)

    def __del__(self):
        # remove sensor wme
        self._sensor_wme_id.DestroyWME()

input_module.register("clock", Clock)
