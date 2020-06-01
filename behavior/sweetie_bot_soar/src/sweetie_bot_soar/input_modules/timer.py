import input_module
import rospy

class Clock:
    def __init__(self, agent, config):
        # common sensor initialization
        self.agent = agent
        self.input_link_id = agent.GetInputLink()
        self.sensor_wme_id = agent.CreateIdWME(self.input_link_id, "clock")
        # create wall clock
        self.time_id = agent.CreateFloatWME(self.sensor_wme_id, "time", rospy.get_rostime().to_sec())
        # cycle counter
        self.cycle = 0
        self.cycle_id = agent.CreateIntWME(self.sensor_wme_id, "cycle", self.cycle)

    def update(self):
        # update wall clock and cycle counter
        self.cycle += 1
        self.agent.Update(self.cycle_id, self.cycle)
        self.agent.Update(self.time_id, rospy.get_rostime().to_sec())

    def __del__(self):
        # remove sensor wme
        self.agent.DestroyWME(self.sensor_wme_id)

input_module.register("clock", Clock)
