import input_module
from random import random

class Rand:

    def __init__(self, agent, config):
        self._agent = agent
        self._input_link_id = agent.GetInputLink()
        # add random value element  
        self._sensor_id = agent.CreateFloatWME(self._input_link_id, "random", random())

    def update(self):
        # update random value
        self._agent.Update(self._sensor_id, random())

    def __del__(self):
        # remove sensor wme
        self._agent.DestroyWME(self._sensor_id)

input_module.register("rand", Rand)
