from . import input_module
from .input_module import InputModule

from random import random

class Rand(InputModule):

    def __init__(self, name, config, agent):
        super(Rand, self).__init__(name)
        self._agent = agent
        self._input_link_id = agent.GetInputLink()
        # add random value element  
        self._sensor_id = agent.CreateFloatWME(self._input_link_id, name, random())

    def update(self):
        # update random value
        self._agent.Update(self._sensor_id, random())

    def __del__(self):
        # remove sensor wme
        self._agent.DestroyWME(self._sensor_id)
        # supercalss destructor
        super(Rand, self).__del__()

input_module.register("rand", Rand)
