from . import input_module
from .input_module import InputModule

from random import random

class Rand:

    def __init__(self, name, config, agent):
        # create WME
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateFloatWME(name, random())

    def update(self):
        # update random value
        self._sensor_id.Update(random())

    def __dell__(self):
        # remove WME
        self._sensor_id.DestroyWME()

input_module.register("rand", Rand)
