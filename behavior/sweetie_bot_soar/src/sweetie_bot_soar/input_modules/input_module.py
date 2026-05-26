import rospy

from ..config_parse import get_config_parameter
from ..module import ModulesLoader, UnlodableModule

class InputModulesLoader(ModulesLoader):
    _loader_name = 'input module loader'
    _modules_registry = {}

class InputModule(UnlodableModule):
    def __init__(self, name, config, agent):
        # create sensor WME
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)
        super(InputModule, self).__init__(name, config, agent)

    # input module interface
    def update(self):
        raise NotImplementedError

    # explicit destructor
    def deinit(self):
        if self._destroyed:
            return
        # remove WME
        self._sensor_id.DestroyWME()
        # deinit module
        super(InputModule, self).deinit()

    # helper methods

    @property
    def sensor_id(self):
        return self._sensor_id

    def getConfigParameter(self, config, name, *args, **kwargs):
        try:
            return get_config_parameter(config, name, *args, **kwargs)
        except (KeyError, ValueError, TypeError) as e:
            e.args = (f"input module '{self._name}'",) + e.args
            raise


