from .output_module import OutputModule, OutputModulesLoader

import rospy

class NOp(OutputModule):

    def _init(self, name, config):
        # configuration
        self._delay = self.getConfigParameter(config, 'delay', allowed_types=(float, int), check_func=lambda d: d > 0.0, error_desc='must be positive')

    def startHook(self, cmd_id):
        rospy.sleep(self._delay)
        return "succeed"

OutputModulesLoader.register("nop", NOp)
