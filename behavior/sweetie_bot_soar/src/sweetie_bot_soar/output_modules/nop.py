from .output_module import OutputModule, OutputModulesLoader

import rospy

class NOp(OutputModule):

    def _init(self, name, config):
        # configuration
        self._delay = config.get("delay")
        if not isinstance(self._delay, (int, float)) or self._delay < 0:
            raise RuntimeError("NOp output module: 'delay' must be provided and be a positive number.")

    def startHook(self, cmd_id):
        rospy.sleep(self._delay)
        return "succeed"

OutputModulesLoader.register("nop", NOp)
