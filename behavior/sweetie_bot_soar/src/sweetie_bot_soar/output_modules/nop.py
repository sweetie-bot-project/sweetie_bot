from . import output_module

import rospy

class NOp(output_module.OutputModule):

    def __init__(self, config):
        super(NOp, self).__init__("nop")
        self._delay = config.get("delay")
        if not isinstance(self._delay, (int, float)) or self._delay < 0:
            raise RuntimeError("NOp output module: 'delay' must be provided and be a positive number.")

    def startHook(self, cmd_id):
        rospy.sleep(self._delay)
        return "succeed"

output_module.register("nop", NOp)
