from .output_module import OutputModule, OutputModulesLoader

import rospy
from sweetie_bot_control_msgs.srv import SetMode, SetModeResponse, SetModeRequest

class SetModeModule(OutputModule):

    def _init(self, name, config):
        # module initialization
        service_ns = self.getConfigParameter(config, "service_ns", allowed_types=str)
        # create Service client
        rospy.wait_for_service(service_ns, timeout=5.0)
        self._set_mode_client = rospy.ServiceProxy(service_ns, SetMode)

    def startHook(self, cmd_id):
        req = SetModeRequest()
        # extract attributes
        mode_id = cmd_id.FindByAttribute("mode", 0)
        if mode_id is None:
            rospy.logerror('set_mode output module: no mode specified.')
            return 'invalid'
        req.mode = mode_id.GetValueAsString()
        res_num = 0
        while True:
            res_id = cmd_id.FindByAttribute("resource", res_num)
            if res_id is None:
                break
            req.resources.append(res_id.GetValueAsString())
        # send request
        resp = self._set_mode_client(req)
        rospy.logdebug('set_mode output module: request: %s, response: %s', req, resp)
        if resp is not None and resp.error_code == SetMode.SUCCESSFUL:
            return "succeed"
        else:
            return 'failed'

OutputModulesLoader.register("set-mode", SetModeModule)
