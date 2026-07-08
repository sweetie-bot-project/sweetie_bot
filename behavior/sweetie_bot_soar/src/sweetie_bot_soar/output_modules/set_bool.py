from .output_module import OutputModule, OutputModulesLoader

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

class SetBoolService(OutputModule):

    def _init(self, name, config):
        # module initialization
        service_ns = self.getConfigParameter(config, "service_ns", allowed_types=str)
        # create Service client
        rospy.wait_for_service(service_ns, timeout=5.0)
        self._set_bool_client = rospy.ServiceProxy(service_ns, SetBool)

    def startHook(self, cmd_id):
        # extract attributes
        value_id = cmd_id.FindByAttribute("value", 0)
        if value_id is None:
            rospy.logerror('set_bool output module: no value specified.')
            return 'invalid'
        value = value_id.GetValueAsString()
        if value in ('true', '1'):
            value = True
        elif value in ('false', '0'):
            value = False
        else:
            rospy.logerror('set_bool output module: value must be "true" or "false".')
            return 'invalid'

        # send request
        try:
            req = SetBoolRequest(data = value)
            resp = self._set_bool_client(req)
            rospy.logdebug('set_bool output module: request: %s, response: %s', req, resp)
            if resp is not None and resp.succeed:
                return "succeed"
            else:
                rospy.logwarn('set bool output module: SetBool failed: %s', resp.message)
                return 'failed'
        except rospy.ServiceException as e:
            rospy.logwarn('set bool output module: SetBool failed: %s', e)
            return 'failed'

OutputModulesLoader.register("set-bool-service", SetBoolService)
