from .output_module import OutputModule, OutputModulesLoader

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_msgs.msg import Bool

class HoldBoolService(OutputModule):

    def _init(self, name, config):
        # module initialization
        service_ns = self.getConfigParameter(config, "service_ns", allowed_types=str)
        # create Service client
        rospy.wait_for_service(service_ns, timeout=5.0)
        self._set_bool_client = rospy.ServiceProxy(service_ns, SetBool)

    def _callSetBool(self, value):
        try:
            req = SetBoolRequest(data = value)
            resp = self._set_bool_client(req)
            rospy.logdebug('set_bool output module: request: %s, response: %s', req, resp)
            if resp.succeed:
                return True
            else:
                rospy.logwarn('set bool output module: SetBool failed: %s', resp.message)
                return False
        except rospy.ServiceException as e:
            rospy.logwarn('set bool output module: SetBool error: %s', e)
            return False

    def startHook(self, cmd_id):
        succeed = self._callSetBool(True)
        if succeed:
            return None
        else:
            return 'failed'

    def updateHook(self, cmd_id, abort_request):
        # check if stop requested
        if (cmd_id is None) or (abort_request is not None):
            succeed = self._callSetBool(False)
            if succeed:
                return 'succeed'
            else:
                return 'failed'
        # return None to continue
        return None

OutputModulesLoader.register("hold-bool-service", HoldBoolService)

class HoldBoolTopic(OutputModule):

    def _init(self, name, config):
        # module initialization
        topic = self.getConfigParameter(config, "topic", allowed_types=str)
        self._active_state = not self.getConfigParameter(config, "inverse", allowed_types=bool, default_value=False)
        # create Service client
        self._set_bool_pub = rospy.Publisher(topic, Bool, queue_size=5)

    def startHook(self, cmd_id):
        self._set_bool_pub.publish(Bool(data = self._active_state))
        return None

    def updateHook(self, cmd_id, abort_request):
        # check if stop requested
        if (cmd_id is None) or (abort_request is not None):
            self._set_bool_pub.publish(Bool(data = not self._active_state))
            return 'succeed'
        # return None to continue
        return None

OutputModulesLoader.register("hold-bool-topic", HoldBoolTopic)
