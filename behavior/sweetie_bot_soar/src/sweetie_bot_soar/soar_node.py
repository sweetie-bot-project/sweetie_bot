import Python_sml_ClientInterface as sml

from . import input_modules
from . import output_modules

import os, sys
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse

from .soar import Soar, SoarState

class SoarNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.configured = False
        # create node interface 
        self.reconfigure_srv = rospy.Service('~reconfigure', Trigger, self.reconfigureCallback)
        self.reload_prod_srv = rospy.Service('~reload_prod', Trigger, self.reloadProdCallback)
        self.set_operational_srv = rospy.Service('~set_operational', SetBool, self.setOperationalCallback)
        self.trigger_operational_srv = rospy.Service('~toggle_operational', Trigger, self.triggerOperationalCallback)
        self.step_srv = rospy.Service('~step', Trigger, self.stepCallback)
        # create SOAR envelopment
        self.soar = Soar()
        self.timer = None
        self.period = 1.0
        # configure node: by default perform 3 attempts with 5 second period
        attempts = rospy.get_param("~reconfiguration_attempts", 3)
        timer = rospy.Rate(0.2)
        while not self.reconfigure():
            if attempts <= 0:
                rospy.logwarn("SOAR abandoned reconfiguration attempts.")
                break
            else:
                attempts -= 1
                rospy.logwarn("SOAR will attemt to reconfigure in 5 seconds.")
                timer.sleep()
                        

    def reconfigureCallback(self, req):
        self.soar.stop()
        self.soar.cleanup()
        success = self.soar.configure()
        return TriggerResponse(success = success)

    def reloadProdCallback(self, req):
        rospy.logerr('reload_prod service not implemented yet.')
        return TriggerResponse(success = False, message = 'Service is not implemented.')

    def setOperationalCallback(self, req):
        state = self.soar.getState()
        if state == SoarState.UNCONFIGURED:
            return SetBoolResponse(success = False, message = 'Node is not configured.')

        if req.data:
            result = self.soar.start()
        else:
            result = self.soar.stop()
    
        return SetBoolResponse(success = result)

    def triggerOperationalCallback(self, req):
        state = self.soar.getState()
        if state == SoarState.UNCONFIGURED:
            return SetBoolResponse(success = False, message = 'Node is not configured.')

        if state == SoarState.STOPPED:
            result = self.soar.start()
        else:
            result = self.soar.stop()

        return TriggerResponse(success = result)

    def stepCallback(self, req):
        state = self.soar.getState()
        if state == SoarState.UNCONFIGURED:
            return SetBoolResponse(success = False, message = 'Node is not configured.')
        # invoke step
        result = self.soar.step()
 
        if result:
            return TriggerResponse(success = True)
        else:
            return TriggerResponse(success = False, message = 'Errors during execution.')

    def reconfigure(self):
        # reset configuration
        self.soar.stop()
        self.soar.cleanup()

        # read timer parameters
        autostart = rospy.get_param("~autostart", True)
        if not isinstance(autostart, bool):
            rospy.logerr("`autostart` parameter must be boolean.")
            return False
        # reconfigure SOAR
        if not self.soar.configure():
            rospy.logerr("SOAR configuration failed.")
            return False
        # if everything OK start timer
        if autostart:
            return self.soar.start()
        # configuration is finished
        return True

def main():
    # SOAR initialization
    soar_node = SoarNode("soar")
    # ROS main loop
    rospy.spin()
