import Python_sml_ClientInterface as sml

from . import input_modules
from . import output_modules

import os, sys
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool

from .soar import Soar, SoarState
from .toggle_debounce import ToggleDebounce

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
        # expose operational state so out-of-SOAR consumers (llm_agent proactive self-talk)
        # can gate on it: latched Bool republished on every start/stop.
        self.operational_pub = rospy.Publisher('~operational', Bool, queue_size=1, latch=True)
        self._publish_operational(False)
        # a TOGGLE is state-relative: an accidental double-fire (rviz double-click, key
        # repeat) inverts the caller's intent. Absorb repeats server-side for every caller.
        self._toggle_debounce = ToggleDebounce(window_s=rospy.get_param('~toggle_debounce', 0.7))
        # create SOAR envelopment
        self.soar = Soar()
        self.timer = None
        self.period = 1.0
        # configure node: by default perform 3 attempts with 5 second period
        attempts = rospy.get_param("~reconfiguration_attempts", 3)
        exit_on_fail = rospy.get_param("~exit_on_configuration_failed", True)
        timer = rospy.Rate(0.2)
        while not self.reconfigure():
            if attempts <= 0:
                if exit_on_fail:
                    sys.exit(1)
                else:
                    rospy.logwarn("SOAR abandoned reconfiguration attempts. Trigger configuration manually.")
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

    def _publish_operational(self, value):
        try:
            self.operational_pub.publish(Bool(bool(value)))
        except Exception:
            pass

    def setOperationalCallback(self, req):
        if req.data:
            result = self.soar.start()
        else:
            result = self.soar.stop()
        # the SetBool path (rviz panel) used to be SILENT - the 2026-07-14 00:42:55
        # operational-off toggle was invisible in the post-mortem logs. Grep seam: keep stable.
        rospy.loginfo('soar: set_operational request: data=%s -> operational=%s (result=%s)',
                      bool(req.data), bool(req.data) and bool(result), bool(result))
        self._publish_operational(bool(req.data) and bool(result))
        return SetBoolResponse(success = result)

    def triggerOperationalCallback(self, req):
        if not self._toggle_debounce.accept(rospy.get_time()):
            rospy.logwarn('soar: toggle_operational repeat within %.1fs debounced '
                          '(double-click / key repeat?)', self._toggle_debounce.window_s)
            return TriggerResponse(success=False, message='debounced: repeated toggle')
        state = self.soar.getState()
        starting = state in (SoarState.STOPPED, SoarState.UNCONFIGURED)
        result = self.soar.start() if starting else self.soar.stop()
        operational = bool(result) if starting else False
        # accepted toggles used to be SILENT (only rejects logged) - that blindness cost a
        # live investigation (T.4). The string below is a grep seam for tests: keep stable.
        rospy.loginfo('soar: toggle_operational accepted: %s -> operational=%s (result=%s)',
                      state, operational, bool(result))
        self._publish_operational(operational)
        return TriggerResponse(success = result)

    def stepCallback(self, req):
        state = self.soar.getState()
        if state != SoarState.STOPPED:
            return SetBoolResponse(success = False, message = 'SOAR must be in STOPPED state.')

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
