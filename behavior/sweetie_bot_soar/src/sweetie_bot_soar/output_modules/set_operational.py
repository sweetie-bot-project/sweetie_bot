from .output_module import OutputModulesLoader, OutputModule
from ..input_modules.swm import ObjectKeyTuple, SpatialWorldModel

import rospy
import actionlib
from  actionlib import GoalStatus

from sweetie_bot_control_msgs.msg import SetOperationalAction, SetOperationalGoal, SetOperationalResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class SetOperational(OutputModule):

    def _init(self, name, config):
        # get configuration
        action_ns = self.getConfigParameter(config, "controller", allowed_types = str)
        # create actionlib client
        self._set_operational_aclient = actionlib.SimpleActionClient(action_ns, SetOperationalAction)

    def startHook(self, cmd_id):
        goal = SetOperationalGoal(operational = True)
        # extract opertional status
        operational_id = cmd_id.FindByAttribute("operational", 0)
        if operational_id is not None:
            operational_value = operational_id.GetValueAsString()
            if operational_value in ("1", "true", "yes"):
                goal.operational = True
            elif operational_value in ("0", "false", "no"):
                goal.operational = False
            else:
                rospy.logwarn("output module '%s': unknown ^operational value '%s'", self._name, operational_value)
                return 'error'
        # get resources
        while True:
            resource_id = cmd_id.FindByAttribute("resource", len(goal.resources))
            if resource_id is not None:
                goal.resources.append(resource_id.GetValueAsString())
            else:
                break
        # start controller
        try:
            self._set_operational_aclient.send_goal(goal)
        except Exception as e:
            rospy.logwarn('lookat output module: failed to send the SetOperational command:\n%s', str(e))
            return 'error'
        return None

    def _cancel_goal(self, reason):
        self._set_operational_aclient.cancel_goal()
        rospy.loginfo(f"output module '%s': %s", self._name, reason)

    def updateHook(self, cmd_id, external_abort_request):
        # get goal state
        status = self._set_operational_aclient.get_state()

        # Controller is active.
        if status in [ GoalStatus.ACTIVE, GoalStatus.PENDING ]:
            # check for abort request from agent
            # look at can always be interrupted so abort request type is unimportant

            # check for agent abort request
            agent_abort_request = False
            if cmd_id is not None:
                # via ^abort WME
                abort_id = cmd_id.FindByAttribute("abort", 0)
                if abort_id is not None:
                    agent_abort_request = True
            else:
                # by deleting command
                agent_abort_request = True
            if agent_abort_request:
                self._cancel_goal(reason="behavior aborted by agent")
                return "succeed"

            # check for exteral abort request
            if external_abort_request:
                self._cancel_goal(reason="behavior aborted by external request")
                return "succeed"

            # continue exection (ACTIVE, PENDING)
            return None

        # Controller is stopped.
        if status in (GoalStatus.REJECTED, GoalStatus.ABORTED):
            rospy.logerr("output module '%s': activation LookAt controller failed.", self._name)
            return "failed"
        elif status in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            rospy.loginfo("output module '%s': stopped by external reason: %s.", self._name, status)
            return "succeed"

        # continue exection (RECALLING, PREEMPTING)
        return None


OutputModulesLoader.register("set-operational", SetOperational)



        



