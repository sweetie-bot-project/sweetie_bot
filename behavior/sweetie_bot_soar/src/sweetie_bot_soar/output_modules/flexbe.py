from . import output_module

import rospy
import actionlib
from  actionlib import GoalStatus

from flexbe_msgs.msg import BehaviorExecutionAction
from flexbe_msgs.msg import BehaviorExecutionGoal
from flexbe_msgs.msg import BehaviorExecutionResult


class FlexBe(output_module.OutputModule):

    def __init__(self, config):
        super(FlexBe, self).__init__("flexbe")
        # module initialization
        action_ns = config.get("action_ns")
        if not action_ns:
            raise RuntimeError("FlexBe output module: 'action_ns' parameter is not defined.")
        # create actionlib client
        self._action_client = actionlib.SimpleActionClient(action_ns, BehaviorExecutionAction)

    def startHook(self, cmd_id):
        goal = BehaviorExecutionGoal()
        # process WME
        # extract behavior
        name_id = cmd_id.FindByAttribute("name", 0);
        if not name_id: 
            rospy.logerr("flexbe output module: behavior name is empty")
            return "error"
        goal.behavior_name = name_id.GetValueAsString()
        # extract parameteres
        params_id = cmd_id.FindByAttribute("param", 0)
        if params_id and params_id.IsIdentifier():
            params_id = params_id.ConvertToIdentifier()
            # parameters are present
            for i in range(params_id.GetNumberChildren()):
                param_id = params_id.GetChild(i)
                # add parameter and value pair to messsge
                goal.arg_keys.append('/' + param_id.GetAttribute())
                goal.arg_values.append(param_id.GetValueAsString())
        # extract input keys
        input_keys_id = cmd_id.FindByAttribute("input", 0)
        if input_keys_id and input_keys_id.IsIdentifier():
            input_keys_id = input_keys_id.ConvertToIdentifier()
            # parameters are present
            for i in range(input_keys_id.GetNumberChildren()):
                input_key_id = input_keys_id.GetChild(i)
                # add parameter and value pair to messsge
                goal.input_input_keys.append(key_id.GetAttribute())
                goal.input_values.append(input_key_id.GetValueAsString())
        # send goal to server
        rospy.loginfo("flexbe output module: executing behavior %s.", goal.behavior_name)
        self._action_client.send_goal(goal)
        return None

    def updateHook(self, cmd_id):
        # check goal state
        status = self._action_client.get_state()
        if status in [ GoalStatus.ACTIVE, GoalStatus.PENDING ]:
            # Goal is active. Check for abort request.
            abort_id = cmd_id.FindByAttribute("abort", 0)
            if abort_id:
                if abort_id.GetValueAsString() == "hard":
                    # cancel goal immediatelly
                    self._action_client.cancel_goal()
            return None
        # Goal is completed.
        if status == GoalStatus.SUCCEEDED:
            result = self._action_client.get_result()
            cmd_id.CreateStringWME("outcome", result.outcome)
            rospy.loginfo("flexbe output module: behavior outcome is %s.", result.outcome)
            return result.outcome
        if status in [ GoalStatus.RECALLED, GoalStatus.PREEMPTED ]:
            rospy.loginfo("flexbe output module:  behavior execution was preempted.")
            return "aborted"
        if status in [ GoalStatus.REJECTED, GoalStatus.ABORTED ]:
            rospy.loginfo("flexbe output module:  behavior execution has failed with error.")
            return "failed"
        else:
            rospy.loginfo("flexbe output module:  behavior execution has failed: %s.", status)
            return "failed"
        
output_module.register("flexbe", FlexBe)



        



