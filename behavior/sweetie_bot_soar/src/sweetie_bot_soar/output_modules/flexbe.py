from .output_module import OutputModule, OutputModulesLoader

import rospy
import actionlib
from  actionlib import GoalStatus

from flexbe_msgs.msg import BehaviorExecutionAction
from flexbe_msgs.msg import BehaviorExecutionGoal
from flexbe_msgs.msg import BehaviorExecutionResult


class FlexBe(OutputModule):

    def _init(self, name, config):
        # module initialization
        action_ns = config.get("action_ns")
        if not action_ns:
            raise RuntimeError("FlexBe output module: 'action_ns' parameter is not defined.")
        # create actionlib client
        self._action_client = actionlib.SimpleActionClient(action_ns, BehaviorExecutionAction)
        self._interruptable = False

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
        # extract interruptable attibute
        self._interruptable = False
        interruptable_id = cmd_id.FindByAttribute("interruptable", 0)
        if interruptable_id is not None:
            if interruptable_id.GetValueAsString() == 'volatile':
                self._interruptable = True
        # send goal to server
        rospy.loginfo("flexbe output module: executing behavior %s with paramters %s", goal.behavior_name, {key: value for key, value in zip(goal.arg_keys, goal.arg_values)})
        self._action_client.send_goal(goal)
        return None

    def _execute_abort_request(self, abort_request, source = ''):
        if abort_request == 'forced_stop':
            self._action_client.cancel_goal()
            rospy.logwarn("output module '%s': %s request for forced abort.", self._name, source)
            return True
        else:
            if abort_request != 'graceful_stop':
                rospy.logwarn("output module '%s': %s request for unknowmn stop method '%s', 'graceful_stop' is assumed.", self._name, source, abort_request)
            if self._interruptable:
                self._action_client.cancel_goal()
                rospy.loginfo("output module '%s': %s request for graceful abort.", self._name, source)
                return True
        return False

    def updateHook(self, cmd_id, external_abort_request):
        # get  goal state
        status = self._action_client.get_state()

        # Goal is active. 
        if status in (GoalStatus.ACTIVE, GoalStatus.PENDING):
            # check for agent abort request.
            agent_abort_request = None
            if cmd_id is None:
                # command was deleted 
                agent_abort_request = 'graceful_stop'
            else:
                abort_id = cmd_id.FindByAttribute("abort", 0)
                if abort_id is not None:
                    agent_abort_request = abort_id.GetValueAsString()
            if agent_abort_request is not None:
                if self._execute_abort_request(agent_abort_request, 'agent'):
                    return 'aborted'

            # check for external request
            if external_abort_request is not None:
                if self._execute_abort_request(external_abort_request, 'external'):
                    return 'aborted'
            
            # continue execution
            return None

        # Goal is completed.
        if status == GoalStatus.SUCCEEDED:
            result = self._action_client.get_result()
            if result != None:
                cmd_id.CreateStringWME("outcome", result.outcome)
                rospy.loginfo("flexbe output module: behavior outcome is %s.", result.outcome)
                return result.outcome
            else:
                cmd_id.CreateStringWME("outcome", "succeed")
                rospy.logwarn("flexbe output module: goal status SUCCEEDED, action result is None, assume behavior outcome is succeed.")
        if status in (GoalStatus.RECALLED, GoalStatus.PREEMPTED):
            rospy.loginfo("flexbe output module:  behavior execution was preempted.")
            return "aborted"
        if status in (GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.LOST):
            rospy.loginfo("flexbe output module:  behavior execution has failed with error.")
            return "failed"

        # Contine execution (RECALLING, PREEMPTING)
        return None
        
OutputModulesLoader.register("flexbe", FlexBe)
