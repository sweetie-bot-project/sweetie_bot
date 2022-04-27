from . import output_module

import rospy
import actionlib
from  actionlib import GoalStatus

from sweetie_bot_text_msgs.msg import TextActionAction, TextActionGoal, TextActionResult

class TextAction(output_module.OutputModule):

    def __init__(self, config):
        super(TextAction, self).__init__("text-action")
        # module initialization
        action_ns = config.get("action_ns")
        if not action_ns:
            raise RuntimeError("TextAction output module: 'action_ns' parameter is not defined.")
        # create actionlib client
        self._action_client = actionlib.SimpleActionClient(action_ns, TextActionAction)

    def startHook(self, cmd_id):
        goal = TextActionGoal()
        # process WME
        # extract command 
        name_id = cmd_id.FindByAttribute("command", 0);
        type_id = cmd_id.FindByAttribute("type", 0);
        if name_id is None or type_id is None:
            rospy.logerr("TextAction output module: type and command attributes must present.")
            return "error"
        goal.command.type = type_id.GetValueAsString()
        goal.command.command = name_id.GetValueAsString()
        # send goal to server
        rospy.loginfo("TextAction output module: executing (%s, %s).", goal.command.type, goal.command.command)
        self._action_client.send_goal(goal)
        return None

    def updateHook(self, cmd_id):
        # check goal state
        status = self._action_client.get_state()
        if status in [ GoalStatus.ACTIVE, GoalStatus.PENDING ]:
            # Goal is active. Check for abort request.
            abort_id = cmd_id.FindByAttribute("abort", 0)
            if abort_id is not None:
                self._action_client.cancel_goal()
            return None
        # Goal is completed.
        if status == GoalStatus.SUCCEEDED:
            return 'succeed'
        if status in [ GoalStatus.RECALLED, GoalStatus.PREEMPTED ]:
            rospy.loginfo("TextAction output module:  behavior execution was preempted.")
            return "aborted"
        if status in [ GoalStatus.REJECTED, GoalStatus.ABORTED ]:
            rospy.loginfo("TextAction output module:  behavior execution has failed with error.")
            return "failed"
        else:
            rospy.loginfo("TextAction output module:  behavior execution has failed: %s.", status)
            return "failed"
        
output_module.register("text-action", TextAction)



        



