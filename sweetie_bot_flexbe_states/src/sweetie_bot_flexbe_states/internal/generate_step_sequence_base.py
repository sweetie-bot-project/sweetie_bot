#!/usr/bin/env python
from flexbe_core import EventState as Dummy
from flexbe_core import Logger
from flexbe_core.proxy import ProxyActionClient

# modules
import rospy
import actionlib
import xmlrpclib

# datatypes
from rospy.rostime import Duration
from std_msgs.msg import Header
from sweetie_bot_clop_generator.msg import MoveBaseAction
from sweetie_bot_clop_generator.msg import MoveBaseGoal
from sweetie_bot_clop_generator.msg import MoveBaseResult
from actionlib_msgs.msg import *


# This is helper class and it should not be interpreted as state. So we trick FlexBe App to ignore it. 
# Dummy is actually EventState but FlexBe App is not able to recognize it.
class GenerateStepSequenceBase(Dummy):
    '''
    Base class for states which synthesize movement with sweetie_bot_clop_generator::MoveBase action request. Action goal is stored on ROS parameter server in serialized form.

    -- controller          string    Action server to execute action.

    <= success 		    Motion have been planned and executed succesfully.
    <= solution_not_found   Planner is unable to find solution.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, obstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failed              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, controller = 'clop_generator', outcomes = ['success', 'solution_not_found', 'partial_movement', 'invalid_pose', 'failure'], *args, **kwargs):
        # Declare outcomes and output keys
	super(GenerateStepSequenceBase, self).__init__(outcomes = outcomes, *args, **kwargs)

        # Connect to action server.
        self._controller = controller
        self._client = ProxyActionClient({self._controller: MoveBaseAction}) # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False

        # log_once support
        self._logged = False
        self._logfunc = { 'info': Logger.loginfo, 'warn': Logger.logwarn, 'err': Logger.logerr, 'hint': Logger.loghint }

    def loadGoalMsg(self, trajectory_ns, trajectory_param):
	if trajectory_ns:
	    trajectory_param = trajectory_ns + '/' + trajectory_param
        try:
            goal_raw = rospy.get_param(trajectory_param)
        except KeyError as e:
            raise KeyError, "Unable to get '" + trajectory_param + "' parameter."

        if not isinstance(goal_raw, xmlrpclib.Binary):
            raise TypeError, "ROS parameter '" + trajectory_param + "' is not a binary data."
        # deserialize
        goal = MoveBaseGoal()
        goal.deserialize(goal_raw.data)
        return goal

    def log_once(self, level, msg):
        if not self._logged:
            self._logfunc[level](msg)
            self._logged = True

    def send_goal(self, goal):
        self._error = False
        self._logged = False
        # Send the goal
        try:
            self._client.send_goal(self._controller, goal)
        except Exception as e:
            Logger.logwarn('GenerateStepSequence: Failed to send the MoveBase command:\n%s' % str(e))
            self._error = True


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'failed'
    
        # Check if the action has been finished
        if self._client.has_result(self._controller):
            state = self._client.get_state(self._controller)
            result = self._client.get_result(self._controller)
            
            if state == GoalStatus.SUCCEEDED:
                # everything is good
                self.log_once('info', 'GenerateStepSequence: MoveBase action succesed: ' + repr(result.error_code) + " " + result.error_string)
                return 'success'
            elif state in [ GoalStatus.ABORTED, GoalStatus.PREEMPTED ]:
                self.log_once('warn', 'GenerateStepSequence: MoveBase action aborted: ' + repr(result.error_code) + " " + result.error_string)
                if result.error_code in [ MoveBase.SOLUTION_NOT_FOUND ]:
                    return 'solution_not_found'
                if result.error_code in [ MoveBase.EXECUTION_FAILED ]:
                    return 'partial_movement'
                if result.error_code in [ MoveBase.INVALID_INITIAL_POSE ]:
                    return 'invalid_pose'

            # Any another kind of error.
            self.log_once('warn', 'GenerateStepSequence: MoveBase action failed (' + repr(state) + '): ' + repr(result.error_code) + " " + result.error_string)
            return 'failure'


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.
        
        if  self._client.is_active(self._controller):
            self._client.cancel(self._controller)
            Logger.loginfo('GenerateStepSequence: Cancel active action goal.')

