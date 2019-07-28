#!/usr/bin/env python
import copy

from flexbe_core import EventState as Dummy
from flexbe_core import Logger
from flexbe_core.proxy import ProxyActionClient

# modules
import rospy
import actionlib
import xmlrpclib

# datatypes
from rospy.rostime import Duration
from sweetie_bot_control_msgs.msg import FollowStepSequenceAction
from sweetie_bot_control_msgs.msg import FollowStepSequenceGoal
from sweetie_bot_control_msgs.msg import FollowStepSequenceResult
from actionlib_msgs.msg import GoalStatus

# This is helper class so trick FlexBe App to  ignore it. 
# Dummy is actually EventState but FlexBe App is not able to recognize it.
class ExecuteStepSequenceBase(Dummy): 
    '''
    Base class for control_msgs::FollowStepSequence action FlexBe states. It fully implements interaction wit action server
    but does not implement FollowStepSequenceGoal message retriving logic.

    -- controller          string    Action server to execute action.

    <= success              Indicate that goal is achived.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, obstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, controller = 'motion/controller/step_sequence', outcomes = ['success', 'partial_movement', 'invalid_pose', 'failure'], input_keys = [], *args, **kwargs):
        # Declare outcomes and output keys
        super(ExecuteStepSequenceBase, self).__init__(outcomes = outcomes, input_keys = input_keys, *args, **kwargs)

        # Connect to action server.
        self._controller = controller
        self._client = ProxyActionClient({self._controller: FollowStepSequenceAction}) # pass required clients as dict (topic: type)

        # Pass outcome from on_enter() or on_resume()
        self._outcome = None

        # log_once support
        self._logged = False
        self._logfunc = { 'info': Logger.loginfo, 'warn': Logger.logwarn, 'err': Logger.logerr, 'hint': Logger.loghint }

    def load_goal_msg(self, trajectory_ns, trajectory_param):
        # derive parameter full name
        if trajectory_ns:
            trajectory_param = trajectory_ns + '/' + trajectory_param

        # Load FollowStepSequenceGoal from Parameter Server
        goal_raw = rospy.get_param(trajectory_param)

        if not isinstance(goal_raw, xmlrpclib.Binary):
            raise TypeError, "ExecuteStepSequence: ROS parameter '" + trajectory_param + "' is not a binary data."
        # deserialize
        goal = FollowStepSequenceGoal()
        goal.deserialize(goal_raw.data)
        return goal

    def log_once(self, level, msg):
        if not self._logged:
            self._logfunc[level](msg)
            self._logged = True

    def send_goal(self, goal):
        self._outcome = None
        self._logged = False
        # Remove odometry information from the goal message
        del goal.base_motion.points[:]
        # Send the goal
        try:
            self._client.send_goal(self._controller, goal)
        except Exception as e:
            Logger.logwarn('ExecuteStepSequence: Failed to send the FollowStepSequence command:\n%s' % str(e))
            self._outcome = 'failure'


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._outcome:
            return self._outcome
    
        # Check if the action has been finished
        if self._client.has_result(self._controller):
            state = self._client.get_state(self._controller)
            result = self._client.get_result(self._controller)
            
            if state == GoalStatus.SUCCEEDED:
                # everything is good
                self.log_once('info', 'ExecuteStepSequence: FollowStepSequence action succesed: ' + repr(result.error_code) + " " + result.error_string)
                return 'success'
            elif state in [ GoalStatus.ABORTED, GoalStatus.PREEMPTED ]:
                # perhaps we stuck in midway due to tolerance error or controller switch.
                self.log_once('warn', 'ExecuteStepSequence: FollowStepSequence action aborted: ' + repr(result.error_code) + " " + result.error_string)
                return 'partial_movement'
            elif state in [ GoalStatus.REJECTED, GoalStatus.RECALLED ]:
                # Execution has not started, perhaps due to invalid goal.
                self.log_once('warn', 'ExecuteStepSequence: FollowStepSequence action rejected: ' + repr(result.error_code) + " " + result.error_string)
                if result.error_code in [ FollowStepSequence.TOLERANCE_VIOLATED, FollowStepSequence.UNABLE_TO_APPEND]:
                    # Starting pose is invalid
                    return 'invalid_pose'
            # Any another kind of error.
            self.log_once('warn', 'ExecuteStepSequence: FollowStepSequence action failed (' + repr(state) + '): ' + repr(result.error_code) + " " + result.error_string)
            return 'failure'


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.
        
        if self._client.is_active(self._controller):
            self._client.cancel(self._controller)
            Logger.loginfo('ExecuteStepSequence: Cancel active action goal.')

