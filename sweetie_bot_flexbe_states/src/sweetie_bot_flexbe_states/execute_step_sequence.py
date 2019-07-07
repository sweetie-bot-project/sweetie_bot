#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# modules
import rospy
import actionlib
import xmlrpclib

# datatypes
from rospy.rostime import Duration
from std_msgs.msg import Header
from sweetie_bot_control_msgs.msg import FollowStepSequenceAction
from sweetie_bot_control_msgs.msg import FollowStepSequenceGoal
from sweetie_bot_control_msgs.msg import FollowStepSequenceResult
from actionlib_msgs.msg import *


class ExecuteStepSequence(EventState):
    '''
    Execute stored control_msgs::FollowStepSequence action. Goal is stored on ROS parameter server in serialized form.

    -- controller          string    Action server to execute action.
    -- trajectory_param    string    ROS parameter in trajectory_ns which stores FollowStepSequence message. If is set to None corresponding key value is used.
    -- trajectory_ns       string    Namespace where trajectores are stored. 

    ># trajectory_param    string    ROS parameter which stores FollowStepSequence message. 

    <= success              Indicate that goal is achived.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, obstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, controller = 'motion/controller/step_sequence', trajectory_param = None, trajectory_ns = 'joint_trajectory'):
        # Declare outcomes and output keys
        super(ExecuteStepSequence, self).__init__(outcomes = ['success', 'partial_movement', 'invalid_pose', 'failure'])

        # Load FollowStepSequenceGoal from Parameter Server
        if trajectory_ns:
            trajectory_param = trajectory_ns + '/' + trajectory_param
        try:
            goal_raw = rospy.get_param(trajectory_param)
        except KeyError as e:
            raise KeyError, "ExecuteStepSequence: Unable to get '" + trajectory_param + "' parameter."

        if not isinstance(goal_raw, xmlrpclib.Binary):
            raise TypeError, "ExecuteStepSequence: ROS parameter '" + trajectory_param + "' is not a binary data."
        # deserialize
        self._goal = FollowStepSequenceGoal()
        self._goal.deserialize(goal_raw.data)

        # Connect to action server.
        self._controller = controller
        self._client = ProxyActionClient({self._controller: FollowStepSequenceAction}) # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False

        # log_once support
        self._logged = False
        self._logfunc = { 'info': Logger.loginfo, 'warn': Logger.logwarn, 'err': Logger.logerr, 'hint': Logger.loghint }


    def log_once(self, level, msg):
        if not self._logged:
            self._logfunc[level](msg)
            self._logged = True

    def on_enter(self, userdata):
        self._error = False
        self._logged = False
        # Send the goal
        try:
            self._client.send_goal(self._controller, self._goal)
        except Exception as e:
            Logger.logwarn('ExecuteStepSequence: Failed to send the FollowStepSequence command:\n%s' % str(e))
            self._error = True


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            userdata.result = None
            return 'failure'
    
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
        
        if not self._client.has_result(self._controller):
            self._client.cancel(self._controller)
            userdata.result = None
            Logger.loginfo('ExecuteStepSequence: Cancel active action goal.')

