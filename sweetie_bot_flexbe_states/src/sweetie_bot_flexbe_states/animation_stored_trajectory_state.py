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
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult
from actionlib_msgs.msg import *
#import trajectory_msgs.msg


class AnimationStoredJointTrajectoryState(EventState):
    '''
    Execute stored control_msgs::FollowJointTrajectoryAction. Goal is stored on ROS parameter server in serialized form.

    -- action_topic        string    Action server to execute action.
    -- trajectory_param     string    ROS parameter which stores FollowJointTrajectoryGoal message.

    #> result       class   Resulting FollowJointTrajectoryResult message.

    <= success              Indicate that goal is achived.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, onbstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''
    ##-- joint_trajectory_storage     string    ROS namespace containing stored trajectories.

    def __init__(self, action_topic, trajectory_param):
        # Declare outcomes and output keys
        super(AnimationStoredJointTrajectoryState, self).__init__(outcomes = ['success', 'partial_movement', 'invalid_pose', 'failure'],
                output_keys = ['result'])

        print(trajectory_param)

        # Load FollowJointTrajectoryGoal from Parameter Server
        goal_raw = rospy.get_param(trajectory_param)
        if not isinstance(goal_raw, xmlrpclib.Binary):
            raise TypeError, "AnimationStoredTrajectoryState: ROS parameter '" + trajectory_param + "' is not a binary data."
        # deserialize
        self._goal = FollowJointTrajectoryGoal()
        self._goal.deserialize(goal_raw.data)

        # Connect to action server.
        self._topic = action_topic
        self._client = ProxyActionClient({self._topic: FollowJointTrajectoryAction}) # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False

    def on_enter(self, userdata):
        # Send the goal
        self._error = False
        try:
            self._client.send_goal(self._topic, self._goal)
        except Exception as e:
            Logger.logwarn('Failed to send the FollowJointTrajectory command:\n%s' % str(e))
            self._error = True


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            userdata.result = None
            return 'failure'
    
        # Check if the action has been finished
        if self._client.has_result(self._topic):
            state = self._client.get_state(self._topic)
            result = self._client.get_result(self._topic)
            
            # Save output key 
            userdata.result = result

            if state == GoalStatus.SUCCEEDED:
                # everything is good
                Logger.loginfo('FollowJointTrajectory succesed: ' + repr(result.error_code) + " " + result.error_string)
                return 'success'
            elif state in [ GoalStatus.ABORTED, GoalStatus.PREEMPTED ]:
                # perhaps we stuck in midway due to tolerance error or controller switch.
                Logger.loginfo('FollowJointTrajectory aborted: ' + repr(result.error_code) + " " + result.error_string)
                return 'partial_movement'
            elif state in [ GoalStatus.REJECTED, GoalStatus.RECALLED ]:
                # Execution has not started, perhaps due to invalid goal.
                Logger.loginfo('FollowJointTrajectory rejected: ' + repr(result.error_code) + " " + result.error_string)
                if result.error_code in [ FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED, FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED]:
                    # Starting pose is invalid
                    return 'invalid_pose'
            # Any another kind of error.
            Logger.loginfo('FollowJointTrajectory failed (' + repr(state) + '): ' + repr(result.error_code) + " " + result.error_string)
            return 'failure'


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.
        
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            userdata.result = None
            Logger.loginfo('Cancelled active action goal.')

