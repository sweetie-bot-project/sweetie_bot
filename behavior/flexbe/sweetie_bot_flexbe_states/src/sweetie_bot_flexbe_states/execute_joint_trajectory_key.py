#!/usr/bin/env python
from .internal.execute_joint_trajectory_base import ExecuteJointTrajectoryBase as EventState
from flexbe_core import Logger

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class ExecuteJointTrajectoryKey(EventState):
    '''
    Execute stored control_msgs.msg.FollowJointTrajectory action. Goal is stored on ROS parameter server in serialized form.
    Parameter name is specified via state input key.

    -- conroller        string    Action server to execute action.
    -- trajectory_ns       string    Namespace where trajectores are stored. 

    ># trajectory_param    string    ROS parameter in trajectory_ns which stores FollowStepSequence message.

    <= success              Indicate that goal is achived.
    <= unavailable          Unable to load desired movement.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, onbstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, controller = 'motion/controller/joint_trajectory', trajectory_ns = 'saved_msgs/joint_trajectory'):
        # Declare outcomes and output keys
        super(ExecuteJointTrajectoryKey, self).__init__(controller = controller, outcomes = ['success', 'unavalible', 'partial_movement', 'invalid_pose', 'failure'], input_keys = ['trajectory_param'])
        # save namspace
        self._trajectory_ns = trajectory_ns

    def on_enter(self, userdata):
        self._outcome = None
        # Load FollowStepSequenceGoal from Parameter Server
        try:
            goal = self.load_goal_msg(self._trajectory_ns, userdata.trajectory_param)
            self._trajectory_info = userdata.trajectory_param
        except Exception as e:
            Logger.logwarn('ExecuteJointTrajectoryKey: unable to load trajectory from `%s/%s` parameter:\n%s' % (self._trajectory_ns, userdata.trajectory_param, str(e)) )
            self._outcome = 'unavaliable'
            return

        # send loaded goal
        self.send_goal(goal)

