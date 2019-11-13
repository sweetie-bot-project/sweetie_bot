#!/usr/bin/env python
from internal.execute_joint_trajectory_base import ExecuteJointTrajectoryBase as EventState

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class ExecuteJointTrajectory(EventState):
    '''
    Execute stored control_msgs.msg.FollowJointTrajectory action. Goal is stored on ROS parameter server in serialized form.
    Parameter name is specified as state parameter.

    -- action_topic        string    Action server to execute action.
    -- trajectory_param    string    ROS parameter which stores FollowJointTrajectoryGoal message.
    -- trajectory_ns       string    Namespace where trajectores are stored. 

    <= success              Indicate that goal is achived.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, onbstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, action_topic = 'motion/controller/joint_trajectory', trajectory_param = '', trajectory_ns = 'saved_msgs/joint_trajectory'):
        # Declare outcomes and output keys
        super(ExecuteJointTrajectory, self).__init__(outcomes = ['success', 'partial_movement', 'invalid_pose', 'failure'])

        # Load FollowJointTrajectoryGoal from Parameter Server
        self._goal = self.load_goal_msg(trajectory_ns, trajectory_param)


    def on_enter(self, userdata):
        # send loaded goal
        self.send_goal(self._goal)
