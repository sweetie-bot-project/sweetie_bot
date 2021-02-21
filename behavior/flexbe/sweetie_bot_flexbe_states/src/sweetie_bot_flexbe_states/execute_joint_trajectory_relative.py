#!/usr/bin/env python
from .internal.execute_joint_trajectory_base import ExecuteJointTrajectoryBase as EventState

from copy import deepcopy
from flexbe_core import Logger
from flexbe_core.proxy import ProxySubscriberCached
from sensor_msgs.msg import JointState

# clamp function 
def clamp(value, lower, upper):
    if lower > value:
        return lower
    if upper < value:
        return upper
    return value

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class ExecuteJointTrajectoryRelative(EventState):
    '''
    Execute stored control_msgs.msg.FollowJointTrajectory action realtive to current pose, i.e. 
    joint positions stored in the trajectory are added to current robot pose. Joint limits 
    are used to limit resulting joint state values.

    Goal is stored on ROS parameter server in serialized form.
    Parameter name is specified as state parameter.

    -- action_topic        string    Action server to execute action.
    -- trajectory_param    string    ROS parameter which stores FollowJointTrajectoryGoal message.
    -- trajectory_ns       string    Namespace where trajectores are stored. 
    -- joints_limits       dict      Dictionary with joints limits: { <joint_name>: (<lower_limit>, <upper_limit>), ... }
    -- joint_states_topic  string    Topic where current robot pose is being published.

    <= success              Indicate that goal is achived.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, onbstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, action_topic = 'motion/controller/joint_trajectory', trajectory_param = '', trajectory_ns = 'saved_msgs/joint_trajectory', joints_limits = [], joint_states_topic = 'joint_states'):
        # Declare outcomes and output keys
        super(ExecuteJointTrajectoryRelative, self).__init__(outcomes = ['success', 'partial_movement', 'invalid_pose', 'failure'])

        # check joints limits
        if not isinstance(joints_limits, dict) or not all( isinstance(name, str) and isinstance(lower, (int, float)) and isinstance(upper, (int, float)) for name, (lower, upper) in joints_limits.items() ):
            raise ValueError('joints_limits must be list of tuples (<joint_name>, <upper_limit>, <lower_limit>)')
        self._joints_limits = joints_limits

        # create pose subscriber
        self._joints_topic = joint_states_topic
        self._joints_subscriber = ProxySubscriberCached({ self._joints_topic: JointState })

        # Load FollowJointTrajectoryGoal from Parameter Server
        self._goal = self.load_goal_msg(trajectory_ns, trajectory_param)

    def on_enter(self, userdata):
        # get current pose from subscriber
        joints = self._joints_subscriber.get_last_msg(self._joints_topic)
        if joints == None or len(joints.name) != len(joints.position):
            Logger.logwarn('ExecuteJointTrajectoryRelative: bad joint state message.')
            joints = None

        # modify goal 

        # create copy
        goal = deepcopy(self._goal)
        trajectory = goal.trajectory
        # convert point positions from tuples to lists (rospy is the worst)
        for point in trajectory.points:
            point.positions = list(point.positions)
        # modify points positions
        for i in range(len(trajectory.joint_names)):
            joint_name = trajectory.joint_names[i]
            # get current position of joint 
            joint_position = None
            for j in range(len(joints.name)):
                if joints.name[j] == joint_name:
                    joint_position = joints.position[j]
                    break
            # modify position in trajectory if necessary
            if joint_position != None:
                for point in trajectory.points:
                    point.positions[i] += joint_position

            # check joints limits
            limits = self._joints_limits.get(joint_name)
            if limits != None:
                for point in trajectory.points:
                    point.positions[i] = clamp(point.positions[i], limits[0], limits[1])

        # convert point positions from lists to tuples (Python is the worst programming language)
        for point in trajectory.points:
            point.positions = tuple(point.positions)
        
        # send loaded goal
        self.send_goal(goal)

