#!/usr/bin/env python
from flexbe_core import Logger
from .internal.set_joint_state_base import SetJointStateBase as EventState

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class SetJointState(EventState):
    '''
    Move robot to named pose using FollowJointState controller. Pose name is specified as parameter.
    
    Pose is loaded from binary parameter from Parameter Server as JointState message.
    Then state activate FollowJointState controller and publish pose.
    Movement is considered finished when position error is less then given tolerance.

    -- controller           string          FollowJointState controller namespace.
    -- pose_param           string          ROS parameter which stores JointState.
    -- pose_ns              string          ROS namespace where parameter is located.
    -- tolerance            float           Position tolerance (rad).
    -- timeout              float           Movement timeout (s).
    -- joint_topic          string          Topic where actual pose published.

    <= done 	                    Finished.
    <= failed 	                    Failed to activate FollowJointState controller.
    <= timeout 	                    Timeout reached.

    '''

    def __init__(self, controller = 'motion/controller/joint_state_head', pose_param = '', pose_ns = 'saved_msgs/joint_state', 
            tolerance = 0.017, timeout = 10.0, joint_topic = "joint_states"):
        super(SetJointState, self).__init__(controller = controller, tolerance = tolerance, timeout = timeout, joint_topic = joint_topic, outcomes = ['done', 'failed', 'timeout'])

        # Load JointState from Parameter Server to self._target_pose 
        self.load_joint_state_msg(pose_ns, pose_param)

