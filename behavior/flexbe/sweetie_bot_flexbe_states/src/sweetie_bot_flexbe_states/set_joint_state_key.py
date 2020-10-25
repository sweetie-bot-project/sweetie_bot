#!/usr/bin/env python
from flexbe_core import Logger
from .internal.set_joint_state_base import SetJointStateBase as EventState

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class SetJointStateKey(EventState):
    '''
    Move robot to named pose using FollowJointState controller. Pose name is specified by input key.
    
    Pose is loaded from binary parameter on Parameter Server as JointState message.
    Then state activate FollowJointState controller and publish pose.
    Movement is considered finished when position error is less then given tolerance.

    -- controller           string          FollowJointState controller namespace.
    -- pose_ns              string          ROS namespace where parameter is located.
    -- tolerance            float           Position tolerance (rad).
    -- timeout              float           Movement timeout (s).
    -- joint_topic          string          Topic where actual pose published.

    ># pose_param           string    ROS parameter in pose_ns which stores JointState message.

    <= done 	                    Finished.
    <= failed 	                    Failed to activate FollowJointState controller.
    <= timeout 	                    Timeout reached.

    '''

    def __init__(self, controller = 'motion/controller/joint_state_head', pose_param = '', pose_ns = 'saved_msgs/joint_state', 
            tolerance = 0.17, timeout = 10.0, joint_topic = "joint_states"):
        super(SetJointStateKey, self).__init__(controller = controller, tolerance = tolerance, timeout = timeout, joint_topic = joint_topic, outcomes = ['done', 'failed', 'timeout'])

        # Cache pose namesapace
        self._pose_ns = pose_ns

    def on_enter(self, userdata):
        self._error = False
        # Load FollowStepSequenceGoal from Parameter Server
        try:
            goal = self.load_joint_state_msg(self._pose_ns, userdata.pose_param)
        except Exception as e:
            Logger.logwarn('SetJointStateKey: unable to load pose from `%s/%s` parameter:\n%s' % (self._pose_ns, userdata.pose_param, str(e)) )
            self._error = True
            return

        # send loaded goal
        self.send_goal(goal)

