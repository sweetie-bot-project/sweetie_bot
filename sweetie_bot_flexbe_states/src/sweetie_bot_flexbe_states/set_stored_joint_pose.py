#!/usr/bin/env python
from itertools import izip
import xmlrpclib

import rospy
from rospy.rostime import Time, Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyActionClient

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from sweetie_bot_control_msgs.msg import SetOperationalAction, SetOperationalGoal, SetOperationalResult


class SetStoredJointPose(EventState):
    '''
    Move robot to named pose using FollowJointState controller. 
    
    Pose is loaded from binary parameter from Parameter Server as JointState message.
    Then state activate FollowJointState controller and publish pose.
    Movement is considered finished when position error is less then given tolerance.

    -- controller           string          FollowJointState controller namespace.
    -- pose_param           string          ROS parameter which stores JointState.
    -- tolerance            float           Position tolerance (rad).
    -- timeout              float           Movement timeout (s).
    -- joint_topic          string          Topic where actual pose published.
    -- resources            string[]        List of controlled chains.

    <= done 	                    Finished.
    <= failed 	                    Failed to activate FollowJointState controller.
    <= timeout 	                    Timeout reached.

    '''

    def __init__(self, controller = 'motion/controller/joint_state_head', pose_param = 'joint_pose', tolerance = 0.17, timeout = 10.0, 
            joint_topic = "joint_states", resources = ['leg1','leg2','leg3','leg4','head','eyes']):
        super(SetStoredJointPose, self).__init__(outcomes = ['done', 'failed', 'timeout'])

        # Store topic parameter for later use.
        self._controller = controller 
        self._joint_topic = joint_topic 
        self._tolerance = tolerance
        self._timeout = Duration.from_sec(timeout)
        self._resources = resources

        # create proxies
        self._action_client = ProxyActionClient({self._controller: SetOperationalAction})
        self._pose_publisher = ProxyPublisher({ self._controller + '/in_joints_ref': JointState })
        self._pose_subscriber = ProxySubscriberCached({ self._joint_topic: JointState })
        
        # Load JointState message from Parameter Server
        try:
            goal_raw = rospy.get_param(pose_param)
        except KeyError as e:
            raise KeyError, "SetStoredJointPose: Unable to get '" + pose_param + "' parameter."
        if not isinstance(goal_raw, xmlrpclib.Binary):
            raise TypeError, "SetStoredJointPose: ROS parameter '" + pose_param + "' is not a binary data."
        # deserialize
        self._target_joint_state = JointState()
        self._target_joint_state.deserialize(goal_raw.data)
        # create joint index to simplify tolerance check
        self._joint_target_pose = { name: position for name, position in izip(self._target_joint_state.name, self._target_joint_state.position) }

        # timestamp
        self._timestamp = None
        # error in enter hook
        self._error = False

    
    def on_enter(self, userdata):
        self._error = False
        # activate controller
        actiavtion_request = SetOperationalGoal()
        actiavtion_request.operational = True
        actiavtion_request.resources = self._resources
        try:
            self._action_client.send_goal(self._controller, actiavtion_request)
        except Exception as e:
            Logger.logwarn('SetStoredJointPose: Failed to send the SetOperational command:\n%s' % str(e))
            self._error = True
            return
        # set start timestamp
        self._timestamp = Time.now()

        Logger.loginfo('SetStoredJointPose started.')

    def execute(self, userdata):
        # error in start hook
        if self._error:
            return 'failed'

        # check if controller is active
        if self._action_client.has_result(self._controller):
            Logger.loginfo('SetStoredJointPose: controller was deactivated by external cause.')
            return 'failed';

        # check if time elasped
        if Time.now() - self._timestamp > self._timeout:
            Logger.loginfo('SetStoredJointPose: controller was deactivated by external cause.')
            return 'timeout'

        # publish goal pose
        self._pose_publisher.publish(self._controller+'/in_joints_ref', self._target_joint_state) 

        # check tolerance
        joints_msg = self._pose_subscriber.get_last_msg(self._joint_topic)

        on_position = True
        for name, pos in izip(joints_msg.name, joints_msg.position):
            target_pos = self._joint_target_pose.get(name)
            if (target_pos != None):
                if abs(target_pos - pos) > self._tolerance:
                    on_position = False
                    break
        
        if on_position:
            Logger.loginfo('SetStoredJointPose: on position')
            return 'done'

    def on_exit(self, userdata):
        if not self._action_client.has_result(self._controller):
            try: 
                self._action_client.cancel(self._controller)
            except Exception as e:
                Logger.logwarn('SetStoredJointPose: failed to deactivate `' + self._controller + '` controller:\n%s' % str(e))

