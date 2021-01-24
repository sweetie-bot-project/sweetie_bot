#!/usr/bin/env python3
import xmlrpc.client
import rospy 
from rospy.rostime import Time, Duration

from flexbe_core import EventState as Dummy
from flexbe_core import Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyActionClient

from sensor_msgs.msg import JointState
from sweetie_bot_control_msgs.msg import SetOperationalAction, SetOperationalGoal, SetOperationalResult


# This is helper class so trick FlexBe App to  ignore it. 
# Dummy is actually EventState but FlexBe App is not able to recognize it.
class SetJointStateBase(Dummy):
    '''
    Base class for states which move robot to named pose using FollowJointState controller. 
    
    Pose is loaded from binary parameter from Parameter Server as JointState message.
    Then state activate FollowJointState controller and publish pose.
    Movement is considered finished when position error is less then given tolerance.

    -- controller           string          FollowJointState controller namespace.
    -- tolerance            float           Position tolerance (rad).
    -- timeout              float           Movement timeout (s).
    -- joint_topic          string          Topic where actual pose published.

    <= done 	                    Finished.
    <= failed 	                    Failed to activate FollowJointState controller.
    <= timeout 	                    Timeout reached.

    '''

    def __init__(self, controller = 'motion/controller/joint_state_head', tolerance = 0.17, timeout = 10.0, 
            joint_topic = "joint_states", outcomes = ['done', 'failed', 'timeout']):
        super(SetJointStateBase, self).__init__(outcomes = outcomes)

        # Store topic parameter for later use.
        self._controller = controller 
        self._joint_topic = joint_topic 
        self._tolerance = tolerance
        self._timeout = Duration.from_sec(timeout)

        # create proxies
        self._action_client = ProxyActionClient({self._controller: SetOperationalAction})
        self._pose_publisher = ProxyPublisher({ self._controller + '/in_joints_ref': JointState })
        self._pose_subscriber = ProxySubscriberCached({ self._joint_topic: JointState })
        
        # timestamp
        self._timestamp = None
        # error in enter hook
        self._error = False

    def load_joint_state_msg(self, pose_ns, pose_param):
        # derive parameter full name
        if pose_ns:
            pose_param = pose_ns + '/' + pose_param
        # Load JointState message from Parameter Server
        try:
            goal_raw = rospy.get_param(pose_param)
        except KeyError as e:
            raise KeyError("SetJointStateBase: Unable to get '" + pose_param + "' parameter.")
        if not isinstance(goal_raw, xmlrpc.client.Binary):
            raise TypeError("SetJointStateBase: ROS parameter '" + pose_param + "' is not a binary data.")
        # deserialize
        self._target_joint_state = JointState()
        self._target_joint_state.deserialize(goal_raw.data)
        # create joint index to simplify tolerance check
        self._joint_target_pose = { name: position for name, position in zip(self._target_joint_state.name, self._target_joint_state.position) }
    
    def on_enter(self, userdata):
        self._error = False
        # activate controller
        actiavtion_request = SetOperationalGoal()
        actiavtion_request.operational = True
        actiavtion_request.resources = self._target_joint_state.name
        try:
            self._action_client.send_goal(self._controller, actiavtion_request)
        except Exception as e:
            Logger.logwarn('SetJointStateBase: Failed to send the SetOperational command:\n%s' % str(e))
            self._error = True
            return
        # set start timestamp
        self._timestamp = Time.now()

    def execute(self, userdata):
        # error in start hook
        if self._error:
            return 'failed'

        # check if controller is active
        if not self._action_client.is_active(self._controller):
            Logger.loginfo('SetJointStateBase: controller was deactivated by external cause.')
            return 'failed';

        # check if time elasped
        if Time.now() - self._timestamp > self._timeout:
            Logger.loginfo('SetJointStateBase: timeout.')
            return 'timeout'

        # publish goal pose
        self._pose_publisher.publish(self._controller+'/in_joints_ref', self._target_joint_state) 

        # check tolerance
        joints_msg = self._pose_subscriber.get_last_msg(self._joint_topic)

        on_position = True
        for name, pos in zip(joints_msg.name, joints_msg.position):
            target_pos = self._joint_target_pose.get(name)
            if (target_pos != None):
                if abs(target_pos - pos) > self._tolerance:
                    on_position = False
                    break
        
        if on_position:
            Logger.loginfo('SetJointStateBase: on position')
            return 'done'

    def on_exit(self, userdata):
        if self._action_client.is_active(self._controller):
            try: 
                self._action_client.cancel(self._controller)
            except Exception as e:
                Logger.logwarn('SetJointStateBase: failed to deactivate `' + self._controller + '` controller:\n%s' % str(e))

