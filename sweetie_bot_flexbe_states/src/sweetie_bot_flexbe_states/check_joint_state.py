#!/usr/bin/env python
from itertools import izip
import xmlrpclib
import rospy 
from rospy.rostime import Time, Duration

from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import JointState

class CheckJointState(EventState):
    '''
    Check if current robot pose corresponds to known joint state. 
    The list of joint states are loaded from parameter server, where they stored as 
    JointState messages in parameters in serialized form. Provided poses are matched in 
    order in which they are supplied.

    -- outcomes             string[]        Poses to check and state outcomes of state at same time. Outcome 'unknown' should alway present.
    -- pose_ns              string          Parameter namespace where poses are stored.
    -- tolerance            float           Position tolerance (rad).
    -- joint_topic          string          Topic where actual pose published.
    -- timeout              float           Waiting timeout (s).
    '''
    def __init__(self, outcomes = ['unknown'], pose_ns = 'saved_msgs/joint_state', tolerance = 0.17, joint_topic = "joint_states", timeout = 1.0):
        # Process supplied pose list. Note it must be processed befor superclass constructor is called,
        # beacuse it changes 'outcomes' list due to side effect.

        # check if 'unknown' outcome is present
        if 'unknown' not in outcomes:
            raise RuntimeError, '"unknown" should presents in state outcomes'
        # remove 'unknown' while preserving items order
        poses_names = [ outcome for outcome in outcomes if outcome != 'unknown' ]
        # Load poses from paramere server. Pose is loaded in form dict(joint name -> position),
        # but they are stored as a list of tuples (pose_name, pose_dict) to preserve order.
        self._poses = [ (name, self.load_joint_state(pose_ns, name)) for name in poses_names ]

        # Call superclass constructor.
        super(CheckJointState, self).__init__(outcomes = outcomes)

        # Subscribe to the joint topic.
        self._joint_topic = joint_topic 
        self._pose_subscriber = ProxySubscriberCached({ self._joint_topic: JointState })
        
        # Store topic parameter for later use.
        self._tolerance = tolerance
        self._timeout = Duration.from_sec(timeout)

        # timestamp
        self._timestamp = None

    def load_joint_state(self, pose_ns, pose_param):
        # derive parameter full name
        if pose_ns:
            pose_param = pose_ns + '/' + pose_param
        # Load JointState message from Parameter Server
        try:
            goal_raw = rospy.get_param(pose_param)
        except KeyError as e:
            raise KeyError, "CheckJointState: Unable to get '" + pose_param + "' parameter."
        if not isinstance(goal_raw, xmlrpclib.Binary):
            raise TypeError, "CheckJointState: ROS parameter '" + pose_param + "' is not a binary data."
        # deserialize
        msg = JointState()
        msg.deserialize(goal_raw.data)
        # create and return joint index to simplify tolerance check
        return { name: position for name, position in izip(msg.name, msg.position) }
    
    def on_enter(self, userdata):
        # set start timestamp
        self._timestamp = Time.now()

    def execute(self, userdata):
        # check if time elasped
        if Time.now() - self._timestamp > self._timeout:
            Logger.loginfo('CheckJointState: timeout reached, pose is unknown')
            return 'unknown'

        # get current robot pose
        joints_msg = self._pose_subscriber.get_last_msg(self._joint_topic)
        if joints_msg:
            # check if one of the poses matches current
            for pose_name, target_joint_states in self._poses:
                # check given pose
                on_position = True
                for joint_name, pos in izip(joints_msg.name, joints_msg.position):
                    target_pos = target_joint_states.get(joint_name)
                    if target_pos != None:
                        if abs(target_pos - pos) > self._tolerance:
                            on_position = False
                            break
                # check if pose 
                if on_position:
                    Logger.loginfo('CheckJointState: detected pose: ' + pose_name)
                    return pose_name
            # Known poses does not matches
            return 'unknown'


