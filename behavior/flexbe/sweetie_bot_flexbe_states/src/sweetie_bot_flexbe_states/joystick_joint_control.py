#!/usr/bin/env python
import rospy
from numpy import clip
#from collections import namedtuple

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

class JoystickJointControl(EventState):
    '''
    Control joint position with joystick. 

    If button is pressed joint is moving with defined in configuration speed. 
    For axes speed is modulated with axis value. If speed is equal to zero,
    then position of joint is defined directly by axis position.

    Note that one button may control multiple joints.
   
    -- joints_topic                     string          Topic with current robot pose (JointState)
    -- goal_joints_topic                string          Topic to publish desired joint positions.
    -- joy_topic                        string          Joystick 
    -- buttons                          object          Array with tuples: (button number, speed, joint name, min, max).
    -- axes                             object          Array with tuples: (axis number, speed, joint name, min, max). 
    -- timeout                          number          Exit state if no joystick input is provided for timeout. If timeout is negative waits forever.

    <= timeout                                          No joystick activity for given timeout.

    '''

    def __init__(self, joints_topic='joint_states', goal_joints_topic='goal_joint_states', joy_topic = 'joy', buttons = [], axes = [], timeout = 5.0):
        super(JoystickJointControl, self).__init__(outcomes = ['done'])

        # store state parameter for later use.
        self._joints_topic = joints_topic
        self._goal_topic = goal_joints_topic
        self._joy_topic = joy_topic
        self._timeout = timeout

        # Event (button or axis) description type.
        # EventDescription = namedtuple('EventDescription', ['ev_num', 'speed', 'joint', 'min', 'max'])
        class EventDescription():
            def __init__(self, args):
                # check arg type
                if not isinstance(args, (list,tuple)) or len(args) != 5:
                    raise ValueError('Button/axis event description must be a tuple with 5 elements.')
                # parse arg in fields
                ( self.ev_num, self.speed, self.joint, self.min, self.max ) = args
                # check fields types
                if not isinstance(self.ev_num, int):
                    raise ValueError('Button/axis number must be integer.')
                if not isinstance(self.speed, (int, float)):
                    raise ValueError('Speself must be float.')
                if not isinstance(self.max, (int, float)) or not isinstance(self.min, (int,float)):
                    raise ValueError('Min/max position must be float.')
                if not isinstance(self.joint, str):
                    raise ValueError('Joint name must be string.')

        # parse buttons and axes
        self._buttons = [ EventDescription(button_conf) for button_conf in buttons ]
        self._axes = [ EventDescription(axis_conf) for axis_conf in axes ]

        # subscribe to topics
        self._joy_subscriber = ProxySubscriberCached({ self._joy_topic: Joy })
        self._joints_subscriber = ProxySubscriberCached({ self._joints_topic: JointState })
        # setup publiser proxies
        self._goal_publisher = ProxyPublisher({ self._goal_topic: JointState })

        # buffers
        self._timestamp = None
        self._activity_timestamp = None
        self._goal_position = {} # map { joint: position } for all controlled joints

    def on_enter(self, userdata):
        # reset timestamp
        self._timestamp = rospy.Time.now()
        self._activity_timestamp = self._timestamp

        # fill joint goal array
        self._goal_position = {}
        # get current joints positions
        joints_msg = self._joints_subscriber.get_last_msg(self._joints_topic)
        # check message
        if joints_msg == None or len(joints_msg.name) != len (joints_msg.position):
            Logger.logerr('JoystickJointControl: robot pose is unavailable or incorrect on topic `%s`' % self._joints_topic)
            raise RuntimeError('JoystickJointControl: robot pose is unavailable or incorrect on topic `%s`' % self._joints_topic)

        for event in self._buttons + self._axes:
            # find coresponding joint
            is_found = False 
            for i in range(0,len(joints_msg.name)):
                if joints_msg.name[i] == event.joint:
                    is_found = True
                    self._goal_position[event.joint] = clip(joints_msg.position[i], event.min, event.max)
            # check if joint is found
            if not is_found:
                Logger.logerr('JoystickJointControl: unable to determine `%s` joint position. (min + max)/2 is used.' % event.joint)
                self._goal_position[event.joint] = (event.min + event.max) / 2.0 # default value

        Logger.loginfo('JoystickJointControl: start monitoring.')

    def execute(self, userdata):
        # calculate time shift
        timestamp = rospy.Time.now()
        dt = (timestamp - self._timestamp).to_sec()
        self._timestamp = timestamp

        activity_detected = False

        # check if new message is available
        if self._joy_subscriber.has_msg(self._joy_topic):
            # get messages
            joy_msg = self._joy_subscriber.get_last_msg(self._joy_topic)

            # TODO skip non-used axes and buttons

            # check if any button is pressed
            for i in range(0, len(joy_msg.buttons)):
                # skip not pressed buttons
                if not joy_msg.buttons[i]:
                    continue
                # find event handlers
                for event in self._buttons:
                    if event.ev_num == i:
                        # modify position accordingly
                        self._goal_position[event.joint] = clip(self._goal_position[event.joint] + event.speed*dt, event.min, event.max)
                        activity_detected = True

            # check if any axis is non-zero
            for i in range(0, len(joy_msg.axes)):
                # find event handlers
                for event in self._axes:
                    if event.ev_num == i:
                        # modify position accordingly
                        if event.speed != 0.0:
                            # add shift
                                self._goal_position[event.joint] = clip(self._goal_position[event.joint] + event.speed*joy_msg.axes[i]*dt, event.min, event.max)
                        else:
                            # set position
                            self._goal_position[event.joint] = 0.5*((event.max - event.min)*joy_msg.axes[i] + event.max + event.min)
                        if joy_msg.axes[i] != 0.0:
                            activity_detected = True 

        # publish pose
        goal_msg = JointState()
        goal_msg.header.stamp = timestamp
        for joint, position in self._goal_position.items():
            goal_msg.name.append(joint)
            goal_msg.position.append(position)

        self._goal_publisher.publish(self._goal_topic, goal_msg)

        # check activity
        if activity_detected:
            self._activity_timestamp = timestamp
            return None

        # check activity timeout
        if (timestamp - self._activity_timestamp).to_sec() > self._timeout:
            return 'done'
        else:
            return None

    def on_exit(self, userdata):
        Logger.loginfo('JoystickJointControl: stop monitoring.')

