#!/usr/bin/env python
import numpy
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyTransformListener, ProxyServiceCaller

from leap_motion.msg import leapros
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import JointState

from proto2.head_ik import HeadIK

class SweetieBotFollowHeadLeapMotion(EventState):
    '''
    SweetieBot head follows object, detected by LeapMotion. 

    -- leap_motion_topic                     string          Leap Motion topic.
    -- focus_point_topic                     string          Topic to publish focus point for vizualizaton purpose (may be Empty).
    -- follow_joint_state_controller         string          FollowJointState controller name.
    -- neck_angle                            float           Joint51 angle (neck base angle).
    -- deactivate                            boolean         Deactivate controller on stop/exit.

    <= failed 	                    Unable to activate state (controller is unavailable and etc)

    '''

    def __init__(self, leap_motion_topic, focus_point_topic, follow_joint_state_controller, neck_angle, deactivate):
        super(SweetieBotFollowHeadLeapMotion, self).__init__(outcomes = ['failed'])

        # store state parameter for later use.
        self._leap_topic = leap_motion_topic
        self._focus_topic = focus_point_topic
        self._controller = 'motion/controller/' + follow_joint_state_controller
        if type(neck_angle) != float:
            raise TypeError('SweetieBotFollowHeadLeapMotion: neck_angle must be float.')
        self._neck_angle = neck_angle
        self._deactivate = deactivate

        # setup proxies
        self._set_operational_caller = ProxyServiceCaller({ self._controller + '/set_operational': SetBool })
        self._leap_subscriber = ProxySubscriberCached({ self._leap_topic: leapros })

        # create publisher
        self._joints_publisher = ProxyPublisher({ self._controller + '/in_joints_ref': JointState })
        if self._focus_topic:
            self._focus_publisher = ProxyPublisher({ self._focus_topic: PointStamped })
        else:
            self._focus_publisher = None

        # head inverse kinematics
        self._ik = HeadIK()

        # error in enter hook
        self._error = False

    def on_enter(self, userdata):

        try: 
            res = self._set_operational_caller.call(self._controller + '/set_operational', True)
        except Exception as e:
            Logger.logwarn('Failed to activate `' + self._controller + '` controller:\n%s' % str(e))
            self._error = True
            return

        if not res.success:
            Logger.logwarn('Failed to activate `' + self._controller + '` controller (SetBoolResponse: success = false).')
            self._error = True

        Logger.loginfo('SweetieBotFollowHeadLeapMotion: controller `{0}` is active.'.format(self._controller))

    def execute(self, userdata):
        if self._error:
            return 'failed'
        
        # check if new message is available
        if self._leap_subscriber.has_msg(self._leap_topic):
            # get object position
            leap_msg = self._leap_subscriber.get_last_msg(self._leap_topic)
            # convert to ROS coordinates
            palmpos = PointStamped()
            # TODO add leap_motion frame
            palmpos.header = Header(frame_id = 'base_link', stamp = rospy.Time.now())
            palmpos.point = Point(x = leap_msg.palmpos.x/1000.0, y = -leap_msg.palmpos.z/1000.0 - 0.5, z = leap_msg.palmpos.y/1000.0)
            # publish focus
            if self._focus_topic:
                self._focus_publisher.publish(self._focus_topic, palmpos)
            # calculate head pose
            head_joints_msg = self._ik.pointDirectionToHeadPose(palmpos, self._neck_angle, 0.0)
            # calculate eyes pose
            eyes_joints_msg = self._ik.pointDirectionToEyesPose(palmpos)
            # publish head pose
            if head_joints_msg:
                if eyes_joints_msg:
                    # join head and eyes pose
                    head_joints_msg.name += eyes_joints_msg.name
                    head_joints_msg.position += eyes_joints_msg.position
                # publish pose
                self._joints_publisher.publish(self._controller + '/in_joints_ref', head_joints_msg)
            elif eyes_joints_msg:
                # publish pose
                self._joints_publisher.publish(self._controller + '/in_joints_ref', eye_joints_msg)

    def on_stop(self):
        if self._deactivate:
            try: 
                res = self._set_operational_caller.call(self._controller + '/set_operational', False)
            except Exception as e:
                Logger.logwarn('Failed to deactivate `' + self._controller + '` controller:\n%s' % str(e))
            Logger.loginfo('SweetieBotFollowHeadLeapMotion: controller `{0}` is deactivated.'.format(self._controller))




