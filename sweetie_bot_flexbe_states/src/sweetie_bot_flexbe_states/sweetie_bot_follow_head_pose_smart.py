#!/usr/bin/env python
import math
import numpy
import rospy
import tf

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyTransformListener, ProxyServiceCaller

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from sensor_msgs.msg import JointState

from proto2.head_ik import HeadIK

class SweetieBotFollowHeadPoseSmart(EventState):
    '''
    SweetieBot follows object with head and eyes. Object is specified by PoseStamped on focus_topic. 
    Robot tries to keep comfort distance beetween object and head. If it is not possible, corresponding 
    outcome may be triggered.

    If distance between head is object is smaller then `distance_uncomfortable` set joint51 to `neck_angle_uncomfortable`.
    If it is greater then `distance_comfortable` set joint51 to `neck_angle_comfortable`.

    -- pose_topic                            string          geometry_msgs.msg.PoseStamped topic, where object pose is published.
    -- follow_joint_state_controller         string          FollowJointState controller name.
    -- discomfort_time                       boolean         If distance beetween head and object is less then `distance_uncomfortable` for `discomfort_time` seconds then `too_close` outcome is triggered.
    -- neck_control_parameteres              float[]         [ neck_angle_cofortable, distance_comfortable, neck_angle_uncofortable, distance_uncomfortable ] 
    -- deactivate                            boolean         Deactivate controller on stop/exit.
    -- controlled_chains                     string[]        List of controlled kinematics chains, may contains 'head', 'eyes'.

    <= failed 	                    Unable to activate state (controller is unavailable and etc)
    <= too_close 	            Object is too close to head.

    '''

    def __init__(self, pose_topic, follow_joint_state_controller = 'joint_state_head', discomfort_time = 1000.0, 
                    neck_control_parameteres = [ -0.13, 0.3, 0.20, 0.2], deactivate = True, controlled_chains = ['head', 'eyes']):
        super(SweetieBotFollowHeadPoseSmart, self).__init__(outcomes = ['failed', 'too_close'])

        # store state parameter for later use.
        self._pose_topic = pose_topic
        if len(neck_control_parameteres) != 4:
            raise TypeError('SweetieBotFollowHeadPoseSmart: neck_control_parameteres must be float[4]')
        self._neck_params = neck_control_parameteres
        self._discomfort_time = discomfort_time
        self._controller = 'motion/controller/' + follow_joint_state_controller
        self._deactivate = deactivate
        self._control_head = 'head' in controlled_chains
        self._control_eyes = 'eyes' in controlled_chains

        # setup proxies
        self._set_operational_caller = ProxyServiceCaller({ self._controller + '/set_operational': SetBool })
        self._pose_subscriber = ProxySubscriberCached({ self._pose_topic: PoseStamped })
        self._joints_publisher = ProxyPublisher({ self._controller + '/in_joints_ref': JointState })

        # head inverse kinematics
        self._ik = HeadIK()
        
        # state
        self._neck_angle = None
        self._comfortable_stamp = None

        # error in enter hook
        self._error = False

    def on_enter(self, userdata):
        self._error = False

        try: 
            res = self._set_operational_caller.call(self._controller + '/set_operational', True)
        except Exception as e:
            Logger.logwarn('Failed to activate `' + self._controller + '` controller:\n%s' % str(e))
            self._error = True
            return

        if not res.success:
            Logger.logwarn('Failed to activate `' + self._controller + '` controller (SetBoolResponse: success = false).')
            self._error = True

        # set default value
        self._neck_angle = self._neck_params[0]
        self._comfortable_stamp = rospy.Time.now()

        Logger.loginfo('SweetieBotFollowHeadLeapMotion: controller `{0}` is active.'.format(self._controller))

    def execute(self, userdata):
        if self._error:
            return 'failed'
        
        # check if new message is available
        if self._pose_subscriber.has_msg(self._pose_topic):
            # get object position
            pose = self._pose_subscriber.get_last_msg(self._pose_topic)
            # convert to PointStamped
            focus_point = PointStamped()
            focus_point.header = Header(frame_id = pose.header.frame_id)
            focus_point.point = pose.pose.position
            
            head_joints_msg = None
            eyes_joints_msg = None
            # CALCULATE HEAD POSITION
            if self._control_head:
                # calculate comfort heck_angle
                try:
                    # convert point coordinates to bone54 frame
                    fp = self._ik._tf.transformPoint('bone54', focus_point).point
                    # distance and direction angle
                    dist = math.sqrt(fp.x**2 + fp.y**2 + fp.z**2)
                    angle = math.acos(fp.z / dist)
                    Logger.loginfo('SweetieBotFollowHeadLeapMotion: dist: %s, angle: %s' % (str(dist), str(angle)))
                    # check comfort distance
                    if angle < math.pi/4:
                        if dist > self._neck_params[1]:
                            self._neck_angle = self._neck_params[0]
                            self._comfortable_stamp = rospy.Time.now()
                        elif dist < self._neck_params[3]:
                            self._neck_angle = self._neck_params[2]
                            # check if discomfort timer is elasped
                            if (rospy.Time.now() - self._comfortable_stamp).to_sec() > self._discomfort_time:
                                return 'too_close'
                        else:
                            self._comfortable_stamp = rospy.Time.now()
                    else:
                        self._comfortable_stamp = rospy.Time.now()
                except tf.Exception as e:
                    Logger.logwarn('Cannot transform to bone54:\n%s' % str(e))
                    self._neck_angle = self._neck_params[0]
                # calculate head pose for given angle
                head_joints_msg = self._ik.pointDirectionToHeadPose(focus_point, self._neck_angle, 0.0)

            # CALCULATE EYES POSE
            if self._control_eyes:
                eyes_joints_msg = self._ik.pointDirectionToEyesPose(focus_point)

            # PUBLISH POSE
            if head_joints_msg:
                if eyes_joints_msg:
                    # join head and eyes pose
                    head_joints_msg.name += eyes_joints_msg.name
                    head_joints_msg.position += eyes_joints_msg.position
                # publish pose
                self._joints_publisher.publish(self._controller + '/in_joints_ref', head_joints_msg)
            elif eyes_joints_msg:
                # publish pose
                self._joints_publisher.publish(self._controller + '/in_joints_ref', eyes_joints_msg)

    def on_stop(self):
        if self._deactivate:
            try: 
                res = self._set_operational_caller.call(self._controller + '/set_operational', False)
            except Exception as e:
                Logger.logwarn('Failed to deactivate `' + self._controller + '` controller:\n%s' % str(e))
            Logger.loginfo('SweetieBotFollowHeadLeapMotion: controller `{0}` is deactivated.'.format(self._controller))




