#!/usr/bin/env python
import numpy
import math
import numpy.linalg
import rospy

from tf.transformations import quaternion_from_matrix

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from leap_motion.msg import leapros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from internal.pose_vectorized import PoseVectorized

class LeapMotionMonitor(EventState):
    '''
    Receive and classify LeapMotion messages. Can detect following situations: `no_object` detected, `still_object`, `moving_object`.
    
    -- leap_motion_topic                     string          Leap Motion topic.
    -- exit_states                           string[]        Stop monitoring and return outcome if detected situation in this list.
    -- pose_topic                            string          Topic for publishing detected object pose for vizualizaton purpose (may be Empty).
    -- parameters                            float[]         Parameteres: detection_period (s), position_tolerance (m), orientation_tolerance (rad).

    #> pose                                  object          Object pose before exit.

    <= no_object                          No object detected. `pose` key contains zero pose.
    <= still_object                       Object presents and it is not moving. `pose` key contains averaged pose.
    <= moving_object                      Object presents and it is moving. `pose` key contains last pose.

    '''


    def __init__(self, leap_motion_topic, exit_states, pose_topic = '', parameters = [2.0, 0.02, 0.2]):
        super(LeapMotionMonitor, self).__init__(outcomes = ['no_object', 'still_object', 'moving_object'],
                                                                output_keys = ['pose'])

        # store state parameter for later use.
        self._leap_topic = leap_motion_topic
        self._pose_topic = pose_topic
        self._exit_states = exit_states
        self._detecton_period = parameters[0]
        self._position_tolerance = parameters[1]
        self._orientation_tolerance = parameters[2]

        # setup proxies
        self._leap_subscriber = ProxySubscriberCached({ self._leap_topic: leapros })
        if self._pose_topic:
            self._pose_publisher = ProxyPublisher({ self._pose_topic: PoseStamped })
        else:
            self._pose_publisher = None

        # pose buffers
        self._pose = None
        self._pose_prev = None
        self._pose_averaged = None
        self._nonstillness_stamp = None


    def on_enter(self, userdata):
        # reset pose buffers
        self._pose = None
        self._pose_prev = None
        self._pose_averaged = None
        self._nonstillness_stamp = None

        Logger.loginfo('LeapMotionMonitor: start monitoring.')

    def execute(self, userdata):
        # check if new message is available
        if self._leap_subscriber.has_msg(self._leap_topic):
            # GET LEAP MOTION message
            leap_msg = self._leap_subscriber.get_last_msg(self._leap_topic)
            #Logger.loginfo('LeapMotionMonitor: leap_motion: \n%s' % str(leap_msg.palmpos))

            # STORE POSE IN BUFFER
            self._pose = PoseVectorized()
            self._pose.header = Header(frame_id = 'leap_motion', stamp = rospy.Time.now())
            self._pose.position = numpy.array([leap_msg.palmpos.x, leap_msg.palmpos.y, leap_msg.palmpos.z]) / 1000.0
            # get axis directions from leap_msg
            # TODO use quaternion_from_rxyx
            axis_z = - numpy.array([leap_msg.normal.x, leap_msg.normal.y,leap_msg.normal.z, 0])
            axis_y = - numpy.array([leap_msg.direction.x, leap_msg.direction.y,leap_msg.direction.z, 0])
            axis_x = numpy.cross(axis_y[:3], axis_z[:3])
            axis_x = numpy.append(axis_x, 0)
            # construct transformation matrix
            mat = numpy.matrix([ axis_x, axis_y, axis_z, numpy.array([0, 0, 0, 1]) ]).transpose()
            #Logger.loginfo('mat: \n%s' % str(mat))
            # obtain quaternion
            self._pose.orientation = quaternion_from_matrix(mat)

            # PUBLISH POSE
            if self._pose_topic:
                self._pose_publisher.publish(self._pose_topic, self._pose.toPoseStamped())

            # CHECK FOR FIRST CALL
            if self._pose_prev == None:
                self._pose_prev = PoseVectorized(self._pose)
                self._pose_averaged = PoseVectorized(self._pose)
                self._nonstillness_stamp = rospy.Time.now()
                return None

            # POSE AVERAGING
            self._pose_averaged.average_exp(self._pose, self._detecton_period)

            # CHECK FOR NO_OBJECT
            if PoseVectorized.eq(self._pose, self._pose_prev):
                # messages are identical, now check how long they are in this state
                if (self._pose.header.stamp - self._pose_prev.header.stamp).to_sec() > self._detecton_period:
                    # NO_OBJECT is detected
                    if 'no_object' in self._exit_states: 
                        Logger.loginfo('LeapMotionMonitor: no_object state is detected.')
                        return 'no_object'
                # wait until detection_period
                return None
            else:
                self._pose_prev = self._pose
            
            # CHECK FOR STILL_OBJECT
            if PoseVectorized.eq_approx(self._pose, self._pose_averaged, self._position_tolerance, self._orientation_tolerance):
                # check if averaging is performed long enought
                if (self._pose.header.stamp - self._nonstillness_stamp).to_sec() > self._detecton_period:
                    # STILL_OBJECT is_detected
                    if 'still_object' in self._exit_states: 
                        Logger.loginfo('LeapMotionMonitor: still_object state is detected.')
                        return 'still_object'
                # wait for detection_period from the start
                return None
            else:
                self._nonstillness_stamp = rospy.Time.now()


            # DEFAULT: MOVING OBJECT
            if 'moving_object' in self._exit_states: 
                Logger.loginfo('LeapMotionMonitor: moving_object state is detected.')
                return 'moving_object'
            else:
                return None

    def on_exit(self, userdata):
        userdata.pose = self._pose.toPoseStamped()
