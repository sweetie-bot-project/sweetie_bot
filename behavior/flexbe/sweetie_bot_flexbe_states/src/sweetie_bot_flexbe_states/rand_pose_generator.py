#!/usr/bin/env python
import random

import rospy, tf
from rospy.rostime import Time, Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxyTransformListener

from geometry_msgs.msg import PoseStamped


class RandPoseGenerator(EventState):
    '''
    Periodically generate geometry_msgs/PoseStamped messge in given parallelepiped. 
    Frames for parallelepiped is defined and the frame in which message is published can be different.

    -- topic        string          Topic where PoseStamped message should be published.
    -- duration     float           How long this state will be executed (seconds).
    -- interval     float[2]        Array of floats, maximal and minimal interval between movements.
    -- minXYZ       float[3]        Max values for x, y, z coordinates.
    -- maxXYZ       float[3]        Min values for x, y, z coordinates.
    -- frame_xyz    string          Coordinate frame of parallelepiped. 
    -- frame_out    string          Frame in which pose is publised.

    <= done 	                    Finished.
    <= failed 	                    Failed to activate FollowJointState controller.

    '''

    def __init__(self, topic = 'motion/controller/look_at/in_pose_ref', duration = 120, interval = [3, 5], maxXYZ = [1, 0.3, 0.5], minXYZ = [1.0, -0.3, 0.0], frame_xyz = 'base_link', frame_out = 'odom_combined'):
        super(RandPoseGenerator, self).__init__(outcomes = ['done', 'failure'])

        # Store topic parameter for later use.
        if not isinstance(topic, str):
            raise ValueError("Topic name must be string.")
        if not isinstance(duration, (int, float)) or duration <= 0:
            raise ValueError("Duration must be positive number.")
        if len(interval) != 2 or any([ not isinstance(t, (int, float)) for t in interval])or any([ t < 0 for t in interval]) or interval[0] > interval[1]:
            raise ValueError("Interval must represent interval of time.")
        if len(minXYZ) != 3 or any([ not isinstance(v, (int, float)) for v in minXYZ]):
            raise ValueError("minXYZ: list of three numbers was expected.")
        if len(maxXYZ) != 3 or any([ not isinstance(v, (int, float)) for v in maxXYZ]):
            raise ValueError("maxXYZ: list of three numbers was expected.")
        if not isinstance(frame_xyz, str) or not isinstance(frame_out, str):
            raise ValueError("Frame names must be string.")

        self._topic = topic
        self._duration = Duration.from_sec(duration)
        self._interval = interval
        self._minXYZ = minXYZ
        self._maxXYZ = maxXYZ
        self._frame_in = frame_xyz
        self._frame_out = frame_out

        # create proxies
        self._publisher = ProxyPublisher({ self._topic : PoseStamped })
        self._tf_listener = ProxyTransformListener().listener()
        self._tf_listener.waitForTransform(self._frame_out, self._frame_in, rospy.Time(), rospy.Duration(1))
        if not self._tf_listener.canTransform(self._frame_out, self._frame_in, rospy.Time(0)):
            raise ValueError("Unable to perform transform between frames %s and %s." % (self._frame_out, self._frame_in))

        # state
        self._start_timestamp = None
        self._movement_timestamp = None
        self._movement_duration = Duration()

        # error in enter hook
        self._error = False
    
    def on_enter(self, userdata):
        self._error = False
        # set start timestamp
        self._start_timestamp = Time.now()
        self._movement_timestamp = Time.now();

        Logger.loginfo('Start random pose generation, topic `%s`.' % self._topic)

    def execute(self, userdata):
        if self._error:
            return 'failure'
       
        time_now = Time.now()
        # check if time elasped
        if time_now - self._start_timestamp > self._duration:
            return 'done'
        # check if we have tosend new point
        if time_now - self._movement_timestamp > self._movement_duration:
            # determine new interval 
            self._movement_timestamp = time_now
            self._movement_duration = Duration.from_sec(random.uniform(*self._interval))
            # form message
            msg = PoseStamped()
            msg.header.frame_id = self._frame_in
            msg.header.stamp = Time(0.0)
            # random position
            msg.pose.position.x = random.uniform(self._minXYZ[0], self._maxXYZ[0])
            msg.pose.position.y = random.uniform(self._minXYZ[1], self._maxXYZ[1])
            msg.pose.position.z = random.uniform(self._minXYZ[2], self._maxXYZ[2])
            # transform to output frame
            try:
                # transform
                self._pose = self._tf_listener.transformPose(self._frame_out, msg)
            except tf.Exception as e:
                Logger.logwarn('Unable to transform from %s to %s: %s' % (self._frame_in, self._frame_out, e))
                return 'failure'
            # publish pose
            msg.header.stamp = self._movement_timestamp
            self._publisher.publish(self._topic, msg)

    def on_exit(self, userdata):
        Logger.loginfo('Done random pose generation.')
 

