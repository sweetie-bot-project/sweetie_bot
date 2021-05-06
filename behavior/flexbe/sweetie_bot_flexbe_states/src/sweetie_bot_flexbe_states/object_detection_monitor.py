#!/usr/bin/env python
import rospy, tf

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyTransformListener

from sweetie_bot_text_msgs.msg import Detection, DetectionArray
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

class ObjectDetectionMonitor(EventState):
    '''
    Receive list of detected objects as sweetie_bot_text_msgs/DetectionArray messages and check if there is an object 
    which can be matches with given (label, type) pair. Based on presece of this object classify situation 
    in `no_detections` state, `have_detections` (there are objects but none of them matches give (label, type)),
    `detection_matches` (desired object found). In the last case its pose published on given topic.
    
    -- detection_topic                       string          DetectionArray message topic.
    -- label                                 string          Object label. Use '*' to match all labals.
    -- type                                  string          Object type. Use '*' to match all types.
    -- detection_period                      float           If nothing was detected during detection_period 'no_detections' outcome happens.
    -- transform_delay                       float           Delay in oject detection tract relative to tf state.
    -- exit_states                           string[]        Stop monitoring and return outcome if detected situation in this list.
    -- pose_topic                            string          Topic for publishing matched object pose (may be Empty).
    -- pose_frame_id                         string          Frame in which pose should be published.

    #> pose                                  object          Object pose before exit.

    <= no_detections 	                     Nothing have been detected for detection_period.
    <= have_detections 	                     Objects were detected, but not of them matches (label, type)
    <= detecton_matches	                     Object presents and it is moving. `pose` key contains last pose.
    <= failure	                             Runtime error.

    '''


    def __init__(self, detection_topic = 'detection', label = '*', type ='*', exit_states = ['no_detections'], pose_topic = '', pose_frame_id = 'odom_combined', detection_period = 15.0, transform_delay = 0.0):
        super(ObjectDetectionMonitor, self).__init__(outcomes = ['no_detections', 'have_detections', 'detection_matches', 'failure'],
                                                                output_keys = ['pose'])

        # store state parameter for later use.
        self._detection_topic = detection_topic
        self._label = label
        self._type = type
        self._pose_topic = pose_topic
        self._frame_id = pose_frame_id
        self._exit_states = exit_states
        self._detection_period = detection_period
        self._transform_delay = transform_delay

        # setup proxies
        self._detection_subscriber = ProxySubscriberCached({ self._detection_topic: DetectionArray })
        self._detection_subscriber.enable_buffer(self._detection_topic)
        if self._pose_topic:
            self._pose_publisher = ProxyPublisher({ self._pose_topic: PoseStamped })
        else:
            self._pose_publisher = None
        self._tf_listener = ProxyTransformListener().listener()

        # check parameters
        if not isinstance(self._frame_id, str): # or not self._tf_listener.frameExists(self._frame_id):
            raise ValueError('ObjectDetectionMonitor: %s is not a valid frame' % self._frame_id)
        if not isinstance(self._label, str) or not isinstance(self._type, str):
            raise ValueError('ObjectDetectionMonitor: label and type parameters must be string.')
        if not isinstance(self._detection_period, float) or self._detection_period < 0.0:
            raise ValueError('ObjectDetectionMonitor: detection_period must be non-negative float.')
        if not isinstance(self._transform_delay, float) or self._transform_delay < 0.0:
            raise ValueError('ObjectDetectionMonitor: transform_delay must be non-negative float.')
        if not isinstance(self._exit_states, list) or not all( [ state in ['no_detections', 'have_detections', 'detection_matches' ] for state in self._exit_states ] ):
            raise ValueError('ObjectDetectionMonitor: incorrect list of exit states.')

        # pose buffers
        self._pose = None
        self._last_detection_stamp = None
        self._last_match_stamp = None
        self._last_match_id = None


    def on_enter(self, userdata):
        # reset pose buffers
        self._pose = None
        stamp = rospy.Time.now()
        self._last_detection_stamp = stamp
        self._last_match_stamp = stamp
        self._last_match_id = None

        Logger.loginfo('ObjectDetectionMonitor: start monitoring.')

    def execute(self, userdata):
        # list of found matches
        have_detections = False
        best_match = None
        match = None
        # check if new message is available
        while self._detection_subscriber.has_buffered(self._detection_topic):
            # get detections msg 
            detections_msg = self._detection_subscriber.get_from_buffer(self._detection_topic)
            # check if have detections
            if len(detections_msg.detections) > 0:
                have_detections = True
            # process detections
            for detection in detections_msg.detections:
                # check match 
                is_match = True
                if self._label != '*' and detection.label != self._label:
                    is_match = False
                if self._type != '*' and detection.type != self._type:
                    is_match = False
                # register match
                if is_match:
                    match = detection
                    if match.id == self._last_match_id:
                        best_match = match
      
        if best_match:
            match = best_match

        # result
        stamp = rospy.Time.now()
        if not have_detections:
            # no object detected
            # check if detection preiod exceeded
            if (stamp - self._last_detection_stamp).to_sec() > self._detection_period:
                # no detections state
                if 'no_detections' in self._exit_states:
                    Logger.loginfo('ObjectDetectionMonitor: no detections.')
                    return 'no_detections'
        elif not match:
            # object detected but it is not match
            self._last_detection_stamp = stamp
            # check if detection perod exceeded
            if (stamp - self._last_match_stamp).to_sec() > self._detection_period:
                # have_detections state
                if 'have_detections' in self._exit_states:
                    Logger.loginfo('ObjectDetectionMonitor: detection.')
                    return 'have_detections'
        else:
            # appropriate match is found
            self._last_detection_stamp = stamp
            self._last_match_stamp = stamp
            self._last_match_id = match.id
            # publish 
            if self._pose_publisher:
                # transform to frame_id
                try:
                    # transform
		    # TODO use message 
                    pose_stamped = PoseStamped(pose = match.pose, header = Header(frame_id = match.header.frame_id, stamp = match.header.stamp - rospy.Duration(self._transform_delay)))
                    self._pose = self._tf_listener.transformPose(self._frame_id, pose_stamped)
                except tf.Exception as e:
                    Logger.logwarn('Unable to transform from %s to %s' % (match.header.frame_id, self._frame_id))
                    return 'failure'
                # publish
                self._pose_publisher.publish(self._pose_topic, self._pose)
            # match_detection state
            if 'detection_matches' in self._exit_states:
                Logger.loginfo('ObjectDetectionMonitor: match.')
                return 'detection_matches'

        return None


    def on_exit(self, userdata):
        userdata.pose = self._pose

