from . import input_module

import math
import rospy
import tf

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from sweetie_bot_text_msgs.msg import DetectionArray as DetectionArrayMsg, Detection as DetectionMsg

from flexbe_core.proxy import ProxyTransformListener

class SpatialObject:
    def __init__(self, id, label, type, timestamp, timeout, frame_id, pose):
        # object general properties
        self.label = label
        self.id = id
        self.type = type
        # time properties
        self.creation_time = timestamp
        self.update_time = timestamp
        self.perceive_end_time = None
        self.memorize_time = timeout
        # spatial properties
        self.frame_id = frame_id
        self.pose = pose

    def updateVisible(self, timestamp, pose):
        self.update_time = timestamp
        self.pose = pose
        self.perceive_end_time = None

    def updateInvisible(self, timestamp):
        if self.perceive_end_time == None:
            self.perceive_end_time = timestamp

    def isOutdated(self, time_now):
        return self.perceive_end_time != None and (self.perceive_end_time + self.memorize_time) < time_now

    def isVisible(self, time_now):
        return self.perceive_end_time != None

    def getPose(self, frame_id, tf_buffer):
        pose_stamped = PoseStamped(pose = self.pose, header = Header(frame_id = self.frame_id))
        return tf_buffer.transformPose(frame_id, pose_stamped).pose

class SpatialObjectSoar(SpatialObject):
    def __init__(self, label, id, type, timestamp, timeout, frame_id, pose, parent_wme_id):
        # create SpatialObject
        super(SpatialObjectSoar, self).__init__(label, id, type, timestamp, timeout, frame_id, pose)
        # create corresponding WME
        self.wme_id = parent_wme_id.CreateIdWME('object')
        # create child id map
        self._child_ids = {}
        # add persistent child WMEs
        self.updateChildWME('label', self.label)
        self.updateChildWME('id', self.id)
        self.updateChildWME('type', self.type)

    def updateChildWME(self, attrib, value):
        # check if corresponding child WME exists
        child_id = self._child_ids.get(attrib)
        if child_id == None:
            # create WME and add it to map
            if isinstance(value, int):
                self._child_ids[attrib] = self.wme_id.CreateIntWME(attrib, value)
            elif isinstance(value, float):
                self._child_ids[attrib] = self.wme_id.CreateFloatWME(attrib, value)
            elif isinstance(value, str):
                self._child_ids[attrib] = self.wme_id.CreateStringWME(attrib, value)
            else:
                raise TypeError('SOAR attribute value must be int, float or string.')
        else:
            # update existing WME
            if child_id.GetValue() != value:
                child_id.Update(value)

    def removeChildWME(self, attrib):
        child_id = self._child_ids.get(attrib)
        if child_id == None:
            raise KeyError('WME with attribute "%s" is not registered')
        else:
            del self._child_ids[attrib]
            child_id.DestroyWME()

    def __del__(self):
        # remove WME
        self.wme_id.DestroyWME()

class BinsMap:
    def __init__(self, param):
        error = RuntimeError("BinsMap parameter must contains numerical list `values` and string list `names`.")
        # check provided configuration parameter
        if not isinstance(param, dict):
            raise error
        # extract names and values lists
        names = param.get('names')
        values = param.get('values')
        if not isinstance(names, list) or not isinstance(values, list):
            raise error
        if len(values) + 1 != len(names):
            raise RuntimeError("BinMap has inconsistent names and values lists lengths." % name)
        # check if parameters are correct
        v_prev = None
        for v in values:
            if not isinstance(v, (int, float)):
                raise error
            if v_prev != None and v_prev >= v:
                raise RuntimeError("BinMap values list must be ordered." % name)
        for n in names:
            if not isinstance(n, str):
                raise error
        # create bin map
        self._values = values
        self._names = names

    def __call__(self, value):
        k = 0
        for v in self._values:
            if v > value:
                break
            k = k + 1
        return self._names[k]

class SpatialWorldModel:
    @staticmethod
    def distance(p):
        return math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z)

    def __init__(self, agent, config):
        self._detections_sub = None
        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME('swm')

        # get configuration from parameters
        detection_topic = config.get('topic')
        if not detection_topic:
            raise RuntimeError('SWM input module: "topic" parameter is not defined.')
        self._seen_timeout = config.get('visibility_timeout')
        if not self._seen_timeout or not isinstance(self._seen_timeout, (int,float)) or self._seen_timeout < 0.0:
            raise RuntimeError('SWM input module: "visibity_timeout" parameter is not defined or incorrect.')
        self._timeout = config.get('timeout')
        if not self._timeout or not isinstance(self._timeout, (int,float)) or self._timeout < 0.0:
            raise RuntimeError('SWM input module: "timeout" parameter is not defined or incorrect.')
        self._world_frame = config.get('world_frame')
        if not self._world_frame or not isinstance(self._world_frame, str):
            raise RuntimeError('SWM input module: "world_frame" parameter must be string.')
        self._head_frame = config.get('head_frame')
        if not self._head_frame or not isinstance(self._head_frame, str):
            raise RuntimeError('SWM input module: "head_frame" parameter must be string.')
        self._body_frame = config.get('body_frame')
        if not self._body_frame or not isinstance(self._body_frame, str):
            raise RuntimeError('SWM input module: "body_frame" parameter must be string.')
        try:
            self._distance_bins_map = BinsMap( config['distance_bins_map'] )
            self._yaw_bins_map = BinsMap( config['yaw_bins_map'] )
            self._time_bins_map = BinsMap( config['time_bins_map'] )
        except KeyError:
            raise RuntimeError('SWM input module: "distance_bins_map" , "yaw_bins_map", "time_bins_map" parameters must present.')

        # add topic subscriber and tf buffer
        self._detections_sub = rospy.Subscriber(detection_topic, DetectionArrayMsg, self.detectionCallback)
        self._tf_listener = ProxyTransformListener().listener()

        # map with memorized objects
        self._object_map = {}

        # message buffers
        self._detections_buffer = []

    def detectionCallback(self, msg):
        # buffer msg into internal buffer
        self._detections_buffer += msg.detections
        # TODO: mesages should be processed immediatelly, but SOAR input link sholud be updated only on call update call
        if len(self._detections_buffer) > 10:
            self._detections_buffer = self._detections_buffer[-11:-1]

    def update(self):
        # iterate over detected objects
        for detection_msg in self._detections_buffer:
            # extract information from detection message
            timestamp = detection_msg.header.stamp.to_sec()
            pose_stamped = PoseStamped(header = Header(frame_id = detection_msg.header.frame_id), pose = detection_msg.pose)
            pose_stamped = self._tf_listener.transformPose(self._world_frame, pose_stamped)

            # search detected object in index
            key_tuple = (detection_msg.id, detection_msg.label, detection_msg.type)
            # check if corrsponding wme exists
            spatial_object = self._object_map.get(key_tuple)
            if spatial_object != None:
                # renew timestamp and pose of SpatialObjectSoar, this does not update any WMEs
                spatial_object.updateVisible(timestamp, pose_stamped.pose)
            else:
                # add new SpatialObjectSoar (it automatically creates corresponding WME and persistent child attributes)
                spatial_object = SpatialObjectSoar( detection_msg.id, detection_msg.label, detection_msg.type, timestamp, self._timeout, self._world_frame, pose_stamped.pose, self._sensor_id )
                self._object_map[key_tuple] = spatial_object

        # clear buffer
        self._detections_buffer.clear()

        # get current time
        time_now = rospy.Time.now().to_sec();
        # check objects for invisible and outdated WMEs
        for key_tuple, spatial_object in self._object_map.items():
            # if object was not detected in last period of time then mark it invisible
            if time_now - spatial_object.update_time > self._seen_timeout:
                spatial_object.updateInvisible(time_now)
            # check if object is outdated
            if spatial_object.isOutdated(time_now):
                # remove outdtated objects (this also removes corresponding WME)
                del self._object_map[key_tuple]
            else:
                # update child WMEs
                # visibility timestamp
                spatial_object.updateChildWME( 'visible', self._time_bins_map(time_now - spatial_object.update_time) ) # string
                # position relative to head
                pose = spatial_object.getPose( self._head_frame, self._tf_listener )
                spatial_object.updateChildWME( 'distance-head', self._distance_bins_map( SpatialWorldModel.distance(pose.position) ) ) # string
                spatial_object.updateChildWME( 'yaw-head', self._yaw_bins_map( math.atan2(pose.position.y, pose.position.x) ) ) # string
                # position relative to bode
                pose = spatial_object.getPose( self._body_frame, self._tf_listener )
                spatial_object.updateChildWME( 'distance-body', self._distance_bins_map( SpatialWorldModel.distance(pose.position) ) ) # string
                spatial_object.updateChildWME( 'yaw-body', self._yaw_bins_map( math.atan2(pose.position.y, pose.position.x) ) ) # string

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        if self._detections_sub:
            self._detections_sub.unregister()

input_module.register("swm", SpatialWorldModel)
