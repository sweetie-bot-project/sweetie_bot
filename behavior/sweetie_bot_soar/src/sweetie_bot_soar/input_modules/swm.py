from . import input_module

import math
import rospy
import tf
from threading import Lock

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from sweetie_bot_text_msgs.msg import DetectionArray as DetectionArrayMsg, Detection as DetectionMsg

from flexbe_core.proxy import ProxyTransformListener
from .bins import BinsMap

class SpatialObject:
    def __init__(self, id, label, type, timestamp, timeout, frame_id, pose):
        # object general properties
        self.label = label
        self.id = id
        self.type = type
        # time properties
        self.creation_time = timestamp
        self.update_time = timestamp
        self.perceive_begin_time = timestamp
        self.perceive_end_time = None
        self.memorize_time = timeout
        # spatial properties
        self.frame_id = frame_id
        self.pose = pose

    def updateVisible(self, timestamp, pose):
        self.update_time = timestamp
        self.pose = pose
        self.perceive_end_time = None
        if self.perceive_begin_time == None:
            self.perceive_begin_time = timestamp

    def updateInvisible(self, timestamp):
        if self.perceive_end_time == None:
            self.perceive_end_time = timestamp
        self.perceive_begin_time = None

    def isOutdated(self, time_now):
        return self.perceive_end_time != None and (self.perceive_end_time + self.memorize_time) < time_now

    def isVisible(self, time_now):
        return self.perceive_end_time != None

    def getPose(self, frame_id, tf_buffer):
        pose_stamped = PoseStamped(pose = self.pose, header = Header(frame_id = self.frame_id))
        return tf_buffer.transformPose(frame_id, pose_stamped).pose

class SpatialObjectSoarView:
    def __init__(self, spatial_object, parent_wme_id):
        # create corresponding WME
        self.wme_id = parent_wme_id.CreateIdWME('object')
        # create child id map
        self._child_ids = {}
        # add persistent child WMEs
        self.updateChildWME('label', spatial_object.label)
        self.updateChildWME('id', spatial_object.id)
        self.updateChildWME('type', spatial_object.type)
    
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
        if child_id != None:
            del self._child_ids[attrib]
            child_id.DestroyWME()
        #else:
        #    raise KeyError('WME with attribute "%s" is not registered')

    def __del__(self):
        # remove WME
        self.wme_id.DestroyWME()

class SpatialWorldModel:
    @staticmethod
    def distance(p):
        return math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z)

    class MemoryElement:
        def __init__(self, spatial_object, soar_view):
            self.spatial_object = spatial_object
            self.soar_view = soar_view

    def __init__(self, name, config, agent):
        self._detections_sub = None

        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)

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
        self._tf_listener = ProxyTransformListener().listener()
        self._detections_sub = rospy.Subscriber(detection_topic, DetectionArrayMsg, self.detectionCallback)

        # map with memorized objects
        self._memory_lock = Lock()
        self._memory_map = {}
        self._soar_view_remove_list = []
        self._last_update_time = rospy.Time.now().to_sec();

    def detectionCallback(self, msg):
        with self._memory_lock:
            # iterate over detected objects 
            for detection_msg in msg.detections:
                # quick fix for id change
                detection_msg.id = 0

                # extract timestamp
                timestamp = detection_msg.header.stamp.to_sec()
                # extract pose from detection_msg
                try:
                    while True:
                        # transform using 'true' time
                        pose_stamped = PoseStamped(header = detection_msg.header, pose = detection_msg.pose)
                        try:
                            pose_stamped = self._tf_listener.transformPose(self._world_frame, pose_stamped)
                            break
                        except tf.ExtrapolationException:
                            pass
                        # transform using most recent transform
                        pose_stamped.header.stamp = rospy.Time()
                        pose_stamped = self._tf_listener.transformPose(self._world_frame, pose_stamped)
                        break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    Logger.logwarn('SWM: unable to transform detection (%s, %s) from %s to %s: %s' % (detection_msg.label, detection_msg.type, pose_stamped.header.frame_id, self._world_frame, e))
                    continue

                # search detected object in index
                key_tuple = (detection_msg.id, detection_msg.label, detection_msg.type)
                # check if corrsponding object exists
                mem_elem = self._memory_map.get(key_tuple)
                if mem_elem != None:
                    # renew timestamp and pose of SpatialObject but not update SOAR view
                    mem_elem.spatial_object.updateVisible(timestamp, pose_stamped.pose)
                else:
                    # add new SpatialObject but do not create corresponding SOAR view
                    spatial_object = SpatialObject( detection_msg.id, detection_msg.label, detection_msg.type, timestamp, self._timeout, self._world_frame, pose_stamped.pose )
                    self._memory_map[key_tuple] = SpatialWorldModel.MemoryElement(spatial_object, None)

            # check time of last update
            time_now = rospy.Time.now().to_sec();
            if time_now - self._last_update_time > self._seen_timeout:
                self.__updateSpatialMemory(time_now)

    def __updateSpatialMemory(self, time_now):
        self._last_update_time = time_now
        # iterate over objects, mark unseen invisible and mark to remove outdated 
        remove_list = []
        for key_tuple, mem_elem in self._memory_map.items():
            spatial_object = mem_elem.spatial_object
            # if object was not detected in last period of time then mark it invisible
            if time_now - spatial_object.update_time > self._seen_timeout:
                spatial_object.updateInvisible(time_now)
            # check if object is outdated
            if spatial_object.isOutdated(time_now):
                # mark to remove remove outdtated objects: they can be roved only inside SOAR context, so place them in remove list
                if mem_elem.soar_view != None:
                    self._soar_view_remove_list.append(mem_elem.soar_view)
                remove_list.append(key_tuple)
        # remove outdated items
        for key_tuple in remove_list:  
            del self._memory_map[key_tuple]

    def update(self):
        with self._memory_lock:
            # get current time
            time_now = rospy.Time.now().to_sec();
            # spatial memory update is mandatory
            self.__updateSpatialMemory(time_now)
            # remove WMEs which corresponds to outdated objects
            self._soar_view_remove_list.clear()
            # update WMEs 
            for mem_elem in self._memory_map.values():
                spatial_object = mem_elem.spatial_object
                soar_view = mem_elem.soar_view
                # check if corresponding WME exists 
                if soar_view == None:
                    # create WME 
                    soar_view = SpatialObjectSoarView(spatial_object, self._sensor_id)
                    mem_elem.soar_view = soar_view
                # visibility timestamp
                soar_view.updateChildWME( 'visible', self._time_bins_map(time_now - spatial_object.update_time) ) # string
                soar_view.updateChildWME( 'appeared', self._time_bins_map(time_now - spatial_object.creation_time) ) # string
                if spatial_object.perceive_begin_time != None:
                    soar_view.updateChildWME('perceive-begin', self._time_bins_map(time_now - spatial_object.perceive_begin_time) ) # string
                else:
                    soar_view.removeChildWME('perceive-begin')
                if spatial_object.perceive_end_time != None:
                    soar_view.updateChildWME('perceive-end', self._time_bins_map(time_now - spatial_object.perceive_end_time) ) # string
                else:
                    soar_view.removeChildWME('perceive-end')
                # position relative to head
                pose = spatial_object.getPose( self._head_frame, self._tf_listener )
                soar_view.updateChildWME( 'distance-head', self._distance_bins_map( SpatialWorldModel.distance(pose.position) ) ) # string
                soar_view.updateChildWME( 'yaw-head', self._yaw_bins_map( math.atan2(pose.position.y, pose.position.x) ) ) # string
                # position relative to bode
                pose = spatial_object.getPose( self._body_frame, self._tf_listener )
                soar_view.updateChildWME( 'distance-body', self._distance_bins_map( SpatialWorldModel.distance(pose.position) ) ) # string
                soar_view.updateChildWME( 'yaw-body', self._yaw_bins_map( math.atan2(pose.position.y, pose.position.x) ) ) # string

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        if self._detections_sub:
            self._detections_sub.unregister()

input_module.register("swm", SpatialWorldModel)
