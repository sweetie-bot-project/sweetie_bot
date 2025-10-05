from .input_module import get_config_parameter, register, InputModule

import math
from threading import Lock
from dataclasses import dataclass
import copy

import rospy
import tf

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from sweetie_bot_text_msgs.msg import DetectionArray as DetectionArrayMsg, Detection as DetectionMsg

from flexbe_core.proxy import ProxyTransformListener
from .bins import BinsMap

@dataclass(eq = True, frozen = True)
class ObjectKeyTuple:
    id : int
    label : str
    type : str

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

    def __repr__(self):
        return f'SpatialObject(label="{self.label}", id={self.id}, type="{self.type}", creation={self.creation_time}, update={self.update_time}, perceive_begin={self.perceive_begin_time}, perceive_end_time={self.perceive_end_time})'

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
        return self.perceive_end_time is not None and (self.perceive_end_time + self.memorize_time) < time_now

    def isVisible(self):
        return self.perceive_end_time is None

    def getPose(self, frame_id, tf_buffer):
        pose_stamped = PoseStamped(pose = self.pose, header = Header(frame_id = self.frame_id))
        return tf_buffer.transformPose(frame_id, pose_stamped).pose

class PoseFilter:
    subclass_map = {}

    def __new__(cls, config):
        # get type name
        type_name = get_config_parameter('PoseFilter', config, 'type', allowed_types=str)
        # factory implementatipacy
        cls = PoseFilter.subclass_map[type_name]
        return super(PoseFilter, cls).__new__(cls)

    def __init_subclass__(cls, name):
        # subcalsses registration
        super(PoseFilter, cls).__init_subclass__()
        PoseFilter.subclass_map[name] = cls

    def __call__(self, spatial_object, detection_msg, detection_pose_transformed, tf_listener):
        raise NotImplemented

class PoseFilterNone(PoseFilter, name = 'none'):
    def __call__(self, spatial_object, detection_msg, detection_pose_trasformed, tf_listener):
        return detection_pose_trasformed

class PoseFilterMaxvel(PoseFilter, name = 'maxvel'):
    def __init__(self, config):
        super(PoseFilterMaxvel, self).__init__()
        # get paramters
        self._difftime = rospy.Duration(get_config_parameter('PoseFilter.Maxvel', config, 'velocity_estimator_difftime', default_value=0.2, check_func=lambda v: v > 0.0))
        self._maxvel_linear = get_config_parameter('PoseFilter.Maxvel', config, 'velocity_linear_threshold', allowed_types=float)
        self._maxvel_angular = get_config_parameter('PoseFilter.Maxvel', config, 'velocity_angular_threshold', allowed_types=float)

    def __call__(self, spatial_object, detection_msg, detection_pose_transformed, tf_listener):
        # check that sensor frame is motionless
        try:
            vel, rot = tf_listener.lookupTwist(detection_msg.header.frame_id, spatial_object.frame_id, detection_msg.header.stamp, self._difftime)
            vel_abs = math.hypot(*vel)
            rot_abs = math.hypot(*rot)
            sensor_frame_is_moving = vel_abs > self._maxvel_linear or rot_abs > self._maxvel_angular
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn('PoseFilterMaxvel: unable to estimate %s frame speed relative to %s frame: %s' % (detection_msg.header.frame_id, spatial_object.frame_id, e))
            sensor_frame_is_moving = True
        # debug output
        # print(f'PoseFilterMaxvel: {spatial_object.frame_id}, {detection_msg.header.frame_id}: ({vel_abs} {rot_abs}) -- {sensor_frame_is_moving}')
        # if frame is moving ignore detected pose
        if sensor_frame_is_moving:
            return spatial_object.pose
        else:
            return detection_pose_transformed

@dataclass(frozen=True)
class MarkerProps:
    type: int
    scale: Vector3
    shift: Vector3
    color: ColorRGBA

class SpatialObjectMarker:
    _marker_id = 0
    _marker_type_map = {
                'human': MarkerProps(Marker.CYLINDER, Vector3(0.1, 0.1, 1.0), Vector3(0.0, 0.0, -0.5), ColorRGBA(0.0, 1.0, 0.0, 1.0)),
                'hand': MarkerProps(Marker.CYLINDER, Vector3(0.15, 0.15, 0.05), Vector3(0.0, 0.0, 0.0), ColorRGBA(0.0, 1.0, 0.0, 1.0)),
                'pony': MarkerProps(Marker.CUBE, Vector3(0.20, 0.20, 0.20), Vector3(0.0, 0.0, 0.0), ColorRGBA(1.0, 1.0, 0.0, 1.0)),
            }
    _marker_type_default = MarkerProps(Marker.SPHERE, Vector3(0.2, 0.2, 0.2), Vector3(0.0, 0.0, 0.0), ColorRGBA(1.0, 0.0, 0.0, 1.0))

    def __init__(self, spatial_object, lifetime = 1.0):
        # get obect properties
        self._marker_props = self._marker_type_map.get(spatial_object.type, self._marker_type_default)
        # object marker
        header = Header(frame_id = spatial_object.frame_id)
        self._object_marker = Marker(header = header, ns = 'swm', id = self._marker_id, action = Marker.ADD, lifetime = rospy.Duration(lifetime),
                                     type = self._marker_props.type, scale = copy.copy(self._marker_props.scale), color = copy.copy(self._marker_props.color))
        # text marker
        self._text_marker = Marker(header = header, ns = 'swm', id = self._marker_id + 1, action = Marker.ADD, lifetime = rospy.Duration(lifetime),
                                                   type = Marker.TEXT_VIEW_FACING, scale = Vector3(0.0, 0.0, 0.02), color = ColorRGBA(1.0, 1.0, 1.0, 1.0))
        # increase marker unique id
        SpatialObjectMarker._marker_id += 10

    def update(self, spatial_object, stamp):
        # object marker
        self._object_marker.header.stamp = stamp
        self._object_marker.color.a = 1.0 if spatial_object.isVisible() else 0.3
        self._object_marker.pose = copy.deepcopy(spatial_object.pose)
        position = self._object_marker.pose.position
        position.x += self._marker_props.shift.x
        position.y += self._marker_props.shift.y
        position.z += self._marker_props.shift.z
        # text marker
        self._text_marker.header.stamp = stamp
        self._text_marker.pose = copy.deepcopy(spatial_object.pose)
        self._text_marker.pose.position.z += 0.2
        # extract paramters
        now = stamp.to_sec()
        label = spatial_object.label
        obj_type = spatial_object.type
        creation = spatial_object.creation_time - now
        update = spatial_object.update_time - now
        if spatial_object.isVisible():
            visibility = f'VISIBLE, perceive_begin: {spatial_object.perceive_begin_time - now:.1f}'
        else:
            visibility = f'INVISIBLE, perceive_end: {spatial_object.perceive_end_time - now:.1f}'
        # update text
        self._text_marker.text = f'({label}, {obj_type})\ncreation: {creation:.1f}, update: {update:.1f}\n{visibility}'

    def getMarkers(self):
        return (self._object_marker, self._text_marker)

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


class SpatialWorldModel(InputModule):
    _swm_instance_ref = None
    _update_callbacks = []

    @staticmethod
    def get_swm():
        swm_ref = SpatialWorldModel._swm_instance_ref
        return swm_ref if swm_ref is not None else None

    @staticmethod
    def add_update_callback(callback):
        if callback not in SpatialWorldModel._update_callbacks:
            SpatialWorldModel._update_callbacks.append(callback)

    @staticmethod
    def remove_update_callback(callback):
        SpatialWorldModel._update_callbacks.remove(callback)

    @staticmethod
    def distance(p):
        return math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z)

    class MemoryElement:
        def __init__(self, spatial_object, soar_view, marker = None):
            self.spatial_object = spatial_object
            self.soar_view = soar_view
            self.marker = marker

    def __init__(self, name, config, agent):
        # superclass constructor
        super(SpatialWorldModel, self).__init__(name)
        # add fileds to prevent AttributeError during destruction
        self._sensor_id = None
        self._detections_sub = None
        # check if SWM exists
        if SpatialWorldModel._swm_instance_ref is not None:
            raise RuntimeError('SWM input_module: only one SWM instance can exist')

        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)

        # get configuration from parameters
        detection_topic = self.getConfigParameter(config, 'topic', allowed_types=str)
        self._seen_timeout = self.getConfigParameter(config, 'visibility_timeout', allowed_types=(int,float), check_func=lambda v: v >= 0.0)
        self._timeout = self.getConfigParameter(config, 'timeout', allowed_types=(int,float), check_func=lambda v: v >= 0.0)
        self._world_frame = self.getConfigParameter(config, 'world_frame', allowed_types=str)
        self._head_frame = self.getConfigParameter(config, 'head_frame', allowed_types=str)
        self._body_frame = self.getConfigParameter(config, 'body_frame', allowed_types=str)
        try:
            self._distance_bins_map = BinsMap( config['distance_bins_map'] )
            self._yaw_bins_map = BinsMap( config['yaw_bins_map'] )
            self._time_bins_map = BinsMap( config['time_bins_map'] )
        except KeyError:
            raise RuntimeError('SWM input module: "distance_bins_map" , "yaw_bins_map", "time_bins_map" parameters must present.')
        filter_config = self.getConfigParameter(config, 'pose_filter', allowed_types=dict, error_desc=f'input module {self._name}: "pose_filter" configuration parameter must present and be a dict with filter declaration.')
        self._markers_period = self.getConfigParameter(config, 'markers_publication_period', 0.0, allowed_types=(int,float), check_func=lambda v: v >= 0.0)
        markers_topic = self.getConfigParameter(config, 'markers_topic', allowed_types=str)

        # add topic subscriber and tf buffer
        self._tf_listener = ProxyTransformListener().listener()
        self._detections_sub = rospy.Subscriber(detection_topic, DetectionArrayMsg, self.detectionCallback)
        # marker publications
        self._markers_pub = rospy.Publisher(markers_topic, MarkerArray, queue_size=5)
        if self._markers_period > 0.0:
            self._markers_timer = rospy.Timer(rospy.Duration(self._markers_period), lambda event: self._publishMarkers())
        else:
            self._markers_timer = None

        # pose filter
        self._pose_filter = PoseFilter(filter_config)

        # map with memorized objects
        self._memory_lock = Lock()
        self._memory_map = {}
        self._soar_view_remove_list = []
        self._last_update_time = rospy.Time.now().to_sec();

        # register SWM
        SpatialWorldModel._swm_instance_ref = self

    @property
    def world_frame(self):
        return self._world_frame

    def get_objects(self, object_filter = None):
        with self._memory_lock:
            if object_filter is None:
                return { key_tuple: mem_elem.spatial_object for key_tuple, mem_elem in self._memory_map.items() }
            else:
                return { key_tuple: mem_elem.spatial_object for key_tuple, mem_elem in self._memory_map.items() if object_filter(mem_elem.spatial_object) }

    def get_object_soar_view(self, key_tuple):
        mem_elem = self._memory_map.get(key_tuple)
        return mem_elem.soar_view if mem_elem is not None else None

    def get_object(self, key_tuple):
        mem_elem = self._memory_map.get(key_tuple)
        return mem_elem.spatial_object if mem_elem is not None else None


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
                        pose_stamped.header.stamp = rospy.Time(0) # TODO: wait for transform
                        pose_stamped = self._tf_listener.transformPose(self._world_frame, pose_stamped)
                        break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn('SWM: unable to transform detection (%s, %s) from %s to %s: %s' % (detection_msg.label, detection_msg.type, pose_stamped.header.frame_id, self._world_frame, e))
                    continue
                # search detected object in index
                key_tuple = ObjectKeyTuple(detection_msg.id, detection_msg.label, detection_msg.type)
                # check if corrsponding object exists
                mem_elem = self._memory_map.get(key_tuple)
                if mem_elem != None:
                    # renew timestamp and pose of SpatialObject but not update SOAR view
                    filtered_pose = self._pose_filter(mem_elem.spatial_object, detection_msg, pose_stamped.pose, self._tf_listener)
                    mem_elem.spatial_object.updateVisible(timestamp, filtered_pose)

                else:
                    # add new SpatialObject but do not create corresponding SOAR view
                    spatial_object = SpatialObject( detection_msg.id, detection_msg.label, detection_msg.type, timestamp, self._timeout, self._world_frame, pose_stamped.pose )
                    marker = SpatialObjectMarker(spatial_object) if self._markers_timer is not None else None
                    self._memory_map[key_tuple] = SpatialWorldModel.MemoryElement(spatial_object, None, marker)

            # check time of last update
            time_now = rospy.Time.now().to_sec();
            if time_now - self._last_update_time > self._seen_timeout:
                self.__updateSpatialMemory(time_now)

        # call callbacks
        for callback in self._update_callbacks:
            callback(self)

    def _publishMarkers(self):
        # form MarkerArray form updated markers
        marker_array = MarkerArray()
        stamp = rospy.Time.now()
        with self._memory_lock:
            for mem_elem in self._memory_map.values():
                marker = mem_elem.marker
                if marker is not None:
                    marker.update(mem_elem.spatial_object, stamp)
                    marker_array.markers.extend(marker.getMarkers())
        # publish
        self._markers_pub.publish(marker_array)

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
                soar_view.updateChildWME( 'perceived-first-time', self._time_bins_map(time_now - spatial_object.creation_time) ) # string
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
                distance = SpatialWorldModel.distance(pose.position)
                soar_view.updateChildWME( 'distance-head', self._distance_bins_map(distance)) # string
                soar_view.updateChildWME( 'distance-head-numeric', distance ) # float
                yaw = math.atan2(pose.position.y, pose.position.x)
                soar_view.updateChildWME( 'yaw-head', self._yaw_bins_map(yaw)) # string
                soar_view.updateChildWME( 'yaw-head-numeric', yaw ) # float
                # position relative to bode
                pose = spatial_object.getPose( self._body_frame, self._tf_listener )
                distance = SpatialWorldModel.distance(pose.position)
                soar_view.updateChildWME( 'distance-body', self._distance_bins_map(distance)) # string
                soar_view.updateChildWME( 'distance-body-numeric', distance ) # float
                yaw = math.atan2(pose.position.y, pose.position.x)
                soar_view.updateChildWME( 'yaw-body', self._yaw_bins_map(yaw)) # string
                soar_view.updateChildWME( 'yaw-body-numeric', math.degrees(yaw) ) # float
                soar_view.updateChildWME( 'yaw-body-abs-numeric', abs(math.degrees(yaw)) ) # float

    def __del__(self):
        # remove sensor wme and ROS subscriber
        if self._sensor_id:
            self._sensor_id.DestroyWME()
        if self._detections_sub:
            self._detections_sub.unregister()
        # unregister SWM
        SpatialWorldModel._swm_instance_ref = None

register("swm", SpatialWorldModel)
