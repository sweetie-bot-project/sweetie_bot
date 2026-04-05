from . import input_module

from threading import Lock

import rospy
from sensor_msgs.msg import JointState

from . import robot_pose
from .bins import BinsMap

class PoseJoints:
    def __init__(self, name, config, agent):
        self._joint_state_sub = None

        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)

        # get configuration from parameters
        joint_state_topic = config.get("topic")
        if not joint_state_topic or not isinstance(joint_state_topic, str):
            raise RuntimeError("Pose input module: 'topic' parameter is not defined or is not string.")
        storage_ns = config.get("storage_ns")
        if not storage_ns or not isinstance(storage_ns, str):
            raise RuntimeError("Pose input module: 'storage_ns' parameter is not defined or is not string.")
        pose_name_list = config.get("pose_list")
        if not pose_name_list or not isinstance(pose_name_list, list) or not all(isinstance(p, str) for p in pose_name_list):
            raise RuntimeError("Pose input module: 'pose_list' must be list of strings.")
        default_tolerance = config.get("tolerance")
        if not default_tolerance or not isinstance(default_tolerance, (float, int)) or default_tolerance < 0:
            raise RuntimeError("Pose input module: 'default_tolerance' must be positive nuber.")
        try:
            self._time_bins_map = BinsMap( config['time_bins_map'] )
        except KeyError:
            raise RuntimeError('Pose input module: "time_bins_map" parameter must present.')
        mov_joints = config.get("movement_detection_joints")
        if not isinstance(mov_joints, list) or not all(isinstance(p, str) for p in mov_joints):
            raise RuntimeError("Pose input module: 'movement_detection_joints' must be list of strings.")
        self._mov_joints = set(mov_joints)
        self._mov_tolerance = config.get("movement_detection_velocity_threshold")
        if not isinstance(self._mov_tolerance, (int, float)) or self._mov_tolerance < 0.0:
            raise RuntimeError("Pose input module: 'movement_detection_velocity_threshold' must be positive number.")

        # load poses mentioned in list from Parameter Server
        self._pose_list = []
        for pose_name in pose_name_list:
            # get pose
            msg = robot_pose.load_joint_state_param(storage_ns + '/joint_state/' + pose_name)
            if msg == None:
                raise RuntimeError("Pose input module: pose parameter '%s' does not exists." % (pose_name, ))
            # get tolerance
            msg_tol = robot_pose.load_joint_state_param(storage_ns + '/joint_state_tolerance/' + pose_name)
            # add pose to list
            if msg_tol == None:
                self._pose_list.append( robot_pose.PoseWithDefaultTolerance(pose_name, msg, default_tolerance) )
            else:
                self._pose_list.append( robot_pose.PoseWithTolerance(pose_name, msg, tol_msg, default_tolerance) )
       
        # subscriber    
        self._joint_state_sub = rospy.Subscriber(joint_state_topic, JointState, self.newJointStateCallback)

        # message buffers
        self._lock = Lock()
        self._joint_state_msg = None
        # WME ids cache
        self._pose_wme_id = self._sensor_id.CreateStringWME("pose", "unknown")
        self._time_wme_id = self._sensor_id.CreateStringWME("change-time", self._time_bins_map(0.0))
        self._mov_wme_id = self._sensor_id.CreateIntWME("moving", 0)
        # last pose index and pose change time
        self._last_pose_index = None
        self._last_pose_change_time = rospy.Time.now()

    def newJointStateCallback(self, msg):
        # buffer msg
        with self._lock:
            self._joint_state_msg = msg

    def update(self):
        # check if input was updated
        with self._lock:
            if self._joint_state_msg == None:
                return
            joint_state_msg = self._joint_state_msg

        # get current time
        time_now = rospy.Time.now()

        # check movement
        moving = False
        for joint, vel in zip(joint_state_msg.name, joint_state_msg.velocity):
            if joint in self._mov_joints and vel > self._mov_tolerance:
                moving = True
                break
        # update movement WME
        if moving != bool(self._mov_wme_id.GetValue()):
            self._mov_wme_id.Update(int(moving))

        # check current pose
        if self._last_pose_index != None:
            if self._pose_list[self._last_pose_index].check(joint_state_msg):
                # pose has not changed: only time update is needed
                time_value = self._time_bins_map( (time_now - self._last_pose_change_time).to_sec() );
                if time_value != self._time_wme_id.GetValue():
                    self._time_wme_id.Update(time_value)
                return

        # find corresponding pose
        for pose_index in range(0, len(self._pose_list)):
            if self._pose_list[pose_index].check( joint_state_msg ):
                # corresponding pose found
                self._last_pose_index = pose_index
                self._last_pose_change_time = time_now
                # update WMEs
                self._pose_wme_id.Update( self._pose_list[pose_index].name )
                self._time_wme_id.Update( self._time_bins_map(0.0) )
                return

        # pose not found
        # check if previous pose is known
        if self._last_pose_index is not None:
            # pose was known: chnge WMEs and reset time
            self._last_pose_change_time = time_now
            self._last_pose_index = None
            self._pose_wme_id.Update('unknown')
            self._time_wme_id.Update( self._time_bins_map(0.0) )
            return
        else:
            # pose remains unknown
            if moving:
                # pose is changing so continously reset time
                self._last_pose_change_time = time_now
                time_value = self._time_bins_map(0.0)
            else:
                # update time
                time_value = self._time_bins_map( (time_now - self._last_pose_change_time).to_sec() );
            # update time WME
            if time_value != self._time_wme_id.GetValue():
                self._time_wme_id.Update(time_value)
            return

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        if self._joint_state_sub:
            self._joint_state_sub.unregister()

input_module.register("pose_joints", PoseJoints)
