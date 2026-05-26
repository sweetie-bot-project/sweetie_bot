from . import input_module
from .input_module import InputModule, InputModulesLoader

from threading import Lock

import rospy
from sensor_msgs.msg import JointState

from . import robot_pose
from .bins import BinsMap

class PoseJoints(InputModule):
    def _init(self, name, config, agent):
        # preinit
        self._joint_state_sub = None

        # get configuration from parameters
        joint_state_topic = self.getConfigParameter(config, 'topic', allowed_types=str)
        storage_ns = self.getConfigParameter(config, 'storage_ns', allowed_types=str)
        pose_name_list = self.getConfigParameter(config, 'pose_list', allowed_types=list, check_func = lambda lst: all(isinstance(p, str) for p in lst), error_desc = 'must be list of string.')
        default_tolerance = self.getConfigParameter(config, 'tolerance', allowed_types=(float, int), check_func = lambda tol: tol >= 0.0, error_desc='must be positive number.')
        try:
            self._time_bins_map = BinsMap( config['time_bins_map'] )
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError('PoseJoint input module: "time_bins_map" parameter is not present or invalid.') from e
        mov_joints = self.getConfigParameter(config, 'movement_detection_joints', allowed_types=list, check_func = lambda lst: all(isinstance(p, str) for p in lst), error_desc = 'must be list of string.')
        self._mov_joints = set(mov_joints)
        self._mov_tolerance= self.getConfigParameter(config, 'movement_detection_velocity_threshold', allowed_types=(float, int), check_func = lambda tol: tol >= 0.0, error_desc='must be positive number.')

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
       
        # subscriber    
        self._joint_state_sub = rospy.Subscriber(joint_state_topic, JointState, self.newJointStateCallback)

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

    def _deinit(self):
        # remove ROS subscriber
        if self._joint_state_sub:
            self._joint_state_sub.unregister()

InputModulesLoader.register("pose_joints", PoseJoints)
