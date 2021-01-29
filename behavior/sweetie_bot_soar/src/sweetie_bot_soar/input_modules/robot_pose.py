from xmlrpc.client import Binary

import rospy
from sensor_msgs.msg import JointState

class PoseWithDefaultTolerance:
    def __init__(self, name, msg, tolerance):
        self.name = name
        self._pose = { joint: position for joint, position in zip(msg.name, msg.position) }
        self._tolerance = tolerance

    def check(self, msg):
        # check if tolerance is exeeded for some joint
        for joint, position in zip(msg.name, msg.position):
            target_position = self._pose.get(joint)
            if target_position == None:
                continue
            if abs(target_position - position) > self._tolerance:
                return False
        return True

class PoseWithTolerance:
    class Pair:
        def __init__(self, position, tolerance):
            self.position, self.tolerance = position, tolerance

    def __init__(self, name, msg, tol_msg, default_tolerance):
        self.name = name
        # get pose
        self._pose = { joint: PoseWithTolerance.Pair(position, default_tolerance) for joint, position in zip(msg.name, msg.position) }
        # set tolerance if it is provided
        for joint, tolerance in zip(tol_msg.name, tol_msg.position):
            pair = self._pose.get(joint)
            if pair != None:
                pair.tolerance = tolerance

    def check(self, msg):
        # check if tolerance is exeeded for some joint
        for joint, position in zip(msg.name, msg.position):
            pair = self._pose.get(joint)
            if pair == None:
                continue
            if abs(pair.position - position) > pair.tolerance:
                return False
        return True

def load_joint_state_param(pose_param):
    # Load JointState message from Parameter Server
    # check if parameter exists
    if not rospy.has_param(pose_param):
        return None
    # get parameter
    pose_raw = rospy.get_param(pose_param)
    if not isinstance(pose_raw, Binary):
        raise RuntimeError("Robot pose: ROS parameter '%s' is not binary data." % pose_param)
    # deserialize
    msg = JointState()
    try:
        msg.deserialize(pose_raw.data)
        if len(msg.name) != len(msg.position):
            raise TypeError
    except:
        raise RuntimeError("Robot_pose: ROS parameter '%s' contains ivalid JointState message." % pose_param)
    return msg
