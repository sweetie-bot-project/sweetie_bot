#!/usr/bin/env python3
import numpy
import numpy.linalg
import math

from tf.transformations import quaternion_inverse, quaternion_multiply

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class PoseVectorized():
    def __init__(self, pose = None):
        if not pose:
            self.header = Header()
            self.position = numpy.array([0, 0, 0])
            self.orientation = numpy.array([0, 0, 0, 1.0])
        elif isinstance(pose, PoseStamped):
            self.header = Header(frame_id = pose.header.frame_id, stamp = pose.header.stamp, seq = pose.header.seq)
            self.position = numpy.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]) / 1000.0
            self.orientation = numpy.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        elif isinstance(pose, PoseVectorized):
            self.header = Header(frame_id = pose.header.frame_id, stamp = pose.header.stamp, seq = pose.header.seq)
            self.position = numpy.copy(pose.position)
            self.orientation = numpy.copy(pose.orientation)
        else:
            raise TypeError('PoseStamped or PoseVectorized is expected as constructor argument.')

    def average_exp(avg_value, new_value, tau):
        T = (new_value.header.stamp - avg_value.header.stamp).to_sec()
        sigma = math.exp(-T/tau)
        avg_value.header.stamp = new_value.header.stamp
        avg_value.position = sigma * avg_value.position + (1.0 - sigma) * new_value.position
        avg_value.orientation = sigma * avg_value.orientation + (1.0 - sigma) * new_value.orientation

    def eq(self, other):
        return self.header.frame_id == other.header.frame_id and numpy.all(self.position == other.position) and numpy.all(self.orientation == other.orientation)

    def eq_approx(self, other, position_tolerance, orientation_tolerance):
        if self.header.frame_id != other.header.frame_id:
            return False
        diff = quaternion_multiply(self.orientation, quaternion_inverse(other.orientation))
        angle = 2*math.atan2(numpy.linalg.norm(diff[:3]), diff[3])
        # check position
        if numpy.linalg.norm(self.position - other.position) > position_tolerance:
            return False

        # check rotation
        if angle > orientation_tolerance:
            return False
        return True

    def toPoseStamped(self):
        return PoseStamped(header = self.header, pose = Pose(position = Point(*self.position), orientation = Quaternion(*self.orientation)))

