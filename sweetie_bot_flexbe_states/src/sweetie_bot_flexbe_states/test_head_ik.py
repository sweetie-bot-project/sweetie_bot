#!/usr/bin/env python
from time import sleep
import numpy

import rospy
from flexbe_core.proxy import ProxyPublisher

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point

from proto2.head_ik import HeadIK

# Init node
rospy.init_node('test_head_ik')

ik = HeadIK()

#pub = rospy.Publisher('/sweetie_bot/motion/controller/joint_state_head/in_joints_ref', JointState, queue_size = 10)
pub = rospy.Publisher('/sweetie_bot/motion/controller/joint_state/out_joints_src_reset', JointState, queue_size = 10)

# Init point header
point = PointStamped()
point.header = Header(stamp = rospy.Time.now(), frame_id = 'base_link')

# helper function
def lookAtPoint(point_stamped, neck_angle = 0.0):
    print 'Head direction point: \n', point
    
    joints = ik.pointDirectionToHeadPose(point_stamped, neck_angle, 0.0)
    print 'IK solution: \n', joints

    pub.publish(joints)

# test sequence

point.point = Point(1.0, 0.0, 0.0)
lookAtPoint(point)
sleep(2.0)

point.point = Point(1.0, 0.0, 0.0)
lookAtPoint(point)
sleep(2.0)

point.point = Point(1.0, 1.0, 0.0)
lookAtPoint(point)
sleep(2.0)

point.point = Point(1.0, -1.0, 0.0)
lookAtPoint(point)
sleep(2.0)

point.point = Point(1.0, -1.0, .3)
lookAtPoint(point)
sleep(2.0)

point.point = Point(1.0, -1.0, .3)
lookAtPoint(point, -numpy.pi/4)
sleep(2.0)

point.point = Point(1.0, -1.0, 0.0)
lookAtPoint(point, -numpy.pi/4)
sleep(2.0)

point.point = Point(1.0, -1.0, 0.0)
lookAtPoint(point, numpy.pi/4)
sleep(2.0)







