#!/usr/bin/env python
import math
import numpy

import rospy
import tf
from tf.transformations import rotation_matrix

from flexbe_core.proxy import ProxyTransformListener
from flexbe_core import Logger

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
#from geometry_msgs.msg import PointStamped


class HeadIK:
    '''
    Implements SweetieBot head and eyes invese kinematics. Meant to be run in FlexBe state enviroment.

    '''

    def __init__(self):
        '''
        Get tf proxy and neessary transform. 
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
        '''
        # get transform listener singlenton
        self._tf = ProxyTransformListener().listener()
        # wait for pose publication
        self._tf.waitForTransform('bone51', 'base_link', rospy.Time(), rospy.Duration(10.0))
        # get neck base translation vector in base_link frame
        (trans, rot) = self._tf.lookupTransform('base_link', 'bone51', rospy.Time())
        self._t5051 = trans;
        # get translation from bone51 to bone52
        (trans, rot) = self._tf.lookupTransform('bone51', 'bone52', rospy.Time())
        self._t5152 = trans;

    def pointDirectionToHeadPose(self, point_stamped, neck_angle, side_angle, limit_joints = True):
        '''
        Calculate Proto2 head pose so it is oriented toward the given point. 
        :param point_stamped: geometry_msg.msg.PointStamped point to look at.
        :param neck_angle: joint51 desired angle in radians.
        :param side_angle: joint54 desired angle in radians.
        :param limit_joints: limit joints positions to prevent self collisions.
        :return: sensor_msg.msg.JointState head pose (joint ranges are not checked). Return `None` if transform is not available.
        '''

        joints = JointState()
        joints.header = Header()
        joints.header.stamp = rospy.Time.now()
        # set known fields
        joints.name = [ 'joint51', 'joint52', 'joint53', 'joint54' ]
        joints.position = [ neck_angle, 0.0, 0.0, 0.0 ]
        
        try:
            # convert point coordinates to base_link frame
            point_stamped = self._tf.transformPoint('base_link', PointStamped(header = Header(frame_id = point_stamped.header.frame_id), point = point_stamped.point))
        except tf.Exception:
            Logger.logwarn('Cannot transform to base_link:\n%s' % str(e))
            return None 

        # move to neck base
        direction = [ point_stamped.point.x, point_stamped.point.y, point_stamped.point.z, 1.0]
        upward = [ 0.0, 0.0, 1.0, 1.0]
        direction[:3] = numpy.subtract(direction[:3], self._t5051)
        upward[:3] = numpy.subtract(upward[:3], self._t5051)
        # rotate to neck_angle around Ox to transform it to desired bone51 frame.
        direction = numpy.dot(rotation_matrix(-numpy.pi/2 + neck_angle, (1, 0, 0)), direction)
        upward = numpy.dot(rotation_matrix(-numpy.pi/2 + neck_angle, (1, 0, 0)), upward)
        # move origin to bone52
        direction[:3] = numpy.subtract(direction[:3], self._t5152)
        upward[:3] = numpy.subtract(upward[:3], self._t5152)
        # calculate rotation angles
        joints.position[1] = -math.atan2(direction[0], direction[2])
        upward = numpy.dot(rotation_matrix(joints.position[1], (0, 1, 0)), upward)
        joints.position[2] = math.atan2(direction[1], math.sqrt(direction[0]**2 + direction[2]**2))
        upward = numpy.dot(rotation_matrix(joints.position[2], (1, 0, 0)), upward)
        joints.position[3] = math.atan2(upward[0], upward[1]) + side_angle
        # limit joints
        if limit_joints:
            joints.position = self.limitHeadJoints51to54(joints.position)

        return joints

    def pointDirectionToEyesPose(self, point_stamped):
        '''
        Calculate Proto2 eyes angles for deprecated eyes module.
        :param point_stamped: geometry_msg.msg.PointStamped point to look at.
        :return: sensor_msg.msg.JointState head pose (joint ranges are not checked). Return `None` if transform is not available.
        '''
        joints = JointState()
        joints.header = Header()
        joints.header.stamp = rospy.Time.now()
        # set known fields
        joints.name = [ 'joint55', 'joint56' ]
        joints.position = [ 0.0, 0.0 ]
        
        try:
            # convert point coordinates to base_link frame
            point_stamped = self._tf.transformPoint('bone54', PointStamped(header = Header(frame_id = point_stamped.header.frame_id), point = point_stamped.point))
        except tf.Exception as e:
            Logger.logwarn('Cannot transform to bone54:\n%s' % str(e))
            return None 
        # calculate angles
        direction = [ point_stamped.point.x, point_stamped.point.y, point_stamped.point.z ]
        joints.position[0] = math.atan2(point_stamped.point.y, math.sqrt(point_stamped.point.x**2 + point_stamped.point.z**2))
        joints.position[1] = -math.atan2(point_stamped.point.x, math.sqrt(point_stamped.point.y**2 + point_stamped.point.z**2))
        # limit joints position
        joints.position = list(numpy.clip(joints.position, -numpy.pi/2, numpy.pi/2))

        return joints

    def limitHeadJoints51to54(self, position):
        '''
        Clip joint51, joint52, joint53, joint54 to collision safe diapazone.
        :param position: array of joint positions in natural order (joint51,.. joint54)
        :return: clipped array.
        '''
        return list( numpy.clip( position, [ -1.00, -1.57, -0.55, -0.65 ], [ 0.25, 1.57, 1.0, 0.65 ]) )



