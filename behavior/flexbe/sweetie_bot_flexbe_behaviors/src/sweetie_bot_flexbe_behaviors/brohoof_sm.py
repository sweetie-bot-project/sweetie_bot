#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from flexbe_manipulation_states.get_joint_values_state import GetJointValuesState
from flexbe_states.calculation_state import CalculationState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import math
import numpy
import tf
from tf.transformations import quaternion_about_axis
from flexbe_core.proxy import ProxyTransformListener

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Wed Aug 02 2017
@author: disRecord
'''
class BrohoofSM(Behavior):
	'''
	Move leg1 up to specified target, perform greeting, wait and pt leg down. 

Robot is assumed to be standing on four legs.
	'''


	def __init__(self):
		super(BrohoofSM, self).__init__()
		self.name = 'Brohoof'

		# parameters of this behavior
		self.add_parameter('wait_time', 5)
		self.add_parameter('neck_angle', 0.25)
		self.add_parameter('brohoof_cone', 0.78)
		self.add_parameter('brohoof_upper_limit', 0.30)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

                self._tf = ProxyTransformListener().listener()
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:228 y:396, x:225 y:240, x:1010 y:659
		_state_machine = OperatableStateMachine(outcomes=['finished', 'unreachable', 'failed'], input_keys=['pose'])
		_state_machine.userdata.pose = PoseStamped(Header(frame_id = 'base_link'), Pose(Point(0.4, 0.0, -0.05), Quaternion(0, 0, 0, 1)))

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:118 y:85
			OperatableStateMachine.add('CheckHandPosition',
										DecisionState(outcomes=['good','bad'], conditions=self.checkHandPosition),
										transitions={'good': 'GetHeadPose', 'bad': 'unreachable'},
										autonomy={'good': Autonomy.Off, 'bad': Autonomy.Off},
										remapping={'input_value': 'pose'})

			# x:890 y:253
			OperatableStateMachine.add('SayHello',
										TextCommandState(type='voice/play_wav', command='hello_im_sweetie_bot_friedship_programms', topic='control'),
										transitions={'done': 'Wait'},
										autonomy={'done': Autonomy.Off})

			# x:663 y:69
			OperatableStateMachine.add('RaiseHead',
										MoveitToJointsState(move_group='head', joint_names=['joint51', 'joint52', 'joint53', 'joint54'], action_topic='move_group'),
										transitions={'reached': 'RaiseLegProgram', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'head_pose'})

			# x:309 y:66
			OperatableStateMachine.add('GetHeadPose',
										GetJointValuesState(joints=['joint51', 'joint52', 'joint53', 'joint54']),
										transitions={'retrieved': 'CalcRaisedHeadPose'},
										autonomy={'retrieved': Autonomy.Off},
										remapping={'joint_values': 'head_pose'})

			# x:485 y:73
			OperatableStateMachine.add('CalcRaisedHeadPose',
										CalculationState(calculation=self.calculateHeadPoseFunc),
										transitions={'done': 'RaiseHead'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'head_pose', 'output_value': 'head_pose'})

			# x:891 y:65
			OperatableStateMachine.add('RaiseLegProgram',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='brohoof_begin', trajectory_ns='/saved_msgs/joint_trajectory'),
										transitions={'success': 'SayHello', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:510 y:396
			OperatableStateMachine.add('ReturnLegProgrammed',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='brohoof_end', trajectory_ns='/saved_msgs/joint_trajectory'),
                                                                                transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:851 y:399
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'ReturnLegProgrammed'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
        def calculateHeadPoseFunc(self, joints):
	        joints = list(joints)
	        joints[0] = 0.0
	        joints[2] = self.neck_angle
	        return joints

	def checkHandPosition(self, input_pose):
	        '''
	        Check if hand is in good position for brohoof. Return 'good' or 'bad'.
	        '''
	        # check if frame header presents
	        if not input_pose.header.frame_id:
	            Logger.logerr('checkHandPosition: Header is missing.')
	            return 'bad'
	        # convert ot base_link frame
	        try:
	            output_pose = self._tf.transformPose('base_link', PoseStamped(Header(frame_id = input_pose.header.frame_id), input_pose.pose))
	        except tf.Exception as e:
	            Logger.logerr('calculateHeadPoseFunc: Cannot transform from `%s` to `base_link` frame.' % input_pose.header.frame_id)
	            return 'bad'
	        pos = output_pose.pose.position
	        Logger.loginfo('calculateHeadPoseFunc: object pos: %s' % str(pos))
	        # Check if boundarie are good
	        if pos.z > self.brohoof_upper_limit or pos.z < -0.15: 
	            # too high or to low
	            return 'bad'
	        if pos.x < 0.25:
	            # too close
                    return 'bad'
                # Move to shoulder1 point
                pos.x -= 0.080
                pos.y -= 0.037
                pos.z += 0.027
                # Check if angle is out of cone
                if abs(math.atan2(pos.y,pos.x)) > self.brohoof_cone:
                    # out of cone
                    return 'bad'
	        return 'good'
    # [/MANUAL_FUNC]
