#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.calculation_state import CalculationState
from sweetie_bot_flexbe_states.set_cartesian_pose import SetCartesianPose
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import tf
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Tue Feb 15 2022
@author: disRecord
'''
class ExecuteSetStanceSM(Behavior):
	'''
	Place robot base on specific height using FollowStance controller
	'''


	def __init__(self):
		super(ExecuteSetStanceSM, self).__init__()
		self.name = 'ExecuteSetStance'

		# parameters of this behavior
		self.add_parameter('target_height', 0.17)
		self.add_parameter('frame_id', 'base_link_path')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:207 y:453, x:547 y:453
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_height = self.target_height

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:149 y:76
			OperatableStateMachine.add('CalcTargetPose',
										CalculationState(calculation=self.create_pose_msg),
										transitions={'done': 'SetPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'target_height', 'output_value': 'pose'})

			# x:217 y:212
			OperatableStateMachine.add('SetPose',
										SetCartesianPose(controller='motion/controller/stance', pose=None, frame_id=self.frame_id, frame_is_moving=False, resources=['leg1','leg2','leg3','leg4'], actuator_frame_id='base_link', tolerance_lin=0.01, tolerance_ang=0.085, timeout=10.0),
										transitions={'done': 'finished', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def create_pose_msg(self, height):
		pose = PoseStamped()
		pose.header.frame_id = self.frame_id
		pose.pose.position.x = 0.0
		pose.pose.position.y = 0.0
		pose.pose.position.z = height
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0) # RPY -> Quaternion
		pose.pose.orientation.x = quaternion[0]
		pose.pose.orientation.y = quaternion[1]
		pose.pose.orientation.z = quaternion[2]
		pose.pose.orientation.w = quaternion[3]
		return pose
	
	# [/MANUAL_FUNC]
