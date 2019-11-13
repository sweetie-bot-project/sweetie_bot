#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.execute_step_sequence import ExecuteStepSequence
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 11 2019
@author: disRecord
'''
class ExecuteStepSequenceSM(Behavior):
	'''
	Execute StepSequence. If robot is standing then it crouchs beforehand.
	'''


	def __init__(self):
		super(ExecuteStepSequenceSM, self).__init__()
		self.name = 'ExecuteStepSequence'

		# parameters of this behavior
		self.add_parameter('action_name', 'walk_fwd_40')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:43 y:392, x:240 y:396, x:477 y:395
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'invalid_pose'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:161 y:101
			OperatableStateMachine.add('StepSequence',
										ExecuteStepSequence(controller='motion/controller/step_sequence', trajectory_param=self.action_name, trajectory_ns='saved_msgs/step_sequence'),
										transitions={'success': 'finished', 'partial_movement': 'invalid_pose', 'invalid_pose': 'Crouch', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:413 y:104
			OperatableStateMachine.add('Crouch',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_begin', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'StepSequence', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
