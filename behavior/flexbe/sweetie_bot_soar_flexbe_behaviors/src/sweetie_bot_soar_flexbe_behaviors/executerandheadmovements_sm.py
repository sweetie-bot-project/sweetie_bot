#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.check_joint_state import CheckJointState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.rand_joints_movements import RandJointsMovements
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Nov 15 2019
@author: disrRecord
'''
class ExecuteRandHeadMovementsSM(Behavior):
	'''
	Move head in random directions
	'''


	def __init__(self):
		super(ExecuteRandHeadMovementsSM, self).__init__()
		self.name = 'ExecuteRandHeadMovements'

		# parameters of this behavior
		self.add_parameter('timeout', 5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:353, x:488 y:318
		_state_machine = OperatableStateMachine(outcomes=['succeed', 'failed'])
		_state_machine.userdata.unused = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:189 y:30
			OperatableStateMachine.add('CheckCrouched',
										CheckJointState(outcomes=['body_crouched', 'unknown'], pose_ns='saved_msgs/joint_state', tolerance=0.17, joint_topic="joint_states", timeout=1.0),
										transitions={'body_crouched': 'StandUp', 'unknown': 'RandMovements'},
										autonomy={'body_crouched': Autonomy.Off, 'unknown': Autonomy.Off})

			# x:196 y:256
			OperatableStateMachine.add('HeadNominal',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'succeed', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:192 y:143
			OperatableStateMachine.add('RandMovements',
										RandJointsMovements(controller='joint_state_head', duration=self.timeout, interval=[2.0,4.0], joints=['head_joint2', 'head_joint3', 'head_joint4'], minimal=[ -0.05, -0.2, -0.2 ], maximal=[ 0.2, 0.2, 0.2 ]),
										transitions={'done': 'HeadNominal', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'config': 'unused'})

			# x:445 y:78
			OperatableStateMachine.add('StandUp',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_end', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'RandMovements', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
