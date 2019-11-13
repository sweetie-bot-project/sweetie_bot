#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.rand_joints_movements import RandJointsMovements
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 29 2019
@author: disRecord
'''
class RandHeadProto3SM(Behavior):
	'''
	Random head movements for proto3.
	'''


	def __init__(self):
		super(RandHeadProto3SM, self).__init__()
		self.name = 'RandHeadProto3'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:353, x:130 y:353
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.config = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:76 y:87
			OperatableStateMachine.add('SetHeadNominal',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'Wait', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:244 y:213
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=20),
										transitions={'done': 'RandMoves'},
										autonomy={'done': Autonomy.Off})

			# x:338 y:62
			OperatableStateMachine.add('RandMoves',
										RandJointsMovements(controller='joint_state_head', duration=30, interval=[4.0,6.0], joints=['head_joint2', 'head_joint3'], minimal=[0.1,-0.3], maximal=[0.7, 0.3 ]),
										transitions={'done': 'SetHeadNominal', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'config': 'config'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
