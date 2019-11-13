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
from sweetie_bot_flexbe_states.compound_action_param_key import CompoundActionParamKey
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Mon Jul 15 2019
@author: disRecord
'''
class RandomCompoundActionSM(Behavior):
	'''
	Select and execute random action from list.
	'''


	def __init__(self):
		super(RandomCompoundActionSM, self).__init__()
		self.name = 'RandomCompoundAction'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:572 y:333, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.unused = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:170 y:138
			OperatableStateMachine.add('SelectAction',
										CalculationState(calculation=lambda x: random.choice(['brohoof', 'menace'])),
										transitions={'done': 'ExecuteCompoundAction'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'unused', 'output_value': 'action_param'})

			# x:330 y:170
			OperatableStateMachine.add('ExecuteCompoundAction',
										CompoundActionParamKey(action_ns='saved_msgs/compound_action'),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'action_param': 'action_param'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
