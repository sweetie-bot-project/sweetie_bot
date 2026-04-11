#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.compound_action_param import CompoundActionParam
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 11 2019
@author: disrecord
'''
class ExecuteCompoundActionSM(Behavior):
	'''
	Execute CompoundAction defined by parameter.
	'''


	def __init__(self):
		super(ExecuteCompoundActionSM, self).__init__()
		self.name = 'ExecuteCompoundAction'

		# parameters of this behavior
		self.add_parameter('action_name', 'brohoof')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:183 y:341, x:352 y:353
		_state_machine = OperatableStateMachine(outcomes=['succeed', 'failed', 'invalid_pose'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:108 y:85
			OperatableStateMachine.add('CompoundAction',
										CompoundActionParam(action_param=self.action_name, action_ns='saved_msgs/compound_action'),
										transitions={'success': 'succeed', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
