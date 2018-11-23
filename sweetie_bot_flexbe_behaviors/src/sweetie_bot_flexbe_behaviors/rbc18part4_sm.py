#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Nov 23 2018
@author: disRecord
'''
class RBC18Part4SM(Behavior):
	'''
	End of RBC2018 presentation: Sweetie says goodbye.
	'''


	def __init__(self):
		super(RBC18Part4SM, self).__init__()
		self.name = 'RBC18Part4'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:27 y:362, x:435 y:185
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:239 y:71
			OperatableStateMachine.add('ReturnToNormal',
										SweetieBotCompoundAction(t1=[0,0.0], type1='eyes/emotion', cmd1='normal', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_node', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'Goodbye', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:234 y:264
			OperatableStateMachine.add('Goodbye',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='spasibo_za_vnimanie', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='introduce_herself_slow', t3=[0,8.0], type3='motion/joint_trajectory', cmd3='bow_begin', t4=[3,4.0], type4='motion/joint_trajectory', cmd4='bow_end'),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
