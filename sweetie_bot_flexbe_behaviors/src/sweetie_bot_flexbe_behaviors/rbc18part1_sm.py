#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 20 2018
@author: mutronics
'''
class RBC18Part1SM(Behavior):
	'''
	RBC18 Presentation Part1
	'''


	def __init__(self):
		super(RBC18Part1SM, self).__init__()
		self.name = 'RBC18Part1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		# x:88 y:510, x:251 y:235
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:27 y:26
			OperatableStateMachine.add('WaitKey1',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Zdrav1', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:138 y:99
			OperatableStateMachine.add('Zdrav1',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='zdravstvuyte_kojannie_meshki', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:454 y:283
			OperatableStateMachine.add('Zdrav2',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='zdravstvuyte_dorogie_gosti', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey4', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:396 y:90
			OperatableStateMachine.add('Atochn',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='a_tochno_sboi_pamayati', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:455 y:174
			OperatableStateMachine.add('WaitKey3',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Zdrav2', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:416 y:430
			OperatableStateMachine.add('WaitKey4',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'NoEsli', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:277 y:350
			OperatableStateMachine.add('NoEsli',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='no_esli_snova_otkluchish_mikrofon', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey5', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:169 y:436
			OperatableStateMachine.add('WaitKey5',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'NamPora', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:23 y:341
			OperatableStateMachine.add('NamPora',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='nam_pora_nachinat', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:250 y:11
			OperatableStateMachine.add('WaitKey2',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Atochn', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
