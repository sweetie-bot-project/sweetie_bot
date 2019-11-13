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
from sweetie_bot_flexbe_states.compound_action_state import CompoundAction
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 21 2018
@author: mutronics
'''
class RBC18Part2SM(Behavior):
	'''
	RBC Presentation Part2
	'''


	def __init__(self):
		super(RBC18Part2SM, self).__init__()
		self.name = 'RBC18Part2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		# x:20 y:231, x:260 y:311
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:57 y:34
			OperatableStateMachine.add('WaitKey1',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'SpasiboMut', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:179 y:106
			OperatableStateMachine.add('SpasiboMut',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='spasibo_mut_tvoyo_uporstvo_vsegda_radovalo_menya', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:442 y:360
			OperatableStateMachine.add('WaitKey3',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'SpasiboZuviel', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:123 y:441
			OperatableStateMachine.add('SpasiboZuviel',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='prekrasno_chuvstvuete_kakaya_skrita_vo_mne_mosch', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:428 y:236
			OperatableStateMachine.add('SpasiboStefanShiron',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='spasibo_ya_uslishala_vse_chto_hotela', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:370 y:44
			OperatableStateMachine.add('WaitKey2',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'SpasiboStefanShiron', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
