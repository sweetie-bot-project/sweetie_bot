#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_derpfest2017presentation')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from behavior_derpfest2017presentation2.derpfest2017presentation2_sm import Derpfest2017Presentation2SM
from behavior_derpfest2017presentation3.derpfest2017presentation3_sm import Derpfest2017Presentation3SM
from behavior_derpfest2017presentation4.derpfest2017presentation4_sm import Derpfest2017Presentation4SM
from behavior_derpfest2017presentation1.derpfest2017presentation1_sm import Derpfest2017Presentation1SM
from flexbe_states.decision_state import DecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 28 2017
@author: disRecord
'''
class Derpfest2017PresentationSM(Behavior):
	'''
	Encapsulates all
	'''


	def __init__(self):
		super(Derpfest2017PresentationSM, self).__init__()
		self.name = 'Derpfest2017Presentation'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(Derpfest2017Presentation2SM, 'Derpfest2017Presentation2')
		self.add_behavior(Derpfest2017Presentation3SM, 'Derpfest2017Presentation3')
		self.add_behavior(Derpfest2017Presentation4SM, 'Derpfest2017Presentation4')
		self.add_behavior(Derpfest2017Presentation1SM, 'Derpfest2017Presentation1')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:61 y:613
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.placeholder = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:217
			OperatableStateMachine.add('EyesNormal',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'Select'},
										autonomy={'done': Autonomy.Off})

			# x:371 y:211
			OperatableStateMachine.add('Derpfest2017Presentation2',
										self.use_behavior(Derpfest2017Presentation2SM, 'Derpfest2017Presentation2'),
										transitions={'finished': 'finished', 'failed': 'Select'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:358 y:412
			OperatableStateMachine.add('Derpfest2017Presentation3',
										self.use_behavior(Derpfest2017Presentation3SM, 'Derpfest2017Presentation3'),
										transitions={'finished': 'Derpfest2017Presentation4', 'failed': 'Select'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:281 y:582
			OperatableStateMachine.add('Derpfest2017Presentation4',
										self.use_behavior(Derpfest2017Presentation4SM, 'Derpfest2017Presentation4'),
										transitions={'finished': 'finished', 'failed': 'Select'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:353 y:71
			OperatableStateMachine.add('Derpfest2017Presentation1',
										self.use_behavior(Derpfest2017Presentation1SM, 'Derpfest2017Presentation1'),
										transitions={'finished': 'Derpfest2017Presentation2', 'failed': 'Select'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:165 y:63
			OperatableStateMachine.add('Select',
										DecisionState(outcomes=['1-begining', '2-construction', '3-evil', '4-end'], conditions=lambda x: '1-begining'),
										transitions={'1-begining': 'Derpfest2017Presentation1', '2-construction': 'Derpfest2017Presentation2', '3-evil': 'Derpfest2017Presentation3', '4-end': 'Derpfest2017Presentation4'},
										autonomy={'1-begining': Autonomy.Full, '2-construction': Autonomy.Off, '3-evil': Autonomy.Off, '4-end': Autonomy.Off},
										remapping={'input_value': 'placeholder'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
