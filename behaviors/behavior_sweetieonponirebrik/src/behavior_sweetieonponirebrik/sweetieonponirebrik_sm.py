#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_sweetieonponirebrik')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.rand_head_movements_state import SweetieRandHeadMovementsState
from flexbe_states.subscriber_state import SubscriberState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Mar 28 2017
@author: disRecord
'''
class SweetieOnPoniRebrikSM(Behavior):
	'''
	Behavior for presentation on PoniRebrik.
	'''


	def __init__(self):
		super(SweetieOnPoniRebrikSM, self).__init__()
		self.name = 'SweetieOnPoniRebrik'

		# parameters of this behavior
		self.add_parameter('joint_state_control_topic', ''motion/controller/joint_state/out_joints_src_reset'')
		self.add_parameter('joint_trajectory_control_topic', 'motion/controller/joint_trajectory')
		self.add_parameter('voice_topic', 'voice/voice')
		self.add_parameter('torque_off_service', 'motion/torque_off/set_operational')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:68 y:535, x:735 y:524
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:277 y:427, x:109 y:425
		_sm_greetings_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_greetings_0:
			# x:98 y:181
			OperatableStateMachine.add('CheckJoyTopic',
										SubscriberState(topic=self.joy_topic, blocking=False, clear=False),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})


		# x:538 y:386, x:347 y:384, x:134 y:383, x:831 y:385, x:906 y:386
		_sm_idlebehavior_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('RandomHeadMovements', 'done')]),
										('finished', [('WaitForJoyEvent', 'received')]),
										('failed', [('WaitForJoyEvent', 'unavailable')])
										])

		with _sm_idlebehavior_1:
			# x:474 y:110
			OperatableStateMachine.add('RandomHeadMovements',
										SweetieRandHeadMovementsState(topic=self.joint_state_control_topic, duration=100, interval=5, max2356=[ 0.3, 0.3, 1.5, 1.5 ], min2356=[ -0.3, -0.3, -1.5, -1.5 ]),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:148 y:118
			OperatableStateMachine.add('WaitForJoyEvent',
										SubscriberState(topic=self.joy_topic, blocking=True, clear=True),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})



		with _state_machine:
			# x:165 y:175
			OperatableStateMachine.add('IdleBehavior',
										_sm_idlebehavior_1,
										transitions={'finished': 'Greetings', 'failed': 'AskForAssistance'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:502 y:411
			OperatableStateMachine.add('AskForAssistance',
										TextCommandState(topic=self.voice_topic, type='voice/play_wav', command='assistance'),
										transitions={'done': 'WaitForAssistance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:303 y:410
			OperatableStateMachine.add('WaitForAssistance',
										WaitState(wait_time=30),
										transitions={'done': 'AskForAssistance'},
										autonomy={'done': Autonomy.Off})

			# x:580 y:130
			OperatableStateMachine.add('Greetings',
										_sm_greetings_0,
										transitions={'finished': 'IdleBehavior', 'failed': 'AskForAssistance'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
