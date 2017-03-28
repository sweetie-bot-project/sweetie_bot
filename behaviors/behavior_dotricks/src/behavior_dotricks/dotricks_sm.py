#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_dotricks')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from flexbe_states.wait_state import WaitState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.calculation_state import CalculationState
from behavior_greeting.greeting_sm import GreetingSM
from behavior_cheer.cheer_sm import CheerSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Mar 28 2017
@author: disRecord
'''
class DoTricksSM(Behavior):
	'''
	Basic interaction with people behavior.
	'''


	def __init__(self):
		super(DoTricksSM, self).__init__()
		self.name = 'DoTricks'

		# parameters of this behavior
		self.add_parameter('wait_time', 10)
		self.add_parameter('be_evil', False)

		# references to used behaviors
		self.add_behavior(GreetingSM, 'Greeting')
		self.add_behavior(CheerSM, 'Cheer')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		# x:33 y:423, x:683 y:568
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.cycle_counter = 0
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:778 y:316, x:75 y:354, x:294 y:358, x:776 y:358, x:777 y:406, x:530 y:356
		_sm_checkjoy_0 = ConcurrencyContainer(outcomes=['no_activity', 'activity_detected', 'failed'], conditions=[
										('no_activity', [('WaitForJoy', 'done')]),
										('failed', [('DetectJoystickMsg', 'unavailable')]),
										('activity_detected', [('DetectJoystickMsg', 'received')])
										])

		with _sm_checkjoy_0:
			# x:85 y:162
			OperatableStateMachine.add('WaitForJoy',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'no_activity'},
										autonomy={'done': Autonomy.Off})

			# x:293 y:147
			OperatableStateMachine.add('DetectJoystickMsg',
										SubscriberState(topic=joy_topic, blocking=True, clear=True),
										transitions={'received': 'activity_detected', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})



		with _state_machine:
			# x:251 y:65
			OperatableStateMachine.add('RandomChoose',
										DecisionState(outcomes=['greeting', 'play', 'cheer', 'bad'], conditions=lambda x: 'greeting'),
										transitions={'greeting': 'Greeting', 'play': 'Cheer', 'cheer': 'Cheer', 'bad': 'Cheer'},
										autonomy={'greeting': Autonomy.Off, 'play': Autonomy.Off, 'cheer': Autonomy.Off, 'bad': Autonomy.Off},
										remapping={'input_value': 'cycle_counter'})

			# x:259 y:387
			OperatableStateMachine.add('CheckJoy',
										_sm_checkjoy_0,
										transitions={'no_activity': 'finished', 'activity_detected': 'AddCycleCounter', 'failed': 'failed'},
										autonomy={'no_activity': Autonomy.Inherit, 'activity_detected': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:256 y:225
			OperatableStateMachine.add('AddCycleCounter',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'RandomChoose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'cycle_counter', 'output_value': 'cycle_counter'})

			# x:483 y:171
			OperatableStateMachine.add('Greeting',
										self.use_behavior(GreetingSM, 'Greeting'),
										transitions={'finished': 'CheckJoy', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:763 y:165
			OperatableStateMachine.add('Cheer',
										self.use_behavior(CheerSM, 'Cheer'),
										transitions={'finished': 'CheckJoy', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
