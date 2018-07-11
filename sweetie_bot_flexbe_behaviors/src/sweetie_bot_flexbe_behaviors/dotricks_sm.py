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
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
from flexbe_states.calculation_state import CalculationState
from behavior_greeting.greeting_sm import GreetingSM
from behavior_cheer.cheer_sm import CheerSM
from behavior_play.play_sm import PlaySM
from behavior_bad.bad_sm import BadSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
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
		self.add_parameter('be_evil', False)
		self.add_parameter('wait_time', 1)

		# references to used behaviors
		self.add_behavior(GreetingSM, 'Greeting')
		self.add_behavior(CheerSM, 'Cheer')
		self.add_behavior(PlaySM, 'Play')
		self.add_behavior(BadSM, 'Bad')

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

		# x:30 y:356, x:644 y:383
		_sm_detectjoyevent_0 = OperatableStateMachine(outcomes=['received', 'unavailable'])

		with _sm_detectjoyevent_0:
			# x:229 y:154
			OperatableStateMachine.add('JoyEvent',
										SubscriberState(topic=joy_topic, blocking=True, clear=False),
										transitions={'received': 'CheckEvent', 'unavailable': 'unavailable'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:229 y:296
			OperatableStateMachine.add('CheckEvent',
										DecisionState(outcomes=['yes', 'no'], conditions=lambda msg: 'yes' if any(msg.buttons) or any(msg.axes[0:5]) else 'no'),
										transitions={'yes': 'received', 'no': 'JoyEvent'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'input_value': 'message'})


		# x:778 y:316, x:75 y:354, x:294 y:358, x:776 y:358, x:777 y:406, x:530 y:356
		_sm_checkjoy_1 = ConcurrencyContainer(outcomes=['no_activity', 'activity_detected', 'failed'], conditions=[
										('no_activity', [('WaitForJoy', 'done')]),
										('failed', [('DetectJoyEvent', 'unavailable')]),
										('activity_detected', [('DetectJoyEvent', 'received')])
										])

		with _sm_checkjoy_1:
			# x:219 y:132
			OperatableStateMachine.add('DetectJoyEvent',
										_sm_detectjoyevent_0,
										transitions={'received': 'activity_detected', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Inherit, 'unavailable': Autonomy.Inherit})

			# x:624 y:152
			OperatableStateMachine.add('WaitForJoy',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'no_activity'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:646 y:31
			OperatableStateMachine.add('RandomChoose',
										DecisionState(outcomes=['greeting', 'play', 'cheer', 'bad'], conditions=self.select_behavior),
										transitions={'greeting': 'Greeting', 'play': 'Play', 'cheer': 'Cheer', 'bad': 'Bad'},
										autonomy={'greeting': Autonomy.Low, 'play': Autonomy.Low, 'cheer': Autonomy.Low, 'bad': Autonomy.Low},
										remapping={'input_value': 'cycle_counter'})

			# x:383 y:460
			OperatableStateMachine.add('CheckJoy',
										_sm_checkjoy_1,
										transitions={'no_activity': 'ResetCounter', 'activity_detected': 'AddCycleCounter', 'failed': 'failed'},
										autonomy={'no_activity': Autonomy.Inherit, 'activity_detected': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:221 y:134
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

			# x:849 y:197
			OperatableStateMachine.add('Cheer',
										self.use_behavior(CheerSM, 'Cheer'),
										transitions={'finished': 'CheckJoy', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:687 y:197
			OperatableStateMachine.add('Play',
										self.use_behavior(PlaySM, 'Play'),
										transitions={'finished': 'CheckJoy', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:991 y:203
			OperatableStateMachine.add('Bad',
										self.use_behavior(BadSM, 'Bad'),
										transitions={'finished': 'CheckJoy', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:115 y:364
			OperatableStateMachine.add('ResetCounter',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'cycle_counter', 'output_value': 'cycle_counter'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
        def select_behavior(self, cycle_counter):
            if cycle_counter == 0:
                return random.choice(['greeting', 'play'])
            elif cycle_counter < 5:
                return random.choice(['greeting', 'play'])
            elif cycle_counter < 10:
                return random.choice(['play', 'bad'])
            else:
                return 'bad'

	# [/MANUAL_FUNC]
