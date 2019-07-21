#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_behaviors.play_sm import PlaySM
from sweetie_bot_flexbe_behaviors.cheer_sm import CheerSM
from sweetie_bot_flexbe_behaviors.bad_sm import BadSM
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_behaviors.greeting_sm import GreetingSM
from flexbe_states.calculation_state import CalculationState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from sweetie_bot_flexbe_states.set_bool_state import SetBoolState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from sweetie_bot_joystick.msg import KeyPressed
# [/MANUAL_IMPORT]


'''
Created on Sat Jul 20 2019
@author: disRecord
'''
class JoyAnimationSM(Behavior):
	'''
	Select animation using joystick. Keys '1','2','3','4' activate Greet,ing, Play, Cheer and Bad behaviors. Between them.
	'''


	def __init__(self):
		super(JoyAnimationSM, self).__init__()
		self.name = 'JoyAnimation'

		# parameters of this behavior
		self.add_parameter('be_evil', False)
		self.add_parameter('timeout', 10)

		# references to used behaviors
		self.add_behavior(PlaySM, 'Play')
		self.add_behavior(CheerSM, 'Cheer')
		self.add_behavior(BadSM, 'Bad')
		self.add_behavior(GreetingSM, 'Greeting')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1037 y:16, x:1042 y:578, x:54 y:340
		_state_machine = OperatableStateMachine(outcomes=['timeout', 'failed', 'unknown_keys'], input_keys=['key_pressed_msg', 'be_evil'], output_keys=['key_pressed_msg'])
		_state_machine.userdata.key_pressed_msg = KeyPressed()
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:729 y:150
		_sm_toggleevilmode_0 = OperatableStateMachine(outcomes=['done'], input_keys=['be_evil'], output_keys=['be_evil'])

		with _sm_toggleevilmode_0:
			# x:89 y:89
			OperatableStateMachine.add('ToggleBeEvil',
										CalculationState(calculation=lambda x: not x),
										transitions={'done': 'ChoiceEvil'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'be_evil', 'output_value': 'be_evil'})

			# x:298 y:139
			OperatableStateMachine.add('ChoiceEvil',
										DecisionState(outcomes=['good','evil'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'good': 'NormalEyes', 'evil': 'RedEyes'},
										autonomy={'good': Autonomy.Off, 'evil': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:484 y:45
			OperatableStateMachine.add('RedEyes',
										TextCommandState(type='eyes/emotion', command='red_eyes', topic='control'),
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Off})

			# x:474 y:231
			OperatableStateMachine.add('NormalEyes',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:353, x:564 y:266, x:699 y:354, x:667 y:416, x:334 y:292, x:206 y:300
		_sm_waitkeypressed_1 = ConcurrencyContainer(outcomes=['key_pressed', 'timeout', 'failed'], input_keys=['key_pressed_msg'], output_keys=['key_pressed_msg'], conditions=[
										('key_pressed', [('WaitKey', 'received')]),
										('failed', [('WaitKey', 'unavailable')]),
										('timeout', [('WaitTimeout', 'done')])
										])

		with _sm_waitkeypressed_1:
			# x:106 y:121
			OperatableStateMachine.add('WaitKey',
										WaitForMessageState(topic='/hmi/joy_decoder/keys_pressed', condition=self.trigger, buffered=False, clear=True),
										transitions={'received': 'key_pressed', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'key_pressed_msg'})

			# x:502 y:101
			OperatableStateMachine.add('WaitTimeout',
										WaitState(wait_time=self.timeout),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:182 y:141
			OperatableStateMachine.add('ProcessKeyPressed',
										DecisionState(outcomes=['greet','play','cheer','bad','switch_evil','unknown','walk','start'], conditions=self.decision),
										transitions={'greet': 'Greeting', 'play': 'Play', 'cheer': 'Cheer', 'bad': 'Bad', 'switch_evil': 'ToggleEvilMode', 'unknown': 'WaitKeyPressed', 'walk': 'unknown_keys', 'start': 'unknown_keys'},
										autonomy={'greet': Autonomy.Off, 'play': Autonomy.Off, 'cheer': Autonomy.Off, 'bad': Autonomy.Off, 'switch_evil': Autonomy.Off, 'unknown': Autonomy.Off, 'walk': Autonomy.Off, 'start': Autonomy.Off},
										remapping={'input_value': 'key_pressed_msg'})

			# x:500 y:257
			OperatableStateMachine.add('Play',
										self.use_behavior(PlaySM, 'Play'),
										transitions={'finished': 'EnableJoystick', 'failed': 'EnableJoystick2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:502 y:377
			OperatableStateMachine.add('Cheer',
										self.use_behavior(CheerSM, 'Cheer'),
										transitions={'finished': 'EnableJoystick', 'failed': 'EnableJoystick2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:504 y:500
			OperatableStateMachine.add('Bad',
										self.use_behavior(BadSM, 'Bad'),
										transitions={'finished': 'EnableJoystick', 'failed': 'EnableJoystick2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:848 y:85
			OperatableStateMachine.add('WaitKeyPressed',
										_sm_waitkeypressed_1,
										transitions={'key_pressed': 'ProcessKeyPressed1', 'timeout': 'timeout', 'failed': 'failed'},
										autonomy={'key_pressed': Autonomy.Inherit, 'timeout': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'key_pressed_msg': 'key_pressed_msg'})

			# x:498 y:164
			OperatableStateMachine.add('Greeting',
										self.use_behavior(GreetingSM, 'Greeting'),
										transitions={'finished': 'EnableJoystick', 'failed': 'EnableJoystick2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:201 y:276
			OperatableStateMachine.add('ToggleEvilMode',
										_sm_toggleevilmode_0,
										transitions={'done': 'WaitKeyPressed'},
										autonomy={'done': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:524 y:14
			OperatableStateMachine.add('ProcessKeyPressed1',
										DecisionState(outcomes=['known','unknown'], conditions=lambda x: 'unknown' if self.decision(x) == 'unknown' else 'known'),
										transitions={'known': 'DisableJoystick', 'unknown': 'WaitKeyPressed'},
										autonomy={'known': Autonomy.Off, 'unknown': Autonomy.Off},
										remapping={'input_value': 'key_pressed_msg'})

			# x:328 y:22
			OperatableStateMachine.add('DisableJoystick',
										SetBoolState(service='motion/controller/joint_state/set_operational', value=False),
										transitions={'true': 'ProcessKeyPressed', 'false': 'ProcessKeyPressed', 'failure': 'failed'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'success': 'success', 'message': 'message'})

			# x:840 y:250
			OperatableStateMachine.add('EnableJoystick',
										SetBoolState(service='motion/controller/joint_state/set_operational', value=True),
										transitions={'true': 'WaitKeyPressed', 'false': 'WaitKeyPressed', 'failure': 'failed'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'success': 'success', 'message': 'message'})

			# x:838 y:462
			OperatableStateMachine.add('EnableJoystick2',
										SetBoolState(service='motion/controller/joint_state/set_operational', value=True),
										transitions={'true': 'failed', 'false': 'failed', 'failure': 'failed'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'success': 'success', 'message': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
        def trigger(self, key_msg):
            if not key_msg:
                return False
            keys = set(key_msg.keys)
            return keys.intersection(('1', '2', '3', '4','start', 'select','left', 'right', 'up', 'down','tumble'))

        def decision(self, key_msg):
            key_mapping = { 
                    '1': 'greet',
                    '2': 'play',
                    '3': 'cheer',
                    '4': 'bad',
                    'select': 'switch_evil',
                    'start': 'start'
                }
            keys = set(key_msg.keys).intersection( key_mapping.keys() )
            if keys:
                return key_mapping[keys.pop()];
            if set(key_msg.keys).intersection(('left', 'right', 'up', 'down')):
                return 'walk'
            return 'unknown'
	
	# [/MANUAL_FUNC]
