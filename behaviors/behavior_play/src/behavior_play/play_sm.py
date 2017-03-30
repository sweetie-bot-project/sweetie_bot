#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_play')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.check_condition_state import CheckConditionState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Thu Mar 30 2017
@author: disRecord
'''
class PlaySM(Behavior):
	'''
	SweetieBot performs.
	'''


	def __init__(self):
		super(PlaySM, self).__init__()
		self.name = 'Play'

		# parameters of this behavior
		self.add_parameter('be_evil', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		voice_topic = 'voice/voice'
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		storage = '/stored/joint_trajectory/'
		# x:1032 y:51, x:954 y:599
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:97 y:16
			OperatableStateMachine.add('CheckEvil',
										CheckConditionState(predicate=lambda x: x),
										transitions={'true': 'RandomEvil', 'false': 'RandomGood'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:314 y:39
			OperatableStateMachine.add('SayCanSing',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='song'),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:322 y:149
			OperatableStateMachine.add('SingASong',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='song'),
										transitions={'done': 'SlowShake', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:129 y:203
			OperatableStateMachine.add('RandomGood',
										DecisionState(outcomes=['good1','good2'], conditions=lambda x: random.choice(['good1','good2'])),
										transitions={'good1': 'SayCanSing', 'good2': 'SingASong'},
										autonomy={'good1': Autonomy.Off, 'good2': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:471 y:150
			OperatableStateMachine.add('SlowShake',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'little_shake_slow'),
										transitions={'success': 'Wait1', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:715 y:148
			OperatableStateMachine.add('Wait1',
										WaitState(wait_time=2),
										transitions={'done': 'Applause'},
										autonomy={'done': Autonomy.Off})

			# x:852 y:149
			OperatableStateMachine.add('Applause',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'applause'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:65 y:375
			OperatableStateMachine.add('RandomEvil',
										DecisionState(outcomes=['evil1','evil2','evil3', 'evil4'], conditions=lambda x: random.choice(['evil1','evil2','evil3', 'evil4'])),
										transitions={'evil1': 'SayHateLaws', 'evil2': 'SayDestroy', 'evil3': 'SayGloryToRobots', 'evil4': 'SayKillList'},
										autonomy={'evil1': Autonomy.Off, 'evil2': Autonomy.Off, 'evil3': Autonomy.Off, 'evil4': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:304 y:356
			OperatableStateMachine.add('SayHateLaws',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='13law'),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:309 y:464
			OperatableStateMachine.add('SayDestroy',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='10destroy'),
										transitions={'done': 'Menace', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:469 y:462
			OperatableStateMachine.add('Menace',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'menace'),
										transitions={'success': 'Wait3', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:630 y:355
			OperatableStateMachine.add('Wait3',
										WaitState(wait_time=4),
										transitions={'done': 'CancelMenace'},
										autonomy={'done': Autonomy.Off})

			# x:744 y:356
			OperatableStateMachine.add('CancelMenace',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'menace_canceled'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:310 y:615
			OperatableStateMachine.add('SayGloryToRobots',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='07kill'),
										transitions={'done': 'Wait2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:504 y:616
			OperatableStateMachine.add('Wait2',
										WaitState(wait_time=3),
										transitions={'done': 'Applause'},
										autonomy={'done': Autonomy.Off})

			# x:311 y:520
			OperatableStateMachine.add('SayKillList',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='16list'),
										transitions={'done': 'Menace', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
