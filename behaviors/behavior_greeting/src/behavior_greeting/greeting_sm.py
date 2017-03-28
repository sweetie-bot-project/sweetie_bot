#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_greeting')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Tue Mar 28 2017
@author: disRecord
'''
class GreetingSM(Behavior):
	'''
	SweetieBot greeting behavior.
	'''


	def __init__(self):
		super(GreetingSM, self).__init__()
		self.name = 'Greeting'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		voice_topic = 'voice/voice'
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		storage = '/hmi/trajectory_storage/'
		# x:877 y:505, x:887 y:13
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:133 y:35
			OperatableStateMachine.add('RandomChoose',
										DecisionState(outcomes=['good1', 'good2', 'evil1', 'evil2'], conditions=lambda evil: random.choice(['good1', 'good2']) if not evil else random.choice(['evil1','evil2'])),
										transitions={'good1': 'SayIRobot', 'good2': 'SayIRobot', 'evil1': 'SayBlaster', 'evil2': 'SayBlaster'},
										autonomy={'good1': Autonomy.Off, 'good2': Autonomy.Off, 'evil1': Autonomy.Off, 'evil2': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:316 y:53
			OperatableStateMachine.add('SayIRobot',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='00irobot'),
										transitions={'done': 'IntroduceHerself', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:507 y:53
			OperatableStateMachine.add('IntroduceHerself',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'introduce_herself'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:313 y:420
			OperatableStateMachine.add('SayBlaster',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='18blaster'),
										transitions={'done': 'HoofStamp', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:513 y:417
			OperatableStateMachine.add('HoofStamp',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
