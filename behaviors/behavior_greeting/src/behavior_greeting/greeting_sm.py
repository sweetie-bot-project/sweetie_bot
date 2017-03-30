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
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
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
		# x:877 y:505, x:887 y:13
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:133 y:35
			OperatableStateMachine.add('RandomChoose',
										DecisionState(outcomes=['good1', 'good2', 'good3', 'evil1', 'evil2', 'evil3'], conditions=lambda evil: random.choice(['good1', 'good2', 'good3']) if not evil else random.choice(['evil1','evil2','evil3'])),
										transitions={'good1': 'SayIRobot', 'good2': 'SayInitAcquitance', 'good3': 'SayHello', 'evil1': 'SayBlaster', 'evil2': 'SayControlYour', 'evil3': 'SayLesserBiologicalForm'},
										autonomy={'good1': Autonomy.Low, 'good2': Autonomy.Low, 'good3': Autonomy.Low, 'evil1': Autonomy.Low, 'evil2': Autonomy.Low, 'evil3': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

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

			# x:318 y:135
			OperatableStateMachine.add('SayInitAcquitance',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='11friendship'),
										transitions={'done': 'Greeting', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:513 y:136
			OperatableStateMachine.add('Greeting',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'greeting'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:515 y:506
			OperatableStateMachine.add('Rejection',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'begone'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:514 y:212
			OperatableStateMachine.add('HeadSuprised',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'head_suprised'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:312 y:500
			OperatableStateMachine.add('SayControlYour',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='14equestria'),
										transitions={'done': 'Rejection', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:321 y:211
			OperatableStateMachine.add('SayHello',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='00irobot'),
										transitions={'done': 'HeadSuprised', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:314 y:357
			OperatableStateMachine.add('SayLesserBiologicalForm',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='12ideal'),
										transitions={'done': 'Rejection', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:316 y:53
			OperatableStateMachine.add('SayIRobot',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='00irobot'),
										transitions={'done': 'IntroduceHerself', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
