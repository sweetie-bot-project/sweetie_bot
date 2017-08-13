#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_greeting')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.wait_state import WaitState
from flexbe_states.decision_state import DecisionState
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
		storage = 'joint_trajectory/'
		# x:877 y:505, x:887 y:13
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:37 y:189
			OperatableStateMachine.add('MoveStandPose',
										SrdfStateToMoveit(config_name='head_basic', move_group='head', action_topic='move_group', robot_name=''),
										transitions={'reached': 'RandomChoose', 'planning_failed': 'MoveToStandPose2', 'control_failed': 'MoveToStandPose2', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:507 y:53
			OperatableStateMachine.add('IntroduceHerself',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'introduce_herself'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:314 y:336
			OperatableStateMachine.add('SayBlaster',
										TextCommandState(type='voice/play_wav', command='18blaster', topic=voice_topic),
										transitions={'done': 'HoofStamp'},
										autonomy={'done': Autonomy.Off})

			# x:506 y:290
			OperatableStateMachine.add('HoofStamp',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:318 y:135
			OperatableStateMachine.add('SayInitAcquitance',
										TextCommandState(type='voice/play_wav', command='11friendship', topic=voice_topic),
										transitions={'done': 'Greeting'},
										autonomy={'done': Autonomy.Off})

			# x:513 y:136
			OperatableStateMachine.add('Greeting',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'greeting'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:510 y:604
			OperatableStateMachine.add('Rejection',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'begone'),
										transitions={'success': 'HoofStompRejection', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:513 y:210
			OperatableStateMachine.add('HeadSuprised',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'head_suprised'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:320 y:439
			OperatableStateMachine.add('SayControlYour',
										TextCommandState(type='voice/play_wav', command='32rule', topic=voice_topic),
										transitions={'done': 'Wait1'},
										autonomy={'done': Autonomy.Off})

			# x:321 y:211
			OperatableStateMachine.add('SayHello',
										TextCommandState(type='voice/play_wav', command='00irobot', topic=voice_topic),
										transitions={'done': 'HeadSuprised'},
										autonomy={'done': Autonomy.Off})

			# x:314 y:576
			OperatableStateMachine.add('SayLesserBiologicalForm',
										TextCommandState(type='voice/play_wav', command='29attention', topic=voice_topic),
										transitions={'done': 'Rejection'},
										autonomy={'done': Autonomy.Off})

			# x:316 y:53
			OperatableStateMachine.add('SayIRobot',
										TextCommandState(type='voice/play_wav', command='24hello_activate', topic=voice_topic),
										transitions={'done': 'IntroduceHerself'},
										autonomy={'done': Autonomy.Off})

			# x:480 y:409
			OperatableStateMachine.add('Wait1',
										WaitState(wait_time=1),
										transitions={'done': 'PointOnHuman'},
										autonomy={'done': Autonomy.Off})

			# x:743 y:562
			OperatableStateMachine.add('HoofStompRejection',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:593 y:388
			OperatableStateMachine.add('PointOnHuman',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'begone'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:152 y:269
			OperatableStateMachine.add('RandomChoose',
										DecisionState(outcomes=['good1', 'good2', 'good3', 'evil1', 'evil2', 'evil3'], conditions=lambda evil: random.choice(['good1', 'good2', 'good3']) if not evil else random.choice(['evil1','evil2','evil3'])),
										transitions={'good1': 'SayIRobot', 'good2': 'SayInitAcquitance', 'good3': 'SayHello', 'evil1': 'SayBlaster', 'evil2': 'SayControlYour', 'evil3': 'SayLesserBiologicalForm'},
										autonomy={'good1': Autonomy.Low, 'good2': Autonomy.Low, 'good3': Autonomy.Low, 'evil1': Autonomy.Low, 'evil2': Autonomy.Low, 'evil3': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

			# x:40 y:408
			OperatableStateMachine.add('MoveToStandPose2',
										SrdfStateToMoveit(config_name='head_basic', move_group='head', action_topic='move_group', robot_name=''),
										transitions={'reached': 'RandomChoose', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
