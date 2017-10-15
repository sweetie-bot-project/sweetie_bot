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
		# x:1044 y:507, x:1035 y:15
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

			# x:321 y:368
			OperatableStateMachine.add('SayPrepareToDie',
										TextCommandState(type='voice/play_wav', command='prepare_to_die', topic=voice_topic),
										transitions={'done': 'Menace'},
										autonomy={'done': Autonomy.Off})

			# x:492 y:368
			OperatableStateMachine.add('Menace',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'menace'),
										transitions={'success': 'Wait3', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:318 y:135
			OperatableStateMachine.add('SayInitAcquitance',
										TextCommandState(type='voice/play_wav', command='acquitance_procedure', topic=voice_topic),
										transitions={'done': 'BrohoofBegin'},
										autonomy={'done': Autonomy.Off})

			# x:471 y:130
			OperatableStateMachine.add('BrohoofBegin',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'brohoof_begin'),
										transitions={'success': 'Wait2', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:500 y:629
			OperatableStateMachine.add('Rejection',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'begone'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:504 y:214
			OperatableStateMachine.add('HeadSuprised',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'head_suprised'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:321 y:536
			OperatableStateMachine.add('SayControlYour',
										TextCommandState(type='voice/play_wav', command='someday_ill_control_you', topic=voice_topic),
										transitions={'done': 'Wait1'},
										autonomy={'done': Autonomy.Off})

			# x:320 y:218
			OperatableStateMachine.add('SayQuestion',
										TextCommandState(type='voice/play_wav', command='are_you_too_collecting_points', topic=voice_topic),
										transitions={'done': 'HeadSuprised'},
										autonomy={'done': Autonomy.Off})

			# x:293 y:622
			OperatableStateMachine.add('SayDoYouWantMyAttention',
										TextCommandState(type='voice/play_wav', command='do_you_want_my_attention', topic=voice_topic),
										transitions={'done': 'Rejection'},
										autonomy={'done': Autonomy.Off})

			# x:317 y:48
			OperatableStateMachine.add('SayImSweetieBot',
										TextCommandState(type='voice/play_wav', command='hello_im_sweetie_bot_friedship_programms', topic=voice_topic),
										transitions={'done': 'IntroduceHerself'},
										autonomy={'done': Autonomy.Off})

			# x:476 y:550
			OperatableStateMachine.add('Wait1',
										WaitState(wait_time=0.3),
										transitions={'done': 'PointOnHuman'},
										autonomy={'done': Autonomy.Off})

			# x:750 y:618
			OperatableStateMachine.add('HoofStompRejection',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:591 y:544
			OperatableStateMachine.add('PointOnHuman',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'begone'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:152 y:269
			OperatableStateMachine.add('RandomChoose',
										DecisionState(outcomes=['good1', 'good2', 'good3', 'good4', 'evil1', 'evil2', 'evil3'], conditions=lambda evil: random.choice(['good1', 'good2', 'good3', 'good4']) if not evil else random.choice(['evil1','evil2','evil3'])),
										transitions={'good1': 'SayImSweetieBot', 'good2': 'SayInitAcquitance', 'good3': 'SayQuestion', 'good4': 'SayMeetUp', 'evil1': 'SayPrepareToDie', 'evil2': 'SayControlYour', 'evil3': 'SayDoYouWantMyAttention'},
										autonomy={'good1': Autonomy.Low, 'good2': Autonomy.Low, 'good3': Autonomy.Low, 'good4': Autonomy.Low, 'evil1': Autonomy.Low, 'evil2': Autonomy.Low, 'evil3': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

			# x:40 y:408
			OperatableStateMachine.add('MoveToStandPose2',
										SrdfStateToMoveit(config_name='head_basic', move_group='head', action_topic='move_group', robot_name=''),
										transitions={'reached': 'RandomChoose', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:690 y:126
			OperatableStateMachine.add('Wait2',
										WaitState(wait_time=3),
										transitions={'done': 'BrohoofEnd'},
										autonomy={'done': Autonomy.Off})

			# x:792 y:131
			OperatableStateMachine.add('BrohoofEnd',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'brohoof_end'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:324 y:277
			OperatableStateMachine.add('SayMeetUp',
										TextCommandState(type='voice/play_wav', command='do_you_want_meet_up', topic=voice_topic),
										transitions={'done': 'Greeting'},
										autonomy={'done': Autonomy.Off})

			# x:502 y:280
			OperatableStateMachine.add('Greeting',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'greeting'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:733 y:364
			OperatableStateMachine.add('Wait3',
										WaitState(wait_time=1.5),
										transitions={'done': 'SayError'},
										autonomy={'done': Autonomy.Off})

			# x:452 y:441
			OperatableStateMachine.add('SayError',
										TextCommandState(type='voice/play_wav', command='error_blasters_are_not_found', topic=voice_topic),
										transitions={'done': 'Wait4'},
										autonomy={'done': Autonomy.Off})

			# x:661 y:446
			OperatableStateMachine.add('Wait4',
										WaitState(wait_time=2),
										transitions={'done': 'MenaceCanceled'},
										autonomy={'done': Autonomy.Off})

			# x:763 y:440
			OperatableStateMachine.add('MenaceCanceled',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'menace_canceled'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
