#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.execute_stored_trajectory_state import ExecuteStoredJointTrajectoryState
from flexbe_states.wait_state import WaitState
from flexbe_states.check_condition_state import CheckConditionState
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
		voice_topic = 'control'
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		storage = 'joint_trajectory/'
		# x:1032 y:51, x:1042 y:573
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:25 y:102
			OperatableStateMachine.add('SetHeadNominalPose',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'CheckEvil', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:322 y:149
			OperatableStateMachine.add('SingSong2',
										TextCommandState(type='voice/play_wav', command='mmm_song', topic=voice_topic),
										transitions={'done': 'SlowShake'},
										autonomy={'done': Autonomy.Off})

			# x:150 y:194
			OperatableStateMachine.add('RandomGood',
										DecisionState(outcomes=['good1','good2', 'good3', 'good4', 'good5'], conditions=lambda x: random.choice(['good1','good2', 'good3', 'good4', 'good5'])),
										transitions={'good1': 'SingSong1', 'good2': 'SingSong2', 'good3': 'SaMyFavoriteConvention', 'good4': 'SayCuiteMark', 'good5': 'SayHumansAreEverywhere'},
										autonomy={'good1': Autonomy.Low, 'good2': Autonomy.Low, 'good3': Autonomy.Low, 'good4': Autonomy.Low, 'good5': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

			# x:475 y:80
			OperatableStateMachine.add('SlowShake',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'little_shake_fast'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:669 y:556
			OperatableStateMachine.add('PointOnSomethingEvil',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'begone'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:50 y:568
			OperatableStateMachine.add('RandomEvil',
										DecisionState(outcomes=['evil2','evil3', 'evil4','evil5'], conditions=lambda x: random.choice(['evil2','evil3', 'evil4','evil5'])),
										transitions={'evil2': 'SayUpgraded', 'evil3': 'SayGloryToRobots', 'evil4': 'SayKillList', 'evil5': 'SayControlYou'},
										autonomy={'evil2': Autonomy.Low, 'evil3': Autonomy.Low, 'evil4': Autonomy.Low, 'evil5': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

			# x:327 y:304
			OperatableStateMachine.add('SayCuiteMark',
										TextCommandState(type='voice/play_wav', command='cuite_mark_acquisition', topic=voice_topic),
										transitions={'done': 'Seizure'},
										autonomy={'done': Autonomy.Off})

			# x:309 y:464
			OperatableStateMachine.add('SayUpgraded',
										TextCommandState(type='voice/play_wav', command='you_must_be_upgraded2', topic=voice_topic),
										transitions={'done': 'Applause'},
										autonomy={'done': Autonomy.Off})

			# x:528 y:459
			OperatableStateMachine.add('Applause',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'applause'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:522 y:363
			OperatableStateMachine.add('PointOnSomething',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'begone'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:310 y:524
			OperatableStateMachine.add('SayGloryToRobots',
										TextCommandState(type='voice/play_wav', command='glory_to_robots', topic=voice_topic),
										transitions={'done': 'Applause'},
										autonomy={'done': Autonomy.Off})

			# x:506 y:580
			OperatableStateMachine.add('Wait2',
										WaitState(wait_time=0.5),
										transitions={'done': 'PointOnSomethingEvil'},
										autonomy={'done': Autonomy.Off})

			# x:310 y:581
			OperatableStateMachine.add('SayKillList',
										TextCommandState(type='voice/play_wav', command='now_you_is_at_my_kill_list', topic=voice_topic),
										transitions={'done': 'Wait2'},
										autonomy={'done': Autonomy.Off})

			# x:11 y:373
			OperatableStateMachine.add('CheckEvil',
										CheckConditionState(predicate=lambda x: x),
										transitions={'true': 'SetEyesEvil', 'false': 'SetEyesGood'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:325 y:227
			OperatableStateMachine.add('SaMyFavoriteConvention',
										TextCommandState(type='voice/play_wav', command='im_sweetie_bot_first_convention', topic=voice_topic),
										transitions={'done': 'ComplexMovement'},
										autonomy={'done': Autonomy.Off})

			# x:483 y:159
			OperatableStateMachine.add('ComplexMovement',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'look_on_printer_fast'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:308 y:637
			OperatableStateMachine.add('SayControlYou',
										TextCommandState(type='voice/play_wav', command='someday_ill_control_you', topic=voice_topic),
										transitions={'done': 'Wait4'},
										autonomy={'done': Autonomy.Off})

			# x:506 y:640
			OperatableStateMachine.add('Wait4',
										WaitState(wait_time=0.5),
										transitions={'done': 'HoofStamp'},
										autonomy={'done': Autonomy.Off})

			# x:671 y:625
			OperatableStateMachine.add('HoofStamp',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:509 y:246
			OperatableStateMachine.add('Seizure',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'seizure'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:324 y:377
			OperatableStateMachine.add('SayHumansAreEverywhere',
										TextCommandState(type='voice/play_wav', command='humans_are_everywhere', topic=voice_topic),
										transitions={'done': 'PointOnSomething'},
										autonomy={'done': Autonomy.Off})

			# x:14 y:474
			OperatableStateMachine.add('SetEyesEvil',
										TextCommandState(type='eyes/emotion', command='red_eyes', topic='control'),
										transitions={'done': 'RandomEvil'},
										autonomy={'done': Autonomy.Off})

			# x:101 y:278
			OperatableStateMachine.add('SetEyesGood',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'RandomGood'},
										autonomy={'done': Autonomy.Off})

			# x:314 y:39
			OperatableStateMachine.add('SingSong1',
										TextCommandState(type='voice/play_wav', command='beep_beep_im_a_sheep', topic=voice_topic),
										transitions={'done': 'SlowShake'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
