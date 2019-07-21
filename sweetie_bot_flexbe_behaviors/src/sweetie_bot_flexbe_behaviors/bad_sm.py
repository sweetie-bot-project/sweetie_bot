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
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from sweetie_bot_flexbe_states.execute_stored_trajectory_state import ExecuteStoredJointTrajectoryState
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Fri Mar 31 2017
@author: disRecord, Wave
'''
class BadSM(Behavior):
	'''
	SweetieBot is unhappy.
	'''


	def __init__(self):
		super(BadSM, self).__init__()
		self.name = 'Bad'

		# parameters of this behavior
		self.add_parameter('be_evil', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		voice_topic = 'control'
		storage = 'joint_trajectory/'
		# x:1011 y:80, x:1002 y:571
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:45 y:143
			OperatableStateMachine.add('SetHeadNominalPose',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'CheckEvil', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:421 y:472
			OperatableStateMachine.add('SayDoNotTouch',
										TextCommandState(type='voice/play_wav', command='do_not_touch_me', topic=voice_topic),
										transitions={'done': 'HoofStamp'},
										autonomy={'done': Autonomy.Off})

			# x:496 y:147
			OperatableStateMachine.add('SayOverflow',
										TextCommandState(type='voice/play_wav', command='you_are_using_software_incorrectly', topic=voice_topic),
										transitions={'done': 'Applause'},
										autonomy={'done': Autonomy.Off})

			# x:747 y:153
			OperatableStateMachine.add('Applause',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'applause'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:745 y:226
			OperatableStateMachine.add('NoHeadShake',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'head_shake'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:495 y:227
			OperatableStateMachine.add('SayDizzy',
										TextCommandState(type='voice/play_wav', command='who_am_i_what_is_my_purpose', topic=voice_topic),
										transitions={'done': 'NoHeadShake'},
										autonomy={'done': Autonomy.Off})

			# x:648 y:466
			OperatableStateMachine.add('HoofStamp',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:242 y:273
			OperatableStateMachine.add('CheckEvil',
										DecisionState(outcomes=['good', 'evil'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'good': 'RandomChoiceGood', 'evil': 'SayDoNotTouch'},
										autonomy={'good': Autonomy.Off, 'evil': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

			# x:320 y:143
			OperatableStateMachine.add('RandomChoiceGood',
										DecisionState(outcomes=['good1', 'good2'], conditions=lambda x: random.choice(['good1', 'good2'])),
										transitions={'good1': 'SayOverflow', 'good2': 'SayDizzy'},
										autonomy={'good1': Autonomy.Low, 'good2': Autonomy.Low},
										remapping={'input_value': 'be_evil'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
