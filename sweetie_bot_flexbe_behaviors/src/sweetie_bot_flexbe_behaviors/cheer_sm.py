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
from sweetie_bot_flexbe_states.execute_stored_trajectory_state import ExecuteStoredJointTrajectoryState
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.compound_action_param import CompoundActionParam
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Tue Mar 28 2017
@author: disRecord
'''
class CheerSM(Behavior):
	'''
	Sweetie cheering state.
	'''


	def __init__(self):
		super(CheerSM, self).__init__()
		self.name = 'Cheer'

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
		# x:901 y:554, x:895 y:38
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:48 y:105
			OperatableStateMachine.add('SetHeadNominalPose',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'CheckEvil', 'failed': 'failed', 'timeout': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:579 y:498
			OperatableStateMachine.add('HoofStamp',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:88 y:447
			OperatableStateMachine.add('RandomChoice',
										DecisionState(outcomes=['evil1','evil2'], conditions=lambda x: random.choice(['evil1','evil2'])),
										transitions={'evil1': 'PointDoNotTouch', 'evil2': 'SayWalk'},
										autonomy={'evil1': Autonomy.High, 'evil2': Autonomy.High},
										remapping={'input_value': 'be_evil'})

			# x:303 y:543
			OperatableStateMachine.add('SayWalk',
										TextCommandState(type='voice/play_wav', command='i_hate_laws', topic=voice_topic),
										transitions={'done': 'Wait1'},
										autonomy={'done': Autonomy.Off})

			# x:467 y:539
			OperatableStateMachine.add('Wait1',
										WaitState(wait_time=0.5),
										transitions={'done': 'HoofStamp'},
										autonomy={'done': Autonomy.Off})

			# x:497 y:102
			OperatableStateMachine.add('PranceMaxFun',
										CompoundActionParam(action_param='prance_maximum_fun', action_ns='saved_msgs/compound_action'),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:437 y:275
			OperatableStateMachine.add('PointDoNotTouch',
										CompoundActionParam(action_param='point_do_not_touch', action_ns='saved_msgs/compound_action'),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:140 y:251
			OperatableStateMachine.add('CheckEvil',
										DecisionState(outcomes=['good', 'evil'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'good': 'PranceMaxFun', 'evil': 'RandomChoice'},
										autonomy={'good': Autonomy.Off, 'evil': Autonomy.Off},
										remapping={'input_value': 'be_evil'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
