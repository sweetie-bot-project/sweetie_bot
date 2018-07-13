#!/usr/bin/env python
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
from flexbe_states.calculation_state import CalculationState
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 17 2017
@author: disRecord
'''
class SwitchEvilModeSM(Behavior):
	'''
	Enter and exit evil mode animation.
	'''


	def __init__(self):
		super(SwitchEvilModeSM, self).__init__()
		self.name = 'SwitchEvilMode'

		# parameters of this behavior
		self.add_parameter('be_evil', False)
		self.add_parameter('new_be_evil', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		storage = 'joint_trajectory/'
		voice_topic = 'voice/voice'
		eye_cmd_topic = 'control'
		# x:1034 y:292, x:622 y:639
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil', 'new_be_evil'], output_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil
		_state_machine.userdata.new_be_evil = self.new_be_evil
		_state_machine.userdata.head_crooked_pose = [0,0,0,0.4]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:96 y:163
			OperatableStateMachine.add('CheckEvil',
										DecisionState(outcomes=['good', 'evil'], conditions=lambda x: 'good' if not x else 'evil'),
										transitions={'good': 'CheckChangeToEvil', 'evil': 'CheckChangeToGood'},
										autonomy={'good': Autonomy.Off, 'evil': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:252 y:50
			OperatableStateMachine.add('CheckChangeToEvil',
										DecisionState(outcomes=['yes','no'], conditions=lambda x: 'yes' if x else 'no'),
										transitions={'yes': 'RedEyes', 'no': 'NormalEyes'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'input_value': 'new_be_evil'})

			# x:426 y:48
			OperatableStateMachine.add('RedEyes',
										TextCommandState(type='eyes/emotion', command='red_eyes', topic=eye_cmd_topic),
										transitions={'done': 'Seizure'},
										autonomy={'done': Autonomy.Off})

			# x:737 y:30
			OperatableStateMachine.add('Seizure',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'seizure_evil'),
										transitions={'success': 'SetEvil', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:961 y:162
			OperatableStateMachine.add('SetEvil',
										CalculationState(calculation=lambda x: True),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'be_evil', 'output_value': 'be_evil'})

			# x:419 y:135
			OperatableStateMachine.add('NormalEyes',
										TextCommandState(type='eyes/emotion', command='normal', topic=eye_cmd_topic),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:195 y:281
			OperatableStateMachine.add('CheckChangeToGood',
										DecisionState(outcomes=['yes','no'], conditions=lambda x: 'no' if x  else 'yes'),
										transitions={'yes': 'NormalEyes2', 'no': 'RedEyes2'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'input_value': 'new_be_evil'})

			# x:416 y:331
			OperatableStateMachine.add('NormalEyes2',
										TextCommandState(type='eyes/emotion', command='normal', topic=eye_cmd_topic),
										transitions={'done': 'HeadAssumeBasicPose'},
										autonomy={'done': Autonomy.Off})

			# x:613 y:321
			OperatableStateMachine.add('ShakeHead',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'head_shake'),
										transitions={'success': 'LookAround', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:894 y:326
			OperatableStateMachine.add('SetGood',
										CalculationState(calculation=lambda x: False),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'be_evil', 'output_value': 'be_evil'})

			# x:671 y:437
			OperatableStateMachine.add('LookAround',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'look_around'),
										transitions={'success': 'SetGood', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:420 y:200
			OperatableStateMachine.add('RedEyes2',
										TextCommandState(type='eyes/emotion', command='red_eyes', topic=eye_cmd_topic),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:444 y:420
			OperatableStateMachine.add('HeadAssumeBasicPose',
										SrdfStateToMoveit(config_name='head_basic', move_group='head', action_topic='move_group', robot_name=''),
										transitions={'reached': 'ShakeHead', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
