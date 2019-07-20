#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from flexbe_states.decision_state import DecisionState
from flexbe_states.calculation_state import CalculationState
from sweetie_bot_flexbe_states.execute_step_sequence_key import ExecuteStepSequenceKey
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 19 2019
@author: disRecord
'''
class TextCmdWalkSM(Behavior):
	'''
	Interperetator of 'motion/step_sequence' commands. Behavior executes commands received on topic '/control' or via input key. If timeout happens or unknown command is received exits immediately.
	'''


	def __init__(self):
		super(TextCmdWalkSM, self).__init__()
		self.name = 'TextCmdWalk'

		# parameters of this behavior
		self.add_parameter('timeout', 10)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		control_topic = '/control'
		# x:902 y:204, x:178 y:606, x:677 y:537
		_state_machine = OperatableStateMachine(outcomes=['failed', 'finished', 'invalid_pose'], input_keys=['text_command'], output_keys=['text_command'])
		_state_machine.userdata.text_command = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:353, x:130 y:353, x:230 y:353, x:330 y:353, x:430 y:353, x:530 y:353
		_sm_waittextcommand_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'timeout'], output_keys=['text_command'], conditions=[
										('finished', [('WaitCommandSub', 'received')]),
										('failed', [('WaitCommandSub', 'unavailable')]),
										('timeout', [('WaitTimeout', 'done')])
										])

		with _sm_waittextcommand_0:
			# x:142 y:113
			OperatableStateMachine.add('WaitCommandSub',
										SubscriberState(topic=control_topic, blocking=True, clear=True),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'text_command'})

			# x:360 y:113
			OperatableStateMachine.add('WaitTimeout',
										WaitState(wait_time=self.timeout),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:52 y:170
			OperatableStateMachine.add('PrepareToWalk',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_begin', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'CheckIfEmpty', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:377 y:240
			OperatableStateMachine.add('CheckIfStepSequence',
										DecisionState(outcomes=['yes', 'no'], conditions=lambda msg: 'yes' if msg.type=='motion/step_sequence' else 'no'),
										transitions={'yes': 'ExtractCmd', 'no': 'ClearTextCommandKey'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'input_value': 'text_command'})

			# x:100 y:358
			OperatableStateMachine.add('StandUp',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_end', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'finished', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:497 y:359
			OperatableStateMachine.add('ExtractCmd',
										CalculationState(calculation=lambda msg: msg.command),
										transitions={'done': 'ExecuteStepSeq'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'text_command', 'output_value': 'command_field'})

			# x:679 y:304
			OperatableStateMachine.add('ExecuteStepSeq',
										ExecuteStepSequenceKey(controller='motion/controller/step_sequence', trajectory_ns='saved_msgs/step_sequence'),
										transitions={'success': 'WaitTextCommand', 'unavailable': 'WaitTextCommand', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'unavailable': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'trajectory_param': 'command_field'})

			# x:224 y:61
			OperatableStateMachine.add('CheckIfEmpty',
										DecisionState(outcomes=['yes','no'], conditions=lambda x: 'yes' if x==None else 'no'),
										transitions={'yes': 'WaitTextCommand', 'no': 'CheckIfStepSequence'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'input_value': 'text_command'})

			# x:271 y:479
			OperatableStateMachine.add('ClearTextCommandKey',
										CalculationState(calculation=None),
										transitions={'done': 'StandUp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'text_command', 'output_value': 'text_command'})

			# x:427 y:106
			OperatableStateMachine.add('WaitTextCommand',
										_sm_waittextcommand_0,
										transitions={'finished': 'CheckIfStepSequence', 'failed': 'failed', 'timeout': 'StandUp'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'text_command': 'text_command'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
