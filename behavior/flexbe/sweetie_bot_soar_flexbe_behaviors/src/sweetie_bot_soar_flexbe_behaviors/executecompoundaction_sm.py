#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.compound_action_param import CompoundActionParam
from sweetie_bot_flexbe_states.check_joint_state import CheckJointState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 11 2019
@author: disrecord
'''
class ExecuteCompoundActionSM(Behavior):
	'''
	Execute CompoundAction defined by parameter.
	'''


	def __init__(self):
		super(ExecuteCompoundActionSM, self).__init__()
		self.name = 'ExecuteCompoundAction'

		# parameters of this behavior
		self.add_parameter('action_name', 'brohoof')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:183 y:341, x:352 y:353
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'invalid_pose'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:584 y:41, x:452 y:346, x:230 y:353
		_sm_container_0 = OperatableStateMachine(outcomes=['finished', 'failed', 'invalid_pose'])

		with _sm_container_0:
			# x:127 y:102
			OperatableStateMachine.add('CheckBodyPose',
										CheckJointState(outcomes=['body_nominal', 'body_crouched', 'unknown'], pose_ns='saved_msgs/joint_state', tolerance=0.17, joint_topic="joint_states", timeout=1.0),
										transitions={'body_nominal': 'finished', 'unknown': 'invalid_pose', 'body_crouched': 'CrouchEnd'},
										autonomy={'body_nominal': Autonomy.Off, 'unknown': Autonomy.Off, 'body_crouched': Autonomy.Off})

			# x:350 y:172
			OperatableStateMachine.add('CrouchEnd',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_end', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'finished', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		# x:30 y:353, x:130 y:353, x:230 y:353, x:330 y:353, x:430 y:353, x:530 y:353, x:630 y:353, x:730 y:353
		_sm_standup_1 = ConcurrencyContainer(outcomes=['finished', 'invalid_pose', 'failed'], conditions=[
										('failed', [('HeadNominal', 'failed')]),
										('finished', [('HeadNominal', 'done'), ('Container', 'finished')]),
										('invalid_pose', [('HeadNominal', 'timeout')]),
										('failed', [('Container', 'failed')]),
										('invalid_pose', [('Container', 'invalid_pose')])
										])

		with _sm_standup_1:
			# x:425 y:127
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed', 'invalid_pose': 'invalid_pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'invalid_pose': Autonomy.Inherit})

			# x:105 y:127
			OperatableStateMachine.add('HeadNominal',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'finished', 'failed': 'failed', 'timeout': 'invalid_pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})



		with _state_machine:
			# x:108 y:85
			OperatableStateMachine.add('CompoundAction',
										CompoundActionParam(action_param=self.action_name, action_ns='saved_msgs/compound_action'),
										transitions={'success': 'finished', 'invalid_pose': 'StandUp', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:405 y:81
			OperatableStateMachine.add('StandUp',
										_sm_standup_1,
										transitions={'finished': 'CompoundAction', 'invalid_pose': 'invalid_pose', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'invalid_pose': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
