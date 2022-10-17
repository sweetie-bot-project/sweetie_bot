#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.check_joint_state import CheckJointState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.rand_pose_generator import RandPoseGenerator
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
from sweetie_bot_flexbe_states.set_operational import SetOperational
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 21 2019
@author: disRecord
'''
class RandHeadMovementsUniversalSM(Behavior):
	'''
	Random head movements for SweetieBot Proto2 and Proto3. Behavior exits after timeout.
	'''


	def __init__(self):
		super(RandHeadMovementsUniversalSM, self).__init__()
		self.name = 'RandHeadMovementsUniversal'

		# parameters of this behavior
		self.add_parameter('timeout', 15)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:353, x:425 y:365
		_state_machine = OperatableStateMachine(outcomes=['succeed', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:353, x:130 y:353, x:230 y:353, x:330 y:353, x:430 y:353, x:530 y:353
		_sm_randheadmovements_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('LookAt', 'done')]),
										('failed', [('LookAt', 'failure')]),
										('finished', [('RandPose', 'done')]),
										('failed', [('RandPose', 'failure')])
										])

		with _sm_randheadmovements_0:
			# x:48 y:117
			OperatableStateMachine.add('LookAt',
										SetOperational(controller='motion/controller/look_at', operational=True, resources=['head'], sync=True),
										transitions={'done': 'finished', 'failure': 'failed'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:286 y:122
			OperatableStateMachine.add('RandPose',
										RandPoseGenerator(topic='motion/controller/look_at/in_pose_ref', duration=self.timeout, interval=[1.0,4.0], maxXYZ=[1.0,2.0,0.6], minXYZ=[1.0,-2.0,0.2], frame_xyz='base_link', frame_out='odom_combined'),
										transitions={'done': 'finished', 'failure': 'failed'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})



		with _state_machine:
			# x:163 y:40
			OperatableStateMachine.add('CheckCrouched',
										CheckJointState(outcomes=['body_crouched', 'unknown'], pose_ns='saved_msgs/joint_state', tolerance=0.17, joint_topic="joint_states", timeout=1.0),
										transitions={'body_crouched': 'StandUp', 'unknown': 'RandHeadMovements'},
										autonomy={'body_crouched': Autonomy.Off, 'unknown': Autonomy.Off})

			# x:150 y:341
			OperatableStateMachine.add('HeadNominal',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'succeed', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:154 y:179
			OperatableStateMachine.add('RandHeadMovements',
										_sm_randheadmovements_0,
										transitions={'finished': 'HeadNominal', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:412 y:68
			OperatableStateMachine.add('StandUp',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_end', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'RandHeadMovements', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
