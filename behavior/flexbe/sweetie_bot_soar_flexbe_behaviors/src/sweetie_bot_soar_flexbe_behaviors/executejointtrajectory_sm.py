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
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.execute_joint_trajectory_relative import ExecuteJointTrajectoryRelative
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
# [/MANUAL_IMPORT]


'''
Created on 2022
@author: disRecord
'''
class ExecuteJointTrajectorySM(Behavior):
	'''
	Execute JointTrajectory absolute or rel.ative.
	'''


	def __init__(self):
		super(ExecuteJointTrajectorySM, self).__init__()
		self.name = 'ExecuteJointTrajectory'

		# parameters of this behavior
		self.add_parameter('joint_trajectory', 'head_node')
		self.add_parameter('is_relative', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1265 y:71, x:1214 y:619, x:1262 y:397
		_state_machine = OperatableStateMachine(outcomes=['succeed', 'failed', 'invalid_pose'])
		_state_machine.userdata.none = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:282 y:95
			OperatableStateMachine.add('ChoiceAnimationType',
										DecisionState(outcomes=['absolute', 'relative'], conditions=lambda x: 'relative' if self.is_relative else 'absolute'),
										transitions={'absolute': 'AnmationAbsolute', 'relative': 'AnimationRelative'},
										autonomy={'absolute': Autonomy.Off, 'relative': Autonomy.Off},
										remapping={'input_value': 'none'})

			# x:763 y:208
			OperatableStateMachine.add('AnmationAbsolute',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param=self.joint_trajectory, trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'succeed', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:756 y:40
			OperatableStateMachine.add('AnimationRelative',
										ExecuteJointTrajectoryRelative(action_topic='motion/controller/joint_trajectory', trajectory_param=self.joint_trajectory, trajectory_ns='saved_msgs/joint_trajectory', joints_limits=self.get_joint_limits(), joint_states_topic='joint_states'),
										transitions={'success': 'succeed', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def get_joint_limits(self):
		robot_name = rospy.get_param('robot_name', None)
		# select joints limits
		if robot_name == 'sweetie_bot_proto3':
			joint_limits = {	'head_joint1' : ( -1.0, 0.2 ), 
								'head_joint2' : ( -0.2, 0.5 ),
								'head_joint3' : ( -0.7, 0.7 ),
								'head_joint4' : ( -0.5, 0.5 ),
							}
		else:
			joint_limits = {}

		return joint_limits
	# [/MANUAL_FUNC]
