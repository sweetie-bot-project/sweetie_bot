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
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.execute_joint_trajectory_relative import ExecuteJointTrajectoryRelative
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
# [/MANUAL_IMPORT]


'''
Created on Sun Feb 21 2021
@author: disRecord
'''
class ExecuteJointTrajectoryAndSaySM(Behavior):
	'''
	Prononce text (TextCommand voice/say) and execute JointTrajectory at the same time.
	'''


	def __init__(self):
		super(ExecuteJointTrajectoryAndSaySM, self).__init__()
		self.name = 'ExecuteJointTrajectoryAndSay'

		# parameters of this behavior
		self.add_parameter('joint_trajectory', 'head_node')
		self.add_parameter('text_delay', -0.5)
		self.add_parameter('is_relative', False)
		self.add_parameter('text', 'I am speaking.')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:716 y:131, x:722 y:347, x:717 y:244
		_state_machine = OperatableStateMachine(outcomes=['succeed', 'failed', 'invalid_pose'])
		_state_machine.userdata.none = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:662 y:84
		_sm_waitandsay_0 = OperatableStateMachine(outcomes=['finished'])

		with _sm_waitandsay_0:
			# x:123 y:80
			OperatableStateMachine.add('WaitTextDelay',
										WaitState(wait_time=max(self.text_delay, 0.0)),
										transitions={'done': 'SayText'},
										autonomy={'done': Autonomy.Off})

			# x:342 y:73
			OperatableStateMachine.add('SayText',
										TextCommandState(type='voice/say', command=self.text, topic='control'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		# x:913 y:78, x:905 y:322, x:912 y:201
		_sm_animation_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'invalid_pose'], input_keys=['none'])

		with _sm_animation_1:
			# x:99 y:134
			OperatableStateMachine.add('WaitAnimation',
										WaitState(wait_time=max(-self.text_delay, 0.0)),
										transitions={'done': 'ChoiceAnimationType'},
										autonomy={'done': Autonomy.Off})

			# x:274 y:137
			OperatableStateMachine.add('ChoiceAnimationType',
										DecisionState(outcomes=['absolute', 'relative'], conditions=lambda x: 'relative' if self.is_relative else 'absolute'),
										transitions={'absolute': 'Animation', 'relative': 'RelativeAnimation'},
										autonomy={'absolute': Autonomy.Off, 'relative': Autonomy.Off},
										remapping={'input_value': 'none'})

			# x:548 y:58
			OperatableStateMachine.add('RelativeAnimation',
										ExecuteJointTrajectoryRelative(action_topic='motion/controller/joint_trajectory', trajectory_param=self.joint_trajectory, trajectory_ns='saved_msgs/joint_trajectory', joints_limits=self.get_joint_limits(), joint_states_topic='joint_states'),
										transitions={'success': 'finished', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:566 y:233
			OperatableStateMachine.add('Animation',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param=self.joint_trajectory, trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'finished', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		# x:321 y:429, x:763 y:412, x:930 y:162, x:931 y:229, x:515 y:419, x:929 y:91
		_sm_animationandsay_2 = ConcurrencyContainer(outcomes=['finished', 'failed', 'invalid_pose'], input_keys=['none'], conditions=[
										('finished', [('WaitAndSay', 'finished'), ('Animation', 'finished')]),
										('failed', [('Animation', 'failed')]),
										('invalid_pose', [('Animation', 'invalid_pose')])
										])

		with _sm_animationandsay_2:
			# x:474 y:182
			OperatableStateMachine.add('Animation',
										_sm_animation_1,
										transitions={'finished': 'finished', 'failed': 'failed', 'invalid_pose': 'invalid_pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'invalid_pose': Autonomy.Inherit},
										remapping={'none': 'none'})

			# x:118 y:186
			OperatableStateMachine.add('WaitAndSay',
										_sm_waitandsay_0,
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})



		with _state_machine:
			# x:259 y:129
			OperatableStateMachine.add('AnimationAndSay',
										_sm_animationandsay_2,
										transitions={'finished': 'succeed', 'failed': 'failed', 'invalid_pose': 'invalid_pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'invalid_pose': Autonomy.Inherit},
										remapping={'none': 'none'})


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
