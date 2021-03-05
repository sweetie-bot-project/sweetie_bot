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
from sweetie_bot_flexbe_states.check_joint_state import CheckJointState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.execute_joint_trajectory_relative import ExecuteJointTrajectoryRelative
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
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
		self.add_parameter('alpha_duration', 0.058)

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

		# x:683 y:221
		_sm_say_0 = OperatableStateMachine(outcomes=['finished'])

		with _sm_say_0:
			# x:107 y:189
			OperatableStateMachine.add('WaitSay',
										WaitState(wait_time=max(self.text_delay, 0.0)),
										transitions={'done': 'SayText'},
										autonomy={'done': Autonomy.Off})

			# x:509 y:199
			OperatableStateMachine.add('WaitSayEnd',
										WaitState(wait_time=self.alpha_duration*len(self.text)),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:272 y:200
			OperatableStateMachine.add('SayText',
										TextCommandState(type='voice/say', command=self.text, topic='control'),
										transitions={'done': 'WaitSayEnd'},
										autonomy={'done': Autonomy.Off})


		# x:796 y:105, x:778 y:418, x:757 y:263
		_sm_animation_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'invaid_pose'])

		with _sm_animation_1:
			# x:182 y:115
			OperatableStateMachine.add('WaitAnimation',
										WaitState(wait_time=max(-self.text_delay, 0.0)),
										transitions={'done': 'RelativeAnimation'},
										autonomy={'done': Autonomy.Off})

			# x:412 y:121
			OperatableStateMachine.add('RelativeAnimation',
										ExecuteJointTrajectoryRelative(action_topic='motion/controller/joint_trajectory', trajectory_param=self.joint_trajectory, trajectory_ns='saved_msgs/joint_trajectory', joints_limits=self.get_joint_limits(), joint_states_topic='joint_states'),
										transitions={'success': 'finished', 'partial_movement': 'invaid_pose', 'invalid_pose': 'invaid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		# x:122 y:483, x:449 y:487, x:284 y:481, x:662 y:471, x:700 y:425, x:711 y:488
		_sm_sayandrelativeanimation_2 = ConcurrencyContainer(outcomes=['finished', 'failed', 'invaid_pose'], conditions=[
										('finished', [('Say', 'finished'), ('Animation', 'finished')]),
										('invaid_pose', [('Animation', 'invaid_pose')]),
										('failed', [('Animation', 'failed')])
										])

		with _sm_sayandrelativeanimation_2:
			# x:81 y:160
			OperatableStateMachine.add('Say',
										_sm_say_0,
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:292 y:138
			OperatableStateMachine.add('Animation',
										_sm_animation_1,
										transitions={'finished': 'finished', 'failed': 'failed', 'invaid_pose': 'invaid_pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'invaid_pose': Autonomy.Inherit})


		# x:729 y:163
		_sm_say_3 = OperatableStateMachine(outcomes=['finished'])

		with _sm_say_3:
			# x:123 y:135
			OperatableStateMachine.add('WaitText',
										WaitState(wait_time=max(self.text_delay, 0.0)),
										transitions={'done': 'SayText'},
										autonomy={'done': Autonomy.Off})

			# x:549 y:153
			OperatableStateMachine.add('WaitSayEnd',
										WaitState(wait_time=self.alpha_duration*len(self.text)),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:308 y:141
			OperatableStateMachine.add('SayText',
										TextCommandState(type='voice/say', command=self.text, topic='control'),
										transitions={'done': 'WaitSayEnd'},
										autonomy={'done': Autonomy.Off})


		# x:762 y:86, x:762 y:342, x:753 y:197
		_sm_animation_4 = OperatableStateMachine(outcomes=['finished', 'failed', 'invaid_pose'])

		with _sm_animation_4:
			# x:132 y:143
			OperatableStateMachine.add('WaitAnimation',
										WaitState(wait_time=max(-self.text_delay, 0.0)),
										transitions={'done': 'ExecuteJointTrajectory'},
										autonomy={'done': Autonomy.Off})

			# x:352 y:151
			OperatableStateMachine.add('ExecuteJointTrajectory',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param=self.joint_trajectory, trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'finished', 'partial_movement': 'invaid_pose', 'invalid_pose': 'invaid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		# x:299 y:423, x:587 y:423, x:473 y:434, x:761 y:487, x:861 y:500, x:810 y:460
		_sm_sayandabsoluteanimation_5 = ConcurrencyContainer(outcomes=['finished', 'failed', 'invalid_pose'], conditions=[
										('finished', [('Say', 'finished'), ('Animation', 'finished')]),
										('invalid_pose', [('Animation', 'invaid_pose')]),
										('failed', [('Animation', 'failed')])
										])

		with _sm_sayandabsoluteanimation_5:
			# x:127 y:147
			OperatableStateMachine.add('Say',
										_sm_say_3,
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:419 y:152
			OperatableStateMachine.add('Animation',
										_sm_animation_4,
										transitions={'finished': 'finished', 'failed': 'failed', 'invaid_pose': 'invalid_pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'invaid_pose': Autonomy.Inherit})



		with _state_machine:
			# x:282 y:95
			OperatableStateMachine.add('ChoiceAnimationType',
										DecisionState(outcomes=['absolute', 'relative'], conditions=lambda x: 'relative' if self.is_relative else 'absolute'),
										transitions={'absolute': 'CheckHeadPose', 'relative': 'SayAndRelativeAnimation'},
										autonomy={'absolute': Autonomy.Off, 'relative': Autonomy.Off},
										remapping={'input_value': 'none'})

			# x:715 y:180
			OperatableStateMachine.add('SayAndAbsoluteAnimation',
										_sm_sayandabsoluteanimation_5,
										transitions={'finished': 'succeed', 'failed': 'failed', 'invalid_pose': 'invalid_pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'invalid_pose': Autonomy.Inherit})

			# x:712 y:39
			OperatableStateMachine.add('SayAndRelativeAnimation',
										_sm_sayandrelativeanimation_2,
										transitions={'finished': 'succeed', 'failed': 'failed', 'invaid_pose': 'invalid_pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'invaid_pose': Autonomy.Inherit})

			# x:576 y:398
			OperatableStateMachine.add('SetHeadNominal',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.03, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'SayAndAbsoluteAnimation', 'failed': 'failed', 'timeout': 'invalid_pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:351 y:333
			OperatableStateMachine.add('CheckHeadPose',
										CheckJointState(outcomes=['head_nominal', 'unknown'], pose_ns='saved_msgs/joint_state', tolerance=0.17, joint_topic="joint_states", timeout=1.0),
										transitions={'head_nominal': 'SayAndAbsoluteAnimation', 'unknown': 'SetHeadNominal'},
										autonomy={'head_nominal': Autonomy.Off, 'unknown': Autonomy.Off})


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
