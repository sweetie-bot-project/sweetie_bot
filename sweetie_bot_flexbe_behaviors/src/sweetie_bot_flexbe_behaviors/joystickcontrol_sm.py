#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from sweetie_bot_flexbe_states.rand_joints_movements import RandJointsMovements
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.joystick_joint_control import JoystickJointControl
from flexbe_states.decision_state import DecisionState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from sensor_msgs.msg import Joy
# [/MANUAL_IMPORT]


'''
Created on Sun Nov 18 2018
@author: disRecord
'''
class JoystickControlSM(Behavior):
	'''
	Head is controlled by joystick.
	'''


	def __init__(self):
		super(JoystickControlSM, self).__init__()
		self.name = 'JoystickControl'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		joint_trajectory_action = '/motion/controller/joint_trajectory'
		# x:30 y:365, x:1031 y:242
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.rand_moves_config = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:397 y:287, x:130 y:297, x:230 y:297, x:330 y:297, x:459 y:297, x:530 y:297
		_sm_joystickmovements_0 = ConcurrencyContainer(outcomes=['failed', 'button1234', 'timeout'], output_keys=['joy_msg'], conditions=[
										('timeout', [('JoystickControl', 'done')]),
										('failed', [('WaitButton1234Pressed', 'unavailable')]),
										('button1234', [('WaitButton1234Pressed', 'received')])
										])

		with _sm_joystickmovements_0:
			# x:385 y:129
			OperatableStateMachine.add('JoystickControl',
										JoystickJointControl(joints_topic='/joint_states', goal_joints_topic='/motion/controller/joint_state/out_joints_src_reset', joy_topic=joy_topic, buttons=[(4,0.6,'ear_l_joint',-3.0,1.0),(6,-0.6,'ear_l_joint',-3.0,1.0),(5,0.6,'ear_r_joint',-3.0,1.0),(7,-0.6,'ear_r_joint',-3.0,1.0)], axes=[(0,0,'eyes_yaw',-1.5,1.5),(1,0,'eyes_pitch',-1.5,1.5),(4,-0.4,'head_joint4',-1.5,1.5),(5,0.4,'head_joint2',-1.5,1.5),(3,0.5,'mouth_joint',-0.6,0)], timeout=5.0),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})

			# x:130 y:136
			OperatableStateMachine.add('WaitButton1234Pressed',
										WaitForMessageState(topic=joy_topic, condition=lambda msg: any(msg.buttons[0:4]), buffered=False, clear=True),
										transitions={'received': 'button1234', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'joy_msg'})


		# x:625 y:339, x:47 y:263, x:323 y:268, x:703 y:252, x:630 y:296, x:443 y:267, x:227 y:263
		_sm_randmovements_1 = ConcurrencyContainer(outcomes=['failed', 'timeout', 'joy_msg'], input_keys=['rand_moves_config'], output_keys=['joy_msg'], conditions=[
										('failed', [('WaitJoystick', 'unavailable')]),
										('failed', [('RandHeadMoves', 'failed')]),
										('joy_msg', [('WaitJoystick', 'received')]),
										('timeout', [('RandHeadMoves', 'done')])
										])

		with _sm_randmovements_1:
			# x:88 y:74
			OperatableStateMachine.add('WaitJoystick',
										WaitForMessageState(topic=joy_topic, condition=lambda msg: any(msg.buttons) or any(msg.axes), buffered=False, clear=False),
										transitions={'received': 'joy_msg', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'joy_msg'})

			# x:332 y:60
			OperatableStateMachine.add('RandHeadMoves',
										RandJointsMovements(controller='joint_state_head', duration=2000, interval=[2.0, 5.0], joints=['head_joint2','head_joint4'], minimal=[-0.2, -0.3], maximal=[0.3, 0.3]),
										transitions={'done': 'timeout', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'config': 'rand_moves_config'})



		with _state_machine:
			# x:57 y:26
			OperatableStateMachine.add('RandMovements',
										_sm_randmovements_1,
										transitions={'failed': 'failed', 'timeout': 'RandMovements', 'joy_msg': 'CheckButtonPressed'},
										autonomy={'failed': Autonomy.Inherit, 'timeout': Autonomy.Inherit, 'joy_msg': Autonomy.Inherit},
										remapping={'rand_moves_config': 'rand_moves_config', 'joy_msg': 'joy_msg'})

			# x:413 y:17
			OperatableStateMachine.add('PlaceHead',
										SrdfStateToMoveit(config_name='head_group_basic', move_group='head_group', action_topic='move_group', robot_name=''),
										transitions={'reached': 'SelectMovement', 'planning_failed': 'Wait', 'control_failed': 'Wait', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:373 y:236
			OperatableStateMachine.add('Greeting',
										ExecuteJointTrajectory(action_topic=joint_trajectory_action, trajectory_param='greeting', trajectory_ns='/saved_msgs/joint_trajectory'),
										transitions={'success': 'JoystickMovements', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:85 y:283
			OperatableStateMachine.add('JoystickMovements',
										_sm_joystickmovements_0,
										transitions={'failed': 'failed', 'button1234': 'PlaceHead', 'timeout': 'RandMovements'},
										autonomy={'failed': Autonomy.Inherit, 'button1234': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'joy_msg': 'joy_msg'})

			# x:612 y:186
			OperatableStateMachine.add('SelectMovement',
										DecisionState(outcomes=['none', 'button1','button2','button3','button4'], conditions=self.move_selector),
										transitions={'none': 'RandMovements', 'button1': 'Greeting', 'button2': 'HeadShake', 'button3': 'GrownUp', 'button4': 'Greeting'},
										autonomy={'none': Autonomy.Off, 'button1': Autonomy.Off, 'button2': Autonomy.Off, 'button3': Autonomy.Off, 'button4': Autonomy.Off},
										remapping={'input_value': 'joy_msg'})

			# x:412 y:317
			OperatableStateMachine.add('HeadShake',
										ExecuteJointTrajectory(action_topic=joint_trajectory_action, trajectory_param='head_shake', trajectory_ns='/saved_msgs/joint_trajectory'),
										transitions={'success': 'JoystickMovements', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:472 y:404
			OperatableStateMachine.add('GrownUp',
										ExecuteJointTrajectory(action_topic=joint_trajectory_action, trajectory_param='very_grown_up', trajectory_ns='/saved_msgs/joint_trajectory'),
										transitions={'success': 'JoystickMovements', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:167 y:134
			OperatableStateMachine.add('CheckButtonPressed',
										DecisionState(outcomes=['button1234', 'other' ], conditions=lambda msg: 'button1234' if any(msg.buttons[0:4]) else 'other'),
										transitions={'button1234': 'PlaceHead', 'other': 'JoystickMovements'},
										autonomy={'button1234': Autonomy.Off, 'other': Autonomy.Off},
										remapping={'input_value': 'joy_msg'})

			# x:686 y:104
			OperatableStateMachine.add('PlaceHead_2',
										SrdfStateToMoveit(config_name='head_group_basic', move_group='head_group', action_topic='move_group', robot_name=''),
										transitions={'reached': 'SelectMovement', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:703 y:18
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=0.5),
										transitions={'done': 'PlaceHead_2'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
        def move_selector(self, joy_msg):
            if (len(joy_msg.buttons) < 4):
                return 'none'
            if (joy_msg.buttons[0]):
                return 'button1'
            if (joy_msg.buttons[1]):
                return 'button2'
            if (joy_msg.buttons[2]):
                return 'button3'
            if (joy_msg.buttons[3]):
                return 'button4'
            return 'none'
	# [/MANUAL_FUNC]
