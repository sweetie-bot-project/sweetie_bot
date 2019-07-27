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
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.joystick_joint_control import JoystickJointControl
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
from flexbe_states.calculation_state import CalculationState
from sweetie_bot_flexbe_states.execute_joint_trajectory_key import ExecuteJointTrajectoryKey
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from sweetie_bot_text_msgs.msg import TextCommand
from sensor_msgs.msg import Joy
import random
# [/MANUAL_IMPORT]


'''
Created on Sun Nov 18 2018
@author: disRecord
'''
class JoyProto3SM(Behavior):
	'''
	Joystick based control for Proto3. Head petrforms random movements, also it position can be controlled by joystick. Different animation can be activated by joystick buttons or by TextCommand.
	'''


	def __init__(self):
		super(JoyProto3SM, self).__init__()
		self.name = 'JoyProto3'

		# parameters of this behavior
		self.add_parameter('action_list', 'greeting,head_shake,applause')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		# x:30 y:365, x:1031 y:242
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.unused = None
		_state_machine.userdata.joy_msg = Joy()
		_state_machine.userdata.text_msg = TextCommand()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:353
		_sm_waituntiltextcmd_0 = OperatableStateMachine(outcomes=['received'], output_keys=['text_msg'])

		with _sm_waituntiltextcmd_0:
			# x:87 y:101
			OperatableStateMachine.add('WaitTextMsg',
										WaitForMessageState(topic='/control', condition=lambda x: x.type == 'flexbe/action', buffered=False, clear=True),
										transitions={'received': 'received', 'unavailable': 'Wait1s'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'text_msg'})

			# x:314 y:97
			OperatableStateMachine.add('Wait1s',
										WaitState(wait_time=1),
										transitions={'done': 'WaitTextMsg'},
										autonomy={'done': Autonomy.Off})


		# x:397 y:287, x:85 y:291, x:504 y:228, x:330 y:297, x:459 y:297, x:530 y:297
		_sm_joystickmovements_1 = ConcurrencyContainer(outcomes=['failed', 'button1234', 'timeout'], input_keys=['joy_msg'], output_keys=['joy_msg'], conditions=[
										('timeout', [('JoystickControl', 'done')]),
										('failed', [('WaitButton1234Pressed', 'unavailable')]),
										('button1234', [('WaitButton1234Pressed', 'received')])
										])

		with _sm_joystickmovements_1:
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


		# x:205 y:274, x:300 y:274, x:68 y:275, x:449 y:266, x:595 y:272, x:833 y:323, x:638 y:392, x:730 y:353, x:949 y:274
		_sm_randmovements_2 = ConcurrencyContainer(outcomes=['failed', 'timeout', 'joy_msg', 'text_msg'], input_keys=['unused', 'joy_msg', 'text_msg'], output_keys=['joy_msg', 'text_msg', 'unused'], conditions=[
										('failed', [('WaitJoystick', 'unavailable')]),
										('failed', [('RandHeadMoves', 'failed')]),
										('joy_msg', [('WaitJoystick', 'received')]),
										('timeout', [('RandHeadMoves', 'done')]),
										('text_msg', [('WaitUntilTextCmd', 'received')])
										])

		with _sm_randmovements_2:
			# x:88 y:74
			OperatableStateMachine.add('WaitJoystick',
										WaitForMessageState(topic=joy_topic, condition=lambda msg: any(msg.buttons) or any(msg.axes), buffered=False, clear=True),
										transitions={'received': 'joy_msg', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'joy_msg'})

			# x:332 y:60
			OperatableStateMachine.add('RandHeadMoves',
										RandJointsMovements(controller='joint_state_head', duration=10, interval=[2.0, 5.0], joints=['head_joint2','head_joint4'], minimal=[-0.2, -0.3], maximal=[0.3, 0.3]),
										transitions={'done': 'timeout', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'config': 'unused'})

			# x:647 y:50
			OperatableStateMachine.add('WaitUntilTextCmd',
										_sm_waituntiltextcmd_0,
										transitions={'received': 'text_msg'},
										autonomy={'received': Autonomy.Inherit},
										remapping={'text_msg': 'text_msg'})



		with _state_machine:
			# x:52 y:63
			OperatableStateMachine.add('RandMovements',
										_sm_randmovements_2,
										transitions={'failed': 'failed', 'timeout': 'RandomChoice', 'joy_msg': 'CheckButtonPressed', 'text_msg': 'ProcessTextMsg'},
										autonomy={'failed': Autonomy.Inherit, 'timeout': Autonomy.Inherit, 'joy_msg': Autonomy.Inherit, 'text_msg': Autonomy.Inherit},
										remapping={'unused': 'unused', 'joy_msg': 'joy_msg', 'text_msg': 'text_msg'})

			# x:85 y:283
			OperatableStateMachine.add('JoystickMovements',
										_sm_joystickmovements_1,
										transitions={'failed': 'failed', 'button1234': 'ProcessButton', 'timeout': 'RandMovements'},
										autonomy={'failed': Autonomy.Inherit, 'button1234': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'joy_msg': 'joy_msg'})

			# x:198 y:176
			OperatableStateMachine.add('CheckButtonPressed',
										DecisionState(outcomes=['button1234', 'other' ], conditions=lambda msg: 'button1234' if any(msg.buttons[0:4]) else 'other'),
										transitions={'button1234': 'ProcessButton', 'other': 'JoystickMovements'},
										autonomy={'button1234': Autonomy.Off, 'other': Autonomy.Off},
										remapping={'input_value': 'joy_msg'})

			# x:690 y:123
			OperatableStateMachine.add('SetHeadNominalPose',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'ExecuteAction', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:425 y:181
			OperatableStateMachine.add('ProcessButton',
										CalculationState(calculation=self.process_joy_msg),
										transitions={'done': 'SetHeadNominalPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'joy_msg', 'output_value': 'action_name'})

			# x:272 y:100
			OperatableStateMachine.add('ProcessTextMsg',
										CalculationState(calculation=self.process_text_msg),
										transitions={'done': 'SetHeadNominalPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'text_msg', 'output_value': 'action_name'})

			# x:457 y:12
			OperatableStateMachine.add('RandomAction',
										CalculationState(calculation=self.random_action),
										transitions={'done': 'SetHeadNominalPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'unused', 'output_value': 'action_name'})

			# x:331 y:321
			OperatableStateMachine.add('ExecuteAction',
										ExecuteJointTrajectoryKey(controller='motion/controller/joint_trajectory', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'JoystickMovements', 'unavalible': 'JoystickMovements', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'unavalible': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'trajectory_param': 'action_name'})

			# x:308 y:7
			OperatableStateMachine.add('RandomChoice',
										DecisionState(outcomes=['move','continue'], conditions=lambda x: 'continue' if random.random() < 0.4 else 'move'),
										transitions={'move': 'RandomAction', 'continue': 'RandMovements'},
										autonomy={'move': Autonomy.Off, 'continue': Autonomy.Off},
										remapping={'input_value': 'unused'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
        def process_joy_msg(self, joy_msg):
            action_list = self.action_list.split(',')
            # find nonzero item in buttons
            index = None
            for k in range(0,min(len(action_list),4)):
                if joy_msg.buttons[k]:
                    index = k
                    break
            # get 
            if index != None:
                return action_list[index]
            else:
                return self.random_action(None)

        def process_text_msg(self, text_msg):
            action_list = self.action_list.split(',')
            # select action
            if text_msg.command in self.action_list:
                return text_msg.command
            else:
                return self.random_action(None)

        def random_action(self, unused):
            action_list = self.action_list.split(',')
            return random.choice(action_list)
	# [/MANUAL_FUNC]
