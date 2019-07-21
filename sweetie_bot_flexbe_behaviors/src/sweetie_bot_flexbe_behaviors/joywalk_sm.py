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
from sweetie_bot_flexbe_states.execute_step_sequence_key import ExecuteStepSequenceKey
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from flexbe_states.wait_state import WaitState
from flexbe_states.check_condition_state import CheckConditionState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from flexbe_states.calculation_state import CalculationState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from sweetie_bot_joystick.msg import KeyPressed
from std_msgs.msg import Bool
# [/MANUAL_IMPORT]


'''
Created on Sat Jul 20 2019
@author: disRecord
'''
class JoyWalkSM(Behavior):
	'''
	Joystick controlled walk behavior. 

Walking is controled by directions keys (left, right, up, down), L1 and L2 plays role of modifiers,
eyes and head are controlled by sticks, other keys causes exit.
	'''


	def __init__(self):
		super(JoyWalkSM, self).__init__()
		self.name = 'JoyWalk'

		# parameters of this behavior
		self.add_parameter('timeout', 10)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:392 y:613, x:739 y:627, x:59 y:602, x:34 y:328
		_state_machine = OperatableStateMachine(outcomes=['timeout', 'failed', 'invalid_pose', 'unknown_keys'], input_keys=['key_pressed_msg'], output_keys=['key_pressed_msg'])
		_state_machine.userdata.key_pressed_msg = KeyPressed()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:353, x:130 y:353, x:626 y:359, x:330 y:353, x:430 y:353, x:530 y:353
		_sm_waitkey_0 = ConcurrencyContainer(outcomes=['received', 'timeout', 'failed'], output_keys=['key_pressed_msg'], conditions=[
										('received', [('WaitKeyPressed', 'received')]),
										('timeout', [('WaitTimeout', 'done')]),
										('failed', [('WaitKeyPressed', 'unavailable')])
										])

		with _sm_waitkey_0:
			# x:281 y:146
			OperatableStateMachine.add('WaitKeyPressed',
										WaitForMessageState(topic='/hmi/joy_decoder/keys_pressed', condition=self.trigger, buffered=False, clear=True),
										transitions={'received': 'received', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'key_pressed_msg'})

			# x:509 y:151
			OperatableStateMachine.add('WaitTimeout',
										WaitState(wait_time=self.timeout),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:51 y:31
			OperatableStateMachine.add('CheckExitEnter',
										DecisionState(outcomes=['animation','exit','walk','unknown'], conditions=self.decision),
										transitions={'animation': 'unknown_keys', 'exit': 'unknown_keys', 'walk': 'PrepareForWalk', 'unknown': 'unknown_keys'},
										autonomy={'animation': Autonomy.Off, 'exit': Autonomy.Off, 'walk': Autonomy.Off, 'unknown': Autonomy.Off},
										remapping={'input_value': 'key_pressed_msg'})

			# x:326 y:259
			OperatableStateMachine.add('CheckExit',
										DecisionState(outcomes=['animation', 'walk', 'exit', 'unknown'], conditions=self.decision),
										transitions={'animation': 'EndWalkUnknownKeys', 'walk': 'ProcessKeyMsg', 'exit': 'EndWalkUnknownKeys', 'unknown': 'WaitKey'},
										autonomy={'animation': Autonomy.Off, 'walk': Autonomy.Off, 'exit': Autonomy.Off, 'unknown': Autonomy.Off},
										remapping={'input_value': 'key_pressed_msg'})

			# x:703 y:410
			OperatableStateMachine.add('ExecuteStepSeq',
										ExecuteStepSequenceKey(controller='motion/controller/step_sequence', trajectory_ns='saved_msgs/step_sequence'),
										transitions={'success': 'WaitKey', 'unavailable': 'WaitKey', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'unavailable': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'trajectory_param': 'motion_param'})

			# x:431 y:381
			OperatableStateMachine.add('WaitKey',
										_sm_waitkey_0,
										transitions={'received': 'CheckExit', 'timeout': 'EndWalkTimeout', 'failed': 'failed'},
										autonomy={'received': Autonomy.Inherit, 'timeout': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'key_pressed_msg': 'key_pressed_msg'})

			# x:719 y:246
			OperatableStateMachine.add('CheckNone',
										CheckConditionState(predicate=lambda x: x == None),
										transitions={'true': 'WaitKey', 'false': 'ExecuteStepSeq'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'motion_param'})

			# x:108 y:289
			OperatableStateMachine.add('EndWalkUnknownKeys',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_end', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'unknown_keys', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:379 y:491
			OperatableStateMachine.add('EndWalkTimeout',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_end', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'timeout', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:408 y:158
			OperatableStateMachine.add('ProcessKeyMsg',
										CalculationState(calculation=self.process_walk),
										transitions={'done': 'CheckNone'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'key_pressed_msg', 'output_value': 'motion_param'})

			# x:211 y:72
			OperatableStateMachine.add('PrepareForWalk',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_begin', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'ProcessKeyMsg', 'partial_movement': 'invalid_pose', 'invalid_pose': 'invalid_pose', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
        def trigger(self, key_msg):
            if not key_msg:
                return False
            keys = set(key_msg.keys)
            return keys.intersection(('1', '2', '3', '4','start', 'select','left', 'right', 'up', 'down','tumble'))

        def decision(self, key_msg):
            keys = set(key_msg.keys)
            if keys.intersection(('1', '2', '3', '4')):
                return 'animation'
            if keys.intersection(('start',)):
                return 'exit'
            if keys.intersection(('left', 'right', 'up', 'down')):
                return 'walk'
            return 'unknown'

        def process_walk(self, key_msg):
            mapping_walk = { 
                    ('left',):  'turn_left_45',
                    ('right',): 'turn_right_45', 
                    ('up',):    'walk_fwd_40',
                    ('down',):  'walk_back_20',
                    ('up','left'):  'turn_left_20_20_45',
                    ('up','right'): 'turn_right_20_20_45',
                    ('L1', 'up'):   'walk_fwd_60',
                    ('L1', 'left'):   'turn_left_90',
                    ('L1', 'right'):   'turn_right_90',
                    ('L1', 'left', 'down'):   'backslide_left_20_20_90',
                    ('L1', 'right', 'down'):   'backslide_right_20_20_90',
                    ('L2', 'left'):            'drift_left',
                    ('L2', 'right'):           'drift_right',
                    ('L2', 'right', 'down'):   'backslide_right_10_00_45',
                    ('L2', 'left', 'down'):   'backslide_left_10_00_45',
                }
            keys = set(key_msg.keys).intersection(('left', 'right', 'up', 'down','L1','L2'))
            for k, val in mapping_walk.iteritems():
                if set(k) == keys:
                    return val
            return None
                
	# [/MANUAL_FUNC]
