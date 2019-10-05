#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
from sweetie_bot_flexbe_behaviors.joyanimation_sm import JoyAnimationSM
from sweetie_bot_flexbe_behaviors.joywalk_sm import JoyWalkSM
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_behaviors.autonomousbehavior2_sm import AutonomousBehavior2SM
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from sweetie_bot_flexbe_states.rand_head_movements import SweetieBotRandHeadMovements
from sweetie_bot_flexbe_behaviors.autonomousbehavior_sm import AutonomousBehaviorSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from sweetie_bot_joystick.msg import KeyPressed
import random
# [/MANUAL_IMPORT]


'''
Created on Sun Jul 21 2019
@author: disRecord
'''
class JoyOverallSM(Behavior):
	'''
	This behavior includes JoyWalk, JoyAnimaton and AutonomousBehavior2.
	'''


	def __init__(self):
		super(JoyOverallSM, self).__init__()
		self.name = 'JoyOverall'

		# parameters of this behavior
		self.add_parameter('timeout', 20)

		# references to used behaviors
		self.add_behavior(JoyAnimationSM, 'JoyAnimation')
		self.add_behavior(JoyWalkSM, 'JoyWalk')
		self.add_behavior(AutonomousBehavior2SM, 'AutonomousBehavior2')
		self.add_behavior(AutonomousBehaviorSM, 'AutonomousBehavior')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:98 y:490, x:1089 y:260
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.be_evil = False
		_state_machine.userdata.key_pressed_msg = KeyPressed()
		_state_machine.userdata.config = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:353, x:558 y:270, x:472 y:279, x:284 y:286, x:430 y:353, x:530 y:353, x:630 y:353
		_sm_waitkeypressed_0 = ConcurrencyContainer(outcomes=['pressed', 'timeout', 'failed'], input_keys=['key_pressed_msg', 'config'], output_keys=['key_pressed_msg'], conditions=[
										('failed', [('RandMovements', 'failed')]),
										('timeout', [('RandMovements', 'done')]),
										('pressed', [('WaitKey', 'received')]),
										('failed', [('WaitKey', 'unavailable')])
										])

		with _sm_waitkeypressed_0:
			# x:70 y:100
			OperatableStateMachine.add('WaitKey',
										WaitForMessageState(topic='/hmi/joy_decoder/keys_pressed', condition=lambda x: x.keys != [], buffered=False, clear=True),
										transitions={'received': 'pressed', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'key_pressed_msg'})

			# x:423 y:91
			OperatableStateMachine.add('RandMovements',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=self.timeout, interval=[3,5], max2356=[0.3,0.3,1.5,1.5], min2356=[-0.3,-0.3,-1.5,-1.5]),
										transitions={'done': 'timeout', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'config': 'config'})



		with _state_machine:
			# x:217 y:310
			OperatableStateMachine.add('SetHeadPoseNominal',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'WaitKeyPressed', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:662 y:48
			OperatableStateMachine.add('JoyAnimation',
										self.use_behavior(JoyAnimationSM, 'JoyAnimation'),
										transitions={'timeout': 'SetHeadPoseNominal', 'failed': 'failed', 'unknown_keys': 'JoyWalk'},
										autonomy={'timeout': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'unknown_keys': Autonomy.Inherit},
										remapping={'key_pressed_msg': 'key_pressed_msg', 'be_evil': 'be_evil'})

			# x:841 y:220
			OperatableStateMachine.add('JoyWalk',
										self.use_behavior(JoyWalkSM, 'JoyWalk'),
										transitions={'timeout': 'SetHeadPoseNominal', 'failed': 'failed', 'invalid_pose': 'failed', 'unknown_keys': 'CheckKeys'},
										autonomy={'timeout': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'invalid_pose': Autonomy.Inherit, 'unknown_keys': Autonomy.Inherit},
										remapping={'key_pressed_msg': 'key_pressed_msg'})

			# x:641 y:329
			OperatableStateMachine.add('CheckKeys',
										DecisionState(outcomes=['walk','animation','start', 'unknown'], conditions=self.decision),
										transitions={'walk': 'JoyWalk', 'animation': 'JoyAnimation', 'start': 'RandomChoice', 'unknown': 'WaitKeyPressed'},
										autonomy={'walk': Autonomy.Off, 'animation': Autonomy.Off, 'start': Autonomy.Off, 'unknown': Autonomy.Off},
										remapping={'input_value': 'key_pressed_msg'})

			# x:443 y:565
			OperatableStateMachine.add('AutonomousBehavior2',
										self.use_behavior(AutonomousBehavior2SM, 'AutonomousBehavior2'),
										transitions={'finished': 'SetHeadPoseNominal', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:194 y:135
			OperatableStateMachine.add('WaitKeyPressed',
										_sm_waitkeypressed_0,
										transitions={'pressed': 'JoyAnimation', 'timeout': 'RandomChoice', 'failed': 'failed'},
										autonomy={'pressed': Autonomy.Inherit, 'timeout': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'key_pressed_msg': 'key_pressed_msg', 'config': 'config'})

			# x:545 y:431
			OperatableStateMachine.add('RandomChoice',
										DecisionState(outcomes=['one','two'], conditions=lambda x: 'one' if random.random()<0.0 else 'two'),
										transitions={'one': 'WaitKeyPressed', 'two': 'WaitKeyPressed'},
										autonomy={'one': Autonomy.Off, 'two': Autonomy.Off},
										remapping={'input_value': 'config'})

			# x:647 y:599
			OperatableStateMachine.add('AutonomousBehavior',
										self.use_behavior(AutonomousBehaviorSM, 'AutonomousBehavior'),
										transitions={'finished': 'SetHeadPoseNominal', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
        def decision(self, key_msg):
            keys = set(key_msg.keys)
            if keys.intersection(('left','right','up','down')):
                return 'walk'
            if keys.intersection(('1','2','3','4')):
                return 'animation'
            if keys.intersection(('start',)):
                return 'start'
            return 'unknown'
	# [/MANUAL_FUNC]
