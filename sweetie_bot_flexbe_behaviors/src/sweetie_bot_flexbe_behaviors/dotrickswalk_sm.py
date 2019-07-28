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
from sweetie_bot_flexbe_behaviors.greeting_sm import GreetingSM
from sweetie_bot_flexbe_behaviors.play_sm import PlaySM
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_behaviors.cheer_sm import CheerSM
from sweetie_bot_flexbe_states.rand_head_movements import SweetieBotRandHeadMovements
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Sat Jul 27 2019
@author: disRecord
'''
class DoTricksWalkSM(Behavior):
	'''
	Do trick during walking.
	'''


	def __init__(self):
		super(DoTricksWalkSM, self).__init__()
		self.name = 'DoTricksWalk'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(GreetingSM, 'Greeting')
		self.add_behavior(PlaySM, 'Play')
		self.add_behavior(CheerSM, 'Cheer')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:902 y:403, x:697 y:491
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.be_evil = False
		_state_machine.userdata.unused = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:32 y:127
			OperatableStateMachine.add('SetStandingPose',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='body_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'SelectTrick', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:371 y:161
			OperatableStateMachine.add('Greeting',
										self.use_behavior(GreetingSM, 'Greeting'),
										transitions={'finished': 'RandHead', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:371 y:284
			OperatableStateMachine.add('Play',
										self.use_behavior(PlaySM, 'Play'),
										transitions={'finished': 'RandHead', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:202 y:154
			OperatableStateMachine.add('SelectTrick',
										DecisionState(outcomes=['play','cheer','greet'], conditions=lambda x: random.choice(['play','cheer','greet'])),
										transitions={'play': 'Play', 'cheer': 'Cheer', 'greet': 'Greeting'},
										autonomy={'play': Autonomy.Off, 'cheer': Autonomy.Off, 'greet': Autonomy.Off},
										remapping={'input_value': 'unused'})

			# x:823 y:274
			OperatableStateMachine.add('PrepareToWalk',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='crouch_begin', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:376 y:38
			OperatableStateMachine.add('Cheer',
										self.use_behavior(CheerSM, 'Cheer'),
										transitions={'finished': 'RandHead', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:774 y:45
			OperatableStateMachine.add('RandHead',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=8, interval=[3,5], max2356=[0.3,0.3,1.5,1.5], min2356=[-0.3,-0.3,-1.5,-1.5]),
										transitions={'done': 'SetHeadNominal', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'config': 'unused'})

			# x:809 y:138
			OperatableStateMachine.add('SetHeadNominal',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'PrepareToWalk', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
