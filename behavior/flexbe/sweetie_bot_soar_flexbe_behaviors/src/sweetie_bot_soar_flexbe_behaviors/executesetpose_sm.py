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
from sweetie_bot_flexbe_states.publisher_state import PublisherState
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from sweetie_bot_kinematics_msgs.msg import SupportState
# [/MANUAL_IMPORT]


'''
Created on Sat May 08 2021
@author: disRecord
'''
class ExecuteSetPoseSM(Behavior):
	'''
	Move robot in specified pose.
	'''


	def __init__(self):
		super(ExecuteSetPoseSM, self).__init__()
		self.name = 'ExecuteSetPose'

		# parameters of this behavior
		self.add_parameter('pose', 'head_nominal')
		self.add_parameter('set_supports', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:207 y:429, x:737 y:257
		_state_machine = OperatableStateMachine(outcomes=['succeed', 'failed'])
		_state_machine.userdata.set_supports = self.set_supports

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:164 y:166
			OperatableStateMachine.add('SetPose',
										SetJointState(controller='motion/controller/joint_state_head', pose_param=self.pose, pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'CheckSetSupports', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:459 y:441
			OperatableStateMachine.add('PublishSupports',
										PublisherState(topic='motion/aggregator_ref/in_supports', msg_type=SupportState, value={'name':['leg1','leg2','leg3','leg4'], 'support':[1.0,1.0,1.0,1.0]}),
										transitions={'done': 'succeed', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:513 y:354
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=0.5),
										transitions={'done': 'PublishSupports'},
										autonomy={'done': Autonomy.Off})

			# x:399 y:292
			OperatableStateMachine.add('CheckSetSupports',
										DecisionState(outcomes=['true', 'false'], conditions=lambda x: 'true' if x else 'false'),
										transitions={'true': 'Wait', 'false': 'succeed'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'set_supports'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
