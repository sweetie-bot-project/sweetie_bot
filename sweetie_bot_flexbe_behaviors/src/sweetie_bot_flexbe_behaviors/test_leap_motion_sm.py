#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.sweetie_bot_follow_head_leap_motion import SweetieBotFollowHeadLeapMotion
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 25 2017
@author: disRecord
'''
class test_leap_motionSM(Behavior):
	'''
	Robot head follows object detected by leap_motion
	'''


	def __init__(self):
		super(test_leap_motionSM, self).__init__()
		self.name = 'test_leap_motion'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:126 y:144
			OperatableStateMachine.add('FollowLeapMotion',
										SweetieBotFollowHeadLeapMotion(leap_motion_topic='/hmi/leap_motion/data', focus_point_topic='/hmi/leap_motion/point', follow_joint_state_controller='joint_state_head', neck_angle=-0.4, deactivate=True),
										transitions={'failed': 'failed'},
										autonomy={'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
