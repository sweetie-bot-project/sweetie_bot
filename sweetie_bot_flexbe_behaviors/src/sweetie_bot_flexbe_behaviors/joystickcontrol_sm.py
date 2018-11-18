#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.joystick_joint_control import JoystickJointControl
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

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
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:120 y:73
			OperatableStateMachine.add('JoystickControl',
										JoystickJointControl(joints_topic='/joint_states', goal_joints_topic='/motion/controller/joint_state/out_joints_src_reset', joy_topic='/hmi/joystick', buttons=[(5,0.3,'head_joint4',-1.0,1.0),(7,-0.3,'head_joint4',-1.0,1.0),(4,0.3,'head_joint2',-0.5,0.9),(6,-0.3,'head_joint2',-0.5,0.9)], axes=[(0,0,'eyes_yaw',-1.5,1.5),(1,0,'eyes_pitch',-1.5,1.5)], timeout=5.0),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
