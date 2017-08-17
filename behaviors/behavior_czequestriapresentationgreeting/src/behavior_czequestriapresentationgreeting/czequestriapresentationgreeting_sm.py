#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_czequestriapresentationgreeting')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 17 2017
@author: mutronics
'''
class CZequestriaPresentationGreetingSM(Behavior):
	'''
	Presentation greeting behavior for CZequestria
	'''


	def __init__(self):
		super(CZequestriaPresentationGreetingSM, self).__init__()
		self.name = 'CZequestriaPresentationGreeting'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		storage = 'joint_trajectory/'
		# x:84 y:357, x:195 y:363
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:34 y:36
			OperatableStateMachine.add('SetNormalEyes',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'SayGreeting'},
										autonomy={'done': Autonomy.Off})

			# x:40 y:171
			OperatableStateMachine.add('GreetingMove',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'greeting'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:186 y:77
			OperatableStateMachine.add('SayGreeting',
										TextCommandState(type='voice/play_wav', command='see_you_here', topic='voice/voice'),
										transitions={'done': 'GreetingMove'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
