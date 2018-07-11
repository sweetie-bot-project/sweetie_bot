#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

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
class CZequestriaPresentationStateMachinesSM(Behavior):
	'''
	State machines move for CZequestria presentation
	'''


	def __init__(self):
		super(CZequestriaPresentationStateMachinesSM, self).__init__()
		self.name = 'CZequestriaPresentationStateMachines'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		storage = 'joint_trajectory/'
		# x:879 y:265, x:922 y:85
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:82 y:127
			OperatableStateMachine.add('SetRedEyes',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'SayStateMachines'},
										autonomy={'done': Autonomy.Off})

			# x:478 y:140
			OperatableStateMachine.add('StateMachinesMove',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:295 y:138
			OperatableStateMachine.add('SayStateMachines',
										TextCommandState(type='voice/play_wav', command='state_machines', topic='control'),
										transitions={'done': 'StateMachinesMove'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
