#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_czequestriapresentationmenace')
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
class CZequestriaPresentationMenaceSM(Behavior):
	'''
	State machines menace for CZequestria presentation
	'''


	def __init__(self):
		super(CZequestriaPresentationMenaceSM, self).__init__()
		self.name = 'CZequestriaPresentationMenace'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		storage = 'joint_trajectory/'
		# x:30 y:322, x:130 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SayMenace',
										TextCommandState(type='voice/play_wav', command='fear', topic='voice/voice'),
										transitions={'done': 'MovmMenace'},
										autonomy={'done': Autonomy.Off})

			# x:30 y:110
			OperatableStateMachine.add('MovmMenace',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'menace'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
