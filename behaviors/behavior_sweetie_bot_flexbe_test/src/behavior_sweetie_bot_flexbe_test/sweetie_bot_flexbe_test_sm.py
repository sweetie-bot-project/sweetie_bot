#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_sweetie_bot_flexbe_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Mar 19 2017
@author: disrecord
'''
class sweetie_bot_flexbe_testSM(Behavior):
	'''
	Test behavior for AnimationStoredJointTrajectory
	'''


	def __init__(self):
		super(sweetie_bot_flexbe_testSM, self).__init__()
		self.name = 'sweetie_bot_flexbe_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:349 y:358
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:240 y:109
			OperatableStateMachine.add('TestMovement',
										AnimationStoredJointTrajectoryState(action_topic='/sweetie_bot/motion/controller/joint_trajectory', trajectory_param='/stored/joint_trajectory/dance'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
