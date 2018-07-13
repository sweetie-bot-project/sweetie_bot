#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from sweetie_bot_flexbe_states.execute_stored_trajectory_state import ExecuteStoredJointTrajectoryState
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
		# x:200 y:516, x:130 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SayMenace',
										TextCommandState(type='voice/play_wav', command='fear', topic='voice/voice'),
										transitions={'done': 'MoveApplause'},
										autonomy={'done': Autonomy.Off})

			# x:664 y:212
			OperatableStateMachine.add('MoveMenace',
										ExecuteStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'menace'),
										transitions={'success': 'EyesNormal', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Full, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:28 y:114
			OperatableStateMachine.add('MoveApplause',
										ExecuteStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'applause'),
										transitions={'success': 'MoveBuck', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:326 y:36
			OperatableStateMachine.add('MoveBuck',
										ExecuteStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'buck3'),
										transitions={'success': 'SetRedEyes', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:610 y:39
			OperatableStateMachine.add('SetRedEyes',
										TextCommandState(type='eyes/emotion', command='red_eyes', topic='control'),
										transitions={'done': 'MoveMenace'},
										autonomy={'done': Autonomy.Off})

			# x:684 y:336
			OperatableStateMachine.add('EyesNormal',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'MoveMenaceCanceled'},
										autonomy={'done': Autonomy.Full})

			# x:630 y:467
			OperatableStateMachine.add('MoveMenaceCanceled',
										ExecuteStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'menace_canceled'),
										transitions={'success': 'MoveHeadShake', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:340 y:480
			OperatableStateMachine.add('MoveHeadShake',
										ExecuteStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'look_around'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
