#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_cheer')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Tue Mar 28 2017
@author: disRecord
'''
class CheerSM(Behavior):
	'''
	Sweetie cheering state.
	'''


	def __init__(self):
		super(CheerSM, self).__init__()
		self.name = 'Cheer'

		# parameters of this behavior
		self.add_parameter('be_evil', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		voice_topic = 'voice/voice'
		storage = '/stored/joint_trajectory/'
		# x:901 y:554, x:895 y:38
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:64 y:76
			OperatableStateMachine.add('CheckEvil',
										DecisionState(outcomes=['good', 'evil'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'good': 'SayMaximumFun', 'evil': 'SayDoNotTouch'},
										autonomy={'good': Autonomy.Off, 'evil': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:504 y:79
			OperatableStateMachine.add('Prance',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'prance'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:325 y:57
			OperatableStateMachine.add('SayMaximumFun',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='fun_level'),
										transitions={'done': 'Prance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:498 y:456
			OperatableStateMachine.add('HoofStamp',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage+'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:309 y:437
			OperatableStateMachine.add('SayDoNotTouch',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='05donottouch'),
										transitions={'done': 'HoofStamp', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
