#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_bad')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Fri Mar 31 2017
@author: disRecord, Wave
'''
class BadSM(Behavior):
	'''
	SweetieBot is unhappy.
	'''


	def __init__(self):
		super(BadSM, self).__init__()
		self.name = 'Bad'

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
		# x:1011 y:80, x:1002 y:571
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:75 y:101
			OperatableStateMachine.add('CheckEvil',
										DecisionState(outcomes=['good', 'evil'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'good': 'RandomChoiceGood', 'evil': 'SayDoNotTouch'},
										autonomy={'good': Autonomy.Off, 'evil': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:320 y:143
			OperatableStateMachine.add('RandomChoiceGood',
										DecisionState(outcomes=['good1', 'good2'], conditions=lambda x: random.choice(['good1', 'good2'])),
										transitions={'good1': 'SayOverflow', 'good2': 'SayDizzy'},
										autonomy={'good1': Autonomy.Off, 'good2': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:450 y:508
			OperatableStateMachine.add('SayDoNotTouch',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='05donottouch'),
										transitions={'done': 'HoofStamp', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:496 y:147
			OperatableStateMachine.add('SayOverflow',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='27queue'),
										transitions={'done': 'Applause', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:747 y:153
			OperatableStateMachine.add('Applause',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'applause'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:745 y:226
			OperatableStateMachine.add('NoHeadShake',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'head_shake'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:495 y:227
			OperatableStateMachine.add('SayDizzy',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='26dizziness'),
										transitions={'done': 'NoHeadShake', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:616 y:500
			OperatableStateMachine.add('HoofStamp',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'hoof_stamp'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
