#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_discordgreeting')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Mar 25 2017
@author: disRecord
'''
class DiscordGreetingSM(Behavior):
	'''
	Little scene play with Discord and Sweetie.
	'''


	def __init__(self):
		super(DiscordGreetingSM, self).__init__()
		self.name = 'DiscordGreeting'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:322 y:652, x:984 y:622
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:67 y:44
			OperatableStateMachine.add('LookAround',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='/hmi/trajectory_storage/look_around'),
										transitions={'success': 'LookAtHoof', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:127 y:254
			OperatableStateMachine.add('SayIamAlive',
										TextCommandState(topic='/sweetie_bot/voice/voice', type='voice/play_wav', command='01alive'),
										transitions={'done': 'WaitSayIamAlive', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:124 y:352
			OperatableStateMachine.add('WaitSayIamAlive',
										WaitState(wait_time=1),
										transitions={'done': 'WarmUp'},
										autonomy={'done': Autonomy.Off})

			# x:102 y:154
			OperatableStateMachine.add('LookAtHoof',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='/hmi/trajectory_storage/look_on_hoof'),
										transitions={'success': 'SayIamAlive', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:96 y:484
			OperatableStateMachine.add('WarmUp',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='/hmi/trajectory_storage/little_shake_fast'),
										transitions={'success': 'WaitBeforeDomination', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:518 y:493
			OperatableStateMachine.add('SayMenace',
										TextCommandState(topic='/sweetie_bot/voice/voice', type='voice/play_wav', command='02technic'),
										transitions={'done': 'WaitSayMenace', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:522 y:390
			OperatableStateMachine.add('WaitSayMenace',
										WaitState(wait_time=2.3),
										transitions={'done': 'SayBattleSystemActivation'},
										autonomy={'done': Autonomy.Off})

			# x:487 y:225
			OperatableStateMachine.add('Menace',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='/hmi/trajectory_storage/menace'),
										transitions={'success': 'TryingToAcquireControl', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:511 y:105
			OperatableStateMachine.add('TryingToAcquireControl',
										WaitState(wait_time=10),
										transitions={'done': 'SayDominationFailed'},
										autonomy={'done': Autonomy.Off})

			# x:501 y:302
			OperatableStateMachine.add('SayBattleSystemActivation',
										TextCommandState(topic='/sweetie_bot/voice/voice', type='voice/play_wav', command='03domination'),
										transitions={'done': 'Menace', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:840 y:91
			OperatableStateMachine.add('SayDominationFailed',
										TextCommandState(topic='/sweetie_bot/voice/voice', type='voice/play_wav', command='04notfound'),
										transitions={'done': 'WaitFailed', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:818 y:281
			OperatableStateMachine.add('MenceCanceled',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='/hmi/trajectory_storage/menace_canceled'),
										transitions={'success': 'WaitBeforeHoofStamp', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:885 y:188
			OperatableStateMachine.add('WaitFailed',
										WaitState(wait_time=1),
										transitions={'done': 'MenceCanceled'},
										autonomy={'done': Autonomy.Off})

			# x:321 y:553
			OperatableStateMachine.add('WaitBeforeDomination',
										WaitState(wait_time=4),
										transitions={'done': 'SayMenace'},
										autonomy={'done': Autonomy.Off})

			# x:856 y:396
			OperatableStateMachine.add('WaitBeforeHoofStamp',
										WaitState(wait_time=1),
										transitions={'done': 'HoofStamp'},
										autonomy={'done': Autonomy.Off})

			# x:826 y:490
			OperatableStateMachine.add('HoofStamp',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='/hmi/trajectory_storage/hoof_stamp'),
										transitions={'success': 'failed', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
