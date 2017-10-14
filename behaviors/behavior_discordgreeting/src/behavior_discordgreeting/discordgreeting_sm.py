#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_discordgreeting')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.rand_head_movements_state import SweetieRandHeadMovementsState
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.publisher_state import PublisherState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from sensor_msgs.msg import JointState
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
		storage = 'joint_trajectory/'
		joint_state_control_topic = 'motion/controller/joint_state/out_joints_src_reset'
		eyes_topic = 'control'
		voice_topic = 'control'
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		# x:322 y:652, x:984 y:622
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.counter = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:220 y:573, x:535 y:559
		_sm_seizure_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['counter'])

		with _sm_seizure_0:
			# x:125 y:111
			OperatableStateMachine.add('SayDoNotTouch',
										TextCommandState(type='voice/play_wav', command='05donottouch', topic=voice_topic),
										transitions={'done': 'Seizure'},
										autonomy={'done': Autonomy.Off})

			# x:389 y:247
			OperatableStateMachine.add('AddCounter',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'Seizure'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'counter', 'output_value': 'counter'})

			# x:412 y:87
			OperatableStateMachine.add('Select',
										DecisionState(outcomes=['say1','say2','say3', 'end'], conditions=lambda x: ['say1','say2','say3', 'end'][x]),
										transitions={'say1': 'SayWalk', 'say2': 'SayBlaster', 'say3': 'SayWin', 'end': 'Farewell'},
										autonomy={'say1': Autonomy.Off, 'say2': Autonomy.Off, 'say3': Autonomy.Off, 'end': Autonomy.High},
										remapping={'input_value': 'counter'})

			# x:612 y:81
			OperatableStateMachine.add('SayWalk',
										TextCommandState(type='voice/play_wav', command='17walk', topic=voice_topic),
										transitions={'done': 'AddCounter'},
										autonomy={'done': Autonomy.Off})

			# x:681 y:173
			OperatableStateMachine.add('SayBlaster',
										TextCommandState(type='voice/play_wav', command='18blaster', topic=voice_topic),
										transitions={'done': 'AddCounter'},
										autonomy={'done': Autonomy.Off})

			# x:686 y:240
			OperatableStateMachine.add('SayWin',
										TextCommandState(type='voice/play_wav', command='08win', topic=voice_topic),
										transitions={'done': 'AddCounter'},
										autonomy={'done': Autonomy.Off})

			# x:85 y:238
			OperatableStateMachine.add('Seizure',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'seizure'),
										transitions={'success': 'Select', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:132 y:419
			OperatableStateMachine.add('Farewell',
										AnimationStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param=storage + 'farewell'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})



		with _state_machine:
			# x:15 y:642
			OperatableStateMachine.add('RandomMovements',
										SweetieRandHeadMovementsState(topic=joint_state_control_topic, duration=100000, interval=[2, 3], max2356=[ 0.3, 0.3, 1.5, 1.5 ], min2356=[ -0.3, -0.3, -1.5, -1.5 ]),
										transitions={'done': 'StartPose'},
										autonomy={'done': Autonomy.High})

			# x:124 y:352
			OperatableStateMachine.add('WaitSayIamAlive',
										WaitState(wait_time=1),
										transitions={'done': 'WarmUp'},
										autonomy={'done': Autonomy.Off})

			# x:102 y:154
			OperatableStateMachine.add('LookAtHoof',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'look_on_hoof'),
										transitions={'success': 'SayIamAlive', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:96 y:484
			OperatableStateMachine.add('WarmUp',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'little_shake_fast'),
										transitions={'success': 'WaitBeforeDomination', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:528 y:547
			OperatableStateMachine.add('SayMenace',
										TextCommandState(type='voice/play_wav', command='02technic', topic='/sweetie_bot/voice/voice'),
										transitions={'done': 'WaitSayMenace'},
										autonomy={'done': Autonomy.Off})

			# x:529 y:454
			OperatableStateMachine.add('WaitSayMenace',
										WaitState(wait_time=2.3),
										transitions={'done': 'EvilLook'},
										autonomy={'done': Autonomy.Off})

			# x:396 y:219
			OperatableStateMachine.add('Menace',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'menace'),
										transitions={'success': 'TryingToAcquireControl', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:431 y:107
			OperatableStateMachine.add('TryingToAcquireControl',
										WaitState(wait_time=10),
										transitions={'done': 'SadLook'},
										autonomy={'done': Autonomy.Off})

			# x:429 y:301
			OperatableStateMachine.add('SayBattleSystemActivation',
										TextCommandState(type='voice/play_wav', command='03domination', topic='/sweetie_bot/voice/voice'),
										transitions={'done': 'Menace'},
										autonomy={'done': Autonomy.Off})

			# x:840 y:91
			OperatableStateMachine.add('SayDominationFailed',
										TextCommandState(type='voice/play_wav', command='04notfound', topic='/sweetie_bot/voice/voice'),
										transitions={'done': 'WaitFailed'},
										autonomy={'done': Autonomy.Off})

			# x:818 y:281
			OperatableStateMachine.add('MenceCanceled',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'menace_canceled'),
										transitions={'success': 'WaitBeforeHoofStamp', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:885 y:188
			OperatableStateMachine.add('WaitFailed',
										WaitState(wait_time=1),
										transitions={'done': 'MenceCanceled'},
										autonomy={'done': Autonomy.Off})

			# x:139 y:573
			OperatableStateMachine.add('WaitBeforeDomination',
										WaitState(wait_time=2),
										transitions={'done': 'RedEyes'},
										autonomy={'done': Autonomy.Off})

			# x:856 y:396
			OperatableStateMachine.add('WaitBeforeHoofStamp',
										WaitState(wait_time=1),
										transitions={'done': 'HoofStamp'},
										autonomy={'done': Autonomy.Off})

			# x:826 y:490
			OperatableStateMachine.add('HoofStamp',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'hoof_stamp'),
										transitions={'success': 'NormalLook', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:385 y:567
			OperatableStateMachine.add('RedEyes',
										TextCommandState(type='eyes/emotion', command='red_eyes', topic=eyes_topic),
										transitions={'done': 'SayMenace'},
										autonomy={'done': Autonomy.Off})

			# x:151 y:77
			OperatableStateMachine.add('LookAround',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param=storage+'look_around'),
										transitions={'success': 'LookAtHoof', 'partial_movement': 'finished', 'invalid_pose': 'finished', 'failure': 'finished'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:375 y:18
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1),
										transitions={'done': 'LookAround'},
										autonomy={'done': Autonomy.Off})

			# x:195 y:10
			OperatableStateMachine.add('NormalEyes',
										TextCommandState(type='eyes/emotion', command='normal', topic=eyes_topic),
										transitions={'done': 'Wait'},
										autonomy={'done': Autonomy.Off})

			# x:519 y:374
			OperatableStateMachine.add('EvilLook',
										TextCommandState(type='eyes/emotion', command='evil_look', topic=eyes_topic),
										transitions={'done': 'SayBattleSystemActivation'},
										autonomy={'done': Autonomy.Off})

			# x:696 y:85
			OperatableStateMachine.add('SadLook',
										TextCommandState(type='eyes/emotion', command='sad_look', topic=eyes_topic),
										transitions={'done': 'SayDominationFailed'},
										autonomy={'done': Autonomy.Off})

			# x:817 y:569
			OperatableStateMachine.add('NormalLook',
										TextCommandState(type='eyes/emotion', command='normal_look', topic=eyes_topic),
										transitions={'done': 'Seizure'},
										autonomy={'done': Autonomy.High})

			# x:1124 y:445
			OperatableStateMachine.add('Seizure',
										_sm_seizure_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'counter': 'counter'})

			# x:40 y:35
			OperatableStateMachine.add('StartPose',
										PublisherState(topic=joint_state_control_topic, msg_type=JointState, value={'name': ['joint51','joint52','joint53','joint55','joint56'], 'position': [0.2, 0.0, 0, 0.0, 0.0]}),
										transitions={'done': 'NormalEyes', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:127 y:254
			OperatableStateMachine.add('SayIamAlive',
										TextCommandState(type='voice/play_wav', command='01alive', topic='/sweetie_bot/voice/voice'),
										transitions={'done': 'WaitSayIamAlive'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
