#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_sweetiebotonchequesruia')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.leap_motion_monitor import LeapMotionMonitor
from sweetie_bot_flexbe_states.sweetie_bot_follow_head_pose_smart import SweetieBotFollowHeadPoseSmart
from behavior_brohoof.brohoof_sm import BrohoofSM
from behavior_greeting.greeting_sm import GreetingSM
from sweetie_bot_flexbe_states.rand_head_movements import SweetieBotRandHeadMovements
from behavior_play.play_sm import PlaySM
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.calculation_state import CalculationState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Wed Aug 02 2017
@author: disRecord
'''
class SweetieBotOnChequesruiaSM(Behavior):
	'''
	SweetieBot beahavior for Cheqestria 2017. 
1. Watch after moving hand.
2. Give leg for brohoof or perform Greeting if hand is still.
3. Perform random action and fall back to random head movements if no object.
	'''


	def __init__(self):
		super(SweetieBotOnChequesruiaSM, self).__init__()
		self.name = 'SweetieBotOnChequesruia'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(BrohoofSM, 'Brohoof')
		self.add_behavior(GreetingSM, 'Greeting')
		self.add_behavior(PlaySM, 'Play')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		leap_topic = '/hmi/leap_motion/data'
		eyes_cmd_topic = 'control'
		voice_topic = 'voice/voice'
		# x:19 y:523, x:989 y:651
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.be_evil = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:979 y:146
		_sm_switchevil_0 = OperatableStateMachine(outcomes=['finished'], input_keys=['be_evil'], output_keys=['be_evil'])

		with _sm_switchevil_0:
			# x:269 y:101
			OperatableStateMachine.add('CheckState',
										DecisionState(outcomes=[ 'evil', 'good' ], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'evil': 'NewState1', 'good': 'NewState2'},
										autonomy={'evil': Autonomy.Off, 'good': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:435 y:63
			OperatableStateMachine.add('NewState1',
										DecisionState(outcomes=['evil', 'good'], conditions=lambda x: 'good' if 0.8 < random.random() else 'evil'),
										transitions={'evil': 'finished', 'good': 'NormalEyes'},
										autonomy={'evil': Autonomy.Off, 'good': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:428 y:195
			OperatableStateMachine.add('NewState2',
										DecisionState(outcomes=['good', 'evil'], conditions=lambda x: 'good' if 0.8 > random.random() else 'evil'),
										transitions={'good': 'finished', 'evil': 'RedEyes'},
										autonomy={'good': Autonomy.Off, 'evil': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:710 y:33
			OperatableStateMachine.add('NormalEyes',
										TextCommandState(topic=eyes_cmd_topic, type='eyes/emotions', command='normal_look'),
										transitions={'done': 'SetGood', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:729 y:264
			OperatableStateMachine.add('RedEyes',
										TextCommandState(topic=eyes_cmd_topic, type='eyes/emotion', command='evil_look'),
										transitions={'done': 'SetEvil', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:953 y:257
			OperatableStateMachine.add('SetEvil',
										CalculationState(calculation=lambda x: True),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'be_evil', 'output_value': 'be_evil'})

			# x:952 y:31
			OperatableStateMachine.add('SetGood',
										CalculationState(calculation=lambda x: False),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'be_evil', 'output_value': 'be_evil'})


		# x:30 y:353, x:343 y:356, x:209 y:349, x:843 y:453, x:473 y:355, x:638 y:361, x:907 y:552
		_sm_randheadmovements_1 = ConcurrencyContainer(outcomes=['object_detected', 'failed'], conditions=[
										('object_detected', [('WaitUntlObject', 'still_object')]),
										('object_detected', [('WaitUntlObject', 'moving_object')]),
										('failed', [('WaitUntlObject', 'no_object')]),
										('object_detected', [('RandHeadMoveme', 'done')]),
										('failed', [('RandHeadMoveme', 'failed')])
										])

		with _sm_randheadmovements_1:
			# x:158 y:110
			OperatableStateMachine.add('WaitUntlObject',
										LeapMotionMonitor(leap_motion_topic='hmi/leap_motion/data', exit_states=['still_object', 'moving_object'], pose_topic=None, parameters=[2.0,0.02,0.2]),
										transitions={'no_object': 'failed', 'still_object': 'object_detected', 'moving_object': 'object_detected'},
										autonomy={'no_object': Autonomy.Off, 'still_object': Autonomy.Off, 'moving_object': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:605 y:103
			OperatableStateMachine.add('RandHeadMoveme',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=120, interval=[3,5], max2356=[0.3,0.3,1.5,1.5], min2356=[-0.3,-0.3,-1.5,-1.5]),
										transitions={'done': 'object_detected', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:353, x:130 y:353, x:330 y:366, x:737 y:355, x:624 y:353, x:899 y:486, x:900 y:533, x:902 y:443
		_sm_watchonleapmotionobject_2 = ConcurrencyContainer(outcomes=['no_object', 'still_object', 'failed'], output_keys=['pose'], conditions=[
										('no_object', [('WaitTillObjectMOving', 'no_object')]),
										('still_object', [('WaitTillObjectMOving', 'still_object')]),
										('failed', [('WaitTillObjectMOving', 'moving_object')]),
										('failed', [('HeadFollowsMovingObject', 'failed')]),
										('failed', [('HeadFollowsMovingObject', 'too_close')])
										])

		with _sm_watchonleapmotionobject_2:
			# x:135 y:135
			OperatableStateMachine.add('WaitTillObjectMOving',
										LeapMotionMonitor(leap_motion_topic='/hmi/leap_motion/data', exit_states=['no_object', 'still_object'], pose_topic=/hmi/leap_motion/pose, parameters=[2.0,0.02,0.2]),
										transitions={'no_object': 'no_object', 'still_object': 'still_object', 'moving_object': 'failed'},
										autonomy={'no_object': Autonomy.Off, 'still_object': Autonomy.Off, 'moving_object': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:583 y:138
			OperatableStateMachine.add('HeadFollowsMovingObject',
										SweetieBotFollowHeadPoseSmart(pose_topic=/hmi/leap_motion/pose, follow_joint_state_controller='joint_state_head', discomfort_time=1000.0, neck_control_parameteres=[-0.13,0.3,0.20,0.2], deactivate=True, controlled_chains=['head','eyes']),
										transitions={'failed': 'failed', 'too_close': 'failed'},
										autonomy={'failed': Autonomy.Off, 'too_close': Autonomy.Off})



		with _state_machine:
			# x:144 y:77
			OperatableStateMachine.add('WatchOnLeapMotionObject',
										_sm_watchonleapmotionobject_2,
										transitions={'no_object': 'Play', 'still_object': 'CheckEvil', 'failed': 'failed'},
										autonomy={'no_object': Autonomy.Inherit, 'still_object': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose'})

			# x:500 y:88
			OperatableStateMachine.add('Brohoof',
										self.use_behavior(BrohoofSM, 'Brohoof'),
										transitions={'finished': 'Play', 'unreachable': 'Greeting', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'unreachable': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:699 y:28
			OperatableStateMachine.add('Greeting',
										self.use_behavior(GreetingSM, 'Greeting'),
										transitions={'finished': 'Play', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:94 y:440
			OperatableStateMachine.add('RandHeadMovements',
										_sm_randheadmovements_1,
										transitions={'object_detected': 'SwitchEvil', 'failed': 'failed'},
										autonomy={'object_detected': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:618 y:388
			OperatableStateMachine.add('Play',
										self.use_behavior(PlaySM, 'Play'),
										transitions={'finished': 'CheckLeapMotionObject', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:345 y:407
			OperatableStateMachine.add('CheckLeapMotionObject',
										LeapMotionMonitor(leap_motion_topic=leap_topic, exit_states=['still_object', 'moving_object', 'no_object'], pose_topic=None, parameters=[2.0,0.02,0.2]),
										transitions={'no_object': 'RandHeadMovements', 'still_object': 'Play', 'moving_object': 'WatchOnLeapMotionObject'},
										autonomy={'no_object': Autonomy.Off, 'still_object': Autonomy.Off, 'moving_object': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:120 y:247
			OperatableStateMachine.add('SwitchEvil',
										_sm_switchevil_0,
										transitions={'finished': 'WatchOnLeapMotionObject'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:387 y:6
			OperatableStateMachine.add('CheckEvil',
										DecisionState(outcomes=['evil','good'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'evil': 'Greeting', 'good': 'Brohoof'},
										autonomy={'evil': Autonomy.Off, 'good': Autonomy.Off},
										remapping={'input_value': 'be_evil'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
