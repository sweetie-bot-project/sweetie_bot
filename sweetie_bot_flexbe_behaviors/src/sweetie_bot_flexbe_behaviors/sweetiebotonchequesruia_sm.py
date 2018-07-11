#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from sweetie_bot_flexbe_behaviors.brohoof_sm import BrohoofSM
from sweetie_bot_flexbe_behaviors.greeting_sm import GreetingSM
from sweetie_bot_flexbe_behaviors.play_sm import PlaySM
from flexbe_states.decision_state import DecisionState
from flexbe_states.calculation_state import CalculationState
from sweetie_bot_flexbe_behaviors.switchevilmode_sm import SwitchEvilModeSM
from sweetie_bot_flexbe_states.leap_motion_monitor import LeapMotionMonitor
from sweetie_bot_flexbe_states.sweetie_bot_follow_head_pose_smart import SweetieBotFollowHeadPoseSmart
from sweetie_bot_flexbe_behaviors.bad_sm import BadSM
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.rand_head_movements import SweetieBotRandHeadMovements
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
from geometry_msgs.msg import PoseStamped
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
		self.add_parameter('hoof_shift', 0)
		self.add_parameter('play_timeout', 12)

		# references to used behaviors
		self.add_behavior(BrohoofSM, 'Brohoof')
		self.add_behavior(GreetingSM, 'Greeting')
		self.add_behavior(PlaySM, 'Play')
		self.add_behavior(SwitchEvilModeSM, 'SwitchEvil/SwitchEvilMode')
		self.add_behavior(BadSM, 'Bad')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		leap_topic = '/hmi/leap_motion/data'
		eyes_cmd_topic = 'control'
		voice_topic = 'control'
		leap_pose_topic = '/hmi/leap_motion/pose'
		# x:286 y:634, x:989 y:651
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.be_evil = False
		_state_machine.userdata.pose = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:353, x:343 y:356, x:209 y:349, x:843 y:453, x:473 y:355, x:638 y:361, x:907 y:552
		_sm_randheadmovements_0 = ConcurrencyContainer(outcomes=['object_detected', 'failed'], conditions=[
										('object_detected', [('WaitUntlObject', 'still_object')]),
										('object_detected', [('WaitUntlObject', 'moving_object')]),
										('failed', [('WaitUntlObject', 'no_object')]),
										('object_detected', [('RandHeadMoveme', 'done')]),
										('failed', [('RandHeadMoveme', 'failed')])
										])

		with _sm_randheadmovements_0:
			# x:158 y:110
			OperatableStateMachine.add('WaitUntlObject',
										LeapMotionMonitor(leap_motion_topic=leap_topic, exit_states=['still_object', 'moving_object'], pose_topic=leap_pose_topic, parameters=[2.0,0.02,0.2]),
										transitions={'no_object': 'failed', 'still_object': 'object_detected', 'moving_object': 'object_detected'},
										autonomy={'no_object': Autonomy.Off, 'still_object': Autonomy.Off, 'moving_object': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:605 y:103
			OperatableStateMachine.add('RandHeadMoveme',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=120, interval=[1,4], max2356=[0.3,0.5,1,1], min2356=[-0.3,-0.1,-1,-1]),
										transitions={'done': 'object_detected', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:353, x:357 y:355, x:671 y:353, x:140 y:353, x:237 y:356, x:530 y:353, x:742 y:441, x:730 y:353, x:452 y:353, x:732 y:406
		_sm_waitnoobjecttimeout_1 = ConcurrencyContainer(outcomes=['no_object', 'timeout', 'failure', 'too_close'], conditions=[
										('no_object', [('WaitNoObject', 'no_object')]),
										('failure', [('WaitNoObject', 'moving_object')]),
										('failure', [('WaitNoObject', 'still_object')]),
										('failure', [('FollowObject', 'failed')]),
										('timeout', [('Wait', 'done')]),
										('too_close', [('FollowObject', 'too_close')])
										])

		with _sm_waitnoobjecttimeout_1:
			# x:162 y:101
			OperatableStateMachine.add('WaitNoObject',
										LeapMotionMonitor(leap_motion_topic=leap_topic, exit_states=['no_object'], pose_topic=leap_pose_topic, parameters=[4.0,0.02,0.2]),
										transitions={'no_object': 'no_object', 'still_object': 'failure', 'moving_object': 'failure'},
										autonomy={'no_object': Autonomy.Off, 'still_object': Autonomy.Off, 'moving_object': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:463 y:95
			OperatableStateMachine.add('FollowObject',
										SweetieBotFollowHeadPoseSmart(pose_topic=leap_pose_topic, follow_joint_state_controller='joint_state_head', discomfort_time=4, neck_control_parameteres=[-0.13,0.3,0.20,0.2], deactivate=True, controlled_chains=['eyes','head']),
										transitions={'failed': 'failure', 'too_close': 'too_close'},
										autonomy={'failed': Autonomy.Off, 'too_close': Autonomy.Off})

			# x:332 y:94
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=self.play_timeout),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:353, x:130 y:353, x:330 y:366, x:737 y:355, x:898 y:596, x:899 y:486, x:900 y:533, x:902 y:443, x:568 y:345
		_sm_watchonleapmotionobject_2 = ConcurrencyContainer(outcomes=['no_object', 'still_object', 'failed', 'to_close'], output_keys=['pose'], conditions=[
										('no_object', [('WaitTillObjectMOving', 'no_object')]),
										('still_object', [('WaitTillObjectMOving', 'still_object')]),
										('failed', [('WaitTillObjectMOving', 'moving_object')]),
										('failed', [('HeadFollowsMovingObject', 'failed')]),
										('to_close', [('HeadFollowsMovingObject', 'too_close')])
										])

		with _sm_watchonleapmotionobject_2:
			# x:135 y:135
			OperatableStateMachine.add('WaitTillObjectMOving',
										LeapMotionMonitor(leap_motion_topic='/hmi/leap_motion/data', exit_states=['no_object', 'still_object'], pose_topic='/hmi/leap_motion/pose', parameters=[1.0,0.02,0.2]),
										transitions={'no_object': 'no_object', 'still_object': 'still_object', 'moving_object': 'failed'},
										autonomy={'no_object': Autonomy.High, 'still_object': Autonomy.High, 'moving_object': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:583 y:138
			OperatableStateMachine.add('HeadFollowsMovingObject',
										SweetieBotFollowHeadPoseSmart(pose_topic='/hmi/leap_motion/pose', follow_joint_state_controller='joint_state_head', discomfort_time=4.0, neck_control_parameteres=[-0.13,0.3,0.20,0.2], deactivate=True, controlled_chains=['head','eyes']),
										transitions={'failed': 'failed', 'too_close': 'to_close'},
										autonomy={'failed': Autonomy.Off, 'too_close': Autonomy.High})


		# x:979 y:146, x:969 y:327
		_sm_switchevil_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'], output_keys=['be_evil'])

		with _sm_switchevil_3:
			# x:269 y:101
			OperatableStateMachine.add('CheckState',
										DecisionState(outcomes=[ 'evil', 'good' ], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'evil': 'NewState1', 'good': 'NewState2'},
										autonomy={'evil': Autonomy.Off, 'good': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:423 y:54
			OperatableStateMachine.add('NewState1',
										CalculationState(calculation=lambda x: random.random() < 0.8),
										transitions={'done': 'SwitchEvilMode'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'be_evil', 'output_value': 'new_be_evil'})

			# x:425 y:159
			OperatableStateMachine.add('NewState2',
										CalculationState(calculation=lambda x: random.random() < 0.2),
										transitions={'done': 'SwitchEvilMode'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'be_evil', 'output_value': 'new_be_evil'})

			# x:678 y:91
			OperatableStateMachine.add('SwitchEvilMode',
										self.use_behavior(SwitchEvilModeSM, 'SwitchEvil/SwitchEvilMode'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil', 'new_be_evil': 'new_be_evil'})



		with _state_machine:
			# x:114 y:550
			OperatableStateMachine.add('PlaceHead',
										SrdfStateToMoveit(config_name='head_upright', move_group='head', action_topic='move_group', robot_name=''),
										transitions={'reached': 'RandHeadMovements', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:500 y:88
			OperatableStateMachine.add('Brohoof',
										self.use_behavior(BrohoofSM, 'Brohoof'),
										transitions={'finished': 'WaitNoObjectTimeout', 'unreachable': 'Greeting', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'unreachable': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose'})

			# x:699 y:28
			OperatableStateMachine.add('Greeting',
										self.use_behavior(GreetingSM, 'Greeting'),
										transitions={'finished': 'WaitNoObjectTimeout', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:455 y:230
			OperatableStateMachine.add('Play',
										self.use_behavior(PlaySM, 'Play'),
										transitions={'finished': 'WaitNoObjectTimeout', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:78 y:247
			OperatableStateMachine.add('SwitchEvil',
										_sm_switchevil_3,
										transitions={'finished': 'WatchOnLeapMotionObject', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:387 y:6
			OperatableStateMachine.add('CheckEvil',
										DecisionState(outcomes=['evil','good'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'evil': 'Greeting', 'good': 'Brohoof'},
										autonomy={'evil': Autonomy.Off, 'good': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:144 y:77
			OperatableStateMachine.add('WatchOnLeapMotionObject',
										_sm_watchonleapmotionobject_2,
										transitions={'no_object': 'Play', 'still_object': 'CheckEvil', 'failed': 'failed', 'to_close': 'Bad'},
										autonomy={'no_object': Autonomy.Inherit, 'still_object': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'to_close': Autonomy.Inherit},
										remapping={'pose': 'pose'})

			# x:283 y:293
			OperatableStateMachine.add('Bad',
										self.use_behavior(BadSM, 'Bad'),
										transitions={'finished': 'SwitchEvil', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:646 y:375
			OperatableStateMachine.add('WaitNoObjectTimeout',
										_sm_waitnoobjecttimeout_1,
										transitions={'no_object': 'PlaceHead', 'timeout': 'Play', 'failure': 'failed', 'too_close': 'Bad'},
										autonomy={'no_object': Autonomy.Inherit, 'timeout': Autonomy.Inherit, 'failure': Autonomy.Inherit, 'too_close': Autonomy.Inherit})

			# x:106 y:411
			OperatableStateMachine.add('RandHeadMovements',
										_sm_randheadmovements_0,
										transitions={'object_detected': 'SwitchEvil', 'failed': 'failed'},
										autonomy={'object_detected': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
