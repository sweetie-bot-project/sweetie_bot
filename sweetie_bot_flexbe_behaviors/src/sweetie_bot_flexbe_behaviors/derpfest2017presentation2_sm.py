#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
from sweetie_bot_flexbe_behaviors.watchpresentaion_sm import WatchPresentaionSM
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 21 2017
@author: disRecord
'''
class Derpfest2017Presentation2SM(Behavior):
	'''
	Second part of presentation: construction.
	'''


	def __init__(self):
		super(Derpfest2017Presentation2SM, self).__init__()
		self.name = 'Derpfest2017Presentation2'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion_3')
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion_4')
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion')
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = 'hmi/joystick'
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		# x:44 y:403, x:375 y:319
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.head_turned_right_joints = [0.15, 0.6, -0.15, 0.0, 0.0, 0.0]
		_state_machine.userdata.head_forward_joints = [0.15, 0.0, -0.15, 0.0, 0.0, 0.0]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:71 y:228
			OperatableStateMachine.add('PlaceHead',
										MoveitToJointsState(move_group='head', joint_names=['joint51', 'joint52', 'joint53', 'joint54'], action_topic='move_group'),
										transitions={'reached': 'WatchPresentaion', 'planning_failed': 'PlaceHead', 'control_failed': 'finished'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'head_turned_right_joints'})

			# x:433 y:67
			OperatableStateMachine.add('SupprisedByProto3',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='do_i_really_be_same', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_suprised', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WatchPresentaion_2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:678 y:447
			OperatableStateMachine.add('DoNotUndestandAnything',
										SweetieBotCompoundAction(t1=[0,0.5], type1='voice/play_wav', cmd1='i_dont_understand', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_shake', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WatchPresentaion_4', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:674 y:329
			OperatableStateMachine.add('WatchPresentaion_3',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion_3', default_keys=['rand_head_config']),
										transitions={'finished': 'DoNotUndestandAnything', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_forward_joints', 'rand_head_config': 'rand_head_config'})

			# x:672 y:574
			OperatableStateMachine.add('WatchPresentaion_4',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion_4', default_keys=['rand_head_config']),
										transitions={'finished': 'AskToContinue ', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_turned_right_joints', 'rand_head_config': 'rand_head_config'})

			# x:439 y:569
			OperatableStateMachine.add('AskToContinue ',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='what_else_intresting_can_you_tell', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='hoof_wave', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'Wait', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:293 y:566
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=4.0),
										transitions={'done': 'What'},
										autonomy={'done': Autonomy.Off})

			# x:96 y:566
			OperatableStateMachine.add('What',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='what_i_dont', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_suprised_aborted', t3=[2,0.0], type3='eyes/emotion', cmd3='blank', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:667 y:202
			OperatableStateMachine.add('ILikeMyMane',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='i_really_like_my_mane', t2=[0,0.5], type2='motion/joint_trajectory', cmd2='head_node', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WatchPresentaion_3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:211 y:42
			OperatableStateMachine.add('WatchPresentaion',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion', default_keys=['rand_head_config']),
										transitions={'finished': 'SupprisedByProto3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_turned_right_joints', 'rand_head_config': 'rand_head_config'})

			# x:667 y:64
			OperatableStateMachine.add('WatchPresentaion_2',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion_2', default_keys=['rand_head_config']),
										transitions={'finished': 'ILikeMyMane', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_turned_right_joints', 'rand_head_config': 'rand_head_config'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
