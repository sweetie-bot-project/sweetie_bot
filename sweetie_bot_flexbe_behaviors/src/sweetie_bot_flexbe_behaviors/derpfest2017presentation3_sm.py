#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_derpfest2017presentation3')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_behaviors.watchpresentaion_sm import WatchPresentaionSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 28 2017
@author: disRecord
'''
class Derpfest2017Presentation3SM(Behavior):
	'''
	Forth part of Derpfest2017 presentation. SweetieBot is offended and tries to eliminate her creators.
	'''


	def __init__(self):
		super(Derpfest2017Presentation3SM, self).__init__()
		self.name = 'Derpfest2017Presentation3'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		moveit_action = 'move_group'
		control_topic = 'control'
		# x:156 y:182, x:526 y:284
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.head_normal_joints = [ 0.15, 0.0, -0.15, 0.0, 0.0, 0.0 ]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:89 y:62
			OperatableStateMachine.add('HeadBasicPose',
										SrdfStateToMoveit(config_name='head_upright', move_group='head', action_topic='move_group', robot_name=''),
										transitions={'reached': 'WakeUp', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:750 y:48
			OperatableStateMachine.add('HeadBasicPose2',
										SrdfStateToMoveit(config_name='head_upright', move_group='head', action_topic=moveit_action, robot_name=''),
										transitions={'reached': 'Menace', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:750 y:149
			OperatableStateMachine.add('Menace',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='turn_right', t2=[0,0.0], type2='voice/play_wav', cmd2='you_can_not_imagine', t3=[1,0.0], type3='motion/joint_trajectory', cmd3='turn_right', t4=[3,0.0], type4='motion/joint_trajectory', cmd4='menace'),
										transitions={'success': 'EyesNormal', 'failure': 'failed'},
										autonomy={'success': Autonomy.Full, 'failure': Autonomy.Off})

			# x:523 y:43
			OperatableStateMachine.add('TurnRight1',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='i_will_not_tolerate', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='turn_right', t3=[2,0.0], type3='eyes/emotion', cmd3='red_eyes', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'HeadBasicPose2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:287 y:50
			OperatableStateMachine.add('WakeUp',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='head_shake_emphasized', t2=[1,0.0], type2='motion/joint_trajectory', cmd2='head_turn_right', t3=[2,0.0], type3='voice/play_wav', cmd3='it_was_rude', t4=[2,1.6], type4=None, cmd4='i_will_not_tolearate'),
										transitions={'success': 'TurnRight1', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:752 y:298
			OperatableStateMachine.add('EyesNormal',
										TextCommandState(type='eyes/emotion', command='normal', topic=control_topic),
										transitions={'done': 'MenaceCanceled'},
										autonomy={'done': Autonomy.Full})

			# x:714 y:453
			OperatableStateMachine.add('MenaceCanceled',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='menace_canceled', t2=[1,0.0], type2='motion/joint_trajectory', cmd2='look_around', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'Wait', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:520 y:608
			OperatableStateMachine.add('Hello',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='hello_im_sweetie_bot_presentation', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='introduce_herself', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WatchPresentaion', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:768 y:606
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1),
										transitions={'done': 'Hello'},
										autonomy={'done': Autonomy.Off})

			# x:297 y:596
			OperatableStateMachine.add('WatchPresentaion',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion'),
										transitions={'finished': 'IsEnd', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_normal_joints'})

			# x:65 y:572
			OperatableStateMachine.add('IsEnd',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='is_it_over_you_promised_an_hour', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_suprised', t3=[0,6.0], type3='voice/play_wav', cmd3='i_do_not_understand_anything', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'HeadBasic3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:84 y:425
			OperatableStateMachine.add('HeadBasic3',
										SrdfStateToMoveit(config_name='head_upright', move_group='head', action_topic=moveit_action, robot_name=''),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
