#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from sweetie_bot_flexbe_states.rand_head_movements import SweetieBotRandHeadMovements
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 20 2018
@author: mutronics
'''
class RBC18Part1SM(Behavior):
	'''
	RBC18 Presentation Part1
	'''


	def __init__(self):
		super(RBC18Part1SM, self).__init__()
		self.name = 'RBC18Part1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		action_button = 12
		# x:105 y:208, x:418 y:227
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.rand_head_config = {'min2356':[-0.5,0.1,-1.0,-1.0], 'max2356':[0.5,0.5,1.0,1.0]}
		_state_machine.userdata.head_pose_joints = [0.0, 0.0, 0.0, 0.0, 0.0]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:284, x:130 y:284, x:230 y:284, x:330 y:284, x:430 y:293, x:530 y:293
		_sm_waitstart_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['rand_head_config'], conditions=[
										('finished', [('WaitKey1', 'received')]),
										('failed', [('WaitKey1', 'unavailable')]),
										('finished', [('RandMovements', 'done')]),
										('failed', [('RandMovements', 'failed')])
										])

		with _sm_waitstart_0:
			# x:98 y:61
			OperatableStateMachine.add('WaitKey1',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[action_button], buffered=False, clear=False),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:335 y:61
			OperatableStateMachine.add('RandMovements',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=120, interval=[1,4], max2356=[0.3,0.3,1.5,1.5], min2356=[-0.3,-0.3,-1.5,-1.5]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'config': 'rand_head_config'})



		with _state_machine:
			# x:52 y:35
			OperatableStateMachine.add('NormalEyes',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'WaitStart'},
										autonomy={'done': Autonomy.Off})

			# x:642 y:281
			OperatableStateMachine.add('HelloGoodThenEvil',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='zdravstvuyte_dorogie_gosti', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='look_on_printer', t3=[2,0.0], type3='motion/joint_trajectory', cmd3='menace', t4=[3,4.0], type4='motion/joint_trajectory', cmd4='menace_canceled'),
										transitions={'success': 'MicrophoneTurnOffReaction', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:634 y:65
			OperatableStateMachine.add('FakeApoplogy',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='a_tochno_sboi_pamayati', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_suprised', t3=[0,2.8], type3='motion/joint_trajectory', cmd3='head_node', t4=[0,6.5], type4='motion/joint_trajectory', cmd4='hoof_knock'),
										transitions={'success': 'WaitKey3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:467 y:314
			OperatableStateMachine.add('WaitKey4',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[action_button], buffered=False, clear=False),
										transitions={'received': 'HeadTurnRightEnd', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:253 y:412
			OperatableStateMachine.add('FakeApologyAndMenace',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='no_esli_snova_otkluchish_mikrofon', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='hoof_knock', t3=[2,0.0], type3='motion/joint_trajectory', cmd3='head_node', t4=[0,6.5], type4='motion/joint_trajectory', cmd4='begone'),
										transitions={'success': 'NormalLook', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:74 y:386
			OperatableStateMachine.add('WaitKey5',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[action_button], buffered=False, clear=False),
										transitions={'received': 'LetsBegin', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:48 y:277
			OperatableStateMachine.add('LetsBegin',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='nam_pora_nachinat', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_node', t3=[1,1.0], type3='motion/joint_trajectory', cmd3='point_right', t4=[0,7.0], type4='motion/joint_trajectory', cmd4='hoof_stamp'),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:439 y:35
			OperatableStateMachine.add('WaitKey2',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[action_button], buffered=False, clear=False),
										transitions={'received': 'FakeApoplogy', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:236 y:35
			OperatableStateMachine.add('HelloEvil',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='zdravstvuyte_kojannie_meshki', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='greeting', t3=[0,0.0], type3='eyes/emotion', cmd3='red_eyes', t4=[0,5.5], type4='motion/joint_trajectory', cmd4='applause'),
										transitions={'success': 'WaitKey2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:243 y:129
			OperatableStateMachine.add('PlaceHead',
										SrdfStateToMoveit(config_name='head_basic', move_group='head', action_topic='move_group', robot_name=''),
										transitions={'reached': 'HelloEvil', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:73 y:124
			OperatableStateMachine.add('WaitStart',
										_sm_waitstart_0,
										transitions={'finished': 'PlaceHead', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'rand_head_config': 'rand_head_config'})

			# x:642 y:365
			OperatableStateMachine.add('MicrophoneTurnOffReaction',
										SweetieBotCompoundAction(t1=[0,0.0], type1='eyes/emotion', cmd1='evil_look', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_turn_right_begin', t3=[2,0.0], type3='motion/joint_trajectory', cmd3='hoof_stamp', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey4', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:231 y:312
			OperatableStateMachine.add('NormalLook',
										SweetieBotCompoundAction(t1=[0,0.0], type1='eyes/emotion', cmd1='normal', t2=[1,0.0], type2='eyes/emotion', cmd2='red_eyes', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey5', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:452 y:401
			OperatableStateMachine.add('HeadTurnRightEnd',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='head_turn_right_end', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'FakeApologyAndMenace', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:628 y:165
			OperatableStateMachine.add('WaitKey3',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[action_button], buffered=False, clear=False),
										transitions={'received': 'HelloGoodThenEvil', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
