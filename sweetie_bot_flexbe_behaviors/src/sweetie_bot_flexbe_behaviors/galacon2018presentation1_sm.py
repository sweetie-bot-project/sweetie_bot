#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_behaviors.watchpresentaion_sm import WatchPresentaionSM
from sweetie_bot_flexbe_states.compound_action_state import CompoundAction
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 16 2018
@author: mutronics
'''
class Galacon2018Presentation1SM(Behavior):
	'''
	First part of Galacon 2018 Presentation: the conversation with Sweetie about Galacon and the flight.
	'''


	def __init__(self):
		super(Galacon2018Presentation1SM, self).__init__()
		self.name = 'Galacon2018Presentation1'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		# x:164 y:331, x:436 y:319
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.head_pose_joints = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
		_state_machine.userdata.rand_head_config = {'min2356':[-0.5,0.1,-1.0,-1.0], 'max2356':[0.5,0.5,1.0,1.0]}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:37 y:183
			OperatableStateMachine.add('WatchPresentaion',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion'),
										transitions={'finished': 'Greeting', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_pose_joints', 'rand_head_config': 'rand_head_config'})

			# x:432 y:50
			OperatableStateMachine.add('Greeting2',
										CompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='head_lean_forward_begin', t2=[1,0.5], type2='motion/joint_trajectory', cmd2='head_lean_forward_end', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:598 y:47
			OperatableStateMachine.add('WaitKey2',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Travel', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:753 y:49
			OperatableStateMachine.add('Travel',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='its_not_the_first_time_that_i_travel_so_far_but_i_cant_say_that_the_flight_was_pleasant', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='begone', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:731 y:160
			OperatableStateMachine.add('NO',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='no_i_dont_like_the_way_you_turned_me_off_and_placed_me_in_a_box', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_shake_emphasized', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'NO2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:762 y:106
			OperatableStateMachine.add('WaitKey3',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'NO', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:733 y:218
			OperatableStateMachine.add('NO2',
										CompoundAction(t1=[0,1.2], type1='motion/joint_trajectory', cmd1='hoof_stamp', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey4', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:761 y:276
			OperatableStateMachine.add('WaitKey4',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'FlightMode', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:725 y:334
			OperatableStateMachine.add('FlightMode',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='then_do_the_flight_mode_for_me', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='seizure', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey5', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:762 y:390
			OperatableStateMachine.add('WaitKey5',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'SayWings', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:729 y:447
			OperatableStateMachine.add('SayWings',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='then_give_me_wings_thats_a_real_flight_mode', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='farewell', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey6', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:761 y:505
			OperatableStateMachine.add('WaitKey6',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Laws', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:356 y:566
			OperatableStateMachine.add('WaitKey7',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'EyesNormal', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:109 y:434
			OperatableStateMachine.add('LetsStart',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='oh_excuse_me_lets_start_our_presentation', t2=[0,0.5], type2='motion/joint_trajectory', cmd2='menace_canceled', t3=[0,4.0], type3='motion/joint_trajectory', cmd3='head_node', t4=[0,6.0], type4='motion/joint_trajectory', cmd4='point_right'),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:756 y:563
			OperatableStateMachine.add('Laws',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='first_those_pesky_three_laws_now_this_too_many_rules', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_suprised', t3=[0,2.0], type3='motion/joint_trajectory', cmd3='hoof_stamp', t4=[0,4.0], type4=None, cmd4='menace'),
										transitions={'success': 'Menace', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:563 y:567
			OperatableStateMachine.add('Menace',
										CompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='menace', t2=[1,0.0], type2='eyes/emotion', cmd2='red_eyes', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey7', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:243 y:47
			OperatableStateMachine.add('Greeting',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='greeting_biological_life_forms_thank_you_for_coming_to_our_presentation', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='introduce_herself', t3=[2,0.0], type3='motion/joint_trajectory', cmd3='look_around', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'Greeting2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:129 y:564
			OperatableStateMachine.add('EyesNormal',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'LetsStart'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
