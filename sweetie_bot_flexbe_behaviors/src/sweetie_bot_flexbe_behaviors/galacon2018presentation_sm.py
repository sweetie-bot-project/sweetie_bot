#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 16 2018
@author: mutronics
'''
class Galacon2018PresentationSM(Behavior):
	'''
	Galacon 2018 Presentation
	'''


	def __init__(self):
		super(Galacon2018PresentationSM, self).__init__()
		self.name = 'Galacon2018Presentation'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:189 y:200, x:436 y:319
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
                control_topic = 'control'
                voice_topic = 'control'
                joint_trajectory_action = 'motion/controller/joint_trajectory'
                joy_topic = '/hmi/joystick'
                moveit_action = 'move_group'
                storage = '/sweetie_bot/joint_trajectory'
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:47
			OperatableStateMachine.add('WaitKey1',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Greeting', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:234 y:46
			OperatableStateMachine.add('Greeting',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='greeting_biological_life_forms_thank_you_for_coming_to_our_presentation', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='introduce_herself', t3=[0,0.0], type3='', cmd3='', t4=[0,1.0], type4=None, cmd4=''),
										transitions={'success': 'Greeting2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:432 y:50
			OperatableStateMachine.add('Greeting2',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='look_around', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
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
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='its_not_the_first_time_that_i_travel_so_far_but_i_cant_say_that_the_flight_was_pleasant', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='begone', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:731 y:160
			OperatableStateMachine.add('NO',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='no_i_dont_like_the_way_you_turned_me_off_and_placed_me_in_a_box', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_shake_emphasized', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
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
										SweetieBotCompoundAction(t1=[0,1.2], type1='motion/joint_trajectory', cmd1='hoof_stamp', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
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
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='then_do_the_flight_mode_for_me', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='seizure', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
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
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='then_give_me_wings_thats_a_real_flight_mode', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='farewell', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey6', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:761 y:505
			OperatableStateMachine.add('WaitKey6',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Laws', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:583 y:576
			OperatableStateMachine.add('WaitKey7',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'LetsStart', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:420 y:582
			OperatableStateMachine.add('WaitKey8',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Chances', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:364 y:522
			OperatableStateMachine.add('Chances',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='the_chances_of_success_are_100_percent', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='little_shake_fast', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey9', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:249 y:577
			OperatableStateMachine.add('WaitKey9',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'NewBody', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:38 y:572
			OperatableStateMachine.add('WaitKey10',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Interesting', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:174 y:519
			OperatableStateMachine.add('NewBody',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='thank_you_mutronics_then_renha_and_shiron_will_tell_about_my_future_body_proto3', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='look_on_printer_fast', t3=[0,1.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey10', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:15 y:403
			OperatableStateMachine.add('WaitKey11',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'Software', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:14 y:458
			OperatableStateMachine.add('Continue',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='but_we_have_to_continue_the_next_topic_is_my_electronic', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_node', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey11', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:10 y:288
			OperatableStateMachine.add('WaitKey12',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'ImHere', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:8 y:346
			OperatableStateMachine.add('Software',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='thank_you_zuviel_i_really_need_the_new_and_more_powerful_on_board_computer', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='look_on_printer', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey12', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:8 y:175
			OperatableStateMachine.add('WaitKey13',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'TurnOff', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:7 y:230
			OperatableStateMachine.add('ImHere',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='look_on_hoof', t2=[0,6.0], type2='voice/play_wav', cmd2='hello_im_here_again_but_youre_monster_anyway', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey13', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:8 y:116
			OperatableStateMachine.add('TurnOff',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='thank_you_for_your_questions_and_attention_our_presentation_terminates_here', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='bow_begin', t3=[0,3.0], type3='motion/joint_trajectory', cmd3='bow_end', t4=[0,4.5], type4='motion/joint_trajectory', cmd4='prance'),
										transitions={'success': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:550 y:519
			OperatableStateMachine.add('LetsStart',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='oh_excuse_me_lets_start_our_presentation', t2=[0,0.5], type2='motion/joint_trajectory', cmd2='menace_canceled', t3=[0,4.0], type3='motion/joint_trajectory', cmd3='head_node', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey8', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:15 y:513
			OperatableStateMachine.add('Interesting',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='its_very_interesting_i_cant_wait_when_my_data_can_be_transferred_inside_my_new_body', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_lean_forward_begin', t3=[0,1.0], type3='motion/joint_trajectory', cmd3='head_suprised', t4=[0,2.5], type4='motion/joint_trajectory', cmd4='head_lean_forward_end'),
										transitions={'success': 'Continue', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:756 y:563
			OperatableStateMachine.add('Laws',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='first_those_pesky_three_laws_now_this_too_many_rules', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_suprised', t3=[0,2.0], type3='motion/joint_trajectory', cmd3='hoof_stamp', t4=[0,4.0], type4='motion/joint_trajectory', cmd4='menace'),
										transitions={'success': 'WaitKey7', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
