#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.operator_decision_state import OperatorDecisionState
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from sweetie_bot_flexbe_behaviors.watchpresentaion_sm import WatchPresentaionSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 16 2018
@author: mutronics
'''
class Galacon2018Presentation2SM(Behavior):
	'''
	Second part of Galacon 2018 Presentation: Sweetie acts as a moderator.
	'''


	def __init__(self):
		super(Galacon2018Presentation2SM, self).__init__()
		self.name = 'Galacon2018Presentation2'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion')
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion_2')
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion_3')
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion_4')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
                
                # set maximal and minimal head rotation angle for WatchPresentation
                max2 = 0.2
                min2 = -0.7

                self.contains['WatchPresentaion'].max2 = max2
                self.contains['WatchPresentaion_2'].max2 = max2
                self.contains['WatchPresentaion_3'].max2 = max2
                self.contains['WatchPresentaion_4'].max2 = max2

                self.contains['WatchPresentaion'].min2 = min2
                self.contains['WatchPresentaion_2'].min2 = min2
                self.contains['WatchPresentaion_3'].min2 = min2
                self.contains['WatchPresentaion_4'].min2 = min2
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		# x:410 y:173, x:436 y:319
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.head_pose_joints = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		_state_machine.userdata.rand_head_config = {'min2356':[-0.5,0.1,-1.0,-1.0], 'max2356':[0.5,0.5,1.0,1.0]}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:322 y:22
			OperatableStateMachine.add('SelectPart',
										OperatorDecisionState(outcomes=['history', 'construction', 'electronics', 'software', 'ending'], hint='Select the part of presentation.', suggestion='history'),
										transitions={'history': 'WatchPresentaion', 'construction': 'WatchPresentaion_2', 'electronics': 'WatchPresentaion_3', 'software': 'WatchPresentaion_4', 'ending': 'ImHere'},
										autonomy={'history': Autonomy.Full, 'construction': Autonomy.Full, 'electronics': Autonomy.Full, 'software': Autonomy.Full, 'ending': Autonomy.Full})

			# x:580 y:313
			OperatableStateMachine.add('NewBody',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='thank_you_mutronics_then_renha_and_shiron_will_tell_about_my_future_body_proto3', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='look_on_printer_fast', t3=[0,1.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WatchPresentaion_3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:174 y:592
			OperatableStateMachine.add('Continue',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='but_we_have_to_continue_the_next_topic_is_my_electronic', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_node', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WatchPresentaion_4', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:44 y:382
			OperatableStateMachine.add('Software',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='thank_you_zuviel_i_really_need_the_new_and_more_powerful_on_board_computer', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='look_on_printer', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:36 y:163
			OperatableStateMachine.add('WaitKey13',
										WaitForMessageState(topic=joy_topic, condition=lambda x: x.buttons[12], buffered=False, clear=False),
										transitions={'received': 'TurnOff', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:31 y:243
			OperatableStateMachine.add('ImHere',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='look_on_hoof', t2=[0,6.0], type2='voice/play_wav', cmd2='hello_im_here_again_but_youre_monster_anyway', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey13', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:37 y:83
			OperatableStateMachine.add('TurnOff',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='thank_you_for_your_questions_and_attention_our_presentation_terminates_here', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='bow_begin', t3=[0,3.0], type3='motion/joint_trajectory', cmd3='bow_end', t4=[0,4.5], type4='motion/joint_trajectory', cmd4='prance'),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:391 y:593
			OperatableStateMachine.add('Interesting',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='its_very_interesting_i_cant_wait_when_my_data_can_be_transferred_inside_my_new_body', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_lean_forward_begin', t3=[0,1.0], type3='motion/joint_trajectory', cmd3='head_suprised', t4=[0,2.5], type4='motion/joint_trajectory', cmd4='head_lean_forward_end'),
										transitions={'success': 'Continue', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:561 y:20
			OperatableStateMachine.add('WatchPresentaion',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion'),
										transitions={'finished': 'Chances', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_pose_joints', 'rand_head_config': 'rand_head_config'})

			# x:557 y:211
			OperatableStateMachine.add('WatchPresentaion_2',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion_2'),
										transitions={'finished': 'NewBody', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_pose_joints', 'rand_head_config': 'rand_head_config'})

			# x:549 y:436
			OperatableStateMachine.add('WatchPresentaion_3',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion_3'),
										transitions={'finished': 'Interesting', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_pose_joints', 'rand_head_config': 'rand_head_config'})

			# x:49 y:468
			OperatableStateMachine.add('WatchPresentaion_4',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion_4'),
										transitions={'finished': 'Software', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_pose_joints', 'rand_head_config': 'rand_head_config'})

			# x:581 y:118
			OperatableStateMachine.add('Chances',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='the_chances_of_success_are_100_percent', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='little_shake_fast', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WatchPresentaion_2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
