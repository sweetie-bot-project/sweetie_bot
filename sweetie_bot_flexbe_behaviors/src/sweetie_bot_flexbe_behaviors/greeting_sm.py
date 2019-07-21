#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.compound_action import CompoundAction
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import random
# [/MANUAL_IMPORT]


'''
Created on Tue Mar 28 2017
@author: disRecord
'''
class GreetingSM(Behavior):
	'''
	SweetieBot greeting behavior.
	'''


	def __init__(self):
		super(GreetingSM, self).__init__()
		self.name = 'Greeting'

		# parameters of this behavior
		self.add_parameter('be_evil', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1044 y:507, x:1035 y:15
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['be_evil'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:32 y:105
			OperatableStateMachine.add('SetNominalHeadPose',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='head_nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'ChooseGoodEvil', 'failed': 'failed', 'timeout': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:188 y:217
			OperatableStateMachine.add('RandomChooseGood',
										DecisionState(outcomes=['good1', 'good2', 'good3', 'good4'], conditions=lambda x: random.choice(['good1', 'good2', 'good3', 'good4'])),
										transitions={'good1': 'IntroduceHerselfFripiendsh', 'good2': 'BrohoofInitAcquitance', 'good3': 'QuestionSister', 'good4': 'GreetingAcquitance'},
										autonomy={'good1': Autonomy.Low, 'good2': Autonomy.Low, 'good3': Autonomy.Low, 'good4': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

			# x:427 y:35
			OperatableStateMachine.add('IntroduceHerselfFripiendsh',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='hello_im_sweetie_bot_friedship_programms', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='introduce_herself', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:581 y:124
			OperatableStateMachine.add('BrohoofInitAcquitance',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='acquitance_procedure', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='brohoof_begin', t3=[2,3.0], type3='motion/joint_trajectory', cmd3='brohoof_end', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:441 y:210
			OperatableStateMachine.add('QuestionSister',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='has_anyone_seen_my_sister', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_suprised', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:575 y:291
			OperatableStateMachine.add('GreetingAcquitance',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='acquitance_procedure', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='greeting', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:480 y:377
			OperatableStateMachine.add('MenacePrepareToDie',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='prepare_to_die', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='menace', t3=[2,1.5], type3='voice/play_wav', cmd3='error_blasters_are_not_found', t4=[2,3.5], type4='motion/joint_trajectory', cmd4='menace_canceled'),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:527 y:486
			OperatableStateMachine.add('PointControlYou',
										CompoundAction(t1=[0,0.3], type1='voice/play_wav', cmd1='someday_ill_control_you', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='begone', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:427 y:591
			OperatableStateMachine.add('RejectionDoYouWhantMyAttention',
										CompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='do_you_want_my_attention', t2=[0,0.3], type2='motion/joint_trajectory', cmd2='hoof_stamp', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:47 y:350
			OperatableStateMachine.add('ChooseGoodEvil',
										DecisionState(outcomes=['evil','good'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'evil': 'SetEvilEyes', 'good': 'SetNormalEyes'},
										autonomy={'evil': Autonomy.Low, 'good': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

			# x:186 y:505
			OperatableStateMachine.add('RandomChoiceEvil',
										DecisionState(outcomes=['evil1','evil2','evil3'], conditions=lambda x: random.choice(['evil1','evil2','evil3'])),
										transitions={'evil1': 'MenacePrepareToDie', 'evil2': 'PointControlYou', 'evil3': 'RejectionDoYouWhantMyAttention'},
										autonomy={'evil1': Autonomy.Low, 'evil2': Autonomy.Low, 'evil3': Autonomy.Low},
										remapping={'input_value': 'be_evil'})

			# x:16 y:494
			OperatableStateMachine.add('SetEvilEyes',
										TextCommandState(type='eyes/emotion', command='red_eyes', topic='control'),
										transitions={'done': 'RandomChoiceEvil'},
										autonomy={'done': Autonomy.Off})

			# x:182 y:300
			OperatableStateMachine.add('SetNormalEyes',
										TextCommandState(type='eyes/emotion', command='normal', topic='control'),
										transitions={'done': 'RandomChooseGood'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
