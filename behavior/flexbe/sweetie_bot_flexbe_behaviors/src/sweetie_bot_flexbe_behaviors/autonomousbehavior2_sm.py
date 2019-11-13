#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.set_cartesian_pose import SetCartesianPose
from sweetie_bot_flexbe_states.compound_action import CompoundAction
from sweetie_bot_flexbe_behaviors.dotrickswalk_sm import DoTricksWalkSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 18 2019
@author: mutronics
'''
class AutonomousBehavior2SM(Behavior):
	'''
	Autonomous Behavior 2
	'''


	def __init__(self):
		super(AutonomousBehavior2SM, self).__init__()
		self.name = 'AutonomousBehavior2'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(DoTricksWalkSM, 'DoTricksWalk')
		self.add_behavior(DoTricksWalkSM, 'DoTricksWalk_2')
		self.add_behavior(DoTricksWalkSM, 'DoTricksWalk_3')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:44 y:33
			OperatableStateMachine.add('WalkingPose',
										SetCartesianPose(controller='motion/controller/stance', pose=[0.0,0.0,0.209,0.0,0.0,0.0], frame_id='base_link_path', frame_is_moving=False, resources=['leg1','leg2','leg3','leg4'], actuator_frame_id='base_link', tolerance_lin=0.001, tolerance_ang=0.0085, timeout=10.0),
										transitions={'done': 'Stage1', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:289 y:24
			OperatableStateMachine.add('Stage1',
										CompoundAction(t1=[0,2.0], type1='motion/step_sequence', cmd1='turn_left_20_20_45', t2=[1,0.0], type2='motion/step_sequence', cmd2='turn_right_20_20_45', t3=[2,0.0], type3='motion/step_sequence', cmd3='turn_left_20_20_45', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'DoTricksWalk_3', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:464 y:25
			OperatableStateMachine.add('Stage2',
										CompoundAction(t1=[0,0.0], type1='motion/step_sequence', cmd1='turn_right_90', t2=[1,0.0], type2='motion/step_sequence', cmd2='turn_right_20_20_45', t3=[2,0.0], type3='motion/step_sequence', cmd3='walk_fwd_20', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'Stage3', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:632 y:29
			OperatableStateMachine.add('Stage3',
										CompoundAction(t1=[0,0.0], type1='motion/step_sequence', cmd1='turn_left_90', t2=[0,0.0], type2=None, cmd2='', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'Stage4', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:629 y:104
			OperatableStateMachine.add('Stage4',
										CompoundAction(t1=[0,0.5], type1='voice/play_wav', cmd1='reverse_beep', t2=[1,0.0], type2='motion/step_sequence', cmd2='walk_back_20', t3=[2,0.0], type3='motion/step_sequence', cmd3='backslide_left_20_20_80', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'DoTricksWalk_2', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:637 y:175
			OperatableStateMachine.add('Stage5',
										CompoundAction(t1=[0,0.0], type1='motion/step_sequence', cmd1='trot_fwd_40', t2=[1,0.0], type2='motion/step_sequence', cmd2='turn_right_90', t3=[2,0.0], type3='motion/step_sequence', cmd3='turn_right_45', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'Stage6', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:636 y:243
			OperatableStateMachine.add('Stage6',
										CompoundAction(t1=[0,0.0], type1='motion/step_sequence', cmd1='walk_fwd_60', t2=[1,0.0], type2='motion/step_sequence', cmd2='turn_left_45', t3=[2,0.0], type3='motion/step_sequence', cmd3='turn_left_45', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'DoTricksWalk', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:636 y:319
			OperatableStateMachine.add('Stage7',
										CompoundAction(t1=[0,0.0], type1='motion/step_sequence', cmd1='turn_left_45', t2=[1,0.0], type2='motion/step_sequence', cmd2='turn_left_20_20_45', t3=[2,0.0], type3='motion/step_sequence', cmd3='turn_left_20_20_45', t4=[3,0.0], type4='motion/step_sequence', cmd4='turn_right_20_20_45'),
										transitions={'success': 'Stage8', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:635 y:399
			OperatableStateMachine.add('Stage8',
										CompoundAction(t1=[0,0.0], type1='motion/step_sequence', cmd1='turn_right_45', t2=[1,0.0], type2='motion/step_sequence', cmd2='turn_right_90', t3=[2,0.0], type3='motion/step_sequence', cmd3='turn_right_05_00_25', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:806 y:264
			OperatableStateMachine.add('DoTricksWalk',
										self.use_behavior(DoTricksWalkSM, 'DoTricksWalk'),
										transitions={'finished': 'Stage7', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:801 y:113
			OperatableStateMachine.add('DoTricksWalk_2',
										self.use_behavior(DoTricksWalkSM, 'DoTricksWalk_2'),
										transitions={'finished': 'Stage5', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:338 y:105
			OperatableStateMachine.add('DoTricksWalk_3',
										self.use_behavior(DoTricksWalkSM, 'DoTricksWalk_3'),
										transitions={'finished': 'Stage2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
