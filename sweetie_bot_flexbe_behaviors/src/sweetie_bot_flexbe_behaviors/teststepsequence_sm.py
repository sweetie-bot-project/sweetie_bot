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
from sweetie_bot_flexbe_states.execute_step_sequence import ExecuteStepSequence
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.compound_action import CompoundAction
from sweetie_bot_flexbe_states.generate_step_sequence import GenerateStepSequence
from sweetie_bot_flexbe_states.compound_action_param import CompoundActionParam
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 07 2019
@author: disRecord
'''
class TestStepSequenceSM(Behavior):
	'''
	Test behavior for SetCartesianPose and ExecuteStepSequence states
	'''


	def __init__(self):
		super(TestStepSequenceSM, self).__init__()
		self.name = 'TestStepSequence'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:821 y:618, x:163 y:383
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.unused = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:88 y:87
			OperatableStateMachine.add('PrepareToWalk',
										SetCartesianPose(controller='motion/controller/stance', pose=[0.0,0.0,0.2085,0.0,0.0,0.0], frame_id='base_link_path', frame_is_moving=False, resources=['leg1','leg2','leg3','leg4'], actuator_frame_id='base_link', tolerance_lin=0.001, tolerance_ang=0.0085, timeout=10.0),
										transitions={'done': 'WalkFwd', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:319 y:31
			OperatableStateMachine.add('WalkFwd',
										ExecuteStepSequence(controller='motion/controller/step_sequence', trajectory_param='walk_fwd_40', trajectory_ns='saved_msgs/step_sequence'),
										transitions={'success': 'GenerateTurnLeft', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:461 y:256
			OperatableStateMachine.add('EndWalk',
										SetJointState(controller='motion/controller/joint_state_head', pose_param='nominal', pose_ns='saved_msgs/joint_state', tolerance=0.017, timeout=10.0, joint_topic="joint_states"),
										transitions={'done': 'Greeting', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:654 y:319
			OperatableStateMachine.add('Greeting',
										ExecuteJointTrajectory(action_topic='motion/controller/joint_trajectory', trajectory_param='greeting', trajectory_ns='saved_msgs/joint_trajectory'),
										transitions={'success': 'CompoundCrouchAndWalk', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:671 y:447
			OperatableStateMachine.add('CompoundCrouchAndWalk',
										CompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='crouch_begin', t2=[1,0.0], type2='motion/step_sequence', cmd2='walk_fwd_40', t3=[2,0.0], type3='generate/step_sequence', cmd3='to_nominal', t4=[3,0.0], type4='set/joint_state', cmd4='nominal'),
										transitions={'success': 'ComponudActionFromParam', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:498 y:89
			OperatableStateMachine.add('GenerateTurnLeft',
										GenerateStepSequence(controller='clop_generator', trajectory_param='turn_left_45', trajectory_ns='saved_msgs/move_base'),
										transitions={'success': 'EndWalk', 'solution_not_found': 'failed', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'solution_not_found': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:507 y:546
			OperatableStateMachine.add('ComponudActionFromParam',
										CompoundActionParam(action_param='brohoof', action_ns='saved_msgs/compound_action'),
										transitions={'success': 'finished', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
