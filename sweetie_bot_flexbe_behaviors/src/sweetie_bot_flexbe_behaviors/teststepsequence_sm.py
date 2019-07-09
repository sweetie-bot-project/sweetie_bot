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
from sweetie_bot_flexbe_states.execute_stored_trajectory_state import ExecuteStoredJointTrajectoryState
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
		# x:365 y:575, x:163 y:383
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.trajectory_param = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:135 y:91
			OperatableStateMachine.add('PrepareToWalk',
										SetCartesianPose(controller='motion/controller/stance', pose=[0.0,0.0,0.2085,0.0,0.0,0.0], frame_id='base_link_path', frame_is_moving=False, resources=['leg1','leg2','leg3','leg4'], actuator_frame_id='base_link', tolerance_lin=0.001, tolerance_ang=0.0085, timeout=10.0),
										transitions={'done': 'WalkFwd', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:286 y:172
			OperatableStateMachine.add('WalkFwd',
										ExecuteStepSequence(controller='motion/controller/step_sequence', trajectory_param='walk_fwd_40', trajectory_ns='saved_msgs/step_sequence'),
										transitions={'success': 'EndWalk', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:479 y:36
			OperatableStateMachine.add('EndWalk',
										SetCartesianPose(controller='motion/controller/stance', pose=[0.0,0.0,0.233,0.0,0.0,0.0], frame_id='base_link_path', frame_is_moving=False, resources=['leg1','leg2','leg3','leg4'], actuator_frame_id='base_link', tolerance_lin=0.001, tolerance_ang=0.0085, timeout=10.0),
										transitions={'done': 'Greeting', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:670 y:144
			OperatableStateMachine.add('Greeting',
										ExecuteStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='/saved_msgs/joint_trajectory/greeting'),
										transitions={'success': 'PrepareToWalk2', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:932 y:428
			OperatableStateMachine.add('Turn45',
										ExecuteStepSequence(controller='motion/controller/step_sequence', trajectory_param='turn_left_45', trajectory_ns='saved_msgs/step_sequence'),
										transitions={'success': 'EndWalk2', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off})

			# x:792 y:546
			OperatableStateMachine.add('EndWalk2',
										SetCartesianPose(controller='motion/controller/stance', pose=[0.0,0.0,0.2085,0.0,0.0,0.0], frame_id='base_link_path', frame_is_moving=False, resources=['leg1','leg2','leg3','leg4'], actuator_frame_id='base_link', tolerance_lin=0.001, tolerance_ang=0.0085, timeout=10.0),
										transitions={'done': 'WalkFwd', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:741 y:286
			OperatableStateMachine.add('PrepareToWalk2',
										SetCartesianPose(controller='motion/controller/stance', pose=[0.0,0.0,0.2085,0.0,0.0,0.0], frame_id='base_link_path', frame_is_moving=False, resources=['leg1','leg2','leg3','leg4'], actuator_frame_id='base_link', tolerance_lin=0.001, tolerance_ang=0.0085, timeout=10.0),
										transitions={'done': 'Turn45', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
