#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_brohoof')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.calculation_state import CalculationState
from sweetie_bot_flexbe_states.moveit_to_pose import MoveitToPose
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from flexbe_manipulation_states.get_joint_values_state import GetJointValuesState
from flexbe_states.decision_state import DecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import math
import tf
from tf.transformations import quaternion_about_axis
from flexbe_core.proxy import ProxyTransformListener

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Wed Aug 02 2017
@author: disRecord
'''
class BrohoofSM(Behavior):
	'''
	Move leg1 up to specified target, perform greeting, wait and pt leg down. 

Robot is assumed to be standing on four legs.
	'''


	def __init__(self):
		super(BrohoofSM, self).__init__()
		self.name = 'Brohoof'

		# parameters of this behavior
		self.add_parameter('wait_time', 3)
		self.add_parameter('neck_angle', 0.25)
		self.add_parameter('hoof_shift', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

                self._tf = ProxyTransformListener().listener()
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:321 y:391, x:319 y:225, x:1010 y:659
		_state_machine = OperatableStateMachine(outcomes=['finished', 'unreachable', 'failed'], input_keys=['pose'])
		_state_machine.userdata.pose = PoseStamped(Header(frame_id = 'base_link'), Pose(Point(0.042, -0.200, 0.108), Quaternion(0.1137, -0.0058, 0.0081, 0.9934)))
		_state_machine.userdata.joint53_pose = [ self.neck_angle ]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:109 y:195
			OperatableStateMachine.add('CheckPoseHeader',
										DecisionState(outcomes=['good','bad'], conditions=lambda x: 'good' if x.header.frame_id else 'bad'),
										transitions={'good': 'CalculateTargetPose', 'bad': 'unreachable'},
										autonomy={'good': Autonomy.Off, 'bad': Autonomy.Off},
										remapping={'input_value': 'pose'})

			# x:286 y:54
			OperatableStateMachine.add('CheckReacibility',
										MoveitToPose(move_group='leg1', plan_only=True, position_tolerance=0.001, orientation_tolerance=0.001),
										transitions={'reached': 'GetHeadPose', 'planning_failed': 'unreachable', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:870 y:62
			OperatableStateMachine.add('RaiseLeg',
										SrdfStateToMoveit(config_name='low_raised1_shift', move_group='legs_front', action_topic='move_group', robot_name=''),
										transitions={'reached': 'MoveLeg', 'planning_failed': 'unreachable', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:881 y:154
			OperatableStateMachine.add('MoveLeg',
										MoveitToPose(move_group='leg1', plan_only=False, position_tolerance=0.001, orientation_tolerance=0.001),
										transitions={'reached': 'SayHello', 'planning_failed': 'PutLegDownUreached', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:654 y:385
			OperatableStateMachine.add('ReturnLeg',
										SrdfStateToMoveit(config_name='low_raised1_shift', move_group='legs_front', action_topic='move_group', robot_name=''),
										transitions={'reached': 'PutLegDown', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:910 y:391
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'ReturnLeg'},
										autonomy={'done': Autonomy.Off})

			# x:470 y:389
			OperatableStateMachine.add('PutLegDown',
										SrdfStateToMoveit(config_name='stand_front', move_group='legs_front', action_topic='move_group', robot_name=''),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:464 y:211
			OperatableStateMachine.add('PutLegDownUreached',
										SrdfStateToMoveit(config_name='stand_front', move_group='legs_front', action_topic='move_group', robot_name=''),
										transitions={'reached': 'unreachable', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:877 y:246
			OperatableStateMachine.add('SayHello',
										TextCommandState(type='voice/play_wav', command='00irobot', topic='voice/voice'),
										transitions={'done': 'Wait'},
										autonomy={'done': Autonomy.Off})

			# x:648 y:42
			OperatableStateMachine.add('RaiseHead',
										MoveitToJointsState(move_group='head', joint_names=['joint51', 'joint52', 'joint53', 'joint54'], action_topic='move_group'),
										transitions={'reached': 'RaiseLeg', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'head_pose'})

			# x:461 y:3
			OperatableStateMachine.add('GetHeadPose',
										GetJointValuesState(joints=['joint51', 'joint52', 'joint53', 'joint54']),
										transitions={'retrieved': 'CalcRaisedHeadPose'},
										autonomy={'retrieved': Autonomy.Off},
										remapping={'joint_values': 'head_pose'})

			# x:459 y:94
			OperatableStateMachine.add('CalcRaisedHeadPose',
										CalculationState(calculation=self.calculateHeadPoseFunc),
										transitions={'done': 'RaiseHead'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'head_pose', 'output_value': 'head_pose'})

			# x:109 y:58
			OperatableStateMachine.add('CalculateTargetPose',
										CalculationState(calculation=self.calculateTargetPoseFunc),
										transitions={'done': 'CheckReacibility'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pose', 'output_value': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
        def calculateHeadPoseFunc(self, joints):
            joints = list(joints)
            joints[0] = 0.0
            joints[2] = self.neck_angle
            return joints

        def calculateTargetPoseFunc(self, input_pose):
            '''
            Calculate target pose for leg1. 
            '''
            try:
                output_pose = self._tf.transformPose('base_link', PoseStamped(Header(frame_id = input_pose.header.frame_id), input_pose.pose))
            except tf.Exception as e:
                Logger.logerr('Cannot transform from `%s` to `base_link` frame.' % input_pose.header.frame_id)
                return None

            # move point forward along Oy of base_link
            output_pose.pose.position = Point(input_pose.pose.position.x, input_pose.pose.position.y + self.hoof_shift, input_pose.pose.position.z)
            # output_pose.pose.orientation = quaternion_about_axis([1, 0, 0], math.pi/2) * input_pose.pose.orientation
            output_pose.pose.orientation = Quaternion(0, 0, 0, 1)
            Logger.loginfo('calculateTargetPoseFunc: target hoof pose: %s' % str(output_pose))
            return output_pose

	
	# [/MANUAL_FUNC]
