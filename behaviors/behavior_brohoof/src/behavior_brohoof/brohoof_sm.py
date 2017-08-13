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
from sweetie_bot_flexbe_states.animation_stored_trajectory_state import AnimationStoredJointTrajectoryState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import math
import numpy
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
		self.add_parameter('wait_time', 4)
		self.add_parameter('neck_angle', 0.25)
		self.add_parameter('hoof_shift', 0)
		self.add_parameter('brohoof_cone', 0.78)
		self.add_parameter('brohof_upper_limit', 0.30)
		self.add_parameter('is_simplified', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

                self._tf = ProxyTransformListener().listener()
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:228 y:396, x:331 y:223, x:1010 y:659
		_state_machine = OperatableStateMachine(outcomes=['finished', 'unreachable', 'failed'], input_keys=['pose'])
		_state_machine.userdata.pose = PoseStamped(Header(frame_id = 'base_link'), Pose(Point(-0.3, -0.40, 0.25), Quaternion(0, 0, 0, 1)))
		_state_machine.userdata.joint53_pose = [ self.neck_angle ]
		_state_machine.userdata.is_simplified = self.is_simplified

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:69 y:120
			OperatableStateMachine.add('CalculateTargetPose',
										CalculationState(calculation=self.calculateTargetPoseFunc2),
										transitions={'done': 'CheckNone'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pose', 'output_value': 'pose'})

			# x:286 y:54
			OperatableStateMachine.add('CheckReacibility',
										MoveitToPose(move_group='leg1', plan_only=True, position_tolerance=0.001, orientation_tolerance=0.001),
										transitions={'reached': 'GetHeadPose', 'planning_failed': 'unreachable', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.High, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:836 y:84
			OperatableStateMachine.add('RaiseLeg',
										SrdfStateToMoveit(config_name='low_raised1', move_group='leg1', action_topic='move_group', robot_name=''),
										transitions={'reached': 'MoveLeg', 'planning_failed': 'unreachable', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:832 y:179
			OperatableStateMachine.add('MoveLeg',
										MoveitToPose(move_group='leg1', plan_only=False, position_tolerance=0.001, orientation_tolerance=0.001),
										transitions={'reached': 'SayHello', 'planning_failed': 'PutLegDownUreached', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:654 y:385
			OperatableStateMachine.add('ReturnLeg',
										SrdfStateToMoveit(config_name='low_raised1', move_group='leg1', action_topic='move_group', robot_name=''),
										transitions={'reached': 'PutLegDown', 'planning_failed': 'ReturnLeg2', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1005 y:389
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'CheckSimplified2'},
										autonomy={'done': Autonomy.Off})

			# x:360 y:393
			OperatableStateMachine.add('PutLegDown',
										SrdfStateToMoveit(config_name='stand1', move_group='leg1', action_topic='move_group', robot_name=''),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:464 y:211
			OperatableStateMachine.add('PutLegDownUreached',
										SrdfStateToMoveit(config_name='stand_front', move_group='legs_front', action_topic='move_group', robot_name=''),
										transitions={'reached': 'unreachable', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:986 y:302
			OperatableStateMachine.add('SayHello',
										TextCommandState(type='voice/play_wav', command='00irobot', topic='voice/voice'),
										transitions={'done': 'Wait'},
										autonomy={'done': Autonomy.Off})

			# x:648 y:42
			OperatableStateMachine.add('RaiseHead',
										MoveitToJointsState(move_group='head', joint_names=['joint51', 'joint52', 'joint53', 'joint54'], action_topic='move_group'),
										transitions={'reached': 'CheckSimplified', 'planning_failed': 'failed', 'control_failed': 'failed'},
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

			# x:84 y:245
			OperatableStateMachine.add('CheckNone',
										DecisionState(outcomes=['good','none'], conditions=lambda x: 'good' if x else 'none'),
										transitions={'good': 'CheckReacibility', 'none': 'unreachable'},
										autonomy={'good': Autonomy.Off, 'none': Autonomy.Off},
										remapping={'input_value': 'pose'})

			# x:514 y:495
			OperatableStateMachine.add('ReturnLeg2',
										SrdfStateToMoveit(config_name='low_raised1', move_group='leg1', action_topic='move_group', robot_name=""),
										transitions={'reached': 'PutLegDown', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:926 y:3
			OperatableStateMachine.add('CheckSimplified',
										DecisionState(outcomes=['simple', 'complex'], conditions=lambda x: 'simple' if x else 'complex'),
										transitions={'simple': 'LegStand', 'complex': 'RaiseLeg'},
										autonomy={'simple': Autonomy.Off, 'complex': Autonomy.Off},
										remapping={'input_value': 'is_simplified'})

			# x:1046 y:140
			OperatableStateMachine.add('RaiseLegProgram',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='joint_trajectory/brohoof_begin'),
										transitions={'success': 'SayHello', 'partial_movement': 'ReturnLeg', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})

			# x:1096 y:36
			OperatableStateMachine.add('LegStand',
										SrdfStateToMoveit(config_name='stand1', move_group='legq', action_topic='move_group', robot_name=''),
										transitions={'reached': 'RaiseLegProgram', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:853 y:412
			OperatableStateMachine.add('CheckSimplified2',
										DecisionState(outcomes=['simple','complex'], conditions=lambda x: 'simple' if x else 'complex'),
										transitions={'simple': 'ReturnLegProgrammed', 'complex': 'ReturnLeg'},
										autonomy={'simple': Autonomy.Off, 'complex': Autonomy.Off},
										remapping={'input_value': 'is_simplified'})

			# x:673 y:533
			OperatableStateMachine.add('ReturnLegProgrammed',
										AnimationStoredJointTrajectoryState(action_topic='motion/controller/joint_trajectory', trajectory_param='joint_trajectory/brohoof_end'),
										transitions={'success': 'finished', 'partial_movement': 'ReturnLeg', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})


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
	        Test point: PoseStamped(Header(frame_id = 'base_link'), Pose(Point(0.03, -0.20, 0.1431), Quaternion(0, 0, 0, 1)))
	        '''
	        # check if frame header presents
	        if not input_pose.header.frame_id:
	            Logger.logerr('calculateTargetPoseFunc: Header is missing.')
	            return None
	        # convert ot base_link frame
	        try:
	            output_pose = self._tf.transformPose('base_link', PoseStamped(Header(frame_id = input_pose.header.frame_id), input_pose.pose))
	        except tf.Exception as e:
	            Logger.logerr('calculateTargetPoseFunc: Cannot transform from `%s` to `base_link` frame.' % input_pose.header.frame_id)
	            return None
	        # move point forward along Oy of base_link
	        output_pose.pose.position = Point(output_pose.pose.position.x, output_pose.pose.position.y + self.hoof_shift, output_pose.pose.position.z)
	        # output_pose.pose.orientation = quaternion_about_axis([1, 0, 0], math.pi/2) * input_pose.pose.orientation
	        output_pose.pose.orientation = Quaternion(0, 0, 0, 1)
	        Logger.loginfo('calculateTargetPoseFunc: target hoof pose: %s' % str(output_pose))
	        return output_pose

	def calculateTargetPoseFunc2(self, input_pose):
	        '''
	        Calculate target pose for leg1: move in direction
	        '''
	        # check if frame header presents
	        if not input_pose.header.frame_id:
	            Logger.logerr('calculateTargetPoseFunc2: Header is missing.')
	            return None
	        # convert ot base_link frame
	        try:
	            output_pose = self._tf.transformPose('base_link', PoseStamped(Header(frame_id = input_pose.header.frame_id), input_pose.pose))
	        except tf.Exception as e:
	            Logger.logerr('calculateTargetPoseFunc2: Cannot transform from `%s` to `base_link` frame.' % input_pose.header.frame_id)
	            return None
	        pos = output_pose.pose.position
	        Logger.loginfo('calculateTargetPoseFunc2: object pos: %s' % str(pos))
	        # Set target orientation
	        if pos.z > 20:
	            # Set hoof orintation: slightly up
	            output_pose.pose.orientation = Quaternion(0.612947, -0.352558, 0.352557, 0.612945)
	        else:
	            output_pose.pose.orientation = Quaternion(0, 0, 0, 1)
	        # Set target position
	        # Check if boundarie is sane
	        if pos.z > self.brohof_upper_limit or pos.z < 0.10: 
	            # too high or to low
	            return None
	        if pos.y > -0.25:
	            # too close
                    return None
                # Move to shoulder1 point
                pos.x -= 0.041
                pos.y += 0.034
                # Check if angle is out of 45 cone
                if abs(math.atan2(pos.x,-pos.y)) > self.brohoof_cone:
                    # out of cone
                    return None
	        # Project x coordinate to y = -0.225+0.034 in coordinates of leg1 base
	        pos.x = pos.x*((-0.225+0.034)/pos.y)
                # Return to base_link frame 
                pos.x += 0.041
	        pos.y = -0.225
	        pos.z = 0.172
	        # Clamp to reachibility zone
                # pos.x = numpy.clip(pos.x, -0.03, 0.12)
                pos.x = numpy.clip(pos.x, -0.01, 0.12)
	        # done: pose must be reachable
	        Logger.loginfo('calculateTargetPoseFunc2: target hoof pose: %s' % str(output_pose))
	        return output_pose
    # [/MANUAL_FUNC]
