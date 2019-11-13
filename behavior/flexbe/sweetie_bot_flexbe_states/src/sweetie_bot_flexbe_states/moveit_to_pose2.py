#!/usr/bin/env python

import rospy
from copy import deepcopy
from moveit_commander import MoveGroupCommander
from moveit_commander.exception import MoveItCommanderException

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal, MoveItErrorCodes
from geometry_msgs.msg import Pose, PoseStamped

from proxy.proxy_moveit_commander import ProxyMoveitCommander

'''
Created on 26.07.2017

@author: disRecord
'''

class MoveitToPose2(EventState):
	'''
	Uses MoveIt to plan and move the move group to the target pose. 
        `move_group` node and `robot_description` parameter must present in current namespace.
        `MoveGroupCommander` is used to plan trajectory. 

	-- move_group		string		Name of the move group to be used for planning.

	># pose		        object		Desired pose as geomery_msgs.msg.PoseStamped message.
								
	<= reached 					Target configuration has been reached.
	<= planning_failed 				Failed to find a plan to the given configuration.
	<= control_failed 				Failed to move along the planned trajectory.

	'''

	def __init__(self, move_group):
		'''
		Constructor
		'''
		super(MoveitToPose2, self).__init__(outcomes=['reached', 'planning_failed', 'control_failed'],
											input_keys=['pose'])
		self._commander = ProxyMoveitCommander()
		self._group = self._commander.getMoveGroupCommander(move_group)
                self._client = ProxyActionClient({ 'execute_trajectory' : ExecuteTrajectoryAction })

		self._planning_failed = False
		self._control_failed = False
		self._success = False
		
	def on_enter(self, userdata):
		self._planning_failed = False
		self._control_failed = False
		self._success = False

                # perform planing using moveit_commander
                try:
                    # start state
                    self._group.set_start_state_to_current_state()
                    # clear old targets and constraints
                    self._group.clear_pose_targets()
                    self._group.clear_path_constraints()
                    # set target
                    # TODO set_joint_value_target is not working
                    # TODO check userdata.pose type
                    pose = PoseStamped(header = userdata.pose.header, pose = userdata.pose.pose)
                    # self._group.set_joint_value_target(pose, True)
                    self._group.set_pose_target(pose)
                    # plan 
                    robot_trajectory = self._group.plan()
                except MoveItCommanderException as e:
                    Logger.logwarn('Unable to plan trajectory for %s:\n%s' % (self._group.get_name(), str(e)))
                    self._planning_failed = True
                    return

                # another way to check failed planning?
                if not robot_trajectory or (len(robot_trajectory.joint_trajectory.points) == 0 and len(robot_trajectory.multi_dof_joint_trajectory.points) == 0):
                    Logger.logwarn('Unable to plan trajectory: move_commander.plan() has returned empty trajectory.')
                    self._planning_failed = True
                    return

                # request trajectory execution
		action_goal = ExecuteTrajectoryGoal()
		action_goal.trajectory = robot_trajectory
		try:
			self._client.send_goal('execute_trajectory', action_goal)
		except Exception as e:
                    Logger.logwarn('Failed to send ExecuteTrajectory goal for group: %s\n%s' % (self._group.get_name(), str(e)))
                    self._control_failed = True
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._planning_failed:
			return 'planning_failed'
		if self._control_failed:
			return 'control_failed'
		if self._success:
			return 'reached'

		if self._client.has_result('execute_trajectory'):
			result = self._client.get_result('execute_trajectory')
			
			if result.error_code.val != MoveItErrorCodes.SUCCESS:
				Logger.logwarn('ExecuteTrajectory action failed with result error code: %s' % str(result.error_code))
				self._control_failed = True
				return 'control_failed'
			else:
				self._success = True
				return 'reached'

	def on_stop(self):
		try:
			if self._client.is_available('execute_trajectory') \
			and not self._client.has_result('execute_trajectory'):
				self._client.cancel('execute_trajectory')
		except:
			# client already closed
			pass

	def on_pause(self):
		self._client.cancel('execute_trajectory')

	def on_resume(self, userdata):
		self.on_enter(userdata)
