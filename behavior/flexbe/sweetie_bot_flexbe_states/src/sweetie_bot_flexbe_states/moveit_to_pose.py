#!/usr/bin/env python

import rospy
from roslib.message import check_type
import xml.etree.ElementTree as ET

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from std_msgs.msg import Header
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, PositionConstraint, BoundingVolume, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

'''
Created on 26.07.2017

@author: disRecord
'''

class MoveitToPose(EventState):
    '''
    Uses MoveIt to plan and move the move group to the target pose. 
    `move_group` node and `robot_description` parameter must present in current namespace.
    State depends only on `move_group` action interface.

    NOTE: sometimes lower tolerance values (e.g. 0.01 m and 0.1 rad) may cause planner to fail.

    -- move_group                string        Name of the move group to be used for planning.
        -- plan_only                            boolean         Do not perform any movements. Only plan trajectory.
    -- position_tolerance                float        Target positon tolerance in meters (default: 0.001 m).
    -- orientation_tolerance            float        Target orientation tolerance in radians (deafult: 0.1 (6 degree))

    ># pose                        object        Desired pose as geomery_msgs.msg.PoseStamped message.

    <= reached                     Target configuration has been reached (or can be reaced).
    <= planning_failed                 Failed to find a plan to the given configuration.
    <= control_failed                 Failed to move along the planned trajectory.

    '''

    def __init__(self, move_group, plan_only = False, position_tolerance = 0.001, orientation_tolerance = 0.001):
        '''
        Constructor
        '''
        super(MoveitToPose, self).__init__(outcomes=['reached', 'planning_failed', 'control_failed'],
                                                                                input_keys=['pose'])
        # store parameters and get end effector
        self._move_group = move_group
        self._position_tolerance = position_tolerance
        self._orientation_tolerance = orientation_tolerance
        self._plan_only = plan_only
        self._client = ProxyActionClient({ 'move_group' : MoveGroupAction })

        # get end effector link
        self._end_effector = None
        srdf_param = rospy.get_param("robot_description_semantic")
        srdf = ET.fromstring(srdf_param)
        for ee in srdf.iter('end_effector'):
            if ee.attrib['group'] == move_group:
                self._end_effector = ee.attrib['parent_link']
                break
        if not self._end_effector:
                    Logger.logerr('MoveitToPose: Unable to determine end effector for group `%s`.' % self._move_group)
                    raise RuntimeError('Unable to determine end effector for group `%s`.' % self._move_group)

        self._planning_failed = False
        self._control_failed = False
        self._success = False

    def on_enter(self, userdata):
        self._planning_failed = False
        self._control_failed = False
        self._success = False

        # TODO check userdata
        # if not isinstance(userdata.pose, PoseStamped):
            #Logger.logwarn('userdata.pose must be geomery_msgs.msg.PoseStamped. `%s` received' % str(type(userdata.pose)))
            #self._planning_failed = True
            #return
        check_type('pose', 'geometry_msgs/PoseStamped', userdata.pose)

        # request planing and execution
        action_goal = MoveGroupGoal()   
        # set palnning options
        action_goal.planning_options.plan_only = self._plan_only
        action_goal.planning_options.replan = False
#        action_goal.planning_options.planning_scene_diff.is_diff = True
#        action_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        # setup request
        action_goal.request.group_name = self._move_group
        action_goal.request.num_planning_attempts = 3
        action_goal.request.allowed_planning_time = 1.0
        action_goal.request.max_velocity_scaling_factor = 1.0
        action_goal.request.max_acceleration_scaling_factor = 1.0
        # start pose
        action_goal.request.start_state.is_diff = True
        # pose constraint
        goal_constraint = Constraints(name = '')
        # set target position
        constraint = PositionConstraint()
        constraint.header = Header(frame_id = userdata.pose.header.frame_id)
        constraint.link_name = self._end_effector
        constraint.constraint_region = BoundingVolume()
        constraint.constraint_region.primitives  = [ SolidPrimitive(type = SolidPrimitive.SPHERE, dimensions = [ self._position_tolerance ]) ]
        constraint.constraint_region.primitive_poses = [ Pose(position = userdata.pose.pose.position) ] 
        constraint.weight = 1.0
        goal_constraint.position_constraints.append(constraint)
        # set target orientation
        constraint = OrientationConstraint()
        constraint.header = Header(frame_id = userdata.pose.header.frame_id)
        constraint.link_name = self._end_effector
        constraint.orientation = userdata.pose.pose.orientation
        constraint.absolute_x_axis_tolerance = self._orientation_tolerance
        constraint.absolute_y_axis_tolerance = self._orientation_tolerance
        constraint.absolute_z_axis_tolerance = self._orientation_tolerance
        constraint.weight = 0.1
        goal_constraint.orientation_constraints.append(constraint)
        # add goal_constraint
        action_goal.request.goal_constraints.append(goal_constraint)
        try:
            self._client.send_goal('move_group', action_goal)
        except Exception as e:
                    Logger.logwarn('MoveitToPose: Failed to send MoveGroupAction goal for group: %s\n%s' % (self._move_group, str(e)))
                    self._planning_failed = True


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

        if self._client.has_result('move_group'):
            result = self._client.get_result('move_group')

            if result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                            Logger.logwarn('MoveitToPose: Control failed for MoveGroupAction of group: %s (error code: %s)' % (self._move_group, str(result.error_code)))
                            self._control_failed = True
                            return 'control_failed'
            elif result.error_code.val != MoveItErrorCodes.SUCCESS:
                            Logger.logwarn('MoveitToPose: Move action failed with result error code: %s' % str(result.error_code))
                            self._planning_failed = True
                            return 'planning_failed'
            else:
                            self._success = True
                            return 'reached'


    def on_stop(self):
        try:
            if self._client.is_available(self._action_topic) \
                    and not self._client.has_result(self._action_topic):
                        self._client.cancel(self._action_topic)
        except:
            # client already closed
            pass

    def on_pause(self):
        self._client.cancel(self._action_topic)

    def on_resume(self, userdata):
        self.on_enter(userdata)
