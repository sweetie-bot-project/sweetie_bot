#!/usr/bin/env python
import math

import rospy
import tf
from rospy.rostime import Time, Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyActionClient, ProxyTransformListener

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sweetie_bot_control_msgs.msg import SetOperationalAction, SetOperationalGoal, SetOperationalResult


class SetCartesianPose(EventState):
    '''
    Move element of the robot to pose using FollowStance or FollowPose controller.e
   
    This state activates controller and then publishs pose as reference position.
    Movement is considered finished when position error is less then given tolerance.

    -- controller           string          Controller namespace.
    -- ref_pose_topic       string          Reference pose topic.
    -- pose                 double[6]       Desired pose: (x,y,z,r,p,y). If set None pose input key is used.
    -- frame_id             string          Desired pose frame_id.
    -- frame_is_moving      bool            Desired pose frame should be updated constantly.
    -- resources            string[]        List of controlled kineamtic chains. 
    -- actuator_frame_id    string          frame_id of actuated part of robot. It is used to detect the end of movement. If set None detection is turned off.
    -- tolerance_lin        float           Position tolerance (m).
    -- tolerance_ang        float           Orientation tolerance (rad).
    -- timeout              float           Movement timeout (s).

    <= done 	                    Finished.
    <= failed 	                    Failed to activate FollowJointState controller.
    <= timeout 	                    Timeout reached (if 

    '''

    def __init__(self, controller = 'motion/controller/stance', 
            pose = [0.0,0.0,0.15,0.0,0.0,0.0], frame_id = 'base_link_path', frame_is_moving = False,
            resources = ['leg1','leg2','leg3','leg4'], actuator_frame_id = 'base_link',
            tolerance_lin = 0.01, tolerance_ang = 0.085, timeout = 10.0):

        if pose: 
            # desired pose is provided, input key is not required
            super(SetCartesianPose, self).__init__(outcomes = ['done', 'failed', 'timeout'])
            # init pose from parameters
            if len(pose) != 6 or any([ not isinstance(v, (int, float)) for v in pose]):
                raise ValueError("pose: list of six numbers was expected.")
            
            # set desired pose from parameteres
            self._pose = PoseStamped()
            self._pose.header.frame_id = frame_id
            self._pose.pose.position.x = pose[0]
            self._pose.pose.position.y = pose[1]
            self._pose.pose.position.z = pose[2]
            quaternion = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5]) # RPY -> Quaternion
            self._pose.pose.orientation.x = quaternion[0]
            self._pose.pose.orientation.y = quaternion[1]
            self._pose.pose.orientation.z = quaternion[2]
            self._pose.pose.orientation.w = quaternion[3]

        else:
            # desired pose should be provided via input key
            super(SetCartesianPose, self).__init__(outcomes = ['done', 'failed', 'timeout'], input_keys = ['pose'])

        # Store topic parameter for later use.
        self._controller = controller 
        self._resources = resources
        self._actuator_frame_id = actuator_frame_id 
        self._tolerance_lin = tolerance_lin
        self._tolerance_ang = tolerance_ang
        self._timeout = Duration.from_sec(timeout)
        self._pose_world = None
        self._frame_is_moving = frame_is_moving

        # create proxies
        self._action_client = ProxyActionClient({self._controller: SetOperationalAction})
        self._pose_publisher = ProxyPublisher({ self._controller + '/in_pose_ref': PoseStamped })
        # get transform listener singlenton
        self._tf = ProxyTransformListener().listener()
        self._tf.waitForTransform(self._actuator_frame_id, 'odom_combined', rospy.Time(), rospy.Duration(10.0))
        
        # timestamp
        self._timestamp = None
        # error in enter hook
        self._error = False

    
    def on_enter(self, userdata):
        self._error = False
        # get pose from key 
        if not self._pose:
            if not isinstnce(userdata.pose, PoseStamped):
                Logger.logwarn('SetCartesianPose: "pose" key must contain PoseStamped msg, but %s received' % str(type(userdata.pose)))
                self._error = True
                return
            self._pose = userdata.pose
        # activate controller
        actiavtion_request = SetOperationalGoal()
        actiavtion_request.operational = True
        actiavtion_request.resources = self._resources
        try:
            self._action_client.send_goal(self._controller, actiavtion_request)
            Logger.loginfo(str(self._action_client.get_state(self._controller)))
        except Exception as e:
            Logger.logwarn('SetCartesianPose: Failed to send the SetOperational command:\n%s' % str(e))
            self._error = True
            return
        # set start timestamp
        self._timestamp = Time.now()
        # Clear pose buffer to force target pose recalculation
        self._pose_world = None

        Logger.loginfo('SetCartesianPose started.')

    def execute(self, userdata):
        # error in start hook
        if self._error:
            return 'failed'

        # check if controller is active
        if self._action_client.has_result(self._controller):
            Logger.loginfo(str(self._action_client.get_state(self._controller)))
            Logger.loginfo(str(self._action_client.get_result(self._controller)))
            Logger.loginfo('SetCartesianPose: controller was deactivated by external cause.')
            return 'failed';

        # check if time elasped
        if Time.now() - self._timestamp > self._timeout:
            Logger.loginfo('SetCartesianPose: controller was deactivated due timeout.')
            return 'timeout'

        # publish goal pose
        # convert point coordinates to world frame (odom_combined)
        if self._pose_world == None or self._frame_is_moving:
            try:
                self._pose_world = self._tf.transformPose('odom_combined', self._pose)
            except tf.Exception as e:
                Logger.logwarn('SetCartesianPose: cannot transform %s from to odom_combined' % self._pose.header.frame_id)
                return 'failed' 
        # publish pose
        self._pose_publisher.publish(self._controller+'/in_pose_ref', self._pose_world) 

        # check tolerance
        try:
            # convert point coordinates to actuator frame
            pose_error = self._tf.transformPose(self._actuator_frame_id, self._pose)
        except tf.Exception as e:
            Logger.logwarn('SetCartesianPose: cannot transform %s from to %s' % (self._pose.header.frame_id, self._actuator_frame_id))
            return 'failed' 

        pos = pose_error.pose.position
        pos_err = math.sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z)
        angle_err = 2*math.acos(pose_error.pose.orientation.w)
        
        if pos_err < self._tolerance_lin and angle_err < self._tolerance_ang:
            Logger.loginfo('SetCartesianPose: on position')
            return 'done'

    def on_exit(self, userdata):
        if not self._action_client.has_result(self._controller):
            try: 
                self._action_client.cancel(self._controller)
            except Exception as e:
                Logger.logwarn('SetCartesianPose: failed to deactivate `' + self._controller + '` controller:\n%s' % str(e))

