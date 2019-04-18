"""Helper module for invoking sweetie_bot_clop_generator using actionlib."""

import rospy
import actionlib
from rospy.rostime import Time
from tf.transformations import quaternion_from_euler

import std_msgs.msg
import geometry_msgs.msg
from sweetie_bot_clop_generator.msg import MoveBaseAction, EndEffectorGoal
from sweetie_bot_clop_generator.msg import MoveBaseGoal as MoveBaseGoalBase

class MoveBaseGoal(MoveBaseGoalBase):
    """Extends sweetie_bot_clop_generator.msg.MoveBaseGoal."""

    def __init__(self, gait_type = "walk_overlap", n_steps = 4, duration = 4, nominal_height = 0.1825):
        """Create MoveBaseGoal message with default field values."""

        super(MoveBaseGoal, self).__init__()

        # create gait generation request
        self.header = std_msgs.msg.Header()
        self.header.frame_id = "base_link_path"

        self.duration = duration;
        self.n_steps = n_steps;
        self.gait_type = gait_type

        self.base_goal = geometry_msgs.msg.Pose()
        self.base_goal.position.x = 0.3
        self.base_goal.position.y = 0.0
        self.base_goal.position.z = nominal_height
        self.base_goal.orientation.x = 0.0
        self.base_goal.orientation.y = 0.0
        self.base_goal.orientation.z = 0.0
        self.base_goal.orientation.w = 1.0

        self.position_tolerance = 0.07
        self.orientation_tolerance = 0.50

    def setTargetBaseShift(self, x, y, angle):
        """Set target base pose in path coordinate system ("base_link_path" frame).

        The origing of path coordinate system is always under robot base,
        x-axis is directed forward along the robot position and z-axis 
        is always directed upward.
        
        Keyword arguments:
        x -- how much robot should shift forward (along x-axis) in meters,
        y -- how much robot should shift left (along y-axis),
        angle -- base rotation around z-axis in radians.

        """
        self.header.frame_id = "base_link_path"
        self.base_goal.position.x = x
        self.base_goal.position.y = y
        q = quaternion_from_euler(0.0, 0.0, angle)
        self.base_goal.orientation.x = q[0]
        self.base_goal.orientation.y = q[1]
        self.base_goal.orientation.z = q[2]
        self.base_goal.orientation.w = q[3]

    def setTargetBasePose(self, x, y, angle):
        """Set target base pose in absolute coordinate system ("odom_combined" frame).

        Keyword arguments:
        x -- position along x-axis (meters),
        y -- position along y-axis,
        angle -- base orientation (rotation around z-axis).

        """
        self = self.msg
        self.header.frame_id = "odom_combined"
        self.base_goal.position.x = x
        self.base_goal.position.y = y
        q = quaternion_from_euler(0.0, 0.0, angle)
        self.base_goal.orientation.x = q[0]
        self.base_goal.orientation.y = q[1]
        self.base_goal.orientation.z = q[2]
        self.base_goal.orientation.w = q[3]

    def addEndEffectorsTargets(self, ee_names = ["leg1", "leg2", "leg3", "leg4"], frame_type = EndEffectorGoal.NOMINAL_POSE):
        """Add end effector target positions. Note it does not set actual target values.

        Keyword arguments:
        ee_names -- names of end effectors (default: ["leg1", "leg2", "leg3", "leg4"])
        frame_type -- how geat generator should interpret target position,
            see sweetie_bot_clop_generator.msg.EndEffectorGoal, default: NOMINAL_POSE
        """
        self.ee_goal = []
        for name in ee_names:
            ee_goal = EndEffectorGoal()
            ee_goal.name = name
            ee_goal.frame_type = frame_type
            ee_goal.contact = True
            self.ee_goal.append(ee_goal)

    def setEndEffectorTargetPose(self, ee_name, position, contact = None, frame_type = None):
        for ee_goal in self.ee_goal:
            if ee_goal.name == ee_name:
                if position != None:
                    ee_goal.position.x = position[0]
                    ee_goal.position.y = position[1]
                    ee_goal.position.z = position[2]
                if contact != None:
                    ee_goal.contact = contact
                if frame_type != None:
                    ee_goal.frame_type = frame_type
                return
        raise AttributeError

class Clopper:
    """sweetie_bot_clop_generator MoveBase aclionlib client."""

    def __init__(self, topic):
        """Init ROS node and connect to sweetie_bot_clop_generator.msg.MoveBaseAction action server.
        
        Keyword arguments:
        topic -- actionlib topic
        """
        rospy.init_node('clop_generator_client_aka_clopper')
        # connect to gait generation server
        self.client = actionlib.SimpleActionClient(topic, MoveBaseAction)
        self.client.wait_for_server()

    def invokeClopGenerator(self, msg):
        """Send MoveBaseGoal to gait generator and print result.
        
        Keyword arguments:
        msg --- sweetie_bot_clop_generator.msg.MoveBaseGoal message.
        """
        # send goal to server
        msg.header.stamp = Time.now()
        self.client.send_goal(msg)
        print("Wait for goal...")
        self.client.wait_for_result()
        print("Result: " + str(self.client.get_result()))

