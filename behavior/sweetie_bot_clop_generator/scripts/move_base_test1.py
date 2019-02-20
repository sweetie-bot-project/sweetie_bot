#!/usr/bin/env python

#
# Test sweetie_bot::ClopClopGenerator
# Repeatedly send different movements request.
# 
# Deploy joint_space_control infrastructure before running test.
#


import rospy
import actionlib
from rospy.rostime import Duration
import std_msgs.msg
import geometry_msgs.msg
import sweetie_bot_control_msgs.msg

if __name__ == '__main__':
    rospy.init_node('test_clop_generator')

    # create goal message
    client = actionlib.SimpleActionClient('clop_generator', sweetie_bot_control_msgs.msg.MoveBaseAction)
    client.wait_for_server()
    goal = sweetie_bot_control_msgs.msg.MoveBaseGoal()
    goal.header = std_msgs.msg.Header()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "base_link_path"
    goal.duration = 3.0;
    goal.n_steps = 4;
    goal.gait_type = "walk_overlap"
    # goal.gait_type = "pace"
    goal.base_goal = geometry_msgs.msg.Pose()
    goal.base_goal.position.x = 0.3
    goal.base_goal.position.y = 0.0
    goal.base_goal.position.z = 0.19
    goal.base_goal.orientation.x = 0.0
    goal.base_goal.orientation.y = 0.0
    goal.base_goal.orientation.z = -0.4
    goal.base_goal.orientation.w = 1.0
    goal.position_tolerance = 0.04
    goal.orientation_tolerance = 0.30;

    # send goal to server
    client.send_goal(goal)
    print("Wait for goal...")
    client.wait_for_result()
    print("Result: " + str(client.get_result()))

