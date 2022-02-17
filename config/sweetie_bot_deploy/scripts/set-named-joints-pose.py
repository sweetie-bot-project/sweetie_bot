#!/usr/bin/env python3

import sys
import rospy, actionlib

from flexbe_msgs.msg import BehaviorExecutionAction, BehaviorExecutionGoal

if len(sys.argv) != 2 :
	print(sys.argv[0] + " <pose_name>")
	print()
	print('''Set robot joints pose by name. This script ignors robot stability and support legs,
joints are simply moved to new positions. 

Poses are stored on Parameter Server under namespace '/saved_msgs/joint_pose' as serialized 
sensor_msgs/JointState messages.  Use `rosparam list` to list them.

This script uses FollowJointState controller to change robot pose. Controller is activated via 
FlexBe subsustem so flexbe node should be running.

''')
	sys.exit(0)

rospy.init_node('set_pose')
# connect to flexbe behavior server
client = actionlib.SimpleActionClient('flexbe/flexbe/execute_behavior', BehaviorExecutionAction)
client.wait_for_server()
# construct request
goal = BehaviorExecutionGoal()
goal.behavior_name = "ExecuteSetPose"
goal.arg_keys = ["pose", "set_supports"]
goal.arg_values = [sys.argv[1], "True"]
# send goal and wait result
print("Executing 'ExecuteSetPose' behavior...")
client.send_goal(goal)
client.wait_for_result()
print("Result: " + str(client.get_result()))
