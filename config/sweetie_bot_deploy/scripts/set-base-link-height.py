#!/usr/bin/env python3

import sys
import rospy, actionlib

from flexbe_msgs.msg import BehaviorExecutionAction, BehaviorExecutionGoal

if len(sys.argv) != 2 :
	print(sys.argv[0] + " <height>")
	print()
	print('''Move robot base to height <height> (meteres) and normalize its orientation 
(set parallel to support surface). All legs are assumed to be in contact with support surface.

This script uses FollowStance controller to change robot pose. Controller is activated via 
FlexBe subsustem so flexbe node should be running.

''')
	sys.exit(0)

rospy.init_node('set_height')
# connect to flexbe behavior server
client = actionlib.SimpleActionClient('flexbe/flexbe/execute_behavior', BehaviorExecutionAction)
client.wait_for_server()
# construct request
goal = BehaviorExecutionGoal()
goal.behavior_name = "ExecuteSetStance"
goal.arg_keys.append("target_height")
goal.arg_values.append(sys.argv[1])
# send goal and wait result
print("Executing 'ExecuteSetStance' behavior...")
client.send_goal(goal)
client.wait_for_result()
print("Result: " + str(client.get_result()))
