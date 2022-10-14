#!/usr/bin/env python3

#
# Test for sweetie_bot::ClopClopGenerator EE 
# Test final pose setting capabilities
# 
# Deploy joint_space_control infrastructure before running test.
#

from math import pi
from sweetie_bot_gait_generator.clopper import Clopper, MoveBaseGoal
from sweetie_bot_gait_generator.msg import EndEffectorGoal

if __name__ == '__main__':
    # create message
    msg = MoveBaseGoal(gait_type = "free", n_steps = 0, duration = 2.0)
    msg.setTargetBaseShift(x = 0.0, y = -0.0, angle = 0.0) 
    msg.base_goal_bounds = MoveBaseGoal.POSE_FREE_XY
    msg.addEndEffectorsTargets(["leg4"], EndEffectorGoal.PATH_INITIAL) 
    msg.setEndEffectorTargetPose("leg4", [-0.080425, -0.0386, 0.01], 0, False) 
    print(msg)

    # send message to server
    clop = Clopper("gait_generator")
    clop.invokeClopGenerator(msg)
