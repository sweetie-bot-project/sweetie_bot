#!/usr/bin/env python3

#
# Test for sweetie_bot::ClopClopGenerator EE 
# Test final pose setting capabilities
# 
# Deploy joint_space_control infrastructure before running test.
#

from math import pi
from sweetie_bot_clop_generator.clopper import Clopper, MoveBaseGoal
from sweetie_bot_clop_generator.msg import EndEffectorGoal

if __name__ == '__main__':
    # create message
    msg = MoveBaseGoal(gait_type = "free", n_steps = 0, duration = 3.0)
    msg.setTargetBaseShift(x = 0.0, y = 0.0, angle = 0.0) 
    msg.addEndEffectorsTargets(["leg1","leg2","leg3","leg4"], EndEffectorGoal.PATH_FINAL) 
    msg.setEndEffectorTargetPose("leg1", [0.080425, 0.0386, 0.0]) 
    msg.setEndEffectorTargetPose("leg2", [0.080425, -0.0386, 0.0]) 
    msg.setEndEffectorTargetPose("leg3", [-0.080425, 0.0386, 0.0]) 
    msg.setEndEffectorTargetPose("leg4", [-0.080425, -0.0386, 0.0]) 
    print(msg)

    # send message to server
    clop = Clopper("clop_generator")
    clop.invokeClopGenerator(msg)
