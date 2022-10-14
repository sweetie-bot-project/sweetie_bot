#!/usr/bin/env python3

#
# Test for sweetie_bot::ClopClopGenerator EE 
# Test final pose setting capabilities
# 
# Deploy joint_space_control infrastructure before running test.
#

from math import pi
from sweetie_bot_gait_generator.clopper import Clopper, MoveBaseGoal

if __name__ == '__main__':
    # create message
    msg = MoveBaseGoal(gait_type = "walk_overlap", n_steps = 4, duration = 3.4)
    msg.setTargetBaseShift(x = 0.4, y = 0.0, angle = 0.0) 
    msg.addEndEffectorsTargets(["leg1","leg2","leg3","leg4"]) 

    # send message to server
    clop = Clopper("gait_generator")
    clop.invokeClopGenerator(msg)
