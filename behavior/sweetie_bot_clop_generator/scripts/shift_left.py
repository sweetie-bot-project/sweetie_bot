#!/usr/bin/env python

#
# Test for sweetie_bot::ClopClopGenerator EE 
# Test final pose setting capabilities
# 
# Deploy joint_space_control infrastructure before running test.
#

from math import pi
from sweetie_bot_clop_generator.clopper import Clopper, MoveBaseGoal

if __name__ == '__main__':
    # create message
    msg = MoveBaseGoal(gait_type = "walk", n_steps = 4, duration = 4.0)
    msg.setTargetBaseShift(x = 0.0, y = 0.2, angle = 0.0) 
    msg.addEndEffectorsTargets(["leg1","leg2","leg3","leg4"]) 

    # send message to server
    clop = Clopper("clop_generator")
    clop.invokeClopGenerator(msg)
