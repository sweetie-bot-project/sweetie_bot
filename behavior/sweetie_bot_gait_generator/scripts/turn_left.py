#!/usr/bin/env python3

#
# Test for sweetie_bot::ClopClopGenerator EE 
# Turn in place on 90 degrees to the left.
# 
# Deploy joint_space_control infrastructure before running test.
#

from math import pi
from sweetie_bot_gait_generator.clopper import Clopper, MoveBaseGoal

if __name__ == '__main__':
    # create message
    msg = MoveBaseGoal(gait_type = "walk", n_steps = 2, duration = 4.0)
    msg.setTargetBaseShift(x = 0, y = 0.0, angle = pi/2) 
    msg.addEndEffectorsTargets(["leg1","leg2","leg3","leg4"]) 

    # send message to server
    clop = Clopper("gait_generator")
    clop.invokeClopGenerator(msg)
