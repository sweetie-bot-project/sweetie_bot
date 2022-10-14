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
    msg = MoveBaseGoal()
    msg.execute_only = True

    # send message to server
    clop = Clopper("gait_generator")
    clop.invokeClopGenerator(msg)
