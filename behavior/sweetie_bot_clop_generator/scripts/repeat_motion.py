#!/usr/bin/env python

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
    msg = MoveBaseGoal()
    msg.execute_only = True

    # send message to server
    clop = Clopper("clop_generator")
    clop.invokeClopGenerator(msg)
