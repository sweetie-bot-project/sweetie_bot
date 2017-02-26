--
-- VIRTUAL ROBOT and JOINT STATE controllers
--
-- controllers: FollowJointState, AnimJointTrajectoryBase
--
-- Intended to be run via config script.
--
require "controller_joint_state"
assert(controller.joint_state:start()	)

require "controller_joint_trajectory"
