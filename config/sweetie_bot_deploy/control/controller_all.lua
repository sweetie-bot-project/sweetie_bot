--
-- VIRTUAL ROBOT and JOINT STATE controllers
--
-- controllers: FollowJointState, ExecuteJointTrajectory, TorqueMainSwitch if herkulex and aggregator are present.
--
-- Intended to be run via config script.
--
require "controller_joint_state"

require "controller_joint_state_head"

require "controller_joint_trajectory"

require "controller_stance"

require "controller_pose"

if (aggregator_real and herkulex) then
	require "controller_torque_off"
end

-- require "reporting"
-- 
-- dir = "/home/oleg/"
-- reporting.add_filereporter("joint_state", { { name = "controller/joint_state" ,  ports= {"out_joints_ref_fixed"} } }, dir .. "joint_state.out", true)
-- reporting.add_filereporter("joint_trajectory", { { name = "controller/joint_trajectory" ,  ports= {"out_joints_ref_fixed"} } }, dir .. "joint_traject.out", true)
-- reporting.add_filereporter("aggregator", {  { name = "aggregator_ref" ,  ports= {"out_joints_sorted"} } }, dir .. "aggregator.out", true)

-- make robot to stand on four legs
set_support(1234)
timer.timer:wait(0, 0.2) -- wait 200 ms

-- start default controller
assert(controller.joint_state:start())

