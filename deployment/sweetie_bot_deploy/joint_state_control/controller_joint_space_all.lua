--
-- VIRTUAL ROBOT and JOINT STATE controllers
--
-- controllers: FollowJointState, AnimJointTrajectoryBase, TorqueMainSwitch if herkulex and agregator are present.
--
-- Intended to be run via config script.
--
require "controller_joint_state"

assert(controller.joint_state:start())

require "controller_joint_state_head"

require "controller_joint_trajectory"

if (agregator_real and herkulex) then
	require "controller_torque_off"
end

-- require "reporting"
-- 
-- dir = "/home/oleg/"
-- reporting.add_filereporter("joint_state", { { name = "controller/joint_state" ,  ports= {"out_joints_ref_fixed"} } }, dir .. "joint_state.out", true)
-- reporting.add_filereporter("joint_trajectory", { { name = "controller/joint_trajectory" ,  ports= {"out_joints_ref_fixed"} } }, dir .. "joint_traject.out", true)
-- reporting.add_filereporter("agregator", {  { name = "agregator_ref" ,  ports= {"out_joints_sorted"} } }, dir .. "agregator.out", true)
