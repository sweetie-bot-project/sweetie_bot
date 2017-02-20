--
-- VIRTUAL ROBOT and JOINT STATE controllers
--
-- Setup FollowJointState controller
require "virtual_motion"

controller = {}

-- depl:loadComponent("controller_joint_state", "sweetie_bot::controller::FollowJointState")
-- controller.joint_state = depl:getPeer("controller_joint_state")
-- rttlib_extra.get_rosparam(controller.joint_state)
-- -- register controller
-- resource_control.register_controller(controller.joint_state)
-- -- data flow: controller -> agregator_ref
-- depl:connect("controller_joint_state.out_joints", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
-- -- prepare to start
-- controller.joint_state:configre()

depl:stream("agregator_ref.in_joints", ros:topic("~agregator_ref/in_joints"))
