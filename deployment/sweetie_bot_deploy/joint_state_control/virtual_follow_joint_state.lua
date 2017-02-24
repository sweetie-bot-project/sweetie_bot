--
-- VIRTUAL ROBOT and JOINT STATE controllers
--
-- Setup FollowJointState controller
require "virtual_motion"

controller = {}

ros:import("sweetie_bot_controllers_joint_space")

-- load controller
depl:loadComponent("controller_joint_state", "sweetie_bot::motion::controller::FollowJointState")
controller.joint_state = depl:getPeer("controller_joint_state")
rttlib_extra.get_peer_rosparams(controller.joint_state)
-- register controller
resource_control.register_controller(controller.joint_state)
-- timer
depl:connect(timer.controller.port, "controller_joint_state.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> agregator_ref
depl:connect("controller_joint_state.out_joints_ref_fixed", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
depl:connect("agregator_ref.out_joints_sorted", "controller_joint_state.in_joints_sorted", rtt.Variable("ConnPolicy"))
-- ros redirect
depl:stream("controller_joint_state.in_joints_ref", ros:topic("~controller_joint_state/in_joints_ref"))
-- connect to RobotModel
depl:connectServices("controller_joint_state", "agregator_ref")
-- prepare to start
controller.joint_state:configure()
