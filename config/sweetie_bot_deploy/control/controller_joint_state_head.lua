--
-- JOINT STATE controllers
--
-- Setup FollowJointState controller
--
-- Intended to be run via config script.
--
require "motion_core"

controller = controller or {}

-- load controller
ros:import("sweetie_bot_controller_joint_space")
depl:loadComponent("controller/joint_state_head", "sweetie_bot::motion::controller::FollowJointState")
controller.joint_state_head = depl:getPeer("controller/joint_state_head")
-- get ROS parameteres ans services
config.get_peer_rosparams(controller.joint_state_head)
-- register controller
resource_control.register_controller(controller.joint_state_head)
-- timer
depl:connect(timer.controller.port, "controller/joint_state_head.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> agregator_ref
depl:connect("controller/joint_state_head.out_joints_ref_fixed", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
depl:connect("controller/joint_state_head.in_joints_sorted", "agregator_ref.out_joints_sorted", rtt.Variable("ConnPolicy"))
-- ROS redirect
depl:stream("controller/joint_state_head.in_joints_ref", ros:topic("~controller/joint_state_head/in_joints_ref"))
depl:stream("controller/joint_state_head.out_joints_src_reset", ros:topic("~controller/joint_state_head/out_joints_src_reset"))
-- connect to RobotModel
depl:connectServices("controller/joint_state_head", "agregator_ref")
-- advertise actionlib interface
controller.joint_state_head:loadService("actionlib")
controller.joint_state_head:provides("actionlib"):connect("~controller/joint_state_head")
-- advertise ROS operation
controller.joint_state_head:loadService("rosservice")
controller.joint_state_head:provides("rosservice"):connect("rosSetOperational", config.node_fullname .. "/controller/joint_state_head/set_operational", "std_srvs/SetBool")
-- prepare to start
assert(controller.joint_state_head:configure(), "ERROR: Unable to configure controller/joint_state_head.")
