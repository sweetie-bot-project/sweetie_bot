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
depl:loadComponent("controller/joint_state", "sweetie_bot::motion::controller::FollowJointState")
controller.joint_state = depl:getPeer("controller/joint_state")
-- get ROS parameteres and services
rttlib_extra.get_peer_rosparams(controller.joint_state)
-- register controller
resource_control.register_controller(controller.joint_state)
-- timer
depl:connect(timer.controller.port, "controller/joint_state.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> agregator_ref
depl:connect("controller/joint_state.out_joints_ref_fixed", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
depl:connect("controller/joint_state.in_joints_sorted", "agregator_ref.out_joints_sorted", rtt.Variable("ConnPolicy"))
-- ROS redirect
depl:stream("controller/joint_state.in_joints_ref", ros:topic("~controller/joint_state/in_joints_ref"))
depl:stream("controller/joint_state.out_joints_src_reset", ros:topic("~controller/joint_state/out_joints_src_reset"))
-- connect to RobotModel
depl:connectServices("controller/joint_state", "agregator_ref")
-- advertise ROS operation
controller.joint_state:loadService("rosservice")
controller.joint_state:provides("rosservice"):connect("rosSetOperational", config.node_fullname .. "/controller/joint_state/set_operational", "std_srvs/SetBool")
-- prepare to start
controller.joint_state:configure()
