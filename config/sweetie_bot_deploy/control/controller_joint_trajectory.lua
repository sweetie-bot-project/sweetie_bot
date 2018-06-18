--
-- VIRTUAL ROBOT and JOINT STATE controllers
--
-- controllers: AnimJointTrajectory
--
-- Intended to be run via config script.
--
require "motion_core"

controller = controller or {}

-- 
-- load AnimJointTrajectory controller
--
ros:import("sweetie_bot_controller_joint_space")

depl:loadComponent("controller/joint_trajectory", "sweetie_bot::motion::controller::AnimJointTrajectory")
controller.joint_trajectory = depl:getPeer("controller/joint_trajectory")
-- load parameters and services
rttlib_extra.get_peer_rosparams(controller.joint_trajectory)
-- register controller
resource_control.register_controller(controller.joint_trajectory)
-- timer
depl:connect(timer.controller.port, "controller/joint_trajectory.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> agregator_ref
depl:connect("controller/joint_trajectory.out_joints_ref_fixed", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
depl:connect("controller/joint_trajectory.out_supports", "agregator_ref.in_supports", rtt.Variable("ConnPolicy"))
depl:connect("agregator_ref.out_joints_sorted", "controller/joint_trajectory.in_joints_sorted", rtt.Variable("ConnPolicy"))
-- configure actionlib
ros:import("rtt_actionlib")
controller.joint_trajectory:loadService("actionlib")
controller.joint_trajectory:provides("actionlib"):connect("~controller/joint_trajectory")
-- connect to RobotModel
depl:connectServices("controller/joint_trajectory", "agregator_ref")
-- prepare to start
assert(controller.joint_trajectory:configure(), "ERROR: controller joint trajectory is not configured.")
