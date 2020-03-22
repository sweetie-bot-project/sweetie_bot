--
-- POSE controller
--
-- Setup LookAt controller: track point by changing head orientation.
--
-- Intended to be run via config script.
--
require "motion_core"

controller = controller or {}

--
-- Pose controller
--

ros:import("sweetie_bot_controller_cartesian")
depl:loadComponent("controller/look_at", "sweetie_bot::motion::controller::LookAtJoints")
controller.look_at = depl:getPeer("controller/look_at")
-- get ROS parameteres and services
config.get_peer_rosparams(controller.look_at)
-- register controller
resource_control.register_controller(controller.look_at)
-- timer
depl:connect(timer.controller.port, "controller/look_at.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller -> aggregator_ref
depl:connect("controller/look_at.out_supports", "aggregator_ref.in_supports", rtt.Variable("ConnPolicy"))
depl:connect("controller/look_at.out_joints_ref", "aggregator_ref.in_joints", rtt.Variable("ConnPolicy"))
-- data flow: controller <- aggregator_ref
depl:connect("controller/look_at.in_joints_sorted", "aggregator_ref.out_joints_sorted", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> kinematics_inv
depl:connectOperations("controller/look_at.poseToJointState", "kinematics_inv.poseToJointState");
-- data flow: controller <- odometry_ref
depl:connect("controller/look_at.in_base", "odometry_ref.out_base", rtt.Variable("ConnPolicy"))
-- data flow: controller <- kinematics_fwd
depl:connect("controller/look_at.in_limbs", "kinematics_fwd.out_limbs_fixed", rtt.Variable("ConnPolicy"))
-- ROS redirect
depl:stream("controller/look_at.in_pose_ref", ros:topic("~controller/look_at/in_pose_ref"))

-- connect to RobotModel
depl:connectServices("controller/look_at", "aggregator_ref")
-- advertise actionlib interface
controller.look_at:loadService("actionlib")
controller.look_at:provides("actionlib"):connect("~controller/look_at")
-- advertise ROS operation
controller.look_at:loadService("rosservice")
controller.look_at:provides("rosservice"):connect("rosSetOperational", config.node_fullname .. "/controller/look_at/set_operational", "std_srvs/SetBool")
-- prepare to start
assert(controller.look_at:configure(), "ERROR: Unable to configure controller/look_at")

