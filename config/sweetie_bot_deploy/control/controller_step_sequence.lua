--
-- STEP SEQUENCE controller
--
-- Setup ExecuteStepSequence controller
--
-- Intended to be run via config script.
--
require "motion_core"

controller = controller or {}

--
-- Stance controller
--

ros:import("sweetie_bot_controller_cartesian")
depl:loadComponent("controller/step_sequence", "sweetie_bot::motion::controller::ExecuteStepSequence")
controller.step_sequence = depl:getPeer("controller/step_sequence")
-- get ROS parameteres and services
config.get_peer_rosparams(controller.step_sequence)
-- register controller
resource_control.register_controller(controller.step_sequence)
-- timer
depl:connect(timer.controller.port, "controller/step_sequence.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller -> aggregator_ref
depl:connect("controller/step_sequence.out_supports", "aggregator_ref.in_supports", rtt.Variable("ConnPolicy"))
-- data flow: controller -> aggregator_ref
depl:connect("controller/step_sequence.out_limbs_ref", "kinematics_inv.in_limbs", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> odometry_ref
if config.get_rosparam("~controller/step_sequence/override_odometry", "bool") then
	-- odometry results are overrided with controller/step_sequence.out_base_ref port
	depl:connect("controller/step_sequence.out_base_ref", "odometry_ref.in_base", rtt.Variable("ConnPolicy"))
end
depl:connect("controller/step_sequence.in_base", "odometry_ref.out_base", rtt.Variable("ConnPolicy"))
-- data flow: controller <- kinematics_fwd
depl:connect("controller/step_sequence.in_limbs_fixed", "kinematics_fwd.out_limbs_fixed", rtt.Variable("ConnPolicy"))
if not config.get_rosparam("~controller/step_sequence/use_kinematics_inv_port", "bool") then
	depl:connectOperations("controller/step_sequence.poseToJointStatePublish", "kinematics_inv.poseToJointStatePublish");
end
-- ROS redirect
-- ROS redirect: debug
-- depl:stream("controller/step_sequence.out_limbs_ref", ros:topic("~controller/step_sequence/out_limbs_ref"))
-- depl:stream("controller/step_sequence.out_base_ref", ros:topic("~controller/step_sequence/out_base_ref"))
-- depl:stream("kinematics_fwd.out_limbs_fixed", ros:topic("~kinematics_fwd/out_limbs"))
-- depl:stream("odometry_ref.out_base", ros:topic("~kinematics_fwd/out_base"))A

-- connect to RobotModel
depl:connectServices("controller/step_sequence", "aggregator_ref")
-- advertise actionlib interface
controller.step_sequence:loadService("actionlib")
controller.step_sequence:provides("actionlib"):connect("~controller/step_sequence")
-- advertise ROS operation
-- prepare to start
assert(controller.step_sequence:configure(),"ERROR: Unable to configure controller/step_sequence.")

