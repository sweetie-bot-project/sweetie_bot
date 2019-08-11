--
-- STANCE controller
--
-- Setup FollowStance controller
--
-- Intended to be run via config script.
--
require "motion_core"

controller = controller or {}

--
-- Stance controller
--

ros:import("sweetie_bot_controller_cartesian")
depl:loadComponent("controller/stance", "sweetie_bot::motion::controller::FollowStance")
controller.stance = depl:getPeer("controller/stance")
-- get ROS parameteres and services
config.get_peer_rosparams(controller.stance)
-- register controller
resource_control.register_controller(controller.stance)
-- timer
depl:connect(timer.controller.port, "controller/stance.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller -> aggregator_ref
depl:connect("controller/stance.out_supports", "aggregator_ref.in_supports", rtt.Variable("ConnPolicy"))
-- data flow: controller -> aggregator_ref
depl:connect("controller/stance.out_limbs_ref", "kinematics_inv.in_limbs", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> odometry_ref
if config.get_rosparam("~controller/stance/override_odometry", "bool") then
	-- odometry results are overrided with controller/stance.out_base_ref port
	depl:connect("controller/stance.out_base_ref", "odometry_ref.in_base", rtt.Variable("ConnPolicy"))
end
depl:connect("controller/stance.in_base", "odometry_ref.out_base", rtt.Variable("ConnPolicy"))
depl:connect("controller/stance.in_balance", "dynamics_inv.out_balance", rtt.Variable("ConnPolicy"))
-- data flow: controller <- kinematics_fwd
depl:connect("controller/stance.in_limbs_fixed", "kinematics_fwd.out_limbs_fixed", rtt.Variable("ConnPolicy"))
if not config.get_rosparam("~controller/stance/use_kinematics_inv_port", "bool") then
	depl:connectOperations("controller/stance.poseToJointStatePublish", "kinematics_inv.poseToJointStatePublish");
end
-- ROS redirect
depl:stream("controller/stance.in_base_ref", ros:topic("~controller/stance/in_pose_ref"))
-- ROS redirect: debug
-- depl:stream("controller/stance.out_limbs_ref", ros:topic("~controller/stance/out_limbs_ref"))
-- depl:stream("controller/stance.out_base_ref", ros:topic("~controller/stance/out_base_ref"))
-- depl:stream("kinematics_fwd.out_limbs_fixed", ros:topic("~kinematics_fwd/out_limbs"))
-- depl:stream("odometry_ref.out_base", ros:topic("~kinematics_fwd/out_base"))A

-- connect to RobotModel
depl:connectServices("controller/stance", "aggregator_ref")
-- advertise actionlib interface
controller.stance:loadService("actionlib")
controller.stance:provides("actionlib"):connect("~controller/stance")
-- advertise ROS operation
controller.stance:loadService("rosservice")
controller.stance:provides("rosservice"):connect("rosSetOperational", config.node_fullname .. "/controller/stance/set_operational", "std_srvs/SetBool")
-- prepare to start
assert(controller.stance:configure(), "ERROR: Unable to configure controller/stance.")


--- Helper function for setting stance support legs
--
-- @param val support legs code: 123 means leg1, leg2, leg3
--
function debug.set_stance(val)
	--- determine new support limbs
	list = {}
	while val >= 1 do
		table.insert(list, "leg" .. tostring(val % 10))
		val = math.floor(val / 10)
	end
	limbs = rtt.Variable("strings")
	limbs:fromtab(list)
	--- reconfigure controller
	controller.stance:stop()
	if #list then
		controller.stance:cleanup()
		controller.stance:getProperty("support_legs"):set(limbs)
		return controller.stance:configure() and  controller.stance:start()
	else
		return true
	end
end

