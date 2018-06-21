--
-- POSE controller
--
-- Setup FollowPose controller: track limb pose in 
--
-- Intended to be run via config script.
--
require "motion_core"

controller = controller or {}

--
-- Pose controller
--

ros:import("sweetie_bot_controller_cartesian")
depl:loadComponent("controller/pose", "sweetie_bot::motion::controller::FollowPose")
controller.pose = depl:getPeer("controller/pose")
-- get ROS parameteres and services
rttlib_extra.get_peer_rosparams(controller.pose)
-- register controller
resource_control.register_controller(controller.pose)
-- timer
depl:connect(timer.controller.port, "controller/pose.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller -> agregator_ref
depl:connect("controller/pose.out_supports", "agregator_ref.in_supports", rtt.Variable("ConnPolicy"))
-- data flow: controller -> kinematics_inv -> agregator_ref
depl:connect("controller/pose.out_limbs_ref", "kinematics_inv.in_limbs", rtt.Variable("ConnPolicy"))
if not rttlib_extra.get_rosparam("~controller/pose/use_kinematics_inv_port", "bool") then
	depl:connectOperations("controller/pose.poseToJointStatePublish", "kinematics_inv.poseToJointStatePublish");
end
-- data flow: controller <- odometry_ref
depl:connect("controller/pose.in_base", "odometry_ref.out_base", rtt.Variable("ConnPolicy"))
-- data flow: controller <- kinematics_fwd
depl:connect("controller/pose.in_limbs", "kinematics_fwd.out_limbs_fixed", rtt.Variable("ConnPolicy"))
-- ROS redirect
depl:stream("controller/pose.in_pose_ref", ros:topic("~controller/pose/in_pose_ref"))

-- connect to RobotModel
depl:connectServices("controller/pose", "agregator_ref")
-- advertise actionlib interface
controller.pose:loadService("actionlib")
controller.pose:provides("actionlib"):connect("~controller/pose")
-- advertise ROS operation
controller.pose:loadService("rosservice")
controller.pose:provides("rosservice"):connect("rosSetOperational", config.node_fullname .. "/controller/pose/set_operational", "std_srvs/SetBool")
-- prepare to start
controller.pose:configure()


---
--- helper function for setting controlled limb
---
function set_pose_controlled_limb(limb)
	--- reconfigure controller
	controller.pose:stop()
	controller.pose:getProperty("controlled_chain"):set(limb)
	return controller.pose:start()
end

