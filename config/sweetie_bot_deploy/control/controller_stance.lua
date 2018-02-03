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
rttlib_extra.get_peer_rosparams(controller.stance)
-- register controller
resource_control.register_controller(controller.stance)
-- timer
depl:connect(timer.controller.port, "controller/stance.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller -> agregator_ref
depl:connect("controller/stance.out_supports", "agregator_ref.in_supports", rtt.Variable("ConnPolicy"))
-- data flow: controller -> agregator_ref
depl:connect("controller/stance.out_limbs_ref", "kinematics_inv.in_limbs", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> odometry_ref
if rttlib_extra.get_rosparam("~controller/stance/override_odometry", "bool") then
	-- odometry results are overrided with controller/stance.out_base_ref port
	depl:connect("controller/stance.out_base_ref", "odometry_ref.in_base", rtt.Variable("ConnPolicy"))
end
depl:connect("controller/stance.in_base", "odometry_ref.out_base", rtt.Variable("ConnPolicy"))
-- data flow: controller <- kinematics_fwd
depl:connect("controller/stance.in_limbs", "kinematics_fwd.out_limbs_fixed", rtt.Variable("ConnPolicy"))
if not rttlib_extra.get_rosparam("~controller/stance/use_kinematics_inv_port", "bool") then
	depl:connectOperations("controller/stance.poseToJointStatePublish", "kinematics_inv.poseToJointStatePublish");
end
-- ROS redirect
depl:stream("controller/stance.in_base_ref", ros:topic("~controller/stance/in_base_ref"))
-- depl:stream("controller/stance.out_base_ref", ros:topic("~controller/stance/out_base_ref"))
-- connect to RobotModel
depl:connectServices("controller/stance", "agregator_ref")
-- advertise ROS operation
controller.stance:loadService("rosservice")
controller.stance:provides("rosservice"):connect("rosSetOperational", config.node_fullname .. "/controller/stance/set_operational", "std_srvs/SetBool")
-- prepare to start
controller.stance:configure()

--
-- debug pose messages and pose control port
-- TODO: move somewhere (write tests).
--
p1 = rtt.Variable("geometry_msgs.PoseStamped")
p1.pose.orientation.w = 1
p1.pose.position.x = 0.00
p1.pose.position.z = 0.22

p2 = rtt.Variable("geometry_msgs.PoseStamped")
p2.pose.orientation.z = -0.094577
p2.pose.orientation.w = 0.9955
p2.pose.position.x = 0.00
p2.pose.position.z = 0.22

p3 = rtt.Variable("geometry_msgs.PoseStamped")
p3.pose.orientation.z = 0.0
p3.pose.orientation.w = 1.0
p3.pose.position.x = 0.10
p3.pose.position.z = 0.22

base_ref_port = rttlib.port_clone_conn( controller.stance:getPort("in_base_ref") )
base_ref_port:write( p1 )

