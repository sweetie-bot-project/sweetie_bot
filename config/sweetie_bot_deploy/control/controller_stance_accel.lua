--
-- STANCE controller
--
-- Setup FollowStance controller
--
-- Intended to be run via config script.
--
require "motion_core"

controller = controller or {}

-- load controller
ros:import("sweetie_bot_controller_cartesian")
depl:loadComponent("controller/stance", "sweetie_bot::motion::controller::FollowStance")
controller.stance = depl:getPeer("controller/stance")
-- get ROS parameteres and services
config.get_peer_rosparams(controller.stance)
-- register controller
resource_control.register_controller(controller.stance)
-- timer
depl:connect(timer.controller.port, "controller/stance.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> aggregator_ref
depl:connect("controller/stance.out_joints_ref", "aggregator_ref.in_joints", rtt.Variable("ConnPolicy"))
depl:connect("controller/stance.out_supports", "aggregator_ref.in_supports", rtt.Variable("ConnPolicy"))
-- data flow: controller <-> dynamics_inv
-- depl:connect("controller/stance.out_base_accel", "dynamics_inv.in_base_accel", rtt.Variable("ConnPolicy"))
depl:connect("controller/stance.in_joints_accel_sorted", "dynamics_inv.out_joints_accel_sorted", rtt.Variable("ConnPolicy"))
-- data flow: controller <- odometry_ref
depl:connect("controller/stance.in_base", "odometry_ref.out_base", rtt.Variable("ConnPolicy"))
-- ROS redirect
depl:stream("controller/stance.in_base_ref", ros:topic("~controller/stance/in_pose_ref"))
-- connect to RobotModel
depl:connectServices("controller/stance", "aggregator_ref")
-- advertise ROS operation
controller.stance:loadService("rosservice")
controller.stance:provides("rosservice"):connect("rosSetOperational", config.node_fullname .. "/controller/stance/set_operational", "std_srvs/SetBool")
-- prepare to start
controller.stance:configure()

--twist0 = rtt.Variable("kdl_msgs.Twist")
--twist = rtt.Variable("kdl_msgs.Twist")
--twist.vel.X = 0.00
--twist.vel.Z = 0.01
--base_accel_port = rttlib.port_clone_conn( dynamics_inv:getPort("in_base_accel") )

--twist0 = rtt.Variable("kdl_msgs.Twist")
--twist = rtt.Variable("kdl_msgs.Twist")
--twist.vel.X = 0.00
--twist.vel.Z = 0.01
--base_accel_port = rttlib.port_clone_conn( dynamics_inv:getPort("in_base_accel") )

--p = rtt.Variable("geometry_msgs.PoseStamped")
--p.pose.orientation.w = 1
--p.pose.position.x = 0.00
--p.pose.position.z = 0.22
--base_ref_port = rttlib.port_clone_conn( controller.stance:getPort("in_base_ref") )
--base_ref_port:write( p )

