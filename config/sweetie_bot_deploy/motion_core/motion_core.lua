-- 
-- MOTION CORE DEPLOYMENT
--
-- Setup hardware independent part of control system. Can be used for simulation.
-- Start logger, agreagator, resource control, timer, kinematics, odometry_ref and dynamics_inv.
--
-- Intended to be run via config script.
--
ros:import("rtt_roscomm")
ros:import("rtt_sensor_msgs")

--
-- logger subsystem
--

require "logger"

local logger_conf = config.get_rosparam("logger_conf", "string")
if not logger_conf then logger_conf = "logger.log4cpp" end

logger.init_loglevels_log4cpp(config.file(logger_conf))
-- logger.log4cpp:addRosAppender("motion", 20)

--
-- init resource_control module
--

require "resource_control"

resource_control.arbiter:configure()
assert(resource_control.arbiter:start(), "ERROR: Unable to start arbiter.")

--
-- timer
--
require "timer"

-------------------------------------------------------------
--              REFERENCE POSE PROCESSING                  --
-------------------------------------------------------------

--
-- aggregator for reference pose
--

ros:import("rtt_roscomm")
ros:import("sweetie_bot_aggregator");
ros:import("sweetie_bot_robot_model");
-- load component
depl:loadComponent("aggregator_ref", "sweetie_bot::motion::Aggregator")
aggregator_ref = depl:getPeer("aggregator_ref")
--set properties: publish only on timer
aggregator_ref:getProperty("publish_on_timer"):set(true)
aggregator_ref:getProperty("publish_on_event"):set(false)
--set properties: autoload
aggregator_ref:loadService("marshalling")
aggregator_ref:provides("marshalling"):loadServiceProperties(config.file("kinematic_chains.cpf"), "robot_model")
aggregator_ref:loadService("rosparam")
aggregator_ref:provides("rosparam"):getParam("", "robot_model")
-- upload robot model parameteres to ROS
aggregator_ref:provides("rosparam"):setParam("~robot_model", "robot_model") 
--get other properties
config.get_peer_rosparams(aggregator_ref)
--timer syncronization
depl:connect(timer.aggregator.port, "aggregator_ref.sync_step", rtt.Variable("ConnPolicy"));
-- publish pose to ROS
depl:stream("aggregator_ref.out_joints_sorted", ros:topic("~aggregator_ref/out_joints_sorted"))
depl:stream("aggregator_ref.out_supports_sorted", ros:topic("~aggregator_ref/out_supports_sorted"))
depl:stream("aggregator_ref.in_supports", ros:topic("~aggregator_ref/in_supports"))
-- configure component
aggregator_ref:configure()

-- 
-- Forward kineamtics component
--

ros:import("sweetie_bot_kinematics");
-- load component
depl:loadComponent("kinematics_fwd","sweetie_bot::motion::KinematicsFwd")
kinematics_fwd = depl:getPeer("kinematics_fwd")
config.get_peer_rosparams(kinematics_fwd)
-- data flow: aggregator_ref -> kinemaitics_fwd -> odometry_ref
depl:connect("aggregator_ref.out_joints_sorted", "kinematics_fwd.in_joints_sorted", rtt.Variable("ConnPolicy"));
-- connect to RobotModel
depl:connectServices("kinematics_fwd", "aggregator_ref")
-- publish pose to ROS
-- depl:stream("kinematics_fwd.out_limbs_fixed", ros:topic("~kinematics_fwd/out_limbs_fixed"))

kinematics_fwd:configure()

--
-- Inverse kinematics
--

ros:import("sweetie_bot_kinematics")
depl:loadComponent("kinematics_inv", "sweetie_bot::motion::KinematicsInv")
kinematics_inv = depl:getPeer("kinematics_inv")
-- get ROS parameteres and services
config.get_peer_rosparams(kinematics_inv)
-- data flow: controller <-> aggregator_ref
depl:connect("kinematics_inv.in_joints_sorted", "aggregator_ref.out_joints_sorted", rtt.Variable("ConnPolicy"))
depl:connect("kinematics_inv.out_joints", "aggregator_ref.in_joints", rtt.Variable("ConnPolicy"))
-- connect to RobotModel
depl:connectServices("kinematics_inv", "aggregator_ref")
-- prepare to start
kinematics_inv:configure()


-- 
-- Odometry
--

ros:import("sweetie_bot_odometry");
-- load component
depl:loadComponent("odometry_ref","sweetie_bot::motion::Odometry")
odometry_ref = depl:getPeer("odometry_ref")
config.get_peer_rosparams(odometry_ref)

-- data flow: aggregator_ref, kinematics_fwd -> odometry_ref
depl:connect("aggregator_ref.out_supports_sorted", "odometry_ref.in_supports_fixed", rtt.Variable("ConnPolicy"));
depl:connect("kinematics_fwd.out_limbs_fixed", "odometry_ref.in_limbs_fixed", rtt.Variable("ConnPolicy"));
-- connect to RobotModel
depl:connectServices("odometry_ref", "aggregator_ref")
-- publish tf to ROS
depl:stream("odometry_ref.out_tf", ros:topic("~odometry_ref/out_tf"))
depl:stream("odometry_ref.out_base", ros:topic("~odometry_ref/out_base"))
-- depl:stream("odometry_ref.in_base", ros:topic("~odometry_ref/in_base"))

odometry_ref:configure()

--- start components

assert(kinematics_inv:start(), "ERROR: Unable to start kinematics_inv.")
assert(aggregator_ref:start(), "ERROR: Unable to start aggregator_ref.") 
assert(kinematics_fwd:start(), "ERROR: Unable to start kinematics_fwd.")
assert(odometry_ref:start(), "ERROR: Unable to start odometry_ref.") 


-------------------------------------------------------------
--                  REAL POSE PROCESSING                   --
-------------------------------------------------------------

--
-- Aggregator for real pose
--

-- load component
depl:loadComponent("aggregator_real", "sweetie_bot::motion::Aggregator");
aggregator_real = depl:getPeer("aggregator_real")
aggregator_real:loadService("marshalling")
aggregator_real:loadService("rosparam")
--set properties: publish on event
aggregator_real:getProperty("publish_on_timer"):set(false)
aggregator_real:getProperty("publish_on_event"):set(true)
--set properties
aggregator_real:provides("marshalling"):loadServiceProperties(config.file("kinematic_chains.cpf"), "robot_model")
aggregator_real:provides("rosparam"):getParam("","robot_model")
--get other properties
config.get_peer_rosparams(aggregator_real)
-- timer syncronization: publish at same time as controllers
depl:connect(timer.controller.port, "aggregator_real.sync_step", rtt.Variable("ConnPolicy"));
-- publish pose to ROS
depl:stream("aggregator_real.out_joints_sorted", ros:topic("~aggregator_real/out_joints_sorted"))
-- configure component
aggregator_real:configure()

--
-- Forward kinematics component for real robot pose
--

-- load component
depl:loadComponent("kinematics_fwd_real","sweetie_bot::motion::KinematicsFwdForce")
kinematics_fwd_real = depl:getPeer("kinematics_fwd_real")
config.get_peer_rosparams(kinematics_fwd_real)
-- data flow: aggregator_real -> kinemaitics_fwd_real
depl:connect("aggregator_real.out_joints_sorted", "kinematics_fwd_real.in_joints_sorted", rtt.Variable("ConnPolicy"));
-- connect to RobotModel
depl:connectServices("kinematics_fwd_real", "aggregator_real")
-- publish pose to ROS
depl:stream("kinematics_fwd_real.out_limbs_fixed", ros:topic("~kinematics_fwd_real/out_limbs_fixed"))

kinematics_fwd_real:configure()


-- start components
assert(aggregator_real:start(), "ERROR: Unable to start aggregator_real.")
assert(kinematics_fwd_real:start(), "ERROR: Unable to start kinematics_fwd_real.")

-------------------------------------------------------------
--                  DYNAMICS                               --
-------------------------------------------------------------

-- 
-- Dynamics
--

ros:import("sweetie_bot_dynamics");
-- load component
depl:loadComponent("dynamics_inv","sweetie_bot::motion::DynamicsInvSimple")
dynamics_inv = depl:getPeer("dynamics_inv")
-- load parameters
dynamics_inv:loadService("rosparam")
dynamics_inv:provides("rosparam"):getParam("robot_description_dynamics", "robot_description")
config.get_peer_rosparams(dynamics_inv)

-- data flow: aggregator_ref, kinematics_fwd, aggregator_real-> dynamics_inv
depl:connect("aggregator_ref.out_joints_sorted", "dynamics_inv.in_joints_ref_sorted", rtt.Variable("ConnPolicy"));
depl:connect("aggregator_ref.out_supports_sorted", "dynamics_inv.in_supports_sorted", rtt.Variable("ConnPolicy"));
depl:connect("aggregator_real.out_joints_sorted", "dynamics_inv.in_joints_real_sorted", rtt.Variable("ConnPolicy"))
depl:connect("odometry_ref.out_base", "dynamics_inv.in_base_ref", rtt.Variable("ConnPolicy"));
depl:connect(timer.aggregator.port, "dynamics_inv.sync_step", rtt.Variable("ConnPolicy"));

-- connect to RobotModel
depl:connectServices("dynamics_inv", "aggregator_ref")
-- publish tf to ROS
depl:stream("dynamics_inv.out_balance", ros:topic("~dynamics_inv/out_balance"))
depl:stream("dynamics_inv.out_wrenches_fixed", ros:topic("~dynamics_inv/out_wrenches_fixed"))
depl:stream("dynamics_inv.out_joints_accel_sorted", ros:topic("~dynamics_inv/out_joints_accel_sorted"))

dynamics_inv:configure()
assert(dynamics_inv:start(), "ERROR: Unable to start dynamics_inv.")


-------------------------------------------------------------
--                  HELPER FUNCTIONS                       --
-------------------------------------------------------------


--- Helper function for setting support
-- @param val Support legs code: 123 means leg1, leg2, leg3
--
function debug.set_support(val)
	local list = {}
	while val >= 1 do
		table.insert(list, "leg" .. tostring(val % 10))
		val = math.floor(val / 10)
	end
	local limbs = rtt.Variable("strings")
	limbs:fromtab(list)
	aggregator_ref:setSupportState(limbs)
end


--- Helper function for resetting base_link pose to identity.
-- @param z base_link new z coordinate (use 0.223 for proto2)
function debug.reset_platform_pose(z)
	-- default platform pose
	local p = rtt.Variable("sweetie_bot_kinematics_msgs.RigidBodyState")
	p.name:resize(1)
	p.name[0] = "base_link"
	p.frame:resize(1)
	p.frame[0].p.Z = z
	p.wrench:resize(1)
	-- override odometry
    local override_odometry_port = rttlib.port_clone_conn( odometry_ref:getPort("in_base") )
	override_odometry_port:write( p )
end

