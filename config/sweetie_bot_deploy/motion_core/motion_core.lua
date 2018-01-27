-- 
-- MOTION CORE DEPLOYMENT
--
-- Setup logger, agreagator, resource control, timer.
--
-- Intended to be run via config script.
--
ros:import("rtt_roscomm")
ros:import("rtt_sensor_msgs")

--
-- logger subsystem
--

require "logger"

local logger_conf = rttlib_extra.get_rosparam("logger_conf", "string")
if not logger_conf then logger_conf = "logger.log4cpp" end

logger.init_loglevels_log4cpp(config.file(logger_conf))
-- logger.log4cpp:addRosAppender("motion", 20)

--
-- init resource_control module
--

require "resource_control"

resource_control.arbiter:configure()
assert(resource_control.arbiter:start())

--
-- timer
--
require "timer"

--
-- load and start pose agregator
--

ros:import("rtt_roscomm")
ros:import("sweetie_bot_agregator");
ros:import("sweetie_bot_robot_model");
-- load component
depl:loadComponent("agregator_ref", "sweetie_bot::motion::Agregator")
agregator_ref = depl:getPeer("agregator_ref")
--set properties: publish only on timer
agregator_ref:getProperty("publish_on_timer"):set(true)
agregator_ref:getProperty("publish_on_event"):set(false)
--set properties: autoload
agregator_ref:loadService("marshalling")
agregator_ref:provides("marshalling"):loadServiceProperties(config.file("kinematic_chains.cpf"), "robot_model")
agregator_ref:loadService("rosparam")
agregator_ref:provides("rosparam"):getParam("", "robot_model")
-- upload robot model parameteres to ROS
agregator_ref:provides("rosparam"):setParam("robot_model", "robot_model") 
--get other properties
rttlib_extra.get_peer_rosparams(agregator_ref)
--timer syncronization
depl:connect(timer.agregator.port, "agregator_ref.sync_step", rtt.Variable("ConnPolicy"));
-- publish pose to ROS
depl:stream("agregator_ref.out_joints_sorted", ros:topic("~agregator_ref/out_joints_sorted"))
depl:stream("agregator_ref.out_supports_sorted", ros:topic("~agregator_ref/out_supports_sorted"))
-- start component
agregator_ref:configure()

-- 
-- Forward kineamtics component
--

ros:import("sweetie_bot_kinematics");
-- load component
depl:loadComponent("kinematics_fwd","sweetie_bot::motion::KinematicsFwd")
kinematics_fwd = depl:getPeer("kinematics_fwd")
rttlib_extra.get_peer_rosparams(kinematics_fwd)

-- data flow: agregator_ref -> servo_inv -> herkulex_sched
depl:connect("agregator_ref.out_joints_sorted", "kinematics_fwd.in_joints_sorted", rtt.Variable("ConnPolicy"));
-- connect to RobotModel
depl:connectServices("kinematics_fwd", "agregator_ref")
-- publish pose to ROS
-- depl:stream("kinematics_fwd.out_limbs_fixed", ros:topic("~kinematics_fwd/out_limbs_fixed"))

kinematics_fwd:configure()

-- 
-- Odometry
--

ros:import("sweetie_bot_odometry");
-- load component
depl:loadComponent("odometry_ref","sweetie_bot::motion::Odometry")
odometry_ref = depl:getPeer("odometry_ref")
rttlib_extra.get_peer_rosparams(odometry_ref)

-- data flow: agregator_ref, kinematics_fwd -> odometry_ref
depl:connect("agregator_ref.out_supports_sorted", "odometry_ref.in_supports_fixed", rtt.Variable("ConnPolicy"));
depl:connect("kinematics_fwd.out_limbs_fixed", "odometry_ref.in_limbs_fixed", rtt.Variable("ConnPolicy"));
-- connect to RobotModel
depl:connectServices("odometry_ref", "agregator_ref")
-- publish tf to ROS
depl:stream("odometry_ref.out_tf", ros:topic("~odometry_ref/out_tf"))
depl:stream("odometry_ref.out_base", ros:topic("~odometry_ref/out_base"))
depl:stream("odometry_ref.in_base", ros:topic("~odometry_ref/in_base"))

odometry_ref:configure()

--- start components

assert(agregator_ref:start()) 
assert(kinematics_fwd:start())
assert(odometry_ref:start()) 

--- helper function for setting support
--- 123 means leg1, leg2, leg3
function set_support(val)
	list = {}
	while val >= 1 do
		table.insert(list, "leg" .. tostring(val % 10))
		val = math.floor(val / 10)
	end
	limbs = rtt.Variable("strings")
	limbs:fromtab(list)
	agregator_ref:setSupportState(limbs)
end


-- 
-- Dynamics
--

ros:import("sweetie_bot_dynamics");
-- load component
depl:loadComponent("dynamics_inv","sweetie_bot::motion::DynamicsInvSimple")
dynamics_inv = depl:getPeer("dynamics_inv")
rttlib_extra.get_peer_rosparams(dynamics_inv)

-- data flow: agregator_ref, kinematics_fwd -> dynamics_inv
depl:connect("agregator_ref.out_joints_sorted", "dynamics_inv.in_joints_sorted", rtt.Variable("ConnPolicy"));
depl:connect("agregator_ref.out_supports_sorted", "dynamics_inv.in_supports_sorted", rtt.Variable("ConnPolicy"));
depl:connect("odometry_ref.out_base", "dynamics_inv.in_base", rtt.Variable("ConnPolicy"));
depl:connect(timer.agregator.port, "dynamics_inv.sync_step", rtt.Variable("ConnPolicy"));
-- connect to RobotModel
depl:connectServices("dynamics_inv", "agregator_ref")
-- publish tf to ROS
depl:stream("dynamics_inv.out_wrenches_fixed", ros:topic("~dynamics_inv/out_wrenches_fixed"))
depl:stream("dynamics_inv.out_joints_accel_sorted", ros:topic("~dynamics_inv/out_joints_accel_sorted"))

dynamics_inv:configure()
assert(dynamics_inv:start())
