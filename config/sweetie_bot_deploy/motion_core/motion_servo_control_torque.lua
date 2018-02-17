-- 
-- SERVO TORQUE CONTROL MODULE
--
-- Setup servo_inv, servo_ident and agregator_real.
--
-- It's second stage control schema. 
--
-- dynamics_inv component provides desired torque, position, velocity and acceleration fo servos
-- servo_inv (ServoInv7Param) uses 7-parameter servo model (inertia, viscous friction, coulomb friction and stribeck friction) 
-- servo feedback coefficient Kp to calculates servo goals.
-- 
-- Servo feedback coeffitients are low enough to ensure compilance.
--
-- This file intendent to be required from a deployment script.
--

--
-- deloy logger, agregator, resource_control, kinematics_fwd
--
require "motion_core"

-- 
-- Servo invertion component.
--

ros:import("sweetie_bot_servo_inv");
-- load component
depl:loadComponent("servo_inv","sweetie_bot::motion::ServoInv7Param")
servo_inv = depl:getPeer("servo_inv")
-- load configuration from cpf
servo_inv:loadService("marshalling")
servo_inv:provides("marshalling"):loadProperties(config.file("servo_models.cpf"))
-- get ROS parameteres and services
rttlib_extra.get_peer_rosparams(servo_inv)

-- data flow: dynamics_inv -> servo_inv -> herkulex_sched
depl:connect("dynamics_inv.out_joints_accel_sorted", "servo_inv.in_joints_accel_fixed", rtt.Variable("ConnPolicy"));

assert(servo_inv:start())

--
-- herkulex subsystem
--

require "herkulex_feedback"

-- data flow: servo_inv -> herkulex/sched
depl:connect("servo_inv.out_goals", "herkulex/sched.in_goals", rtt.Variable("ConnPolicy"));
-- data flow (setup): herkulex/array -> agregator_ref
depl:connect("herkulex/array.out_joints", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
herkulex.array:publishJointStates()

-- 
-- Servo identification component.
--

-- load component
depl:loadComponent("servo_ident","sweetie_bot::motion::ServoIdent")
servo_ident = depl:getPeer("servo_ident")
-- load configuration from cpf
servo_ident:loadService("marshalling")
servo_ident:provides("marshalling"):loadProperties(config.file("servo_models.cpf"))
-- get ROS parameteres and services
rttlib_extra.get_peer_rosparams(servo_ident)

-- data flow: herkulex_sched, dynamics_ident -> servo_ident
depl:connect("herkulex/sched.out_joints", "servo_ident.in_joints_measured", rtt.Variable("ConnPolicy"));
depl:connect("dynamics_inv.out_joints_accel_sorted", "servo_ident.in_joints_accel_fixed", rtt.Variable("ConnPolicy"));
-- data flow: servo_ident -> servo_inv
depl:connect("servo_ident.out_servo_models", "servo_inv.in_servo_models", rtt.Variable("ConnPolicy"));
-- timer syncronization: start of next control cycle
depl:connect(timer.controller.port, "servo_ident.sync_step", rtt.Variable("ConnPolicy"));

assert(servo_ident:start())
--
-- agregator for real pose
--

-- load component
depl:loadComponent("agregator_real", "sweetie_bot::motion::Agregator");
agregator_real = depl:getPeer("agregator_real")
agregator_real:loadService("marshalling")
agregator_real:loadService("rosparam")
--set properties: publish on event
agregator_real:getProperty("publish_on_timer"):set(false)
agregator_real:getProperty("publish_on_event"):set(true)
--set properties
agregator_real:provides("marshalling"):loadProperties(config.file("kinematic_chains.cpf"));
agregator_real:provides("marshalling"):loadServiceProperties(config.file("kinematic_chains.cpf"), "robot_model")
agregator_real:provides("rosparam"):getParam("","robot_model")
--get other properties
rttlib_extra.get_peer_rosparams(agregator_real)
-- timer syncronization: publish at same time as controllers
depl:connect(timer.controller.port, "agregator_real.sync_step", rtt.Variable("ConnPolicy"));
-- publish pose to ROS
depl:stream("agregator_real.out_joints_sorted", ros:topic("~agregator_real/out_joints_sorted"))
-- start component
agregator_real:configure()
assert(agregator_real:start())

-- data flow: herkulex_sched -> agregator_real
depl:connect("herkulex/sched.out_joints", "agregator_real.in_joints", rtt.Variable("ConnPolicy"))

--- start herkulex scheduler
if herkulex.array:isConfigured() then
	herkulex.sched:start()
	print "herkulex.sched is started!"
else
	print "WARNING: herkulex.array is not configured. herkulex.sched is not started."
end


