-- 
-- SERVO TORQUE CONTROL MODULE
--
-- Setup servo_inv, servo_ident.
--
-- It's second stage control schema. 
--
-- dynamics_inv component provides desired torque, position, velocity and acceleration for servos
-- servo_inv (ServoInvParam) uses parameterized servo model (inertia, viscous friction, coulomb friction)
-- servo feedback coefficient Kp to calculates servo goals.
-- 
-- Servo feedback coeffitients are low enough to ensure compilance.
--
-- This file intendent to be required from a deployment script.
--

--
-- deloy logger, aggregator, resource_control, kinematics_fwd
--
require "motion_core"

-- 
-- Servo invertion component.
--

ros:import("sweetie_bot_servo_inv");
-- load component
depl:loadComponent("servo_inv","sweetie_bot::motion::ServoInvParam")
servo_inv = depl:getPeer("servo_inv")
-- load configuration from cpf
servo_inv:loadService("marshalling")
servo_inv:provides("marshalling"):loadProperties(config.file("servo_models.cpf"))
-- get ROS parameteres and services
config.get_peer_rosparams(servo_inv)

-- data flow: dynamics_inv -> servo_inv -> herkulex_sched
depl:connect("dynamics_inv.out_joints_accel_sorted", "servo_inv.in_joints_accel_fixed", rtt.Variable("ConnPolicy"));

-- assert(servo_inv:start())

--
-- herkulex subsystem
--

require "herkulex_feedback"

-- data flow: servo_inv -> herkulex/sched
for name, group in pairs(herkulex) do
	depl:connect("servo_inv.out_goals", "herkulex/"..name.."/sched.in_goals", rtt.Variable("ConnPolicy"))
end
-- data flow (setup): herkulex/array -> aggregator_ref
for name, group in pairs(herkulex) do
	depl:connect("herkulex/"..name.."/array.out_joints", "aggregator_ref.in_joints", rtt.Variable("ConnPolicy"))
	group.array:publishJointStates()
end

-- data flow: herkulex_sched -> aggregator_real
for name, group in pairs(herkulex) do
	depl:connect("herkulex/"..name.."/sched.out_joints", "aggregator_real.in_joints", rtt.Variable("ConnPolicy"))
end

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
config.get_peer_rosparams(servo_ident)

-- data flow: herkulex_sched, dynamics_inv, servo_inv -> servo_ident
for name, group in pairs(herkulex) do
	depl:connect("herkulex/"..name.."/sched.out_joints", "servo_ident.in_joints_measured", rtt.Variable("ConnPolicy"))
end
depl:connect("dynamics_inv.out_joints_accel_sorted", "servo_ident.in_joints_accel_ref_fixed", rtt.Variable("ConnPolicy"));
depl:connect("servo_inv.out_goals", "servo_ident.in_goals_fixed", rtt.Variable("ConnPolicy"));
-- data flow: servo_ident -> servo_inv
depl:connect("servo_ident.out_servo_models", "servo_inv.in_servo_models", rtt.Variable("ConnPolicy"));
-- timer syncronization: start of next control cycle
depl:connect(timer.controller.port, "servo_ident.sync_step", rtt.Variable("ConnPolicy"));

-- assert(servo_ident:start())

--- start herkulex scheduler
for name, group in pairs(herkulex) do
	if group.array:isConfigured() then
		group.sched:start()
		print("herkulex."..name..".sched is started!")
	else
		print("WARNING: herkulex."..name..".array is not configured. herkulex."..name..".sched is not started.")
	end
end
