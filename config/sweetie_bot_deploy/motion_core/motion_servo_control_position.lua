-- 
-- POSITION SERVO CONTROL MODULE
--
-- Setup servo_inv, herkulex_* and agregator_real.
--
-- It's first stage control schema. Servo goals are set to position published by agregator_ref.
-- Feedback coeffitients are high enough to ensure position tracking.
--
-- This file intendent to be required from a deployment script.
--

--
-- deloy logger, agregator, resource_control, kinematics_fwd
--
require "motion_core"

-- 
-- Servo trajectore generator invertion component.
--

ros:import("sweetie_bot_servo_inv");
-- load component
depl:loadComponent("servo_inv","sweetie_bot::motion::ServoInvLead")
servo_inv = depl:getPeer("servo_inv")
config.get_peer_rosparams(servo_inv)

-- timer syncronization
depl:connect(timer.agregator.port, "servo_inv.sync_step", rtt.Variable("ConnPolicy"));

-- data flow: agregator_ref -> servo_inv -> herkulex_sched
depl:connect("agregator_ref.out_joints_sorted", "servo_inv.in_joints_fixed", rtt.Variable("ConnPolicy"));

assert(servo_inv:start(), "ERROR: Unable to start servo_inv.")

--
-- herkulex subsystem
--

require "herkulex_feedback"

-- data flow: servo_inv -> herkulex/sched
for name, group in pairs(herkulex) do
	depl:connect("servo_inv.out_goals", "herkulex/"..name.."/sched.in_goals", rtt.Variable("ConnPolicy"))
end
-- data flow (setup): herkulex/array -> agregator_ref
for name, group in pairs(herkulex) do
	depl:connect("herkulex/"..name.."/array.out_joints", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
	group.array:publishJointStates()
end

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
config.get_peer_rosparams(agregator_real)
-- timer syncronization: publish at same time as controllers
depl:connect(timer.controller.port, "agregator_real.sync_step", rtt.Variable("ConnPolicy"));
-- publish pose to ROS
depl:stream("agregator_real.out_joints_sorted", ros:topic("~agregator_real/out_joints_sorted"))
-- start component
agregator_real:configure()
assert(agregator_real:start(), "ERROR: Unable to start agregator_real.")

-- data flow: herkulex_sched -> agregator_real
for name, group in pairs(herkulex) do
	depl:connect("herkulex/"..name.."/sched.out_joints", "agregator_real.in_joints", rtt.Variable("ConnPolicy"))
end

--- start herkulex scheduler
for name, group in pairs(herkulex) do
	if group.array:isConfigured() then
		group.sched:start()
		print("herkulex."..name..".sched is started!")
	else
		print("WARNING: herkulex."..name..".array is not configured. herkulex."..name..".sched is not started.")
	end
end


