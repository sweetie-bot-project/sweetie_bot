-- 
-- POSITION SERVO CONTROL MODULE
--
-- Setup servo_inv, herkulex_* and aggregator_real.
--
-- It's first stage control schema. Servo goals are set to position published by aggregator_ref.
-- Feedback coeffitients are high enough to ensure position tracking.
--
-- This file intendent to be required from a deployment script.
--

--
-- deloy logger, aggregator, resource_control, kinematics_fwd
--
require "motion_core"

-- 
-- Servo trajectore generator invertion component.
--

ros:import("sweetie_bot_servo_inv");
-- load component
depl:loadComponent("servo_inv","sweetie_bot::motion::ServoInvExtrapolate")
servo_inv = depl:getPeer("servo_inv")
config.get_peer_rosparams(servo_inv)

-- timer syncronization
-- depl:connect(timer.aggregator.port, "servo_inv.sync_step", rtt.Variable("ConnPolicy"));

-- data flow: aggregator_ref -> servo_inv -> herkulex_sched
depl:connect("aggregator_ref.out_joints_sorted", "servo_inv.in_joints_fixed", rtt.Variable("ConnPolicy"));

assert(servo_inv:start(), "ERROR: Unable to start servo_inv.")

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
	depl:stream("herkulex/"..name.."/sched.out_states", ros:topic("~herkulex/joint_states"))
        depl:stream("herkulex/"..name.."/array.out_states", ros:topic("~herkulex/servo_states"))
        array = depl:getPeer("herkulex/"..name.."/array")
        array:setPeriod(0.057)
end

--- start herkulex scheduler and array
for name, group in pairs(herkulex) do
	if group.array:isConfigured() then
		group.sched:start()
		group.array:start()
		print("herkulex."..name..".sched is started!")
	else
		print("WARNING: herkulex."..name..".array is not configured. herkulex."..name..".sched is not started.")
	end
end


