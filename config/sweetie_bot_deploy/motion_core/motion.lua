-- 
-- BASIC MOTION DEPLOYMENT
--
-- Setup logger, agreagator_ref, resource control, servo_inv, herkulex_*, agregator_real
--
-- Intended to be run via config script.
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
rttlib_extra.get_peer_rosparams(servo_inv)

-- timer syncronization
depl:connect(timer.agregator.port, "servo_inv.sync_step", rtt.Variable("ConnPolicy"));

-- data flow: agregator_ref -> servo_inv -> herkulex_sched
depl:connect("agregator_ref.out_joints_sorted", "servo_inv.in_joints_fixed", rtt.Variable("ConnPolicy"));

assert(servo_inv:start())

-- herkulex subsystem
require "herkulex_feedback"

-- data flow: servo_inv -> herkulex/sched
depl:connect("servo_inv.out_goals", "herkulex/sched.in_goals", rtt.Variable("ConnPolicy"));
-- data flow (setup): herkulex/array -> agregator_ref
depl:connect("herkulex/array.out_joints", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
herkulex.array:publishJointStates()

-- agregator for real pose
ros:import("rtt_roscomm")
ros:import("sweetie_bot_agregator");
ros:import("sweetie_bot_robot_model");
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


