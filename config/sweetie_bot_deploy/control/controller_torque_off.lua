--
-- TORQUE OFF controller
--
-- Setup TorqueMainSwitch controller
--
-- Intended to be run via config script.
--
require "motion"

controller = controller or {}

-- load controller
ros:import("sweetie_bot_controller_joint_space")
depl:loadComponent("controller/torque_off", "sweetie_bot::motion::controller::TorqueMainSwitch")
controller.torque_off = depl:getPeer("controller/torque_off")
-- register controller
resource_control.register_controller(controller.torque_off)
-- timer
depl:connect(timer.controller.port, "controller/torque_off.sync", rtt.Variable("ConnPolicy"))
-- data flow: controller -> agregator_ref
depl:connect("controller/torque_off.out_joints_ref", "agregator_ref.in_joints", rtt.Variable("ConnPolicy"))
-- data flow: agregator_real -> controller
depl:connect("agregator_real.out_joints_sorted", "controller/torque_off.in_joints_actual", rtt.Variable("ConnPolicy"))
-- connect to RobotModel
depl:connectServices("controller/torque_off", "agregator_ref")
-- present herkulex subsystem and set corresponding options
local herkulex_arrays = {}
local herkulex_scheds = {}
for name, group in pairs(herkulex) do
	depl:addPeer("controller/torque_off", group.array:getName())
	depl:addPeer("controller/torque_off", group.sched:getName())
	-- form herkulex_arrays and herkulex_scheds lists
	table.insert(herkulex_scheds, "herkulex/"..name.."/sched")
	table.insert(herkulex_arrays, "herkulex/"..name.."/array")
end
-- set herkulex_arrays and herkulex_scheds properties
rttlib_extra.set_property(controller.torque_off, 'herkulex_arrays', 'string[]', herkulex_arrays )
rttlib_extra.set_property(controller.torque_off, 'herkulex_scheds', 'string[]', herkulex_scheds )
-- get ROS configuration
rttlib_extra.get_peer_rosparams(controller.torque_off)
-- advertise actionlib interface
controller.torque_off:loadService("actionlib")
controller.torque_off:provides("actionlib"):connect("~controller/torque_off")
-- advertise ROS operation
controller.torque_off:loadService("rosservice")
controller.torque_off:provides("rosservice"):connect("rosSetOperational", config.node_fullname .. "/controller/torque_off/set_torque_off", "std_srvs/SetBool")

-- prepare to start
assert(controller.torque_off:configure(), "ERROR: unable to configure controller/torque_off")
