-- 
-- HERKULEX_* SUBSYSTEM 
--
-- Setup herkulex_driver, herkulex_array, herkulex_sched
--
-- Intended to be run via config script.
--

require "timer"

ros:import("sweetie_bot_herkulex_control");

--- Load HerkulexDriver, HerkulexSched, HerkulexArray components. 
--
-- Components are named "herkulex/<group>/driver", "herkulex/<group>/array", "herkulex/<group>/sched".
-- Componts lua object are placed in herkulex (first parameter) table and can be accesed as herkulex.<group>.driver, 
-- herkulex.<group>.array and herkulex.<group>.sched.
--
-- @param herkulex table to hold reference to components. 
-- @param group new servos group name (string).
-- @param servos_description_cpf parameter  is .cpf configuration file name for HerkulexArray (string). It is processed with config.file function.
-- @return true if components are succesfully configured and HerkulexDriver is started.
--
local function setup_herkulex_subsystem(herkulex, group, servos_description_cpf)
	if type(group) ~= 'string' or string.len(group) == 0 then
		print("ERROR: setup_herkulex_subsystem group parameter must be string.")
		return false
	end
	-- add table for herkulex components
	local basename = 'herkulex/'..group
	herkulex[group] = {}

	-- LOAD HERKULEX_* COMPONENTS
	
	-- load HerkulexArray
	depl:loadComponent(basename .. "/array", "herkulex::HerkulexArray");
	herkulex[group].array = depl:getPeer(basename .. "/array")
	herkulex[group].array:loadService("marshalling")
	herkulex[group].array:provides("marshalling"):loadProperties(config.file(servos_description_cpf))
	config.get_peer_rosparams(herkulex[group].array)
	-- load HerkulexDriver
	depl:loadComponent(basename .. "/driver", "herkulex::HerkulexDriver");
	herkulex[group].driver = depl:getPeer(basename .. "/driver")
	config.get_peer_rosparams(herkulex[group].driver)
	-- load HerkulexSched
	depl:loadComponent(basename .. "/sched", "herkulex::HerkulexSched");
	herkulex[group].sched = depl:getPeer(basename .. "/sched")
	config.get_peer_rosparams(herkulex[group].sched)

	-- CONNECT OPERATIONS OF HERKULEX_* subsystem

	depl:connectOperations(basename.."/array.sendPacketCM", basename.."/sched.sendPacketCM");
	depl:connectOperations(basename.."/sched.receivePacketCM", basename.."/array.receivePacketCM");
	depl:connectOperations(basename.."/sched.sendPacketDL", basename.."/driver.sendPacket");
	depl:connectOperations(basename.."/driver.receivePacket", basename.."/sched.receivePacketDL");
	depl:connectServices(basename.."/array",basename.."/sched");

	-- Scheduler waits for completion of JOG command if those operations are connected.
	depl:connectOperations(basename.."/sched.waitSendPacketDL", basename.."/driver.waitSendPacket");

	-- TIME PERIODS CONFIGURATION

	-- Check configuration sanity
	local herkulex_round_duration = herkulex[group].sched:getProperty("period_RT_JOG"):get() + herkulex[group].sched:getProperty("period_RT_read"):get() 
		+ herkulex[group].sched:getProperty("period_CM"):get() + 2*herkulex[group].sched:getProperty("timeout"):get() 
	local timer_period = timer.herkulex_period
	local herkulex_array_timeout = herkulex[group].array:getProperty("timeout"):get()
	local herkulex_sched_timeout = herkulex[group].sched:getProperty("timeout"):get()
	local herkulex_sched_poll_size = herkulex[group].sched:getProperty("poll_round_size"):get()
	local herkulex_sched_period_RT_read = herkulex[group].sched:getProperty("period_RT_read"):get() + herkulex[group].sched:getProperty("timeout"):get()

	print(group..": timer_period", timer_period)
	print(group..": herkulex_round_duration", herkulex_round_duration)
	print(group..": herkulex_sched_period_RT_read", herkulex_sched_period_RT_read)
	print(group..": herkulex_sched_poll_size", herkulex_sched_poll_size)

	assert(herkulex_round_duration <= timer_period)
	assert(timer_period <= herkulex_array_timeout)
	assert(herkulex_sched_period_RT_read >= herkulex_sched_poll_size*herkulex_sched_timeout)

	-- connect to timer
	depl:connect(timer.herkulex.port, basename.."/sched.sync", rtt.Variable("ConnPolicy"));
	-- publish servo states for ROS
	depl:stream(basename.."/array.out_states", ros:topic("~herkulex/servo_states"))
	depl:stream(basename.."/sched.out_states", ros:topic("~herkulex/out_joints_ext"))

	-- START HERKULEX_* SUBSYSTEM (without scheduler)

	local success
	success = herkulex[group].driver:configure() and herkulex[group].driver:start()

	success = success and herkulex[group].sched:configure()
	success = success and herkulex[group].array:configure()

	return success
end

-- print all servo positions for given herkulex array
function debug.herkulex_print_position(herk_array)
    local servos = herk_array:listServos():totab()
	for i, servo in pairs(servos) do
		print(servo, herk_array:getRegisterRAM(servo, "absolute_position"))
	end
end

-- load herkulex groups
herkulex = {}

local groups = config.get_rosparam('~herkulex/groups', 'string[]')
assert(groups, "ERROR: Unable to load herkulex/groups parameter. Herkulex subsystem is not loaded.")

for i, group in ipairs(groups) do
	print("Setup herkulex group " .. group)
	setup_herkulex_subsystem(herkulex, group, 'herkulex_servos_'..group..'.cpf')
end


