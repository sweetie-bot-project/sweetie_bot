-- 
-- HERKULEX_* SUBSYSTEM 
--
-- Setup herkulex_driver, herkulex_array, herkulex_sched
--
-- Intended to be run via config script.
--

ros:import("sweetie_bot_herkulex_control");

-- LOAD HERKULEX_* COMPONENTS
herkulex = {}

depl:loadComponent("herkulex_array","herkulex::HerkulexArray");
herkulex.array = depl:getPeer("herkulex_array")
herkulex.array:loadService("marshalling")
herkulex.array:provides("marshalling"):loadProperties(config.file("sweetie_bot_servos.cpf"))
rttlib_extra.get_rosparam(herkulex.array)

depl:loadComponent("herkulex_driver","herkulex::HerkulexDriver");
herkulex.driver = depl:getPeer("herkulex_driver")
rttlib_extra.get_rosparam(herkulex.driver)

depl:loadComponent("herkulex_sched","herkulex::HerkulexSched");
herkulex.sched = depl:getPeer("herkulex_sched")
rttlib_extra.get_rosparam(herkulex.sched)

-- CONNECT OPERATIONS OF HERKULEX_* subsystem

depl:connectOperations("herkulex_array.sendPacketCM", "herkulex_sched.sendPacketCM");
depl:connectOperations("herkulex_sched.receivePacketCM", "herkulex_array.receivePacketCM");
depl:connectOperations("herkulex_sched.sendPacketDL", "herkulex_driver.sendPacket");
depl:connectOperations("herkulex_driver.receivePacket", "herkulex_sched.receivePacketDL");
depl:connectServices("herkulex_array","herkulex_sched");

-- Scheduler waits for completion of JOG command if those operations are connected.
depl:connectOperations("herkulex_sched.waitSendPacketDL", "herkulex_driver.waitSendPacket");

-- TIME PERIODS CONFIGURATION
-- Check configuration sanity
local herkulex_round_duration = herkulex.sched:getProperty("period_RT_JOG"):get() + herkulex.sched:getProperty("period_RT_read"):get() 
    + herkulex.sched:getProperty("period_CM"):get() + 2*herkulex.sched:getProperty("timeout"):get() 
local timer_period = tonumber(rttlib_extra.rosparam_get_string(config.node_fullname .. "/period"))
local herkulex_array_timeout = herkulex.array:getProperty("timeout"):get()
local herkulex_sched_timeout = herkulex.sched:getProperty("timeout"):get()
local herkulex_sched_poll_size = herkulex.sched:getProperty("poll_round_size"):get()
local herkulex_sched_period_RT_read = herkulex.sched:getProperty("period_RT_read"):get() + herkulex.sched:getProperty("timeout"):get()

print("timer_period", timer_period)
print("herkulex_round_duration", herkulex_round_duration)
print("herkulex_sched_period_RT_read", herkulex_sched_period_RT_read)
print("herkulex_sched_poll_size", herkulex_sched_poll_size)

assert(herkulex_round_duration <= timer_period)
assert(timer_period <= herkulex_array_timeout)
assert(herkulex_sched_period_RT_read >= herkulex_sched_poll_size*herkulex_sched_timeout)

-- connect to timer
-- TODO require "timer"
depl:connect("timer.timer_20", "herkulex_sched.sync", rtt.Variable("ConnPolicy"));

-- START HERKULEX_* SUBSYSTEM (without scheduler)

herkulex.driver:configure()
assert(herkulex.driver:start())

assert(herkulex.sched:configure())
assert(herkulex.array:configure())

