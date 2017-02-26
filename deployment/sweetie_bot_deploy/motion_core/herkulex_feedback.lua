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

depl:loadComponent("herkulex/array","herkulex::HerkulexArray");
herkulex.array = depl:getPeer("herkulex/array")
herkulex.array:loadService("marshalling")
herkulex.array:provides("marshalling"):loadProperties(config.file("sweetie_bot_servos.cpf"))
rttlib_extra.get_peer_rosparams(herkulex.array)

depl:loadComponent("herkulex/driver","herkulex::HerkulexDriver");
herkulex.driver = depl:getPeer("herkulex/driver")
rttlib_extra.get_peer_rosparams(herkulex.driver)

depl:loadComponent("herkulex/sched","herkulex::HerkulexSched");
herkulex.sched = depl:getPeer("herkulex/sched")
rttlib_extra.get_peer_rosparams(herkulex.sched)

-- CONNECT OPERATIONS OF HERKULEX_* subsystem

depl:connectOperations("herkulex/array.sendPacketCM", "herkulex/sched.sendPacketCM");
depl:connectOperations("herkulex/sched.receivePacketCM", "herkulex/array.receivePacketCM");
depl:connectOperations("herkulex/sched.sendPacketDL", "herkulex/driver.sendPacket");
depl:connectOperations("herkulex/driver.receivePacket", "herkulex/sched.receivePacketDL");
depl:connectServices("herkulex/array","herkulex/sched");

-- Scheduler waits for completion of JOG command if those operations are connected.
depl:connectOperations("herkulex/sched.waitSendPacketDL", "herkulex/driver.waitSendPacket");

-- TIME PERIODS CONFIGURATION
require "timer"

-- Check configuration sanity
local herkulex_round_duration = herkulex.sched:getProperty("period_RT_JOG"):get() + herkulex.sched:getProperty("period_RT_read"):get() 
    + herkulex.sched:getProperty("period_CM"):get() + 2*herkulex.sched:getProperty("timeout"):get() 
local timer_period = timer.period
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
depl:connect(timer.herkulex.port, "herkulex/sched.sync", rtt.Variable("ConnPolicy"));

-- START HERKULEX_* SUBSYSTEM (without scheduler)

herkulex.driver:configure()
--assert(herkulex.driver:start())

assert(herkulex.sched:configure())
--assert(herkulex.array:configure())

