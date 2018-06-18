--
-- TIMER DEPLOYMENT MODULE
--
require 'rttlib'
require 'rttlib_extra'

-- get Deployer
local depl = rtt.getTC():getPeer("Deployer")

-- import ros facilities
depl:import("rtt_ros") 
local ros = rtt.provides("ros")

-- Module table
timer = {}

-- load timer
depl:loadComponent("timer", "OCL::TimerComponent")
timer.timer = depl:getPeer("timer")
rttlib_extra.get_peer_rosparams(timer.timer)

-- set timer period
timer.period = rttlib_extra.get_rosparam("~timer/period", "float")
assert(timer.period, "ERROR: unable to get `~timer/period`")
timer.herkulex_period = rttlib_extra.get_rosparam("~timer/herkulex_period", "float")
assert(timer.herkulex_period, "ERROR: unable to get `~timer/herkulex_period`")

print("timer_period:" .. tostring(timer.period))
print("herkulex_timer_period:" .. tostring(timer.herkulex_period))

-- start timers:
--    timer_10: controllers timer,
--    timer_15: herkulex timer,
--    timer_20: aggregator timer: causes kinematic and dynamic calculatons
timer.timer:startTimer(10, timer.period)
timer.timer:startTimer(15, timer.herkulex_period)
timer.timer:wait(0, timer.period/2)
timer.timer:startTimer(20, timer.period)
-- register groups
timer.controller = { port = "timer.timer_10", shift = 0 }
timer.agregator = { port = "timer.timer_20", shift = timer.period/2 }
if timer.period == timer.herkulex_period then
	-- use the same timer for herkulex and controller components
	timer.timer:killTimer(15)
	timer.herkulex = { port = "timer.timer_10", shift = 0 }
else
	timer.herkulex = { port = "timer.timer_15", shift = 0 }
end
