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

-- load and start timer
depl:loadComponent("timer", "OCL::TimerComponent")
timer.period = rttlib_extra.get_rosparam("~period", "float")
timer.timer = depl:getPeer("timer")
-- start timers:
--    timer_10: controllers timer
--    timer_20: herkulex timer
timer.timer:startTimer(10, timer.period)
timer.timer:wait(0, timer.period/2)
timer.timer:startTimer(20, timer.period)
-- register groups
timer.controller = { port = "timer.timer_10", shift = 0 }
timer.herkulex = { port = "timer.timer_20", shift = timer.period/2 }
