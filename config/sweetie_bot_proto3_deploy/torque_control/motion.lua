-- 
-- MOTION DEPLOYMENT FOR REAL ROBOT
--
-- Utilizes torque control schema with compilance.
--
-- Intended to be run via config script.
--

--
-- hardware independent modules
--
require "motion_core"

-- 
-- deploy servo position control schema
--
require "motion_servo_control_torque"
