-- 
-- BASIC MOTION DEPLOYMENT FOR REAL ROBOT
--
-- Utilizes postion control schema (no compilance).
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
require "motion_servo_control_position"

-- 
-- deploy imu based odometry
--
require "odometry_imu"
