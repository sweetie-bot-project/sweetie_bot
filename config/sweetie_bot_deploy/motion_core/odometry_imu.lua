--
-- ODOMETRY: IMU component
--
-- Setup imu_real componentbased on RTIMULib library and forward kinematics for 
--
-- Intended to be run via config script.
--

--
-- imu component
--

ros:import("sweetie_bot_imu_fusion")
depl:loadComponent("imu_real", "sweetie_bot::motion::PoseFusionRTIMULib")
imu_real = depl:getPeer("imu_real")
-- configuration file: split filename in path and basename, ensure that basename has '.ini' extension
local config_filepath = config.file("RTIMULib.ini")
if config_filepath then
	local path, basename = string.match(config_filepath, "(.*)/(.*)")
	if string.sub(basename, -4) == '.ini' then
		basename = string.sub(basename, 0, -5)
	else
		lfs.link(config_filepath, path .. '/' .. basename .. '.ini', true)
	end
	imu_real:getProperty("rtimulib_config_file"):set(basename)
	imu_real:getProperty("rtimulib_config_path"):set(path)
end 
-- get ROS parameteres and services
config.get_peer_rosparams(imu_real)
-- data flow to ROS
depl:stream("imu_real.out_imu", ros:topic("~imu_real/out_imu"))
depl:stream("imu_real.out_tf", ros:topic("~imu_real/out_tf"))

-- configure and start component

local success = imu_real:configure() and imu_real:start()
if not success then
	print("WARNING: imu_real component is not started!")
end

