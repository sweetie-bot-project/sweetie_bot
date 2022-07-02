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
depl:loadComponent("imu_real", "sweetie_bot::motion::PoseFusionBNO080RVC")
imu_real = depl:getPeer("imu_real")
-- get ROS parameteres and services
config.get_peer_rosparams(imu_real)
-- data flow
depl:connect("odometry_ref.out_base", "imu_real.in_base_ref", rtt.Variable("ConnPolicy"));
-- timer
depl:connect(timer.controller.port, "imu_real.sync", rtt.Variable("ConnPolicy"))
-- data flow to ROS
depl:stream("imu_real.out_imu", ros:topicBuffer("~imu_real/out_imu", 50))
depl:stream("imu_real.out_tf", ros:topic("~imu_real/out_tf"))

-- configure and start component

local success = imu_real:configure() and imu_real:start()
if not success then
	print("WARNING: imu_real component is not started!")
end

