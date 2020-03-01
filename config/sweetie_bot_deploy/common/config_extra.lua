require 'rttlib'

-- get Deployer
local depl = rtt.getTC():getPeer("Deployer")
-- import ros facilities
depl:import("rtt_ros") 
local ros = rtt.provides("ros")
-- import
ros:import("rtt_rosparam")
depl:loadService("Deployer", "rosparam")

config = config or {}
config.rosparam = depl:provides("rosparam")

--- Property setter for complex types.
-- It creates rtt.Variale of corresponding type, assign it value with fromtab() and set property.
-- @param peer OROCOS component (TaskContext)
-- @param prop property name (string)
-- @param value table which would be converted to desired type with fromtab() call.
function config.set_property(peer, prop, value) 
	local prop = peer:getProperty(prop)
	local prop_type = prop:info()["type"]

	var = rtt.Variable(prop_type);
	var:fromtab(value)
	prop:set(var)
end

--- Get parameter from ROS Parameter Server
-- Get parameter using rospram service as rtt.Variable, return totab() output,
-- @param name  parameter name (absolute, relative or private path)
-- @param typename string with rtt_ros typename: "bool", "int", "float", "double", "string" with, possibly, "[]" postfix.
-- @return table which contains totab() result.
function config.get_rosparam(name, typename)
	typename = string.lower(typename)
	local op_name
	if typename:sub(-2) == '[]' then
		op_name = 'getVectorOf' .. typename:sub(1,1):upper() .. typename:sub(2,-3)
	else
		op_name = 'get' .. typename:sub(1,1):upper() .. typename:sub(2)
	end
	local getOp = config.rosparam:getOperation(op_name)
	if getOp then
		local var = rtt.Variable(typename)
		if getOp(name, var) then
			return var:totab()
		end
	end
	return nil
end

--- Get Property object from peer.
-- This function envelops getProperty() call but does not produce error if property does not exists.
-- @param peer OROCOS component (TaskContext)
-- @param property_name property name (string)
-- @return rtt.Property or nil if property does not exists.
function config.get_property(peer, property_name)
	prop_names = peer:getPropertyNames()
	for i, name in pairs(prop_names) do
		if name == property_name then
			return peer:getProperty(name)
		end
	end
	return nil
end


--- Load properties to peer from ROS parameter server
-- This function uses "~<peer_name>/" namespace to find properties. Note, that if component name contains slash
-- it would be processed as separator between namespaces names. This can be used to organize components in groups.
--
-- Some parameters has special meaning:
-- * @c period component property as default value uses "~timer/period" ROS parameter.
-- * @c priority ROS parameter (int) is used to set peer RT priority.
-- * @c services ROS parameter (string[]) contains list of services that should be loaded in compnent.
--
-- After special parameters is processed component's and subservices' properties are assigned
-- by getAll() call (rosparam).
--
-- @param peer component (TaskContext). It name is used to construct parameter namespace. 
--
function config.get_peer_rosparams(peer)
	-- process some properties specifically
	-- `period` property is allways set to `~timer/period'
	local prop = config.get_property(peer, "period")
	if prop then
		local period = config.get_rosparam("~timer/period", "float")
		if period then
			prop:set(period) 
		else
			print("WARNING: `~timer/period` is paramerer is not set")
		end
	end
	-- use rosparam `priority` to set component RT priority
	local priority = config.get_rosparam("~" .. peer:getName() .. "/priority", "int")
	if priority then
		depl:setActivity(peer:getName(), 0, priority, rtt.globals.ORO_SCHED_RT)
	end
	-- use rosparam `activity_period` to set component period
	local period = config.get_rosparam("~" .. peer:getName() .. "/activity_period", "float")
	if period then
		peer:setPeriod(period)
	end
	-- use rosparam `services` to load additionaol services
	services = config.get_rosparam("~" .. peer:getName() .. "/services", "string[]")
	if services then
		for k, service in ipairs(services) do 
			-- print("Load service " .. service .. " into " .. peer:getName())
			peer:loadService(service)
		end
	end
	-- load parameters from parameter server
	peer:loadService("rosparam")
	peer:provides("rosparam"):getAll();
end

