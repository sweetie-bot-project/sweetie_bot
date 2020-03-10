require "rttlib"
require "rttros"
require "lfs"

require "readline"
require "complete"
rttlib.color = true

tc = rtt.getTC()
depl = tc:getPeer("Deployer")

-- table for deployment-related function and variables
config = {} 
-- table for robot debug/testing code, filled in modules.
debug = {} 

-- 0. Simplify access to OROCOS facilities in children scripts

-- Deplyer
depl = rtt.getTC():getPeer("Deployer")
-- GlobalService
gs = rtt.provides()
-- ROS
depl:import("rtt_ros") 
ros = rtt.provides("ros")
-- ROS plugins
ros:import("rtt_rosnode")
ros:import("rtt_rosparam")
ros:import("rtt_actionlib")
ros:import("rtt_dynamic_reconfigure")

--
local function str_array_join(array) 
	result = ""
	if array then
		for i, s in ipairs(array) do result = result .. " " .. s end
	end
	return result
end

-- 0. Parse arguments: divide scripts, overlays and ros parameters
config.path = rttros.rospack_find("sweetie_bot_deploy") 
config.modules = {}
config.overlay_paths = {} 
for i, a in ipairs(arg) do
	if string.find(a, "^__") then
		-- argument is ROS parameter
		if string.find(a, "^__name:=[^ ]+") then
			config.node_fullname = string.sub(a, 9)
		elseif string.find(a, "^__ns:=[^ ]+") then
			config.node_namespace = string.sub(a, 7)
		elseif string.find(a, "^__param_ns:=[^ ]+") then
			config.param_namespace = string.sub(a, 13)
		end
	elseif string.find(a, "\.lua$") then
		-- argument is lua script name, i.e. module to run
		a = string.sub(a, 1, -5)
		table.insert(config.modules, a)
		print(config.modules[1])
	elseif not string.find(a, "[^ ]+:=[^ ]+") then
		-- argument is overlay directory
		if not string.find(a, "^/") then
			-- path is not absolute
			a = config.path .. "/" .. a
		end
		table.insert(config.overlay_paths, a)
	end
end
print("modules:", str_array_join(config.modules))
print("overlays:", str_array_join(config.overlay_paths))
		
-- 1. Setup overalays

-- Add default paths
table.insert(config.overlay_paths, config.path .. "/common")
 -- Allow to call scripts from any overlay using dot notation: "motion_core.motion.lua"
table.insert(config.overlay_paths, config.path)

-- Set LUA_PATH to access overlays
local lua_path = ";./?.lua;" 
for i, path in ipairs(config.overlay_paths) do
	lua_path = lua_path .. path .. "/?.lua;"
end
config.package_path_old = package.path
package.path = lua_path .. string.gsub(package.path, "^;*%./%?%.lua;*", "")

-- print("LUA_PATH", package.path)

-- TODO Function to add overlays from children scripts

-- 2. Set prefix using arg1 It is used to set
--    * logger categories,
--    * rosparam server.

-- get node name and namespace
-- if ros additional functions are supported 
if ros.getNodeName then
	config.node_fullname = ros:getNodeName()
	config.node_namespace = ros:getNamespace()
else
	-- otherwise: use args
	config.node_fullname = config.node_fullname or "motion"
	config.node_namespace = config.node_namespace or os.getenv("ROS_NAMESPACE")
	if not config.node_namespace then
		config.node_namespace = "/"
		config.node_fullname = "/" .. config.node_fullname
	else
		config.node_fullname = config.node_namespace .. "/" .. config.node_fullname
	end
end
if not config.param_namespace then
	config.param_namespace = config.node_namespace
end
print("node_fullname:", config.node_fullname)
print("node_namespace:", config.node_namespace)
print("param_namespace:", config.param_namespace)

-- construct category name
config.logger_root_category = string.gsub(string.gsub(config.node_fullname, "^/", ""), "/", ".")
-- configure logger
require "logger"
logger.set_root_category(config.logger_root_category)

print("logger_root: ", config.logger_root_category)

-- 3. Setup working dir to first overlay
lfs.chdir(config.overlay_paths[1])

-- 4. Provide helper functions

-- Helper functions: parameters access and etc
require "config_extra"

--- Find configuration file and return full path to it. Return nul on failure
--
-- First of all it tries to find parameter with the same name on ROS parameter server.
-- If path is relative  "conf_file/" namespace is assumed. If parameter does not exists then 
-- search for configuration file under defined overlays. 
--
-- All not valid characters in parameter name is replaced by "_"
--
-- @param conf_file configuration file name (string).
--
function config.file(conf_file)
	-- add prefix to parameter name and replace not valid characters
	local conf_file_param = string.gsub(conf_file, "[^A-Za-z0-9/~_]", "_")
	if not string.find(conf_file_param, "^[~/]") then
		conf_file_param = "conf_file/" .. conf_file_param
	end
	-- try to get rosparam
	local buffer = config.get_rosparam(conf_file_param, 'string')
	if buffer then
		print("config.file: use ROS parameter ", conf_file_param)
		-- create directory in /tmp/<node_fullname>
		local full_path = "/tmp" .. config.node_namespace 
		local cd = lfs.currentdir()
		full_path = "/tmp" .. config.node_namespace
		local success = lfs.chdir(full_path) or lfs.mkdir(full_path)
		full_path = "/tmp" .. config.node_fullname
		success = success and (lfs.chdir(full_path) or lfs.mkdir(full_path))
		lfs.chdir(cd)
		assert(success, "config.file: Unable to create directory " .. full_path)
		-- create file
		full_path = full_path .. "/" .. string.gsub(conf_file_param, "[~/]", "_")
		local file = io.open(full_path, "w");
		assert(file, "config.file: Unable to create file " .. full_path)
		-- store parameter to file
		success = file:write(buffer) and file:close()
		assert(success, "config.file: Unable to write file " .. full_path)
		-- return full path
		return full_path
	end
	-- try to find file in overlays
	for i, path in ipairs(config.overlay_paths) do
		local full_path = path .. "/" .. conf_file
		local fd = io.open(full_path, "r")
		if fd then
			io.close(fd)
			print("config.file: use ", full_path)
			return full_path
		end
	end
	-- file not found
	print("config.file: not found ", conf_file)
	return nil
end

-- 5. Pass control to modules
for i,mod in ipairs(config.modules) do
	require(mod)
end
