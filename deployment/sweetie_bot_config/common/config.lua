require "rttlib"
require "rttros"
require "lfs"

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
ros:import("rtt_dynamic_reconfigure")

config = {}

-- 1. Setup overalays

config.path = rttros.rospack_find("sweetie_bot_config") 

-- Set overlays fullpaths
config.overlay_paths = {} 
for i = 2,#arg do
	local overlay_path = arg[i]
	if not string.find(overlay_path, "^__") then
		if not string.find(overlay_path, "^/") then
			overlay_path = config.path .. "/" .. overlay_path
		end
		table.insert(config.overlay_paths, overlay_path)
	end
end
table.insert(config.overlay_paths, config.path .. "/common")

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

config.rosnode = gs:provides("rosnode")
config.node_fullname = config.rosnode:getName()
config.node_namespace = config.rosnode:getNamespace()
config.logger_root_category = string.gsub(string.gsub(config.node_fullname, "^/", ""), "/", ".")
-- configure logger
require "logger"
logger.set_root_category(config.logger_root_category)

-- Helper functions: parameters access and etc
require "rttlib_extra"

--print("logger_root", config.logger_root_category)
--print("node_name", config.node_fullname)

-- 3. Setup working dir to first overlay
lfs.chdir(config.overlay_paths[1])

-- 4. Provide configuration file finding function

-- Find configuration file in overlay and return full path to it. Return nul on failure
function config.file(conf_file)
	for i, path in ipairs(config.overlay_paths) do
		local full_path = path .. "/" .. conf_file
		local fd = io.open(full_path, "r")
		if fd then
			io.close(fd)
			print("config.file: use ", full_path)
			return full_path
		end
	end
	print("config.file: not found ", conf_file)
	return nil
end

-- 5. Pass control to overlay
require(arg[1])
