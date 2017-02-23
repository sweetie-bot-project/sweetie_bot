-- 
-- HERKULEX_* SUBSYSTEM REPORTING
--
-- Setup reporter components for servo_inv and herkulex_sched.
-- Component 
--
-- Intended to be run via config script.
--

reporting = {}

-- get directory
local directory = rttlib_extra.get_rosparam("~reporting/directory", "string")
directory = directory or "~"

-- helper function to setup reporter
local function setup_reporting(reporter_name, peer_ports, file, complex_decompose)
	complex_decompose = complex_decompose or false
	-- load reporter 
	if not depl:loadComponent("reporting/" .. reporter_name, "OCL::FileReporting") then
		return false
	end
	reporter = depl:getPeer("reporting/" .. reporter_name)
	reporting[reporter_name] = reporter
	-- configuere reporter: default
	reporter:getProperty("WriteHeader"):set(true)
	reporter:getProperty("ReportFile"):set(file)
	-- configuere reporter: rosparam 
	rttlib_extra.get_peer_rosparams(reporter)
	-- TODO period
	-- add access to peers
	for i, peer in ipairs(peer_ports) do
		depl:addPeer("reporting/" .. reporter_name, peer.name)
		-- report ports
		for i, port in ipairs(peer.ports) do
			reporter:reportPort(peer.name, port)
		end
	end
end

-- herkulex_sched statistics
setup_reporting("herkulex_statistics", { { name = "herkulex_sched" ,  ports= {"statistics"} } }, directory .. "/statistics.out")
-- servo states
setup_reporting("herkulex_states", { { name = "herkulex_sched" ,  ports= {"out_states"} } }, directory .. "/states.out")
-- control signals
setup_reporting("herkulex_control", { { name = "agregator_ref" ,  ports= {"out_joints_sorted"} }, { name = "agregator_real" ,  ports= {"out_joints_sorted"} } }, directory .. "/control.out")
