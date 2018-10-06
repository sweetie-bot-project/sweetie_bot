-- LOGGER DEPLOYMENT MODULE
--
-- Simplify logger initialization.

-- get Deployer
local depl = rtt.getTC():getPeer("Deployer")

-- get directory
reporting = {}
reporting.peers = {}
reporting.directory = config.get_rosparam("~reporting/directory", "string") or ""

-- Setup file reporter.
-- Usage example: 
-- add_filereporter("herkulex_statistics", { { name = "herkulex/sched" ,  ports= {"statistics"} } }, directory .. "/statistics.out", true)
function reporting.add_filereporter(reporter_name, peer_ports, file, complex_decompose)
	complex_decompose = complex_decompose or true
	success = true
	-- setup file
	if not string.find(file, "^/") then
		file = reporting.directory .. "/" .. file
	end
	-- load reporter 
	if not depl:loadComponent("reporting/" .. reporter_name, "OCL::FileReporting") then
		return nil
	end
	reporter = depl:getPeer("reporting/" .. reporter_name)
	-- configuere reporter: default
	reporter:getProperty("WriteHeader"):set(true)
	reporter:getProperty("ComplexDecompose"):set(complex_decompose)
	reporter:getProperty("ReportFile"):set(file)
	-- configuere reporter: rosparam 
	config.get_peer_rosparams(reporter)
	-- TODO period
	-- add access to peers
	for i, peer in ipairs(peer_ports) do
		success = depl:addPeer("reporting/" .. reporter_name, peer.name) and success
		-- report ports
		for i, port in ipairs(peer.ports) do
			success = reporter:reportPort(peer.name, port) and success
		end
	end
	if success then
		reporting[reporter_name] = reporter
		reporting.peers[reporter_name] = reporter
		return reporter
	else
		depl:unloadComponent(reporter:getName())
		return nil
	end
end

-- Unload reporter
function reporting.remove_reporter(reporter)
	if type(reporter) ~= "string" then
		reporter = reporter:getName()
	end
	table.remove(reporting, reporter)
	table.remove(reporting.peers, reporter)
	depl:unloadComponent(reporter)
end

-- Reconfigure all reporters
function reporting.reconfigure()
	for name, reporter in pairs(reporting.peers) do
		reporter:cleanup()
		reporter:configure()
	end
end

-- Stop all reporters
function reporting.start()
	for name, reporter in pairs(reporting.peers) do
		reporter:start()
	end
end

-- Start all reporters
function reporting.stop()
	for name, reporter in pairs(reporting.peers) do
		reporter:stop()
	end
end
