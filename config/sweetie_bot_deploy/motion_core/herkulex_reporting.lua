-- 
-- HERKULEX_* SUBSYSTEM REPORTING
--
-- Setup reporter components for servo_inv and herkulex_sched.
-- Component 
--
-- Intended to be run via config script.
--
require "reporting"

-- herkulex_sched statistics
reporting.add_filereporter("herkulex_statistics", { { name = "herkulex/sched" ,  ports= {"statistics"} } }, directory .. "/statistics.out")
-- servo statej
reporting.add_filereporter("herkulex_states", { { name = "herkulex/sched" ,  ports= {"out_states"} } }, directory .. "/states.out")
-- control signals
reporting.add_filereporter("herkulex_control", { { name = "agregator_ref" ,  ports= {"out_joints_sorted"} }, { name = "agregator_real" ,  ports= {"out_joints_sorted"} } }, directory .. "/control.out")
