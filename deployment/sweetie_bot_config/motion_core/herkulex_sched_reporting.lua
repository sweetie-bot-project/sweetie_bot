# LOAD REPORTING COMPONET FOR REPORTING HERKULEX_SCHED

# statistics
loadComponent("reporter_herk_stat", "OCL::FileReporting");
reporter_herk_stat.WriteHeader = 0;
reporter_herk_stat.ReportFile = "statistics.out";
addPeer("reporter_herk_stat", "herk_sched");
reporter_herk_stat.reportPort("herk_sched", "statistics");

# servo detailed states
loadComponent("reporter_herk_state", "OCL::FileReporting");
reporter_herk_state.WriteHeader = 0;
reporter_herk_state.ComplexDecompose = 1;
reporter_herk_state.ReportFile = "state.out";
addPeer("reporter_herk_state", "herk_sched");
reporter_herk_state.reportPort("herk_sched", "out_states");
