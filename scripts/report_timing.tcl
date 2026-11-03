# Prints the worst setup paths on clk_74a (run from src/fpga: quartus_sta -t ../../scripts/report_timing.tcl)
project_open ap_core -revision ap_core
create_timing_netlist
read_sdc
update_timing_netlist
report_timing -setup -npaths 25 -detail summary -to_clock clk_74a -file output_files/worst_paths.txt
report_timing -setup -npaths 2 -detail full_path -to_clock clk_74a -file output_files/worst_full.txt
delete_timing_netlist
project_close
