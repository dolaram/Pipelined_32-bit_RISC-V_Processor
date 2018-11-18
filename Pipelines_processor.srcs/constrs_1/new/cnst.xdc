## Clock signal
set_property PACKAGE_PIN W5 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -add -name sys_clk_pin -period 40 -waveform {0 20} [get_ports clk]
##Buttons
set_property PACKAGE_PIN U18 [get_ports rst]
set_property IOSTANDARD LVCMOS33 [get_ports rst]
## LEDs
set_property PACKAGE_PIN U16 [get_ports {R4_out[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {R4_out[0]}]
set_property PACKAGE_PIN E19 [get_ports {R4_out[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {R4_out[1]}]
set_property PACKAGE_PIN U19 [get_ports {R4_out[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {R4_out[2]}]
set_property PACKAGE_PIN V19 [get_ports {R4_out[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {R4_out[3]}]
set_property PACKAGE_PIN W18 [get_ports {R4_out[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {R4_out[4]}]
set_property PACKAGE_PIN U15 [get_ports {R4_out[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {R4_out[5]}]
set_property PACKAGE_PIN U14 [get_ports {R4_out[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {R4_out[6]}]
set_property PACKAGE_PIN V14 [get_ports {R4_out[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {R4_out[7]}]