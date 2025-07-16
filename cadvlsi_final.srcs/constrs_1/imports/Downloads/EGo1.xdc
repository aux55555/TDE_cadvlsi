# 设置时钟约束
create_clock -period 10.000 [get_ports clk]

# 复位信号 rst
# 設置時鐘信號的引腳位置和 I/O 標準
set_property PACKAGE_PIN P17 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]

set_property PACKAGE_PIN P15 [get_ports rst]
set_property IOSTANDARD LVCMOS33 [get_ports rst]

# 输入信号 x (10 位)
set_property PACKAGE_PIN R1 [get_ports {x[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[0]}]
set_property PACKAGE_PIN N4 [get_ports {x[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[1]}]
set_property PACKAGE_PIN M4 [get_ports {x[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[2]}]
set_property PACKAGE_PIN R2 [get_ports {x[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[3]}]
set_property PACKAGE_PIN P2 [get_ports {x[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[4]}]
set_property PACKAGE_PIN P3 [get_ports {x[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[5]}]
set_property PACKAGE_PIN P4 [get_ports {x[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[6]}]
set_property PACKAGE_PIN P5 [get_ports {x[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[7]}]
set_property PACKAGE_PIN T5 [get_ports {x[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[8]}]
set_property PACKAGE_PIN T3 [get_ports {x[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {x[9]}]

# 输入信号 y (10 位)
set_property PACKAGE_PIN R3 [get_ports {y[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[0]}]
set_property PACKAGE_PIN V4 [get_ports {y[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[1]}]
set_property PACKAGE_PIN V5 [get_ports {y[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[2]}]
set_property PACKAGE_PIN V2 [get_ports {y[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[3]}]
set_property PACKAGE_PIN U2 [get_ports {y[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[4]}]
set_property PACKAGE_PIN U3 [get_ports {y[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[5]}]
set_property PACKAGE_PIN K3 [get_ports {y[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[6]}]
set_property PACKAGE_PIN M1 [get_ports {y[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[7]}]
set_property PACKAGE_PIN L1 [get_ports {y[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[8]}]
set_property PACKAGE_PIN K6 [get_ports {y[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {y[9]}]

# 输出信号 busy
set_property PACKAGE_PIN J5 [get_ports busy]
set_property IOSTANDARD LVCMOS33 [get_ports busy]

# 输出信号 valid
set_property PACKAGE_PIN H5 [get_ports valid]
set_property IOSTANDARD LVCMOS33 [get_ports valid]

# 输出信号 d (8 位)
set_property PACKAGE_PIN H6 [get_ports {d[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {d[0]}]
set_property PACKAGE_PIN K1 [get_ports {d[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {d[1]}]
set_property PACKAGE_PIN K2 [get_ports {d[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {d[2]}]
set_property PACKAGE_PIN J2 [get_ports {d[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {d[3]}]
set_property PACKAGE_PIN J3 [get_ports {d[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {d[4]}]
set_property PACKAGE_PIN H4 [get_ports {d[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {d[5]}]
set_property PACKAGE_PIN J4 [get_ports {d[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {d[6]}]
set_property PACKAGE_PIN G3 [get_ports {d[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {d[7]}]




# 七段顯示器的段位信號 seg7[7:0]
set_property PACKAGE_PIN D4 [get_ports {seg7[0]}]
set_property PACKAGE_PIN E3 [get_ports {seg7[1]}]
set_property PACKAGE_PIN D3 [get_ports {seg7[2]}]
set_property PACKAGE_PIN F4 [get_ports {seg7[3]}]
set_property PACKAGE_PIN F3 [get_ports {seg7[4]}]
set_property PACKAGE_PIN E2 [get_ports {seg7[5]}]
set_property PACKAGE_PIN D2 [get_ports {seg7[6]}]
set_property PACKAGE_PIN H2 [get_ports {seg7[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7_sel[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7_sel[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7_sel[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg7_sel[0]}]


set_property PACKAGE_PIN G1 [get_ports {seg7_sel[3]}]
set_property PACKAGE_PIN F1 [get_ports {seg7_sel[2]}]
set_property PACKAGE_PIN E1 [get_ports {seg7_sel[1]}]
set_property PACKAGE_PIN G6 [get_ports {seg7_sel[0]}]




