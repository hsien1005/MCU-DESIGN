onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /testbench/clk
add wave -noupdate /testbench/reset
add wave -noupdate -radix hexadecimal /testbench/port_b_out
add wave -noupdate -radix hexadecimal /testbench/c1/spr/ram(37)
add wave -noupdate -radix hexadecimal /testbench/c1/w_q
