module testbench;
logic clk, reset;
logic [7:0] port_b_out;

  //module cpu(w_out, clk, reset);
cpu c1(port_b_out, clk, reset);
  always #5 clk = ~clk;
  
initial
  begin
    clk = 0; reset = 1;
    #10 reset = 0;
    #5000 $stop;
  end
endmodule
