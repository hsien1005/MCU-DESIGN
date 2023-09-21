module cpu(port_b_out, clk, rst);

input clk, rst;
output logic [7:0]port_b_out;

logic [7:0]w_q;
logic [10:0]pc_q, mar_q, pc_next, pc_n;
logic [13:0]ir_q, prog_data;
logic [3:0]op;
logic [7:0]alu_out, ram_out, mux_out, databus, RAM_mux, bcf_mux, bsf_mux;
logic load_pc, load_mar, load_ir, load_w, reset_ir, sel_pc, ram_en, d, sel_alu, sel_bus;
logic load_port_b, addr_port_b;
logic [2:0]ps, ns, sel_bit;
logic btfsc_skip_bit, btfss_skip_bit, btfsc_btfss_skip_bit, aluout_zero;
logic [1:0]sel_RAM_mux;

//-----------------PC
assign pc_n = pc_q + 1;

//always @(posedge clk)
//  begin
//    if (sel_pc) pc_next <= ir_q[10:0];
//    else pc_next <= pc_n;
//end
assign pc_next = sel_pc ? ir_q[10:0] : pc_n;
  
always @(posedge clk)
  begin
    if (rst) pc_q <= 11'b0;
    else if (load_pc) pc_q <= pc_next;
end

//------------------MAR
always @(posedge clk)
  begin
    if (rst) mar_q <= 11'b0;
    else if (load_mar) mar_q <= pc_q;
  end

//------------------IR
assign reset_ir = rst;

always @(posedge clk)
  begin
    if (reset_ir) ir_q <= 14'b0;
    else if (load_ir) ir_q <= prog_data;
  end
  
//------------------Rom
Program_Rom prom(prog_data, mar_q);

//------------------RAM
single_port_ram_128X8 spr(databus, ir_q[6:0], ram_en, clk, ram_out);

//always @(posedge clk)
//  begin
//    if (sel_alu) mux_out <= ram_out;
//    else mux_out <= ir_q[7:0];
//  end
  
assign mux_out = sel_alu ?  RAM_mux : ir_q[7:0] ;

//------------------ALU
always @(*)
begin
  case(op)
    4'h0: alu_out = mux_out + w_q;
    4'h1: alu_out = mux_out - w_q;
    4'h2: alu_out = mux_out & w_q;
    4'h3: alu_out = mux_out | w_q;
    4'h4: alu_out = mux_out ^ w_q;
    4'h5: alu_out = mux_out;
	4'h6: alu_out = mux_out + 1;
	4'h7: alu_out = mux_out - 1;
	4'h8: alu_out = 0;
	4'h9: alu_out = ~mux_out;
	4'hA: alu_out = {mux_out[7], mux_out[7:1]};//ASRF:算術向右移
	4'hB: alu_out = {mux_out[6:0], 1'b0};
	4'hC: alu_out = {1'b0, mux_out[7:1]};
	4'hD: alu_out = {mux_out[6:0], mux_out[7]};
	4'hE: alu_out = {mux_out[0], mux_out[7:1]};
	4'hF: alu_out = {mux_out[3:0], mux_out[7:4]};
	
    default: alu_out = mux_out + w_q;
  endcase
end

//------------------Port_b
always_ff @(posedge clk)
	if(rst) port_b_out <= 0;
	else if(load_port_b) port_b_out <= databus;

assign addr_port_b = (ir_q[6:0] == 7'h0d);


assign ANDWF  = (ir_q[13:8] 	== 6'b00_0101);
assign ADDWF  = (ir_q[13:8] 	== 6'b00_0111);
assign GOTO   = (ir_q[13:11] 	== 3'b10_1);
assign CLRF   = (ir_q[13:7] 	== 7'b00_0001_1);
assign CLRW   = (ir_q[13:2] 	== 12'b00_0001_0000_00);
assign COMF   = (ir_q[13:8] 	== 6'b00_1001);
assign DECF   = (ir_q[13:8] 	== 6'b00_0011);
assign MOVLW  = (ir_q[13:8] 	== 6'b11_0000);
assign ADDLW  = (ir_q[13:8] 	== 6'b11_1110);
assign SUBLW  = (ir_q[13:8] 	== 6'b11_1100);
assign ANDLW  = (ir_q[13:8] 	== 6'b11_1001);
assign IORLW  = (ir_q[13:8] 	== 6'b11_1000);
assign XORLW  = (ir_q[13:8] 	== 6'b11_1010);
assign INCF   = (ir_q[13:8] 	== 6'b00_1010);
assign IORWF  = (ir_q[13:8] 	== 6'b00_0100);
assign MOVF   = (ir_q[13:8] 	== 6'b00_1000);
assign MOVWF  = (ir_q[13:7] 	== 7'b00_0000_1);
assign SUBWF  = (ir_q[13:8] 	== 6'b00_0010);
assign XORWF  = (ir_q[13:8] 	== 6'b00_0110);
assign BCF	  = (ir_q[13:10] 	== 4'b01_00);
assign BSF	  = (ir_q[13:10] 	== 4'b01_01);
assign BTFSC  = (ir_q[13:10] 	== 4'b01_10);
assign BTFSS  = (ir_q[13:10] 	== 4'b01_11);
assign DECFSZ = (ir_q[13:8]	    == 6'b00_1011);
assign INCFSZ = (ir_q[13:8] 	== 6'b00_1111);
assign ASRF	  = (ir_q[13:8] 	== 6'b11_0111);
assign LSLF   = (ir_q[13:8] 	== 6'b11_0101);
assign LSRF   = (ir_q[13:8] 	== 6'b11_0110);
assign RLF	  = (ir_q[13:8] 	== 6'b00_1101);
assign RRF    = (ir_q[13:8] 	== 6'b00_1100);
assign SWAPF  = (ir_q[13:8] 	== 6'b00_1110);


 always @(posedge clk)
    begin 
      if (rst) w_q <= 8'b0;
      else if (load_w) w_q <= alu_out;
    end
   
   
assign databus = sel_bus ?  w_q : alu_out ;

//controller

parameter T0 = 0;
parameter T1 = 1;
parameter T2 = 2;
parameter T3 = 3;
parameter T4 = 4;
parameter T5 = 5;
parameter T6 = 6;

//------------------state
always @(posedge clk)
begin
  if (rst) ps <= 0;
  else ps <= ns;
end

assign d = ir_q[7];
//BSF BCF
assign sel_bit = ir_q[9:7];

always_comb 
begin
	case(sel_RAM_mux)
		0: RAM_mux = ram_out;
		1: RAM_mux = bcf_mux;
		2: RAM_mux = bsf_mux;
	endcase
end
//bcf_mux
always_comb 
begin
	case(sel_bit)
		3'b000: bcf_mux = ram_out & 8'b1111_1110;
		3'b001: bcf_mux = ram_out & 8'b1111_1101;
		3'b010: bcf_mux = ram_out & 8'b1111_1011;
		3'b011: bcf_mux = ram_out & 8'b1111_0111;
		3'b100: bcf_mux = ram_out & 8'b1110_1111;
		3'b101: bcf_mux = ram_out & 8'b1101_1111;
		3'b110: bcf_mux = ram_out & 8'b1011_1111;
		3'b111: bcf_mux = ram_out & 8'b0111_1111;
	endcase
end

//bsf_mux
always_comb 
begin
	case(sel_bit)
		3'b000: bsf_mux = ram_out | 8'b0000_0001;
		3'b001: bsf_mux = ram_out | 8'b0000_0010;
		3'b010: bsf_mux = ram_out | 8'b0000_0100;
		3'b011: bsf_mux = ram_out | 8'b0000_1000;
		3'b100: bsf_mux = ram_out | 8'b0001_0000;
		3'b101: bsf_mux = ram_out | 8'b0010_0000;
		3'b110: bsf_mux = ram_out | 8'b0100_0000;
		3'b111: bsf_mux = ram_out | 8'b1000_0000;
	endcase
end

//BTFSC & BTFSS
assign btfsc_skip_bit = RAM_mux[ir_q[9:7]] == 0;
assign btfss_skip_bit = RAM_mux[ir_q[9:7]] == 1;
assign btfsc_btfss_skip_bit = (BTFSC & btfsc_skip_bit)|(BTFSS & btfss_skip_bit);

//DECFSZ & INCFSZ
assign aluout_zero = (alu_out == 0) ? 1'b1 : 1'b0;

//------------------control
always @(*)
  begin
    load_w = 0;
    load_pc = 0;
    load_mar = 0;
    load_ir = 0;
	sel_pc = 0;
	sel_alu = 0;
	sel_bus = 0;
	ram_en = 0;
	sel_RAM_mux = 0;
	load_port_b = 0;
    case(ps)
      T0: 
		begin
			ns = T1;
		end
      T1: 
        begin
          load_mar = 1;
          ns = T2;
        end
      T2: 
        begin
            load_pc = 1;
            ns = T3;
        end 
      T3: 
        begin
          load_ir = 1;
          ns = T4;
        end
      T4:
        begin
			
        if (MOVLW | ADDLW | SUBLW | ANDLW | IORLW | XORLW)
            begin 
              if (MOVLW) 
                begin
                  load_w = 1;
                  op = 5;
                end
              else if (ADDLW) 
                begin
                  load_w = 1;
                  op = 0;
                end
              else if (SUBLW)
                begin
                  load_w = 1;
                  op = 2;
                end
              else if (ANDLW)
                begin
                  load_w = 1;
                  op = 2;
                end
              else if (IORLW) 
                begin
                  load_w = 1;
                  op = 3;
                end
              else if (XORLW) 
                begin
                  load_w = 1;
                  op = 4;
                end
			end
        else if (ANDWF | ADDWF | GOTO | CLRF | CLRW | COMF | DECF)
            begin 
              if (ADDWF) 
                begin
					if(d)
						ram_en = 1;
					else
						load_w = 1;
					sel_alu = 1;
					op = 0;
                end
              else if (GOTO) 
                begin
					load_pc = 1;
					sel_pc = 1;
                end
              else if (ANDWF)
                begin
					if(d)
						ram_en = 1;
					else
						load_w = 1;
					sel_alu = 1;
					op = 2;
                end
              else if (CLRF)
                begin
					ram_en = 1;
					op = 8;
                end
              else if (CLRW) 
                begin
					load_w = 1;
					op = 8;
                end
              else if (COMF) 
                begin
					ram_en = 1;
					sel_alu = 1;
					op = 9;
                end
				  else if (DECF) 
                begin
					ram_en = 1;
					sel_alu = 1;
					op = 7;
                end
            end
		else if (INCF | IORWF | MOVF | MOVWF | SUBWF | XORWF)
            begin 
              if (INCF) 
                begin
					if(d)
						begin
							ram_en = 1;
							sel_bus = 0;
						end
					else
						load_w = 1;
					sel_alu = 1;
					op = 6;
                end
              else if (IORWF) 
                begin
					if(d)
						begin
							ram_en = 1;
							sel_bus = 0;
						end
					else
						load_w = 1;
					sel_alu = 1;
					op = 3;
                end
              else if (MOVF)
                begin
					if(d)
						begin
							ram_en = 1;
							sel_bus = 0;
						end
					else
						load_w = 1;
					sel_alu = 1;
					op = 5;
                end
              else if (MOVWF)
                begin
					sel_bus = 1;
					if (addr_port_b)
						load_port_b = 1;
					else
						ram_en = 1;
                end
              else if (SUBWF) 
                begin
					if(d)
						begin
							ram_en = 1;
							sel_bus = 0;
						end
					else
						load_w = 1;
					sel_alu = 1;
					op = 1;
                end
              else if (XORWF) 
                begin
					if(d)
						begin
							ram_en = 1;
							sel_bus = 0;
						end
					else
						load_w = 1;
					sel_alu = 1;
					op = 4;
                end
            end
		else if (BCF | BSF | BTFSC | BTFSS | DECFSZ | INCFSZ)
            begin 
              if (BCF) 
                begin
					sel_alu = 1;
					sel_RAM_mux = 1;
					op = 5;
					sel_bus = 0;
					ram_en = 1;
                end
              else if (BSF) 
                begin
					sel_alu = 1;
					sel_RAM_mux = 2;
					op = 5;
					sel_bus = 0;
					ram_en = 1;
                end
              else if (BTFSC)
                begin
					if(btfsc_btfss_skip_bit)
						begin
							load_pc = 1;
							sel_pc = 0;
						end
                end
              else if (BTFSS)
                begin
					if(btfsc_btfss_skip_bit)
						begin
							load_pc = 1;
							sel_pc = 0;
						end
                end
              else if (DECFSZ) 
                begin
					sel_alu = 1;
					op = 7;
					if(d)
						begin
							ram_en = 1;
							sel_bus = 0;
							if(aluout_zero)
								begin
									load_pc = 1;
									sel_pc = 0;
								end
						end
					else
						begin
							load_w = 1;
							if(aluout_zero)
								begin
									load_pc = 1;
									sel_pc = 0;
								end
						end
                end
              else if (INCFSZ) 
                begin
					sel_alu = 1;
					op = 6;
					if(d)
						begin
							ram_en = 1;
							sel_bus = 0;
							if(aluout_zero)
								begin
									load_pc = 1;
									sel_pc = 0;
								end
						end
					else
						begin
							load_w = 1;
							if(aluout_zero)
								begin
									load_pc = 1;
									sel_pc = 0;
								end
						end
					
                end
            end
		else if (ASRF | LSLF | LSRF | RLF | RRF | SWAPF)
            begin 
              if (ASRF) 
                begin
					sel_alu = 1;
					sel_RAM_mux = 0;
					op = 4'hA;
					if(d)
						begin
							sel_bus = 0;
							ram_en = 1;
						end
					else
						load_w = 1;
                end
              else if (LSLF) 
                begin
					sel_alu = 1;
					sel_RAM_mux = 0;
					op = 4'hB;
					if(d)
						begin
							sel_bus = 0;
							ram_en = 1;
						end
					else
						load_w = 1;
                end
              else if (LSRF)
                begin
					sel_alu = 1;
					sel_RAM_mux = 0;
					op = 4'hC;
					if(d)
						begin
							sel_bus = 0;
							ram_en = 1;
						end
					else
						load_w = 1;
                end
              else if (RLF)
                begin
					sel_alu = 1;
					sel_RAM_mux = 0;
					op = 4'hD;
					if(d)
						begin
							sel_bus = 0;
							ram_en = 1;
						end
					else
						load_w = 1;
                end
              else if (RRF) 
                begin
					sel_alu = 1;
					sel_RAM_mux = 0;
					op = 4'hE;
					if(d)
						begin
							sel_bus = 0;
							ram_en = 1;
						end
					else
						load_w = 1;
                end
              else if (SWAPF) 
                begin
					sel_alu = 1;
					sel_RAM_mux = 0;
					op = 4'hF;
					if(d)
						begin
							sel_bus = 0;
							ram_en = 1;
						end
					else
						load_w = 1;
                end
            end
			ns = T5;
        end
        T5:
			begin
				ns = T6;
			end
        T6:
          begin
				ns = T1;
			end
    endcase
  end 
endmodule


