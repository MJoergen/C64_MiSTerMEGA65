/*
   Adjusted to MiSTer2MEGA65 by sy2002 in March 2022:
   
   1. The iecdrv_sync needs appropriate set_false_path settings in the XDC for each instantiation.
   
   2. iecdrv_mem's data loading mechanism did not work in Vivado: Solution: Creating two modules: One for
      ROM loading that is a wrapper for MiSTer2MEGA65's dualport_2clk_ram, which adds the advantage of being
      QNICE compatible (falling edge). Additionally a RAM module that is identical to the original module iecdrv_mem. 
     
   3. Vivado was not able to synthesize iecdrv_bitmem. One of many differences between Intel/Quartus and Xilinx/Vivado.
      So I commented it out and turned it into a wrapper for MiSTer2MEGA65's dualport_2clk_ram, which also adds the
      advantage that it is directly compatible with QNICE's falling edge logic for reading and writing.
*/
   


module iecdrv_sync #(parameter WIDTH = 1) 
(
	input                  clk,
	input      [WIDTH-1:0] in,
	output reg [WIDTH-1:0] out
);

reg [WIDTH-1:0] s1,s2;
always @(posedge clk) begin
	s1 <= in;
	s2 <= s1;
	if(s1 == s2) out <= s2;
end

endmodule

// -------------------------------------------------------------------------------
// RAM
// -------------------------------------------------------------------------------

module iecdrv_mem #(parameter DATAWIDTH, ADDRWIDTH)
(
	input	                     clock_a,
	input	     [ADDRWIDTH-1:0] address_a,
	input	     [DATAWIDTH-1:0] data_a,
	input	                     wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input	                     clock_b,
	input	     [ADDRWIDTH-1:0] address_b,
	input	     [DATAWIDTH-1:0] data_b,
	input	                     wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

reg [DATAWIDTH-1:0] ram[1<<ADDRWIDTH];

reg                 wren_a_d;
reg [ADDRWIDTH-1:0] address_a_d;
always @(posedge clock_a) begin
	wren_a_d    <= wren_a;
	address_a_d <= address_a;
end

always @(posedge clock_a) begin
	if(wren_a_d) begin
		ram[address_a_d] <= data_a;
		q_a <= data_a;
	end else begin
		q_a <= ram[address_a_d];
	end
end

reg                 wren_b_d;
reg [ADDRWIDTH-1:0] address_b_d;
always @(posedge clock_b) begin
	wren_b_d    <= wren_b;
	address_b_d <= address_b;
end

always @(posedge clock_b) begin
	if(wren_b_d) begin
		ram[address_b_d] <= data_b;
		q_b <= data_b;
	end else begin
		q_b <= ram[address_b_d];
	end
end

endmodule

// -------------------------------------------------------------------------------
// ROM
// -------------------------------------------------------------------------------

module iecdrv_mem_rom #(parameter DATAWIDTH, ADDRWIDTH, INITFILE=" ", FALLING_A=1'b0, FALLING_B=1'b0)
(
	input	                     clock_a,
	input	     [ADDRWIDTH-1:0] address_a,
	input	     [DATAWIDTH-1:0] data_a,
	input	                     wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input	                     clock_b,
	input	     [ADDRWIDTH-1:0] address_b,
	input	     [DATAWIDTH-1:0] data_b,
	input	                     wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

dualport_2clk_ram #(
   .ADDR_WIDTH(ADDRWIDTH),
   .DATA_WIDTH(DATAWIDTH),
   .FALLING_A(FALLING_A),
   .FALLING_B(FALLING_B),
   .ROM_PRELOAD(INITFILE != " " ? 1'b1 : 1'b0),
   .ROM_FILE(INITFILE),
   .ROM_FILE_HEX(1'b1)
) ram (
   .clock_a(clock_a),
   .address_a(address_a_d),
   .data_a(data_a),
   .wren_a(wren_a_d),
   .q_a(q_a),

   .clock_b(clock_b),
   .address_b(address_b_d),
   .data_b(data_b),
   .wren_b(wren_b_d),
   .q_b(q_b) 
);

// delay signals (MiSTer's original implementation does that so do we)
reg                 wren_a_d;
reg [ADDRWIDTH-1:0] address_a_d;

if (FALLING_A == 1'b0) begin
   always @(posedge clock_a) begin
      wren_a_d    <= wren_a;
      address_a_d <= address_a;
   end
end else begin // QNICE expects the data to flow instantly on the falling edge, not delayed
   assign wren_a_d    = wren_a;
   assign address_a_d = address_a;
end

reg                 wren_b_d;
reg [ADDRWIDTH-1:0] address_b_d;

if (FALLING_B == 1'b0) begin
   always @(posedge clock_b) begin
      wren_b_d    <= wren_b;
      address_b_d <= address_b;
   end
end else begin // QNICE expects the data to flow instantly on the falling edge, not delayed
   assign wren_b_d    = wren_b;
   assign address_b_d = address_b;
end

endmodule

/*
// -------------------------------------------------------------------------------
// Dual width memory aka "bitmem"
// -------------------------------------------------------------------------------

module iecdrv_bitmem #(parameter ADDRWIDTH, FALLING_A=1'b0, FALLING_B=1'b0)
(
	input	                     clock_a,
	input	     [ADDRWIDTH-1:0] address_a,
	input	               [7:0] data_a,
	input	                     wren_a,
	output reg           [7:0] q_a,

	input	                     clock_b,
	input	     [ADDRWIDTH+2:0] address_b,
	input	                     data_b,
	input	                     wren_b,
	output reg                 q_b
);

// delay signals (MiSTer's original implementation does that so do we)
reg                 wren_a_d;
reg [ADDRWIDTH-1:0] address_a_d;
reg           [7:0] data_a_d;

if (FALLING_A == 1'b0) begin
   always @(posedge clock_a) begin
      wren_a_d    <= wren_a;
      address_a_d <= address_a;
      data_a_d    <= data_a;
   end
end else begin // QNICE expects the data to flow instantly on the falling edge, not delayed
   assign wren_a_d    = wren_a;
   assign address_a_d = address_a;
   assign data_a_d    = data_a;
end

reg                 wren_b_d;
reg [ADDRWIDTH+2:0] address_b_d;
reg                 data_b_d;

if (FALLING_B == 1'b0) begin
   always @(posedge clock_b) begin
      wren_b_d    <= wren_b;
      address_b_d <= address_b;
      data_b_d    <= data_b;
   end
end else begin // QNICE expects the data to flow instantly on the falling edge, not delayed
   assign wren_b_d    = wren_b;
   assign address_b_d = address_b;
   assign data_b_d    = data_b;
end

// Use 8 1-bit RAMs to implement the dual-port RAM with different data-widths
integer bit_selector_d;
assign bit_selector_d = address_b_d[2:0];
integer bit_selector_dd;
always @(posedge clock_b) begin
   bit_selector_dd <= bit_selector_d;
end

generate
	genvar i;
	for(i=0; i<8; i=i+1) begin : bitrams

      wire q_a_bit, q_b_bit;
      dualport_2clk_ram #(
         .ADDR_WIDTH(ADDRWIDTH),
         .DATA_WIDTH(1),
         .FALLING_A(FALLING_A),
         .FALLING_B(FALLING_B)
      ) ram (
         .clock_a(clock_a),
         .address_a(address_a_d),
         .data_a(data_a_d[i]),
         .wren_a(wren_a_d),
         .q_a(q_a_bit),
      
         .clock_b(clock_b),
         .address_b(address_b_d[ADDRWIDTH+2:3]),
         .data_b(data_b_d),
         .wren_b(wren_b_d & (bit_selector_d == i)),
         .q_b(q_b_bit)  
      );
      
      assign q_a[i] = q_a_bit;
   end
endgenerate

assign q_b = (bitrams[0].q_b_bit & (bit_selector_dd == 0)) |
             (bitrams[1].q_b_bit & (bit_selector_dd == 1)) |
             (bitrams[2].q_b_bit & (bit_selector_dd == 2)) |
             (bitrams[3].q_b_bit & (bit_selector_dd == 3)) |
             (bitrams[4].q_b_bit & (bit_selector_dd == 4)) |
             (bitrams[5].q_b_bit & (bit_selector_dd == 5)) |
             (bitrams[6].q_b_bit & (bit_selector_dd == 6)) |
             (bitrams[7].q_b_bit & (bit_selector_dd == 7));

endmodule
*/

/* Original MiSTer Intel/Quartus code that does not synthesize with Vivado v2019.2.
   I did not try newer Vivado versions but replaced the code with a wrapper of 1-bit RAMs made with dualport_2clk_ram
   done by sy2002 in March 2022
    
reg [7:0] ram[1<<ADDRWIDTH];

reg                 wren_a_d;
reg [ADDRWIDTH-1:0] address_a_d;
reg           [7:0] data_a_d;
always @(posedge clock_a) begin
	wren_a_d    <= wren_a;
	address_a_d <= address_a;
	data_a_d    <= data_a;
end

always @(posedge clock_a) begin
	if(wren_a_d) begin
		ram[address_a_d] <= data_a_d;
		q_a <= data_a_d;
	end else begin
		q_a <= ram[address_a_d];
	end
end

reg                 wren_b_d;
reg [ADDRWIDTH+2:0] address_b_d;
reg                 data_b_d;
always @(posedge clock_b) begin
	wren_b_d    <= wren_b;
	address_b_d <= address_b;
	data_b_d    <= data_b;
end

always @(posedge clock_b) begin
	if(wren_b_d) begin
		ram[address_b_d[ADDRWIDTH+2:3]][address_b_d[2:0]] <= data_b_d;
		q_b <= data_b_d;
	end else begin
		q_b <= ram[address_b_d[ADDRWIDTH+2:3]][address_b_d[2:0]];
	end
end
*/
