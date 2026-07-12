//-------------------------------------------------------------------------------
//
// C1541 multi-drive implementation with shared ROM
// (C) 2021 Alexey Melnikov
//
// Input clock/ce 16MHz
//
//-------------------------------------------------------------------------------


module c1581_multi #(parameter PARPORT=1,DUALROM=1,DRIVES=2)
(
	//clk ports
	input         clk,
	input   [N:0] reset,
	input         ce,

	input         pause,

	input   [N:0] img_mounted,
	input         img_readonly,
	input  [31:0] img_size,

	output  [N:0] act_led,
	output  [N:0] pwr_led,

	input         iec_atn_i,
	input         iec_data_i,
	input         iec_clk_i,
	input         iec_fclk_i,
	output        iec_data_o,
	output        iec_clk_o,
	output        iec_fclk_o,

	// parallel bus
	input   [7:0] par_data_i,
	input         par_stb_i,
	output reg [7:0] par_data_o,
	output        par_stb_o,

	//clk_sys ports
	input         clk_sys,

	output [31:0] sd_lba[NDR],
	output  [N:0] sd_rd,
	output  [N:0] sd_wr,
	input   [N:0] sd_ack,
	input   [8:0] sd_buff_addr,
	input   [7:0] sd_buff_dout,
	output  [7:0] sd_buff_din[NDR],
	input         sd_buff_wr,

	input  [14:0] rom_addr,
	input   [7:0] rom_data,
	output  [7:0] rom_data_o,
	input         rom_wr,
	input         rom_std,

	// ---------------------------------------------------------------------
	// MEGA65 physical internal 1581 (issue #90): drive-0 (internal) phys ABI,
	// threaded from iec_drive down to the drive-0 c1581_drv/fdc1772. Only drive
	// index 0 has a physical controller; any other generated drive stays virtual
	// (its fdc phys inputs are tied 0 / outputs left open). phys_mode=0 keeps the
	// image path byte-identical.
	// ---------------------------------------------------------------------
	input         phys_mode,

	output        phys_active,
	output        phys_cia_motor_on,
	output        phys_cia_side,
	output        phys_step_req_tgl,
	output        phys_step_outward,
	output        phys_rd_req_tgl,
	output  [2:0] phys_rd_op,
	output  [7:0] phys_rd_track,
	output        phys_rd_side,
	output  [7:0] phys_rd_sector,
	output        phys_rd_cancel_tgl,
	output        phys_byte_ovf,
	output        phys_byte_rd_en,

	input         phys_step_ack_tgl,
	input         phys_rd_done_tgl,
	input   [4:0] phys_rd_result,
	input         phys_rd_crc_err,
	input         phys_rd_rnf,
	input         phys_rd_deleted,
	input   [7:0] phys_rd_c,
	input   [7:0] phys_rd_h,
	input   [7:0] phys_rd_r,
	input   [7:0] phys_rd_n,
	input   [7:0] phys_byte_data,
	input         phys_byte_empty,
	input         phys_media_ready,
	input         phys_index,
	input         phys_track0,
	input         phys_wprot,
	input         phys_change,
	input         phys_motor_on,
	input         phys_head_settled
);

localparam NDR = (DRIVES < 1) ? 1 : (DRIVES > 4) ? 4 : DRIVES;
localparam N   = NDR - 1;

wire iec_atn, iec_data, iec_clk, iec_fclk;
iecdrv_sync atn_sync(clk,  iec_atn_i,  iec_atn);
iecdrv_sync dat_sync(clk,  iec_data_i, iec_data);
iecdrv_sync clk_sync(clk,  iec_clk_i,  iec_clk);
iecdrv_sync fclk_sync(clk, iec_fclk_i, iec_fclk);

wire [N:0] reset_drv;
iecdrv_sync #(NDR) rst_sync(clk, reset, reset_drv);

wire stdrom = (DUALROM || PARPORT) ? rom_std : 1'b1;

reg ph2_r;
reg ph2_f;
reg wd_ce;
always @(posedge clk) begin
	reg [2:0] div;
	reg       ena, ena1;

	ena1 <= ~pause;
	if(div[1:0]) ena <= ena1;

	ph2_r <= 0;
	ph2_f <= 0;
	wd_ce  <= 0;
	if(ce) begin
		div <= div + 1'd1;
		ph2_r <= ena && !div[2] && !div[1:0];
		ph2_f <= ena &&  div[2] && !div[1:0];
		wd_ce  <= ena && !div[0];
	end
end

reg  [14:0] mem_a;
wire [7:0] rom_do;
wire [7:0] romstd_do;
wire [7:0] qnice_rom_do;       // MEGA65: readback of the writable (custom DOS) slot -> rom_data_o
generate
	if(PARPORT || DUALROM) begin
		// MEGA65 (sy2002/D81 enable): the original `iecdrv_mem #(8,15,"./c1581_rom.mif") rom`
		// was a Quartus-only elaboration error (3 positional params to the 2-param Vivado
		// iecdrv_mem, posedge-A, .mif). Mirror the romstd instance below: a Vivado-compatible
		// iecdrv_mem_rom on the QNICE falling edge. INITFILE preloads the writable slot with
		// the STANDARD 1581 DOS (upstream semantics) so a missing jd-c1581.bin degrades to
		// stock 1581 instead of a dead drive. q_a feeds the QNICE readback (rom_data_o).
		iecdrv_mem_rom #(
		   .DATAWIDTH(8),
		   .ADDRWIDTH(15),
		   .INITFILE("../../C64_MiSTerMEGA65/rtl/iec_drive/c1581_rom.mif.hex"),
		   .FALLING_A(1'b1)
		) rom
		(
			.clock_a(clk_sys),
			.address_a(rom_addr),
			.data_a(rom_data),
			.wren_a(rom_wr),
			.q_a(qnice_rom_do),

			.clock_b(clk),
			.address_b(mem_a),
			.q_b(rom_do)
		);
	end
	else begin
		assign rom_do       = romstd_do;
		assign qnice_rom_do = 8'hFF;   // no writable slot in this config
	end
endgenerate
assign rom_data_o = qnice_rom_do;

iecdrv_mem_rom #(
   .DATAWIDTH(8),
   .ADDRWIDTH(15),
   .INITFILE("../../C64_MiSTerMEGA65/rtl/iec_drive/c1581_rom.mif.hex"),
   .FALLING_A(1'b1)
) romstd (
	.clock_a(clk_sys),
	.address_a(rom_addr),
	.data_a(rom_data),
	.wren_a((DUALROM || PARPORT) ? 1'b0 : rom_wr),

	.clock_b(clk),
	.address_b(mem_a),
	.q_b(romstd_do)
);

wire [14:0] drv_addr[NDR];
reg   [7:0] drv_data[4];
always @(posedge clk) begin
	reg [2:0] state;
	reg [14:0] mem_d;
	
	if(~&state) state <= state + 1'd1;
	if(ph2_f)   state <= 0;

	case(state)
		0,1,2,3: mem_a <= drv_addr[state[1:0]];
	endcase
	
	case(state)
		3,4,5,6: drv_data[state[1:0] - 2'd3] <= stdrom ? romstd_do : rom_do;
	endcase
end

wire [N:0] iec_data_d, iec_clk_d, iec_fclk_d;
assign     iec_clk_o  = &{iec_clk_d  | reset_drv};
assign     iec_fclk_o = &{iec_fclk_d | reset_drv};
assign     iec_data_o = &{iec_data_d | reset_drv};

wire [7:0] par_data_d[NDR];
wire [N:0] par_stb_d;
assign     par_stb_o = &{par_stb_d | reset_drv};
always_comb begin
	par_data_o = 8'hFF;
	for(int i=0; i<NDR; i=i+1) if(~reset_drv[i]) par_data_o = par_data_o & par_data_d[i];
end

wire [N:0] act_led_drv, pwr_led_drv;
assign     act_led = act_led_drv & ~reset_drv;
assign     pwr_led = pwr_led_drv & ~reset_drv;

// MEGA65 (#90): collect each generated drive's phys outputs; the module-level
// phys_* bundle exposes only drive 0 (the internal 1581). Drives i>0 route to
// unused wires (effectively open) and get their phys inputs tied 0 below.
wire [N:0] phys_active_d, phys_cia_motor_on_d, phys_cia_side_d;
wire [N:0] phys_step_req_tgl_d, phys_step_outward_d, phys_rd_req_tgl_d;
wire [N:0] phys_rd_side_d, phys_rd_cancel_tgl_d, phys_byte_ovf_d, phys_byte_rd_en_d;
wire [2:0] phys_rd_op_d[NDR];
wire [7:0] phys_rd_track_d[NDR];
wire [7:0] phys_rd_sector_d[NDR];

assign phys_active        = phys_active_d[0];
assign phys_cia_motor_on  = phys_cia_motor_on_d[0];
assign phys_cia_side      = phys_cia_side_d[0];
assign phys_step_req_tgl  = phys_step_req_tgl_d[0];
assign phys_step_outward  = phys_step_outward_d[0];
assign phys_rd_req_tgl    = phys_rd_req_tgl_d[0];
assign phys_rd_op         = phys_rd_op_d[0];
assign phys_rd_track      = phys_rd_track_d[0];
assign phys_rd_side       = phys_rd_side_d[0];
assign phys_rd_sector     = phys_rd_sector_d[0];
assign phys_rd_cancel_tgl = phys_rd_cancel_tgl_d[0];
assign phys_byte_ovf      = phys_byte_ovf_d[0];
assign phys_byte_rd_en    = phys_byte_rd_en_d[0];

generate
	genvar i;
	for(i=0; i<NDR; i=i+1) begin :drives
		c1581_drv c1581_drv
		(
			.clk(clk),
			.reset(reset_drv[i]),

			.ce(ce),
			.wd_ce(wd_ce),
			.ph2_r(ph2_r),
			.ph2_f(ph2_f),

			.img_mounted(img_mounted[i]),
			.img_readonly(img_readonly),
			.img_size(img_size),

			.drive_num(i),
			.act_led(act_led_drv[i]),
			.pwr_led(pwr_led_drv[i]),

			.iec_atn_i(iec_atn),
			.iec_data_i(iec_data & iec_data_o),
			.iec_clk_i(iec_clk & iec_clk_o),
			.iec_fclk_i(iec_fclk & iec_fclk_o),
			.iec_data_o(iec_data_d[i]),
			.iec_clk_o(iec_clk_d[i]),
			.iec_fclk_o(iec_fclk_d[i]),

			.par_data_i(par_data_i),
			.par_stb_i(par_stb_i),
			.par_data_o(par_data_d[i]),
			.par_stb_o(par_stb_d[i]),

			.rom_addr(drv_addr[i]),
			.rom_data(drv_data[i]),

			.clk_sys(clk_sys),

			.sd_lba(sd_lba[i]),
			.sd_rd(sd_rd[i]),
			.sd_wr(sd_wr[i]),
			.sd_ack(sd_ack[i]),
			.sd_buff_addr(sd_buff_addr),
			.sd_buff_dout(sd_buff_dout),
			.sd_buff_din(sd_buff_din[i]),
			.sd_buff_wr(sd_buff_wr),

			// MEGA65 (#90): physical 1581 ABI. Only drive 0 is the internal drive;
			// other drives get phys inputs tied off and their outputs left unused.
			.phys_mode        ( (i==0) ? phys_mode         : 1'b0 ),
			.phys_active      ( phys_active_d[i]      ),
			.phys_cia_motor_on( phys_cia_motor_on_d[i]),
			.phys_cia_side    ( phys_cia_side_d[i]    ),
			.phys_step_req_tgl( phys_step_req_tgl_d[i]),
			.phys_step_outward( phys_step_outward_d[i]),
			.phys_rd_req_tgl  ( phys_rd_req_tgl_d[i]  ),
			.phys_rd_op       ( phys_rd_op_d[i]       ),
			.phys_rd_track    ( phys_rd_track_d[i]    ),
			.phys_rd_side     ( phys_rd_side_d[i]     ),
			.phys_rd_sector   ( phys_rd_sector_d[i]   ),
			.phys_rd_cancel_tgl(phys_rd_cancel_tgl_d[i]),
			.phys_byte_ovf    ( phys_byte_ovf_d[i]    ),
			.phys_byte_rd_en  ( phys_byte_rd_en_d[i]  ),
			.phys_step_ack_tgl( (i==0) ? phys_step_ack_tgl : 1'b0 ),
			.phys_rd_done_tgl ( (i==0) ? phys_rd_done_tgl  : 1'b0 ),
			.phys_rd_result   ( (i==0) ? phys_rd_result    : 5'd0 ),
			.phys_rd_crc_err  ( (i==0) ? phys_rd_crc_err   : 1'b0 ),
			.phys_rd_rnf      ( (i==0) ? phys_rd_rnf       : 1'b0 ),
			.phys_rd_deleted  ( (i==0) ? phys_rd_deleted   : 1'b0 ),
			.phys_rd_c        ( (i==0) ? phys_rd_c         : 8'd0 ),
			.phys_rd_h        ( (i==0) ? phys_rd_h         : 8'd0 ),
			.phys_rd_r        ( (i==0) ? phys_rd_r         : 8'd0 ),
			.phys_rd_n        ( (i==0) ? phys_rd_n         : 8'd0 ),
			.phys_byte_data   ( (i==0) ? phys_byte_data    : 8'd0 ),
			.phys_byte_empty  ( (i==0) ? phys_byte_empty   : 1'b1 ),
			.phys_media_ready ( (i==0) ? phys_media_ready  : 1'b0 ),
			.phys_index       ( (i==0) ? phys_index        : 1'b0 ),
			.phys_track0      ( (i==0) ? phys_track0       : 1'b0 ),
			.phys_wprot       ( (i==0) ? phys_wprot        : 1'b0 ),
			.phys_change      ( (i==0) ? phys_change       : 1'b0 ),
			.phys_motor_on    ( (i==0) ? phys_motor_on     : 1'b0 ),
			.phys_head_settled( (i==0) ? phys_head_settled : 1'b0 )
		);
	end
endgenerate

endmodule
