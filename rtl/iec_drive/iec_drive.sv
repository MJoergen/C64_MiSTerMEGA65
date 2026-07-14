//-------------------------------------------------------------------------------
//
// C1541/C1581 selector
// (C) 2021 Alexey Melnikov
//
//-------------------------------------------------------------------------------
 
module iec_drive #(parameter PARPORT=1,DUALROM=1,DRIVES=2)
(
	//clk ports
	input         clk,
	input   [N:0] reset,
	input         ce,

	input         pause,

	input   [N:0] img_mounted,
	input         img_readonly,
	input  [31:0] img_size,
	
	// 00 - 1541 emulated GCR(D64)
	// 01 - 1541 real GCR mode (G64,D64)
	// 10 - 1581 (D81)
	input   [1:0] img_type,

	// MEGA65 physical internal 1581 (issue #90): mode select for drive 0 (the
	// internal Commodore drive 8). A single bit is sufficient: only drive 0 can
	// be backed by the real internal 3.5" drive; drives 1..N stay virtual. When
	// set it forces the 1581 engine active, holds the 1541 engine in reset and
	// masks drive-0 image DMA (sd_rd/sd_wr) off. See phys_mode_vec below.
	input         physical_mode,

	output  [N:0] led,

	input         iec_atn_i,
	input         iec_data_i,
	input         iec_clk_i,
	output        iec_data_o,
	output        iec_clk_o,

	// parallel bus
	input   [7:0] par_data_i,
	input         par_stb_i,
	output  [7:0] par_data_o,
	output        par_stb_o,

	//clk_sys ports
	input         clk_sys,

	output reg [31:0] sd_lba[NDR],
	output reg  [5:0] sd_blk_cnt[NDR],
	output reg  [N:0] sd_rd,
	output reg  [N:0] sd_wr,
	input   [N:0] sd_ack,
	input  [13:0] sd_buff_addr,
	input   [7:0] sd_buff_dout,
	output reg [7:0] sd_buff_din[NDR],
	input         sd_buff_wr,

	input  [15:0] rom_addr_i,
	input   [7:0] rom_data_i,
	output  [7:0] rom_data_o,
	input         rom_wr_i,
	input         rom_std_i,

	// ---------------------------------------------------------------------
	// MEGA65 physical internal 1581 (issue #90): drive-0 flat toggle/level ABI
	// to the VHDL physical_1581_controller (50 MHz, instantiated in main.vhd).
	// Threaded straight through c1581_multi -> drive-0 c1581_drv -> fdc1772.
	// Inert unless physical_mode=1. main.vhd connects these ports.
	// ---------------------------------------------------------------------
	// phys OUTPUTS (fdc1772 -> controller)
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
	output  [1:0] phys_rd_seq,       // op sequence tag (quasi-static before rd_req_tgl)
	output        phys_byte_ovf,
	output        phys_byte_rd_en,

	// phys INPUTS (controller -> fdc1772)
	input         phys_step_ack_tgl,
	input         phys_rd_done_tgl,
	input   [1:0] phys_rd_done_seq,  // seq of the op being completed (quasi-static before rd_done_tgl)
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
	input         phys_head_settled,

	// fdc1772 diagnostic event toggles + per-op presented-byte count (drive
	// clock; main.vhd 2-FF-syncs the toggles into the 50 MHz diag domain,
	// pres_cnt crosses unsynced as a quasi-static bus captured on the fin edge)
	output        phys_dbg_lost_tgl,
	output        phys_dbg_drain_tgl,
	output        phys_dbg_staledone_tgl,
	output        phys_dbg_busycmd_tgl,
	output        phys_dbg_fin_tgl,
	output [10:0] phys_dbg_pres_cnt
);

localparam NDR = (DRIVES < 1) ? 1 : (DRIVES > 4) ? 4 : DRIVES;
localparam N   = NDR - 1;

// MEGA65 (#90): drive-0-only physical-mode vector. physical_mode is a single
// bit (internal drive 8); zero-extension puts it on bit 0 and leaves bits N..1
// at 0, so only drive 0's engine-select and sd_rd/sd_wr gating are affected.
wire [N:0] phys_mode_vec = physical_mode;

reg [N:0] dtype[2];
wire        c1541_iec_data, c1541_iec_clk, c1541_stb_o;
wire  [7:0] c1541_par_o;
wire  [N:0] c1541_led;
wire  [7:0] c1541_sd_buff_dout[NDR];
wire [31:0] c1541_sd_lba[NDR];
wire  [N:0] c1541_sd_rd, c1541_sd_wr;
wire  [5:0] c1541_sd_blk_cnt[NDR];

wire        c1581_iec_data, c1581_iec_clk, c1581_stb_o;
wire  [7:0] c1581_par_o;
wire  [N:0] c1581_led;
wire  [7:0] c1581_sd_buff_dout[NDR];
wire [31:0] c1581_sd_lba[NDR];
wire  [N:0] c1581_sd_rd, c1581_sd_wr;

// MEGA65 (D81 enable, sy2002): re-homed from clk_sys to clk. Upstream MiSTer sources
// img_mounted/img_size/img_type from hps_io on clk_sys, so latching on posedge clk_sys was
// same-domain. In the M2M port these three come from vdrives, which resynchronizes them into
// the CORE clock domain (clk = clk_main_i). Sampling them on the QNICE clock (clk_sys) was an
// unsynchronized CDC: D64 survived it only because dtype powers up to 0 (= 1541), but a D81
// (img_type=10) needs this latch to actually capture a non-zero value across the boundary.
// Latch in the signals own domain instead. dtype is quasi-static (changes only on a mount),
// so its use as the clk_sys-domain sd_lba/sd_rd mux select stays safe.
always @(posedge clk) for(int i=0; i<NDR; i=i+1) if(img_mounted[i] && img_size) {dtype[1][i],dtype[0][i]} <= img_type;

assign led          = c1581_led      | c1541_led;     // MEGA65 (D81): 1581 engine enabled
assign iec_data_o   = c1581_iec_data & c1541_iec_data;
assign iec_clk_o    = c1581_iec_clk  & c1541_iec_clk;
assign par_stb_o    = c1581_stb_o    & c1541_stb_o;
assign par_data_o   = c1581_par_o    & c1541_par_o;

// MEGA65 (D81): ROM readback mux. c1541 custom-DOS ROM when rom_addr_i[15]=0, c1581 when
// =1. The ROM slots are FALLING_A (QNICE writes/reads on the falling edge of clk_sys), so
// the select bit is captured on the SAME falling edge to stay aligned with the q_a read
// data. (Readback only -- the auto-loader never reads ROMs back; this is forward-proofing
// for any future verify-after-write of the 1581 JiffyDOS image.)
wire [7:0] c1541_rom_data_o, c1581_rom_data_o;
reg        rom_sel_d;
always @(negedge clk_sys) rom_sel_d <= rom_addr_i[15];
assign rom_data_o = rom_sel_d ? c1581_rom_data_o : c1541_rom_data_o;

always_comb for(int i=0; i<NDR; i=i+1) begin
	sd_buff_din[i] = (dtype[1][i] ? c1581_sd_buff_dout[i] : c1541_sd_buff_dout[i] );
	sd_lba[i]      = (dtype[1][i] ? c1581_sd_lba[i] << 1  : c1541_sd_lba[i]       );
	// MEGA65 (#90): in physical mode the internal drive is backed by the real
	// disk, so mask its image DMA off. sd_lba/sd_blk_cnt/sd_buff_din are left as
	// is -- harmless while sd_rd=sd_wr=0 (vdrives only acts on rd|wr).
	sd_rd[i]       = phys_mode_vec[i] ? 1'b0 : (dtype[1][i] ? c1581_sd_rd[i] : c1541_sd_rd[i]);
	sd_wr[i]       = phys_mode_vec[i] ? 1'b0 : (dtype[1][i] ? c1581_sd_wr[i] : c1541_sd_wr[i]);
	sd_blk_cnt[i]  = (dtype[1][i] ? 6'd1                  : c1541_sd_blk_cnt[i]   );
end

c1541_multi #(.PARPORT(PARPORT), .DUALROM(DUALROM), .DRIVES(DRIVES)) c1541
(
	.clk(clk),
	// MEGA65 (#90): also hold the 1541 engine in reset in physical mode -- only
	// one engine may own the AND-wired IEC bus, and physical mode is 1581-only.
	.reset(reset | dtype[1] | phys_mode_vec),
	.ce(ce),

	.gcr_mode(dtype[0]),

	.iec_atn_i (iec_atn_i),
	.iec_data_i(iec_data_i & c1581_iec_data),
	.iec_clk_i (iec_clk_i  & c1581_iec_clk),
	.iec_data_o(c1541_iec_data),
	.iec_clk_o (c1541_iec_clk),

	.led(c1541_led),

	.par_data_i(par_data_i),
	.par_stb_i(par_stb_i),
	.par_data_o(c1541_par_o),
	.par_stb_o(c1541_stb_o),

	.clk_sys(clk_sys),
	.pause(pause),

	.rom_addr_i(rom_addr_i[14:0]),
	.rom_data_i(rom_data_i),
	.rom_data_o(c1541_rom_data_o),
	.rom_wr_i(~rom_addr_i[15] & rom_wr_i),
	.rom_std_i(rom_std_i),

	.img_mounted(img_mounted),
	.img_size(img_size),
	.img_readonly(img_readonly),

	.sd_lba(c1541_sd_lba),
	.sd_blk_cnt(c1541_sd_blk_cnt),
	.sd_rd(c1541_sd_rd),
	.sd_wr(c1541_sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(c1541_sd_buff_dout),
	.sd_buff_wr(sd_buff_wr)
);


// MEGA65 (D81 enable, sy2002): the 1581 engine is now active. Reset is released only when
// drive 8 has a D81 mounted (dtype[1]=1); a D64 holds it in reset so only one engine drives
// the IEC bus at a time (AND-wired, safe by construction -- a reset drive contributes '1').
// The stale signal names in the original commented block (rom_addr/rom_data/rom_wr/rom_std)
// are corrected to the actual _i-suffixed ports; bit15 of rom_addr_i selects the 1581 ROM
// window. iec_fclk_o and pwr_led are intentionally left unconnected (C64 has no fast serial).
c1581_multi #(.PARPORT(PARPORT), .DUALROM(DUALROM), .DRIVES(DRIVES)) c1581
(
	.clk(clk),
	// MEGA65 (#90): release the 1581 engine when a D81 is mounted (dtype[1]=1)
	// OR when physical mode is selected -- physical mode has no D81 image but
	// must still run the 1581 engine (backed by the real internal drive).
	.reset(reset | ~(dtype[1] | phys_mode_vec)),
	.ce(ce),

	.iec_atn_i (iec_atn_i),
	.iec_data_i(iec_data_i & c1541_iec_data),
	.iec_clk_i (iec_clk_i  & c1541_iec_clk),
	.iec_fclk_i (1),
	.iec_data_o(c1581_iec_data),
	.iec_clk_o (c1581_iec_clk),

	.act_led(c1581_led),

	.par_data_i(par_data_i),
	.par_stb_i(par_stb_i),
	.par_data_o(c1581_par_o),
	.par_stb_o(c1581_stb_o),

	.clk_sys(clk_sys),
	.pause(pause),

	.rom_addr(rom_addr_i[14:0]),
	.rom_data(rom_data_i),
	.rom_data_o(c1581_rom_data_o),
	.rom_wr(rom_addr_i[15] & rom_wr_i),
	.rom_std(rom_std_i),

	.img_mounted(img_mounted),
	.img_size(img_size),
	.img_readonly(img_readonly),

	.sd_lba(c1581_sd_lba),
	.sd_rd(c1581_sd_rd),
	.sd_wr(c1581_sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr[8:0]),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(c1581_sd_buff_dout),
	.sd_buff_wr(sd_buff_wr),

	// MEGA65 (#90): physical internal 1581 ABI, threaded to drive 0.
	.phys_mode(physical_mode),
	.phys_active(phys_active),
	.phys_cia_motor_on(phys_cia_motor_on),
	.phys_cia_side(phys_cia_side),
	.phys_step_req_tgl(phys_step_req_tgl),
	.phys_step_outward(phys_step_outward),
	.phys_rd_req_tgl(phys_rd_req_tgl),
	.phys_rd_op(phys_rd_op),
	.phys_rd_track(phys_rd_track),
	.phys_rd_side(phys_rd_side),
	.phys_rd_sector(phys_rd_sector),
	.phys_rd_cancel_tgl(phys_rd_cancel_tgl),
	.phys_rd_seq(phys_rd_seq),
	.phys_byte_ovf(phys_byte_ovf),
	.phys_byte_rd_en(phys_byte_rd_en),
	.phys_step_ack_tgl(phys_step_ack_tgl),
	.phys_rd_done_tgl(phys_rd_done_tgl),
	.phys_rd_done_seq(phys_rd_done_seq),
	.phys_rd_result(phys_rd_result),
	.phys_rd_crc_err(phys_rd_crc_err),
	.phys_rd_rnf(phys_rd_rnf),
	.phys_rd_deleted(phys_rd_deleted),
	.phys_rd_c(phys_rd_c),
	.phys_rd_h(phys_rd_h),
	.phys_rd_r(phys_rd_r),
	.phys_rd_n(phys_rd_n),
	.phys_byte_data(phys_byte_data),
	.phys_byte_empty(phys_byte_empty),
	.phys_media_ready(phys_media_ready),
	.phys_index(phys_index),
	.phys_track0(phys_track0),
	.phys_wprot(phys_wprot),
	.phys_change(phys_change),
	.phys_motor_on(phys_motor_on),
	.phys_head_settled(phys_head_settled),
	.phys_dbg_lost_tgl(phys_dbg_lost_tgl),
	.phys_dbg_drain_tgl(phys_dbg_drain_tgl),
	.phys_dbg_staledone_tgl(phys_dbg_staledone_tgl),
	.phys_dbg_busycmd_tgl(phys_dbg_busycmd_tgl),
	.phys_dbg_fin_tgl(phys_dbg_fin_tgl),
	.phys_dbg_pres_cnt(phys_dbg_pres_cnt)
);
endmodule
