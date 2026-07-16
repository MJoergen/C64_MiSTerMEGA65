//
// fdc1772.v
//
// Copyright (c) 2015 Till Harbaum <till@harbaum.org>
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

// TODO: 
// - 30ms settle time after step before data can be read
// - implement sector size 0
//
// Discovery by sy2002 in March 2022:
// Vivado needs interpret this as SystemVerilog even though it is "just" a ".v" file

// MEGA65 (iverilog/#90): the parameters were originally declared in the module
// body (after the port list), but the port list uses `W` (= FD_NUM-1). Vivado's
// SystemVerilog front-end tolerates the forward reference; iverilog -g2012 does
// not. Moving the parameters (and the derived `W`) into an ANSI #(...) header is
// a pure declaration-order fix -- no logic change; defaults are byte-identical.
module fdc1772 #(
	parameter CLK_EN           = 16'd8000, // in kHz
	parameter FD_NUM           = 1,    // number of supported floppies (changed by sy2002; should be a generic?)
	parameter MODEL            = 2,    // 0 - wd1770, 1 - fd1771, 2 - wd1772, 3 = wd1773/fd1793
	parameter SECTOR_SIZE_CODE = 2'd3, // sec size 0=128, 1=256, 2=512, 3=1024
	parameter SECTOR_BASE      = 1'b0, // number of first sector on track (archie 0, dos 1)
	parameter EXT_MOTOR        = 1'b0, // != 0 if motor is controlled externally by floppy_motor
	parameter INVERT_HEAD_RA   = 1'b0, // != 0 - invert head in READ_ADDRESS reply
	parameter W                = FD_NUM - 1  // MSB of the per-drive vectors (was a body localparam)
) (
	input            clkcpu, // system cpu clock.
	input            clk_sys, // MEGA65: QNICE clock for the SD/vdrives interface (CDC, sy2002/D81)
	input            clk8m_en,

	// external set signals
	input      [W:0] floppy_drive,
	input            floppy_side,
	input            floppy_reset,
	output           floppy_step,
	input            floppy_motor,
	output           floppy_ready,
	output           fdc_busy,     // MEGA65 (D81 drive LED): expose WD1772 command-busy for the 1581 activity LED

	// interrupts
	output reg       irq,
	output reg       drq, // data request

	input      [1:0] cpu_addr,
	input            cpu_sel,
	input            cpu_rw,
	input      [7:0] cpu_din,
	output reg [7:0] cpu_dout,

	// place any signals that need to be passed up to the top after here.
	input      [W:0] img_mounted, // signaling that new image has been mounted
	input      [W:0] img_wp,      // write protect
	input            img_ds,      // double-sided image (for BBC Micro only)
	input     [31:0] img_size,    // size of image in bytes
	output reg[31:0] sd_lba,
	output reg [W:0] sd_rd,
	output reg [W:0] sd_wr,
	input            sd_ack,
	input      [8:0] sd_buff_addr,
	input      [7:0] sd_dout,
	output     [7:0] sd_din,
	input            sd_dout_strobe,

	// ---------------------------------------------------------------------
	// MEGA65 physical internal 1581 (issue #90): flat toggle/level ABI to the
	// VHDL physical_1581_controller (50 MHz) and its external read FIFO.
	// Everything below is gated on phys_mode; with phys_mode=0 the block is
	// inert and the image (floppy.v / sd_*) path above is byte-identical.
	// ---------------------------------------------------------------------
	input            phys_mode,         // 1 = drive 8 backed by the real internal 1581

	// controller INPUTS driven BY fdc1772 (raw toggles/levels; controller syncs them)
	output           phys_active,       // capable + running enable for the controller
	output           phys_cia_motor_on, // motor-on request (PA2 sense)
	output           phys_cia_side,     // side select (PA0 sense)
	output reg       phys_step_req_tgl, // Type-I one-step request (toggle)
	output reg       phys_step_outward, // 1 = step toward track 0
	output reg       phys_rd_req_tgl,   // read-op request (toggle)
	output reg [2:0] phys_rd_op,        // RDOP_READ_SECTOR/READ_ADDRESS/VERIFY
	output reg [7:0] phys_rd_track,     // WD track register at request time
	output reg       phys_rd_side,      // current head/side
	output reg [7:0] phys_rd_sector,    // WD sector register at request time
	output reg       phys_rd_cancel_tgl,// force-interrupt / cancel (toggle)
	output reg [1:0] phys_rd_seq,       // op sequence tag: bumped for EVERY issued read op
	                                    // (incl. reissues); quasi-static before rd_req_tgl
	output           phys_byte_ovf,     // FIFO-overrun back to controller (see note; tied 0)

	// controller OUTPUTS consumed by fdc1772 (synced here into clkcpu)
	input            phys_step_ack_tgl, // step complete (toggle)
	input            phys_rd_done_tgl,  // read op complete (toggle)
	input      [1:0] phys_rd_done_seq,  // seq of the op being completed (quasi-static
	                                    // before rd_done_tgl; must match phys_rd_seq)
	input      [4:0] phys_rd_result,    // RES_* code (informational)
	input            phys_rd_crc_err,   // data/id CRC error
	input            phys_rd_rnf,       // record-not-found / not-ready family
	input            phys_rd_deleted,   // deleted data mark seen
	input      [7:0] phys_rd_c,         // ID field C/H/R/N (C -> sector reg on Read Address)
	input      [7:0] phys_rd_h,
	input      [7:0] phys_rd_r,
	input      [7:0] phys_rd_n,

	// external read FIFO port (read side is native clkcpu -> no sync needed)
	output reg       phys_byte_rd_en,   // pop the FIFO head
	input      [7:0] phys_byte_data,    // FIFO head byte (first-word-fall-through)
	input            phys_byte_empty,   // FIFO empty

	// live normalized drive state (level; synced into clkcpu below)
	input            phys_media_ready,
	input            phys_index,
	input            phys_track0,
	input            phys_wprot,
	input            phys_change,
	input            phys_motor_on,
	input            phys_head_settled,

	// diagnostic event toggles + per-op presented-byte count (drive-clock
	// domain; 2-FF-synced and counted in the QNICE diag device upstream).
	// Toggles are never reset: each edge marks exactly one event.
	output reg        phys_dbg_lost_tgl,      // presentation overwrote an unconsumed byte (LOST DATA)
	output reg        phys_dbg_drain_tgl,     // a between-ops drain episode started
	output reg        phys_dbg_staledone_tgl, // a done edge was ignored (seq mismatch / no op pending)
	output reg        phys_dbg_busycmd_tgl,   // a non-Force-Interrupt command write while busy was ignored
	output reg        phys_dbg_fin_tgl,       // a physical read op finalized (busy release)
	output reg [10:0] phys_dbg_pres_cnt       // bytes presented by the op just finalized
	                                          // (quasi-static after each fin toggle)
);

localparam SECTOR_SIZE = 11'd128 << SECTOR_SIZE_CODE;
localparam WIDX = $clog2(FD_NUM);

// -------------------------------------------------------------------------
// MEGA65 (iverilog/#90): forward declarations. The signals below are read in
// continuous assigns / always @(*) / wire initializers that appear textually
// before their original declaration. Vivado accepts that; iverilog -g2012 does
// not ("declared after use"). Declaring them here and turning the original
// `wire x = expr` sites into plain `assign` (or dropping the duplicate `reg`)
// is a pure reorder -- no logic change.
// -------------------------------------------------------------------------
localparam FDC_REG_CMDSTATUS = 0;
localparam FDC_REG_TRACK     = 1;
localparam FDC_REG_SECTOR    = 2;
localparam FDC_REG_DATA      = 3;

reg  [7:0]  track /* verilator public */;
reg  [7:0]  sector;
reg  [7:0]  data_out;
reg         step_dir;
reg         data_lost;
reg  [7:0]  cmd /* verilator public */;
reg         cmd_rx /* verilator public */;
wire        cmd_type_1;
wire        cmd_type_2;
wire        cmd_type_3;
wire        cmd_type_4;
reg         s_odd;      // odd sector
reg  [10:0] fifo_cpuptr;
wire        fd_doubleside;
wire [4:0]  fd_spt;
reg         data_transfer_start;
reg         data_transfer_done;
reg         sd_card_write;
reg         sd_card_read;
reg         cpu_rw_data;
wire        sd_done_tgl_c;

// -------------------------------------------------------------------------
// MEGA65 physical 1581 (#90): CDC + state
// -------------------------------------------------------------------------
// live level state 2-FF/iecdrv_sync'd into clkcpu
wire       phys_media_ready_c, phys_index_c, phys_track0_c, phys_wprot_c;
wire       phys_change_c, phys_motor_on_c, phys_head_settled_c;
wire [4:0] phys_rd_result_c;
wire       phys_rd_crc_err_c, phys_rd_rnf_c, phys_rd_deleted_c;
wire [7:0] phys_rd_c_c;
wire [1:0] phys_rd_done_seq_c;
// inbound toggles synced into clkcpu (edge-detected against *_serviced below)
wire       phys_step_ack_c, phys_rd_done_c;

iecdrv_sync      phys_mrdy_sync    (clkcpu, phys_media_ready,  phys_media_ready_c);
iecdrv_sync      phys_index_sync   (clkcpu, phys_index,        phys_index_c);
iecdrv_sync      phys_trk0_sync    (clkcpu, phys_track0,       phys_track0_c);
iecdrv_sync      phys_wp_sync      (clkcpu, phys_wprot,        phys_wprot_c);
iecdrv_sync      phys_chg_sync     (clkcpu, phys_change,       phys_change_c);
iecdrv_sync      phys_mot_sync     (clkcpu, phys_motor_on,     phys_motor_on_c);
iecdrv_sync      phys_hs_sync      (clkcpu, phys_head_settled, phys_head_settled_c);
iecdrv_sync #(5) phys_res_sync     (clkcpu, phys_rd_result,    phys_rd_result_c);
iecdrv_sync      phys_crc_sync     (clkcpu, phys_rd_crc_err,   phys_rd_crc_err_c);
iecdrv_sync      phys_rnf_sync     (clkcpu, phys_rd_rnf,       phys_rd_rnf_c);
iecdrv_sync      phys_del_sync     (clkcpu, phys_rd_deleted,   phys_rd_deleted_c);
iecdrv_sync #(8) phys_c_sync       (clkcpu, phys_rd_c,         phys_rd_c_c);
iecdrv_sync #(2) phys_dseq_sync    (clkcpu, phys_rd_done_seq,  phys_rd_done_seq_c);
iecdrv_sync      phys_stepack_sync (clkcpu, phys_step_ack_tgl, phys_step_ack_c);
iecdrv_sync      phys_rddone_sync  (clkcpu, phys_rd_done_tgl,  phys_rd_done_c);

// MEGA65 (#90 review): consume the rd-done toggle two clkcpu AFTER it resolves. The
// controller writes the result flags/C byte/done seq in the SAME 50 MHz cycle it
// flips the toggle, and every signal crosses through its own independent
// iecdrv_sync -- whose two-sample agreement filter may resolve each signal one
// destination cycle apart. Acting on the raw synced toggle could therefore latch
// stale flags (a stale rnf/crc would report a clean status for an error result and
// could wrongly continue a multi-sector chain; a stale done seq would misclassify
// the completion). By the time the edge is two cycles old, every flag launched
// with it is guaranteed stable.
reg phys_rd_done_c_d1 = 1'b0;
reg phys_rd_done_c_d2 = 1'b0;
always @(posedge clkcpu) begin
	phys_rd_done_c_d1 <= phys_rd_done_c;
	phys_rd_done_c_d2 <= phys_rd_done_c_d1;
end

// physical read/step engine state (clkcpu)
reg         phys_step_busy;         // waiting for a step ack
reg         phys_rd_pending;        // a read op has been requested, not yet reported done
reg         phys_reading;           // enable byte pace (drain the FIFO)
reg         phys_done_latched;      // controller reported done; release clean FIFO / drain error
reg         phys_verify;            // the in-flight read op is a Type-I verify
reg         phys_reissue;           // deferred re-issue for a multiple-sector read
reg         phys_rnf_l, phys_crc_l, phys_del_l;   // latched result flags
reg  [7:0]  phys_c_l;               // latched ID C (Read Address -> sector reg)
reg         phys_step_ack_serviced; // last serviced step-ack toggle value
reg         phys_rd_done_serviced;  // last serviced rd-done toggle value
reg         phys_rd_start;          // 1-cycle "new op" strobe: re-arm pace, clear pres/lost
reg         phys_cmd_clear = 1'b0;  // 1-cycle: phys command accepted -> clear DRQ + LOST DATA
reg         phys_set_sector;        // Read Address: write phys_c_l into sector reg
reg  [7:0]  phys_step_tally;        // steps issued by the current command (RESTORE bound)

// MEGA65 (#90 delivery v2): DISK-PACED presentation. One DD MFM byte-time is
// 32 us = 252 clk8m_en ticks (~7.88 MHz); the pace counter spaces byte
// presentations exactly like the real WD1772 moves bytes from its shift
// register into the data register -- independent of the drive CPU. The pres
// counter and lost flag are per-op diagnostics/status (never gate completion).
localparam [7:0] PHYS_PACE_TICKS = 8'd252;
reg  [7:0]  phys_pace_cnt = 8'd0;   // clk8m ticks until the next presentation (0 = expired)
reg  [10:0] phys_pres_cnt = 11'd0;  // bytes PRESENTED this op (diagnostics only)
reg         phys_lost_l   = 1'b0;   // phys LOST DATA flag (status bit 2, Type II/III)
reg         phys_draining = 1'b0;   // drain-episode tracker for phys_dbg_drain_tgl

// MEGA65 (#90 round 11): MINIMUM Type-I busy duration in phys mode. A zero-step
// SEEK (track already equals the target) or a RESTORE with TR00 already active
// completes in ~3 clk8m ticks (~380 ns) in this RTL, but the real WD1772's
// internal microcode keeps busy set for on the order of a millisecond even
// when no step pulses are needed. The 1581 ROM RELIES on that: its command
// writer at $CBF4 spins in "wait busy-SET" ($CBFA: BIT $6000 / BEQ, one poll
// every ~3.5 us at 2 MHz) AFTER writing the command -- a sub-microsecond busy
// pulse is invisible to it and the DOS hangs forever with I set (motor frozen
// on, LED frozen off, zero further WD traffic; observed on hardware when the
// error-recovery job $C0 issued its re-positioning zero-step seek at $CB0F).
// Type II/III ops are inherently slow (controller round trip >= ms) and Force
// Interrupt must stay immediate, so only Type-I completion is gated. 12000
// ticks ~= 1.5 ms, matching the real chip's order of magnitude (and the
// rom_emu reference model's max(1,n)*1.5 ms).
localparam [13:0] PHYS_T1_MIN_TICKS = 14'd12000;
reg  [13:0] phys_t1_min_cnt = 14'd0; // clk8m ticks; Type-I may not finish before 0

// MEGA65 (#90 round 10 hardening): never present INTO an open drive-CPU READ
// access of the WD data register. The T65 keeps cpu_sel/cpu_addr asserted for
// the whole 2 MHz bus cycle (~16 clkcpu) and latches cpu_dout at the CLOSING
// enable tick, so a paced presentation landing inside that window would
//  (a) overwrite data_out mid-read: the CPU latches the NEW byte at its
//      closing tick and then reads the same byte again at its DRQ -- one
//      corrupted byte plus a duplicate, under clean status;
//  (b) land in the same clkcpu as the registered drq_clr of that read (the
//      drq register is clear-dominant), silently swallowing the byte's DRQ;
//  (c) sample drq=1 for a byte consumed in that very cycle -> false LOST DATA.
// Deferring while the access is open fixes all three with one mechanism: the
// presentation proceeds on the next eligible clk8m tick after the access
// closes. The deferral is bounded by the bus access length (~16 clkcpu, i.e.
// ~0.5 us versus the 32 us pace) -- still disk-time-bounded, NOT consumption-
// coupled: it waits for the bus cycle to close, never for the byte to be
// consumed (back-to-back data-register reads are always separated by opcode
// fetches, which deassert the select). A short-select implementation can drop
// cpu_sel before the registered cpu_rw_data pulse clears DRQ, so that pulse is
// a second mandatory exclusion window.
wire phys_cpu_rd_data_open = cpu_sel && cpu_rw && (cpu_addr == FDC_REG_DATA);

// Physical bytes are SPECULATIVE until the controller has checked the complete
// field CRC. The production 512-byte async FIFO is therefore also the sector
// quarantine: presentation starts only after a tag-matched CLEAN done. An
// error done drops phys_reading and the residue drain discards the FIFO without
// ever asserting DRQ. This gives the proven WD/ROM side a completed-sector
// transaction instead of exposing a live, unvalidated magnetic stream.
//
// Once released, presentation fires on a clk8m tick with the pace expired and
// a byte available; it never waits for the previous byte to be consumed (A2).
// The pace counter holds at 0 across a read-access deferral, so a pace-expired
// presentation happens immediately after both exclusion windows close.
wire phys_present_now = phys_mode && phys_reading
                        && phys_done_latched && !phys_rnf_l && !phys_crc_l
                        && (phys_pace_cnt == 8'd0) && !phys_byte_empty
                        && !phys_cpu_rd_data_open && !cpu_rw_data;

// controller INPUTS that are pure combinational (EXT_MOTOR=1 for the 1581, so
// fd_motor == floppy_motor). phys_byte_ovf is tied low: fdc1772 only sees the
// FIFO read side, so it cannot detect a write-side overrun -- the real
// rdfifo full flag is wired to the controller at the top level, not through here.
assign phys_active       = phys_mode & floppy_reset;
assign phys_cia_motor_on = floppy_motor;
assign phys_cia_side     = floppy_side;
assign phys_byte_ovf     = 1'b0;

// MEGA65 (#90): the request toggles are NOT reset on floppy_reset (that would
// inject a phantom edge into the controller); give them a defined power-up value
// so the very first `~tgl` is a real edge (FPGA config value; matches HW). The
// same applies to the sequence tag and the diagnostic event toggles.
initial begin
	phys_step_req_tgl  = 1'b0;
	phys_rd_req_tgl    = 1'b0;
	phys_rd_cancel_tgl = 1'b0;
	phys_rd_seq        = 2'd0;
	phys_dbg_lost_tgl      = 1'b0;
	phys_dbg_drain_tgl     = 1'b0;
	phys_dbg_staledone_tgl = 1'b0;
	phys_dbg_busycmd_tgl   = 1'b0;
	phys_dbg_fin_tgl       = 1'b0;
	phys_dbg_pres_cnt      = 11'd0;
end

// -------------------------------------------------------------------------
// --------------------- IO controller image handling ----------------------
// -------------------------------------------------------------------------

// MEGA65 (D81 enable): sd_lba is computed combinationally here from the (clkcpu-domain,
// quasi-static) track/sector, but DRIVEN onto the output by the clk_sys SD FSM (label3),
// which latches this stable value at the start of each transfer. See sd_lba_comb -> sd_lba.
reg [31:0] sd_lba_comb;
always @(*) begin
	case (SECTOR_SIZE_CODE)
	// archie
	3: sd_lba_comb = {(16'd0 + (fd_spt*track[6:0]) << fd_doubleside) + (floppy_side ? 5'd0 : fd_spt) + sector[4:0], s_odd };
	// st
	2: sd_lba_comb = ((fd_spt*track[6:0]) << fd_doubleside) + (floppy_side ? 5'd0 : fd_spt) + sector[4:0] - 1'd1;
	// bbc micro
	1: sd_lba_comb = (((fd_spt*track[6:0]) << fd_doubleside) + (floppy_side ? 5'd0 : fd_spt) + sector[4:0]) >> 1;
	default: sd_lba_comb = 0;
	endcase
end

reg  [10:0] fdn_sector_len[FD_NUM];
reg   [4:0] fdn_spt[FD_NUM];     // sectors/track
reg   [9:0] fdn_gap_len[FD_NUM]; // gap len/sector
reg         fdn_doubleside[FD_NUM];
reg         fdn_hd[FD_NUM];
reg         fdn_fm[FD_NUM];
reg         fdn_present[FD_NUM];

reg  [11:0] image_sectors;
reg  [11:0] image_sps; // sectors/side
reg   [4:0] image_spt; // sectors/track
reg   [9:0] image_gap_len;
reg         image_doubleside;
wire        image_hd = img_size[20];
reg         image_fm;

always @(*) begin
	case (SECTOR_SIZE_CODE)
	3: begin
		// archie, 1024 bytes/sector
		image_fm = 0;
		image_sectors = img_size[21:10];
		image_doubleside = 1'b1;
		image_spt = image_hd ? 5'd10 : 5'd5;
		image_gap_len = 10'd220;
	end
	2: begin
		// this block is valid for the .st format (or similar arrangement), 512 bytes/sector
		image_fm = 0;
		image_sectors = img_size[20:9];
		image_doubleside = 1'b0;
		image_sps = image_sectors;
		if (image_sectors > (85*12)) begin
			image_doubleside = 1'b1;
			image_sps = image_sectors >> 1'b1;
		end
		if (image_hd) image_sps = image_sps >> 1'b1;

		// spt : 9-12, tracks: 79-85
		case (image_sps)
			711,720,729,738,747,756,765   : image_spt = 5'd9;
			790,800,810,820,830,840,850   : image_spt = 5'd10;
			948,960,972,984,996,1008,1020 : image_spt = 5'd12;
			default : image_spt = 5'd11;
		endcase;

		if (image_hd) image_spt = image_spt << 1'b1;

		// SECTOR_GAP_LEN = BPT/SPT - (SECTOR_LEN + SECTOR_HDR_LEN) = 6250/SPT - (512+6)
		case (image_spt)
			5'd9, 5'd18: image_gap_len = 10'd176;
			5'd10,5'd20: image_gap_len = 10'd107;
			5'd11,5'd22: image_gap_len = 10'd50;
			default : image_gap_len = 10'd2;
		endcase;
	end
	1: begin
		// 256 bytes/sector single density (BBC SSD/DSD, TI99/4A)
		image_fm = 1;
		image_sectors = img_size[19:8];
		image_doubleside = img_ds;
		if (img_ds)
			image_sps = image_sectors >> 1'b1;
		else
			image_sps = image_sectors;
		case (image_sps)
			360: image_spt = 9; // TI99/4A
			default: image_spt = 10; // BBC Micro
		endcase
		image_gap_len = 10'd50;
	end
	default: begin
		image_fm = 0;
		image_sectors = 0;
		image_doubleside = 0;
		image_spt = 0;
		image_gap_len = 0;
	end

	endcase
end

always @(posedge clkcpu) begin : label0
	reg [W:0] img_mountedD;
	integer i;
	img_mountedD <= img_mounted;
	
	for(i = 0; i < FD_NUM; i = i+1'd1) begin
		if (~img_mountedD[i] && img_mounted[i]) begin
			fdn_present[i] <= |img_size;
			fdn_sector_len[i] <= SECTOR_SIZE;
			fdn_spt[i] <= image_spt;
			fdn_gap_len[i] <= image_gap_len;
			fdn_doubleside[i] <= image_doubleside;
			fdn_hd[i] <= image_hd;
			fdn_fm[i] <= image_fm;
		end
	end
end

// -------------------------------------------------------------------------
// ---------------------------- IRQ/DRQ handling ---------------------------
// -------------------------------------------------------------------------
reg cpu_selD;
reg cpu_rwD;
always @(posedge clkcpu) begin
	cpu_rwD <= cpu_sel & ~cpu_rw;
	cpu_selD <= cpu_sel;
end

wire cpu_we = cpu_sel & ~cpu_rw & ~cpu_rwD;

reg irq_set;

// floppy_reset and read of status register/write of command register clears irq
reg cpu_rw_cmdstatus;
always @(posedge clkcpu)
  cpu_rw_cmdstatus <= ~cpu_selD && cpu_sel && cpu_addr == FDC_REG_CMDSTATUS;

wire irq_clr = !floppy_reset || cpu_rw_cmdstatus;

always @(posedge clkcpu) begin
	if(irq_clr) irq <= 1'b0;
	else if(irq_set) irq <= 1'b1;
end

reg drq_set;

always @(posedge clkcpu)
	cpu_rw_data <= ~cpu_selD && cpu_sel && cpu_addr == FDC_REG_DATA;

// MEGA65 (#90 delivery v2): accepting a Type-I/II/III command in phys mode also
// clears DRQ (phys_cmd_clear, 1 cycle) -- real-WD1772 command-start semantics.
// In image mode phys_cmd_clear never pulses, so drq_clr is byte-identical.
wire drq_clr = !floppy_reset || cpu_rw_data || phys_cmd_clear;

always @(posedge clkcpu) begin
	if(drq_clr) drq <= 1'b0;
	else if(drq_set) drq <= 1'b1;
end

// -------------------------------------------------------------------------
// -------------------- virtual floppy drive mechanics ---------------------
// -------------------------------------------------------------------------

wire       fdn_index[FD_NUM];
wire       fdn_ready[FD_NUM];
wire [6:0] fdn_track[FD_NUM];
wire [4:0] fdn_sector[FD_NUM];
wire       fdn_sector_hdr[FD_NUM];
wire       fdn_sector_data[FD_NUM];
wire       fdn_dclk[FD_NUM];

reg [WIDX:0] fdn;
always @(*) begin : label1
	integer i;

	fdn = 0;
	for(i = FD_NUM-1; i >= 0; i = i - 1) if(!floppy_drive[i]) fdn = i[WIDX:0];
end

wire       fd_any = ~&floppy_drive;

reg step_in, step_out;
reg motor_on /* verilator public */ = 1'b0;
wire fd_motor = EXT_MOTOR ? floppy_motor : motor_on;

generate
	genvar i;
	
	for(i=0; i < FD_NUM; i = i+1) begin :fdd

		floppy #(.CLK_EN(CLK_EN)) floppy
		(
			.clk         ( clkcpu             ),
			.clk8m_en    ( clk8m_en           ),

			// control signals into floppy
			.select      ( fd_any && fdn == i ),
			.motor_on    ( fd_motor           ),
			.step_in     ( step_in            ),
			.step_out    ( step_out           ),

			// physical parameters
			.sector_len  ( fdn_sector_len[i]  ),
			.spt         ( fdn_spt[i]         ),
			.sector_gap_len ( fdn_gap_len[i]  ),
			.sector_base ( SECTOR_BASE[0]     ),
			.hd          ( fdn_hd[i]          ),
			.fm          ( fdn_fm[i]          ),

			// status signals generated by floppy
			.dclk_en     ( fdn_dclk[i]        ),
			.track       ( fdn_track[i]       ),
			.sector      ( fdn_sector[i]      ),
			.sector_hdr  ( fdn_sector_hdr[i]  ),
			.sector_data ( fdn_sector_data[i] ),
			.ready       ( fdn_ready[i]       ),
			.index       ( fdn_index[i]       )
		);
	end
endgenerate

// -------------------------------------------------------------------------
// ----------------------------- floppy demux ------------------------------
// -------------------------------------------------------------------------

wire       fd_index       = fd_any ? fdn_index[fdn]       : 1'b0;
wire       fd_ready       = fd_any ? fdn_ready[fdn]       : 1'b0;
wire [6:0] fd_track       = fd_any ? fdn_track[fdn]       : 7'd0;
wire [4:0] fd_sector      = fd_any ? fdn_sector[fdn]      : 5'd0;
wire       fd_sector_hdr  = fd_any ? fdn_sector_hdr[fdn]  : 1'b0;
//wire     fd_sector_data = fd_any ? fdn_sector_data[fdn] : 1'b0;
wire       fd_dclk_en     = fd_any ? fdn_dclk[fdn]        : 1'b0;
wire       fd_present     = fd_any ? fdn_present[fdn]     : 1'b0;
wire       fd_writeprot   = fd_any ? img_wp[fdn]          : 1'b1;

assign     fd_doubleside  = fdn_doubleside[fdn];   // MEGA65 (#90): forward-declared above
assign     fd_spt         = fdn_spt[fdn];          // MEGA65 (#90): forward-declared above

// MEGA65 (#90): in phys_mode PA1 ready sense comes from the physical controller.
assign floppy_ready = phys_mode ? phys_media_ready_c : (fd_ready && fd_present);

// MEGA65 (#90 bring-up): index source for the WD's own index-based housekeeping
// (motor idle timeout, spin-up countdown, Force-Interrupt-on-index) -- the real
// mechanism index in phys mode, the image floppy model otherwise.
wire fd_index_eff = phys_mode ? phys_index_c : fd_index;

// -------------------------------------------------------------------------
// ----------------------- internal state machines -------------------------
// -------------------------------------------------------------------------

// --------------------------- Motor handling ------------------------------

// if motor is off and type 1 command with "spin up sequnce" bit 3 set
// is received then the command is executed after the motor has
// reached full speed for 5 rotations (800ms spin-up time + 5*200ms =
// 1.8sec) If the floppy is idle for 10 rotations (2 sec) then the
// motor is switched off again
localparam MOTOR_IDLE_COUNTER = 4'd10;
reg [3:0] motor_timeout_index /* verilator public */;
reg indexD;
reg busy /* verilator public */;
reg [3:0] motor_spin_up_sequence /* verilator public */;

// consider spin up done either if the motor is not supposed to spin at all or
// if it's supposed to run and has left the spin up sequence
wire motor_spin_up_done = (!motor_on) || (motor_on && (motor_spin_up_sequence == 0));

// ---------------------------- step handling ------------------------------

localparam STEP_PULSE_LEN = 16'd1;
localparam STEP_PULSE_CLKS = STEP_PULSE_LEN * CLK_EN;
reg [15:0] step_pulse_cnt;

// the step rate is only valid for command type I
wire [15:0] step_rate_clk = 
           (cmd[1:0]==2'b00)               ? (16'd6 *CLK_EN-1'd1):   //  6ms
           (cmd[1:0]==2'b01)               ? (16'd12*CLK_EN-1'd1):   // 12ms
           (MODEL == 2 && cmd[1:0]==2'b10) ? (16'd2 *CLK_EN-1'd1):   //  2ms
           (cmd[1:0]==2'b10)               ? (16'd20*CLK_EN-1'd1):   // 20ms
           (MODEL == 2)                    ? (16'd3 *CLK_EN-1'd1):   //  3ms
                                             (16'd30*CLK_EN-1'd1);   // 30ms

reg [15:0] step_rate_cnt;
reg [23:0] delay_cnt;

assign floppy_step = step_in | step_out;
assign fdc_busy    = busy;   // MEGA65 (D81 drive LED): WD1772 command-busy -> 1581 activity LED (clkcpu domain, no CDC)

// flag indicating that a "step" is in progress
wire step_busy = (step_rate_cnt != 0);
wire delaying = (delay_cnt != 0);

wire fd_track0 = (fd_track == 0);

reg [7:0] step_to;
reg RNF;
reg sector_inc_strobe;
reg track_inc_strobe;
reg track_dec_strobe;
reg track_clear_strobe;

always @(posedge clkcpu) begin : label2
	reg [1:0] seek_state;
	reg notready_wait;
	reg sector_not_found;
	reg irq_at_index;
	reg [1:0] data_transfer_state;
	reg       sd_io_idle;
	reg       sd_done_tgl_cD;

	sector_inc_strobe <= 1'b0;
	track_inc_strobe <= 1'b0;
	track_dec_strobe <= 1'b0;
	track_clear_strobe <= 1'b0;
	irq_set <= 1'b0;

	// MEGA65 (#90): 1-cycle physical-mode strobes
	phys_set_sector <= 1'b0;
	phys_rd_start   <= 1'b0;
	phys_cmd_clear  <= 1'b0;

	// MEGA65 (D81 enable): sd_io_idle is the CDC-safe replacement for the old
	// `sd_state == SD_IDLE` gate (sd_state now lives in clk_sys, label3). It is cleared
	// in the very cycle/branch that issues an SD request (race-free) and set again here
	// when the clk_sys SD FSM toggles sd_done_tgl (synced to clkcpu as sd_done_tgl_c).
	sd_done_tgl_cD <= sd_done_tgl_c;
	if (sd_done_tgl_c ^ sd_done_tgl_cD) sd_io_idle <= 1'b1;

	if(!floppy_reset) begin
		motor_on <= 1'b0;
		busy <= 1'b0;
		step_in <= 1'b0;
		step_out <= 1'b0;
		sd_card_read <= 0;
		sd_card_write <= 0;
		sd_io_idle <= 1'b1;
		data_transfer_start <= 1'b0;
		seek_state <= 0;
		notready_wait <= 1'b0;
		sector_not_found <= 1'b0;
		irq_at_index <= 1'b0;
		data_transfer_state <= 2'b00;
		RNF <= 1'b0;

		// MEGA65 (#90): physical engine reset (toggle OUTPUTS are left untouched to
		// avoid a phantom edge; the *_serviced trackers are re-armed to the current
		// synced value so no stale ack/done is seen after reset).
		phys_step_busy    <= 1'b0;
		phys_rd_pending   <= 1'b0;
		phys_reading      <= 1'b0;
		phys_done_latched <= 1'b0;
		phys_verify       <= 1'b0;
		phys_reissue      <= 1'b0;
		phys_rnf_l <= 1'b0; phys_crc_l <= 1'b0; phys_del_l <= 1'b0;
		phys_step_tally <= 8'd0;
		phys_t1_min_cnt <= 14'd0;
		phys_step_ack_serviced <= phys_step_ack_c;
		phys_rd_done_serviced  <= phys_rd_done_c_d2;
	end else if (clk8m_en) begin
		sd_card_read <= 0;
		sd_card_write <= 0;
		data_transfer_start <= 1'b0;

		// disable step signal after 1 msec
		if(step_pulse_cnt != 0) 
			step_pulse_cnt <= step_pulse_cnt - 16'd1;
		else begin
			step_in <= 1'b0;
			step_out <= 1'b0;
		end

		 // step rate timer
		if(step_rate_cnt != 0) 
			step_rate_cnt <= step_rate_cnt - 16'd1;

		// delay timer
		if(delay_cnt != 0)
			delay_cnt <= delay_cnt - 1'd1;

		// MEGA65 (#90 round 11): minimum Type-I busy time (see declaration)
		if(phys_t1_min_cnt != 0)
			phys_t1_min_cnt <= phys_t1_min_cnt - 14'd1;

		// just received a new command
		if(cmd_rx) begin
			busy <= 1'b1;
			notready_wait <= 1'b0;
			sector_not_found <= 1'b0;
			data_transfer_state <= 2'b00;

			// MEGA65 (#90): start every command from a clean physical-engine state
			phys_rd_pending   <= 1'b0;
			phys_reading      <= 1'b0;
			phys_done_latched <= 1'b0;
			phys_verify       <= 1'b0;
			phys_step_busy    <= 1'b0;
			phys_reissue      <= 1'b0;
			phys_step_tally   <= 8'd0;

			if(cmd_type_1 || cmd_type_2 || cmd_type_3) begin
				RNF <= 1'b0;
				motor_on <= 1'b1;
				// 'h' flag '0' -> wait for spin up
				if (!motor_on && !cmd[3]) motor_spin_up_sequence <= 6;   // wait for 6 full rotations

				// MEGA65 (#90 delivery v2): real-WD1772 command-start semantics --
				// accepting a command clears DRQ, LOST DATA and the latched result
				// flags. phys_cmd_clear reaches the DRQ clear and the delivery
				// block (which owns phys_lost_l). Image mode untouched.
				if (phys_mode) begin
					phys_cmd_clear <= 1'b1;
					phys_rnf_l <= 1'b0; phys_crc_l <= 1'b0; phys_del_l <= 1'b0;
					// round 11: arm the minimum Type-I busy time (see declaration)
					if (cmd_type_1) phys_t1_min_cnt <= PHYS_T1_MIN_TICKS;
				end
			end

			// handle "forced interrupt"
			if(cmd_type_4) begin
				busy <= 1'b0;
				if(cmd[3]) irq_set <= 1'b1;
				if(cmd[3:2] == 2'b01) irq_at_index <= 1'b1;
				// From Hatari: Starting a Force Int command when idle should set the motor bit and clear the spinup bit (verified on STF)
				if (!busy) motor_on <= 1'b1;

				// MEGA65 (#90): abort any in-flight physical operation
				if (phys_mode) begin
					phys_rd_cancel_tgl <= ~phys_rd_cancel_tgl;
					phys_rd_pending    <= 1'b0;
					phys_reading       <= 1'b0;
					phys_done_latched  <= 1'b0;
					phys_verify        <= 1'b0;
					phys_step_busy     <= 1'b0;
					phys_reissue       <= 1'b0;
				end
			end
		end

		// execute command if motor is not supposed to be running or
		// wait for motor spinup to finish
		// MEGA65 (#90): in phys_mode the controller owns spin-up/readiness (and the
		// image-mode index pulses do not exist), so do not gate on motor_spin_up_done.
		if(busy && (phys_mode ? 1'b1 : motor_spin_up_done) && !step_busy && !delaying) begin

			// ------------------------ TYPE I -------------------------
			if(cmd_type_1) begin
				// MEGA65 (#90): in phys_mode Type-I commands run UNCONDITIONALLY, like
				// on the real WD1772 (the 177x has no READY input; only the 179x gated
				// commands on readiness). Gating them on media-ready would deadlock the
				// whole drive: media-ready requires the disk-change latch to be clear,
				// the latch is only cleared by a step, and every step arrives as a
				// Type-I command. The 1581 DOS resolves power-up/change by stepping;
				// it must always be able to. track0 still comes from the controller.
				if(!phys_mode && !fd_present) begin
					// no image/disk selected -> send irq after 6 ms
					if (!notready_wait) begin
						delay_cnt <= 16'd6*CLK_EN;
						notready_wait <= 1'b1;
					end else begin
						RNF <= 1'b1;
						busy <= 1'b0;
						irq_set <= 1'b1; // emit irq when command done
					end
				end else
				// evaluate command
				case (seek_state)
				0: begin
					// restore
					if(cmd[7:4] == 4'b0000) begin
						if (phys_mode ? phys_track0_c : fd_track0) begin
							track_clear_strobe <= 1'b1;
							seek_state <= 2;
						end else if (phys_mode && phys_step_tally == 8'd255) begin
							// MEGA65 (#90 review): real-WD1772 RESTORE bound -- if TR00
							// never asserts after 255 step pulses (mechanism absent or
							// track0 sensor broken), terminate with Seek Error + INTRQ
							// instead of stepping forever with busy stuck high. (The
							// image path needs no bound: fd_track reaches 0 by construction.)
							RNF <= 1'b1;
							seek_state <= 3;
						end else begin
							step_dir <= 1'b1;
							seek_state <= 1;
						end
					end

					// seek
					if(cmd[7:4] == 4'b0001) begin
						if (track == step_to) seek_state <= 2;
						else begin
							step_dir <= (step_to < track);
							seek_state <= 1;
						end
					end

					// step
					if(cmd[7:5] == 3'b001) seek_state <= 1;

					// step-in
					if(cmd[7:5] == 3'b010) begin
						step_dir <= 1'b0;
						seek_state <= 1;
					end

					// step-out
					if(cmd[7:5] == 3'b011) begin
						step_dir <= 1'b1;
						seek_state <= 1;
					end
				   end

				// do the step
				1: begin
					if (phys_mode) begin
						// MEGA65 (#90): one acknowledged physical STEP. Do not advance the
						// seek FSM here -- the synced step-ack (central block) does that.
						if (!phys_step_busy) begin
							phys_step_outward <= step_dir;          // 1 = toward track 0
							phys_step_req_tgl <= ~phys_step_req_tgl;
							phys_step_busy    <= 1'b1;
							if (phys_step_tally != 8'd255)
								phys_step_tally <= phys_step_tally + 8'd1;
							// update the track register (same U-flag rules as the image path)
							if( (!cmd[6] && !cmd[5]) || ((cmd[6] || cmd[5]) && cmd[4]))
								if (step_dir)
									track_dec_strobe <= 1'b1;
								else
									track_inc_strobe <= 1'b1;
						end
					end else begin
						if (step_dir)
							step_in <= 1'b1;
						else
							step_out <= 1'b1;

						// update the track register if seek/restore or the update flag set
						if( (!cmd[6] && !cmd[5]) || ((cmd[6] || cmd[5]) && cmd[4]))
							if (step_dir)
								track_dec_strobe <= 1'b1;
							else
								track_inc_strobe <= 1'b1;

						step_pulse_cnt <= STEP_PULSE_CLKS - 1'd1;
						step_rate_cnt <= step_rate_clk;

						seek_state <= (!cmd[6] && !cmd[5]) ? 0 : 2; // loop for seek/restore
					end
				   end

				// verify
				2: begin
					if (phys_mode) begin
						if (cmd[2]) begin
							// MEGA65 (#90): after the last step wait for head-settle, then
							// issue a physical VERIFY read op (completion routes to finish).
							if (phys_head_settled_c && !phys_rd_pending && !phys_verify) begin
								phys_rd_op      <= 3'b010;    // RDOP_VERIFY (streams no bytes)
								phys_rd_track   <= track;
								phys_rd_side    <= floppy_side;
								phys_rd_sector  <= sector;
								phys_rd_seq     <= phys_rd_seq + 2'd1;
								phys_rd_req_tgl <= ~phys_rd_req_tgl;
								phys_rd_pending <= 1'b1;
								phys_verify     <= 1'b1;
								phys_rd_start   <= 1'b1;       // re-arm pace, clear pres/lost
							end
						end else
							seek_state <= 3;
					end else begin
						if (cmd[2]) begin
							delay_cnt <= 16'd3*CLK_EN; // TODO: implement verify, now just delay one more step
						end
						seek_state <= 3;
					end
				   end

				// finish
				3: begin
					// MEGA65 (#90 round 11): in phys mode hold busy until the
					// minimum Type-I busy time has elapsed -- a zero-step seek or
					// restore must stay observable to the ROM's wait-busy-set poll
					// (see the PHYS_T1_MIN_TICKS declaration). Image mode unchanged.
					if (!phys_mode || phys_t1_min_cnt == 14'd0) begin
						busy <= 1'b0;
						irq_set <= 1'b1; // emit irq when command done
						seek_state <= 0;
					end
				   end
				endcase
			end // if (cmd_type_1)

			// ------------------------ TYPE II -------------------------
			if(cmd_type_2 && phys_mode) begin
				// MEGA65 (#90): physical Type-II. Read Sector issues one physical read
				// op (streamed via the external FIFO, finished on rd_done). Every
				// accepted op completes ONLY through the controller done handshake --
				// there is NO unilateral WD-side completion (a not-ready mechanism is
				// bounded by the controller at ~1.1 s -> RES_NOT_READY, an unmatched
				// search at 5 index edges / 1.3 s -> RNF), so the controller can never
				// be left running an op the WD already gave up on (that desync paired
				// a stale result with the next command). Write Sector is BLOCKED
				// (read-only milestone): the disk is presented write-protected (status
				// WP bit forced above) and the command completes without touching
				// media (no sd_card_write, no FIFO).
				if (cmd[7:5] == 3'b100) begin
					// read sector
					if (!phys_rd_pending && !phys_done_latched && !phys_reissue) begin
						phys_rd_op      <= 3'b000;    // RDOP_READ_SECTOR
						phys_rd_track   <= track;
						phys_rd_side    <= floppy_side;
						phys_rd_sector  <= sector;
						phys_rd_seq     <= phys_rd_seq + 2'd1;
						phys_rd_req_tgl <= ~phys_rd_req_tgl;
						phys_rd_pending <= 1'b1;
						phys_reading    <= 1'b1;
						phys_rd_start   <= 1'b1;
					end
				end else if (cmd[7:5] == 3'b101) begin
					// write sector -> blocked; complete write-protected
					busy    <= 1'b0;
					irq_set <= 1'b1;
				end
			end else
			if(cmd_type_2) begin
				if(!fd_present) begin
					// no image selected -> send irq after 6 ms
					if (!notready_wait) begin
						delay_cnt <= 16'd6*CLK_EN;
						notready_wait <= 1'b1;
					end else begin
						RNF <= 1'b1;
						busy <= 1'b0;
						irq_set <= 1'b1; // emit irq when command done
					end
				end else if (sector_not_found) begin
					busy <= 1'b0;
					irq_set <= 1'b1; // emit irq when command done
					RNF <= 1'b1;
				end else if (cmd[2] && !notready_wait) begin
					// e flag: 15 ms settling delay
					delay_cnt <= 16'd15*CLK_EN;
					notready_wait <= 1'b1;
					// read sector
				end else begin
					if(cmd[7:5] == 3'b100) begin
						if ((sector - SECTOR_BASE[0]) >= fd_spt) begin
							// wait 5 rotations (1 sec) before setting RNF
							sector_not_found <= 1'b1;
							delay_cnt <= 24'd1000 * CLK_EN;
						end else if (sd_io_idle) begin
							case (data_transfer_state)

							2'b00: if (fifo_cpuptr == 0) begin
								// SD Card phase
								sd_card_read <= 1;
								sd_io_idle <= 1'b0;   // MEGA65: mark SD busy in the same cycle (race-free)
								data_transfer_state <= 2'b01;
							end

							2'b01: begin
								// CPU phase
								// we are busy until the right sector header passes under 
								// the head and the sd-card controller indicates the sector
								// is in the fifo
								if(fd_ready && fd_sector_hdr && (fd_sector == sector)) data_transfer_start <= 1'b1;

								if(data_transfer_done) begin
									data_transfer_state <= 2'b00;
									if (cmd[4]) sector_inc_strobe <= 1'b1; // multiple sector transfer
									else begin
										busy <= 1'b0;
										irq_set <= 1'b1; // emit irq when command done
									end
								end
							end

							default :;
							endcase

						end
					end

					// write sector
					if(cmd[7:5] == 3'b101) begin
						if ((sector - SECTOR_BASE[0]) >= fd_spt) begin
							// wait 5 rotations (1 sec) before setting RNF
							sector_not_found <= 1'b1;
							delay_cnt <= 24'd1000 * CLK_EN;
						end else if (sd_io_idle) begin
							case (data_transfer_state)
							2'b00: begin
								// pre-read phase
									if (SECTOR_SIZE_CODE < 2) begin
										sd_card_read <= 1;
										sd_io_idle <= 1'b0;   // MEGA65: race-free SD-busy (size<2 pre-read only)
									end
									data_transfer_state <= 2'b10;
								end
							2'b10: begin
								// CPU phase
								if (fifo_cpuptr == 0 && fd_ready && fd_sector_hdr && (fd_sector == sector)) data_transfer_start <= 1'b1;
								if (data_transfer_done) begin
									sd_card_write <= 1;
									sd_io_idle <= 1'b0;   // MEGA65: mark SD busy in the same cycle (race-free)
									data_transfer_state <= 2'b11;
								end
							end

							2'b11: begin
								// SD Card phase
								data_transfer_state <= 2'b00;
								if (cmd[4]) sector_inc_strobe <= 1'b1; // multiple sector transfer
								else begin
									busy <= 1'b0;
									irq_set <= 1'b1; // emit irq when command done
								end
							end

							default: ;
							endcase

						end
					end
				end
			end

			// ------------------------ TYPE III -------------------------
			if(cmd_type_3 && phys_mode) begin
				// MEGA65 (#90): physical Type-III. Read Address issues a physical
				// READ_ADDRESS op; the controller streams the 6 reply bytes
				// (C,H,R,N,CRC-hi,CRC-lo) through the FIFO and, on completion, WD
				// copies C into the sector register. Like Type-II, the op completes
				// ONLY through the controller done handshake (not-ready is bounded
				// there); no unilateral WD-side completion. Read/Write Track are not
				// physically supported and simply finish (as upstream fakes them).
				if (cmd[7:4] == 4'b1100) begin
					// read address
					if (!phys_rd_pending && !phys_done_latched) begin
						phys_rd_op      <= 3'b001;    // RDOP_READ_ADDRESS
						phys_rd_track   <= track;
						phys_rd_side    <= floppy_side;
						phys_rd_sector  <= sector;
						phys_rd_seq     <= phys_rd_seq + 2'd1;
						phys_rd_req_tgl <= ~phys_rd_req_tgl;
						phys_rd_pending <= 1'b1;
						phys_reading    <= 1'b1;
						phys_rd_start   <= 1'b1;
					end
				end else begin
					// read track / write track: not physically supported -> finish
					busy    <= 1'b0;
					irq_set <= 1'b1;
				end
			end else
			if(cmd_type_3) begin
				if(!fd_present) begin
					// no image selected -> send irq immediately
					RNF <= 1'b1;
					busy <= 1'b0;
					irq_set <= 1'b1; // emit irq when command done
				end else begin
					// read track TODO: fake
					if(cmd[7:4] == 4'b1110) begin
						busy <= 1'b0;
						irq_set <= 1'b1; // emit irq when command done
					end

					// write track TODO: fake
					if(cmd[7:4] == 4'b1111) begin
						busy <= 1'b0;
						irq_set <= 1'b1; // emit irq when command done
					end

					// read address
					if(cmd[7:4] == 4'b1100) begin
						// we are busy until the next setor header passes under the head
						if(fd_ready && fd_sector_hdr)
							data_transfer_start <= 1'b1;

						if(data_transfer_done) begin
							busy <= 1'b0;
							irq_set <= 1'b1; // emit irq when command done
						end
					end
				end
			end
		end

		// ---------------------------------------------------------------------
		// MEGA65 (#90): physical step-ack + read-completion handshakes. Runs on
		// clk8m_en, outside the execute gate, so it advances the seek FSM and
		// finalizes a read even while the WD is otherwise idle-waiting. Toggles
		// are compared against their *_serviced copy (level compare -> no lost
		// edge across the 50 MHz -> clkcpu CDC).
		// ---------------------------------------------------------------------
		if (phys_mode) begin
			// one physical STEP acknowledged -> advance the Type-I seek FSM
			if (phys_step_ack_c != phys_step_ack_serviced) begin
				phys_step_ack_serviced <= phys_step_ack_c;
				if (phys_step_busy) begin
					phys_step_busy <= 1'b0;
					// loop for seek/restore (cmd[6:5]==00), else go verify/finish
					seek_state <= (!cmd[6] && !cmd[5]) ? 2'd0 : 2'd2;
				end
			end

			// controller reported a read op done -> latch its result flags. The
			// two-cycle-delayed copy of the synced toggle is used so the flags
			// (synchronized independently) are guaranteed stable -- see the
			// phys_rd_done_c_d1/_d2 comment at the synchronizers.
			//
			// MEGA65 (#90 delivery v2): the done must carry the CURRENT op's
			// sequence tag (phys_rd_done_seq echoes the seq the controller
			// accepted; cancelled/aborted ops complete with THEIR OWN seq). A
			// done whose tag mismatches -- or that arrives with no op pending --
			// is CONSUMED (the serviced tracker advances) but IGNORED, so a
			// stale result can never pair with a newer command (wrong-sector
			// data under clean status). Each ignore toggles the staledone diag.
			// KNOWN ACCEPTED HOLE (documented, not fixed): a QNICE-domain-only
			// reset mid-op idles the controller without a done toggle, leaving
			// the WD busy; in M2M the QNICE reset never occurs without the
			// accompanying core reset, which clears this engine via floppy_reset.
			if (phys_rd_done_c_d2 != phys_rd_done_serviced) begin
				phys_rd_done_serviced <= phys_rd_done_c_d2;
				if (phys_rd_pending && phys_rd_done_seq_c == phys_rd_seq) begin
					phys_rnf_l <= phys_rd_rnf_c;
					phys_crc_l <= phys_rd_crc_err_c;
					phys_del_l <= phys_rd_deleted_c;
					phys_c_l   <= phys_rd_c_c;
					phys_rd_pending   <= 1'b0;
					phys_done_latched <= 1'b1;
					// A failed capture is not sector data. Close presentation and
					// let the existing residue path discard every quarantined byte.
					if (phys_rd_rnf_c || phys_rd_crc_err_c)
						phys_reading <= 1'b0;
				end else begin
					phys_dbg_staledone_tgl <= ~phys_dbg_staledone_tgl;
				end
			end

			// MEGA65 (#90): finalize -- the DISK-PACED completion of a physical
			// read op. Clean captures are released only after their CRC-valid done;
			// failed captures are drained unseen. Fires on the first clk8m tick
			// where (a) the controller done has been received and tag-matched,
			// (b) the read FIFO is empty (clean bytes presented, failed bytes
			// discarded), and (c) the pace counter has expired again with no
			// byte presenting this tick, i.e. at least one full byte-time after
			// the last presentation. That is the real WD1772 shape: busy outlives
			// the last DRQ by >= 1 byte-time (the image engine's
			// data_transfer_cnt = N+1 produces the same tail). The 1581 ROM
			// transfer loops poll BUSY FIRST and DRQ second ($CD17: AND #$03 /
			// LSR / BCC done), so this tail guarantees the loop cannot exit with
			// the final byte still unread; the ROM then software-CRC-checks the
			// 6-byte Read Address reply ($DA63, CCITT preset $B230 over
			// C,H,R,N,CRC,CRC -> residue 0). Completion depends ONLY on
			// disk-time-shaped events (done + FIFO level + pace) -- never on the
			// drive CPU consuming anything and never on a wall-clock watchdog: a
			// stalled CPU gets LOST DATA (status bit 2), not a wedged WD, and an
			// error result with partial pushes presents its bytes at pace and
			// then completes with the error status.
			// (round 11: a verify completion is a Type-I completion, so it also
			// honors the minimum Type-I busy time -- see PHYS_T1_MIN_TICKS.)
			if (phys_done_latched && phys_byte_empty && phys_pace_cnt == 8'd0
			    && !phys_present_now
			    && (!phys_verify || phys_t1_min_cnt == 14'd0)) begin
				phys_done_latched <= 1'b0;
				phys_reading      <= 1'b0;
				RNF               <= phys_rnf_l;
				phys_dbg_pres_cnt <= phys_pres_cnt;      // diagnostics: bytes presented
				phys_dbg_fin_tgl  <= ~phys_dbg_fin_tgl;  // by the op finalized right here
				if (phys_verify) begin
					// Type-I verify: seek error mirrors RNF; finish the command
					phys_verify <= 1'b0;
					busy        <= 1'b0;
					irq_set     <= 1'b1;
					seek_state  <= 2'd0;
				end else if (cmd[7:5] == 3'b100 && cmd[4] && !phys_rnf_l && !phys_crc_l) begin
					// multiple-sector read: advance sector, re-issue next cycle.
					// Only a CLEAN completion (rnf=0, crc=0) continues the chain;
					// any error terminates the command through the else below.
					sector_inc_strobe <= 1'b1;
					phys_reissue      <= 1'b1;
				end else begin
					if (cmd[7:4] == 4'b1100)  // Read Address: WD writes C into sector reg
						phys_set_sector <= 1'b1;
					busy    <= 1'b0;
					irq_set <= 1'b1;
				end
			end

			// deferred re-issue for a multiple-sector read (sector already
			// incremented). A reissue is a NEW op: it gets a fresh sequence tag,
			// and phys_rd_start re-arms the pace counter and clears LOST DATA
			// and the presented-byte counter.
			// MEGA65 (#90 round 10 hardening): gate on !cmd_rx. A command strobe
			// while busy can only be Force Interrupt (every other write is
			// dropped wholesale in the cpu-register-write block), and its cancel
			// branch above clears phys_reissue only for FUTURE ticks -- without
			// this gate the OLD value would still launch a phantom op (req
			// toggle + seq bump) on the very edge that also toggles the cancel.
			// NOTE: the FI-at-finalize-tick alignment additionally relies on
			// cmd_rx spanning TWO consecutive clk8m ticks for FI-while-busy
			// (the type-4 cmd_rx_i clear requires !busy, which only updates
			// after the first tick): the finalize can re-arm phys_reissue
			// AFTER the first FI execution (last-write-wins), the gate blocks
			// the launch on the intermediate tick, and the SECOND FI execution
			// clears the re-armed flag. Do not narrow cmd_rx to a single tick
			// without re-deriving this collision.
			if (phys_reissue && !cmd_rx) begin
				phys_reissue    <= 1'b0;
				phys_rd_op      <= 3'b000;    // RDOP_READ_SECTOR
				phys_rd_track   <= track;
				phys_rd_side    <= floppy_side;
				phys_rd_sector  <= sector;    // incremented by sector_inc_strobe last cycle
				phys_rd_seq     <= phys_rd_seq + 2'd1;
				phys_rd_req_tgl <= ~phys_rd_req_tgl;
				phys_rd_pending <= 1'b1;
				phys_reading    <= 1'b1;
				phys_rd_start   <= 1'b1;
			end
		end

		// stop motor if there was no command for 10 index pulses.
		// MEGA65 (#90 bring-up): this block also counts down the spin-up sequence
		// (WD status bit 5 for Type-I) and fires the Force-Interrupt-on-index IRQ.
		// In phys mode the image floppy model never pulses fd_index, which left
		// motor_spin_up_done and irq_at_index dead -- use the real (synced) index.
		indexD <= fd_index_eff;
		if(indexD && !fd_index_eff) begin
			irq_at_index <= 1'b0;
			if (irq_at_index) irq_set <= 1'b1;

			// let motor timeout run once fdc is not busy anymore
			if(!busy && motor_spin_up_done) begin
				if(motor_timeout_index != 0)
					motor_timeout_index <= motor_timeout_index - 4'd1;
				else if(motor_on)
					motor_timeout_index <= MOTOR_IDLE_COUNTER;

				if(motor_timeout_index == 1)
					motor_on <= 1'b0;
			end

			if(motor_spin_up_sequence != 0)
				motor_spin_up_sequence <= motor_spin_up_sequence - 4'd1;
		end
		if(busy) motor_timeout_index <= 0;
	end
end

// floppy delivers data at a floppy generated rate (usually 250kbit/s), so the start and stop
// signals need to be passed forth and back from cpu clock domain to floppy data clock domain
// MEGA65 (#90): data_transfer_start/done forward-declared above.

// ==================================== FIFO ==================================

// 0.5/1 kB buffer used to receive a sector as fast as possible from from the io
// controller. The internal transfer afterwards then runs at 250000 Bit/s
// MEGA65 (#90): fifo_cpuptr, s_odd forward-declared above.
reg  [9:0] fifo_cpuptr_adj;
wire [7:0] fifo_q;
reg  [9:0] fifo_sdptr;
reg  [7:0] data_in;
reg        data_in_strobe;

always @(*) begin
	if (SECTOR_SIZE_CODE == 3)
		fifo_sdptr = { s_odd, sd_buff_addr };
	else
		fifo_sdptr = { 1'b0, sd_buff_addr };

	if (SECTOR_SIZE_CODE == 1)
		fifo_cpuptr_adj = { 1'b0, sector[0], fifo_cpuptr[7:0] };
	else
		fifo_cpuptr_adj = fifo_cpuptr[9:0];
end

// MEGA65 (D81 enable): dual-CLOCK FIFO. Port A (SD/io side) is clocked by clk_sys (the
// QNICE/vdrives domain): address_a=fifo_sdptr={1'b0,sd_buff_addr} (no core term for the
// 1581's SECTOR_SIZE_CODE==2), data_a=sd_dout, wren_a=sd_dout_strobe&sd_ack, q_a=sd_din --
// all vdrives-domain. Port B (drive/cpu side) stays on clkcpu. The dual-port RAM performs
// the data-path CDC; the SD FSM (label3) is on clk_sys so the whole SD interface matches
// vdrives. (Was single-clock on clkcpu -> would have meta-stabled the QNICE-domain inputs.)
fdc1772_dpram #(8, 10) fifo
(
	.clock_a(clk_sys),
	.address_a(fifo_sdptr),
	.data_a(sd_dout),
	.wren_a(sd_dout_strobe & sd_ack),
	.q_a(sd_din),

	.clock_b(clkcpu),
	.address_b(fifo_cpuptr_adj),
	.data_b(data_in),
	.wren_b(data_in_strobe),
	.q_b(fifo_q)
);

// ------------------ SD card control ------------------------
//
// MEGA65 (D81 enable, sy2002): the entire SD-request FSM (label3) is re-clocked onto
// clk_sys (the QNICE/vdrives domain), mirroring sy2002's c1541_track rework. As a result
// sd_rd, sd_wr, sd_lba, sd_ack and sd_buff_* all live in the SAME clock domain as
// vdrives.vhd, so vdrives needs no synchronizers and no changes. The clkcpu command FSM
// (label2) hands off to this clk_sys FSM via:
//   - request:    label2's 1-cycle sd_card_read/sd_card_write pulses become stable level
//                 toggles (sd_rd_req_tgl/sd_wr_req_tgl), pulse-synchronized into clk_sys.
//   - completion: sd_done_tgl is toggled here when a transfer finishes and synchronized
//                 back to clkcpu (sd_done_tgl_c), where it releases label2's sd_io_idle gate.
//   - sd_lba:     latched here (clk_sys) from the stable combinational sd_lba_comb.
// A reset clause (clk_sys-synced floppy_reset) clears a stale sd_rd/sd_wr after an unmount
// mid-transfer -- the upstream FSM had no reset term, leaving a zombie sd_rd asserted.
localparam SD_IDLE = 0;
localparam SD_READ = 1;
localparam SD_WRITE = 2;

reg [1:0] sd_state = SD_IDLE;
// MEGA65 (#90): sd_card_write, sd_card_read forward-declared above.

// clkcpu: turn label2's 1-cycle request pulses into stable level toggles, so they can be
// pulse-synchronized into clk_sys (a plain level-sync would race the short pulse).
reg sd_rd_req_tgl = 1'b0;
reg sd_wr_req_tgl = 1'b0;
always @(posedge clkcpu) begin : label_sdreq
	reg sd_card_readD, sd_card_writeD;
	sd_card_readD  <= sd_card_read;
	sd_card_writeD <= sd_card_write;
	if (~sd_card_readD  & sd_card_read)  sd_rd_req_tgl <= ~sd_rd_req_tgl;
	if (~sd_card_writeD & sd_card_write) sd_wr_req_tgl <= ~sd_wr_req_tgl;
end

// request toggles + reset synchronized INTO clk_sys; done toggle synchronized BACK to clkcpu
wire sd_rd_req_s, sd_wr_req_s, floppy_reset_s;
reg  sd_done_tgl = 1'b0;
// MEGA65 (#90): sd_done_tgl_c forward-declared above.
iecdrv_sync sd_rdreq_sync (clk_sys, sd_rd_req_tgl, sd_rd_req_s);
iecdrv_sync sd_wrreq_sync (clk_sys, sd_wr_req_tgl, sd_wr_req_s);
iecdrv_sync sd_frst_sync  (clk_sys, floppy_reset,  floppy_reset_s);
iecdrv_sync sd_done_sync  (clkcpu,  sd_done_tgl,   sd_done_tgl_c);

always @(posedge clk_sys) begin : label3
	reg sd_ackD;
	reg sd_rd_req_sD, sd_wr_req_sD;

	sd_ackD      <= sd_ack;
	sd_rd_req_sD <= sd_rd_req_s;
	sd_wr_req_sD <= sd_wr_req_s;
	if (sd_ack) {sd_rd, sd_wr} <= 0;

	case (sd_state)
	SD_IDLE:
	begin
		s_odd <= 1'b0;
		if (sd_rd_req_s ^ sd_rd_req_sD) begin
			sd_rd[fdn] <= 1;
			sd_lba     <= sd_lba_comb;
			sd_state   <= SD_READ;
		end
		else if (sd_wr_req_s ^ sd_wr_req_sD) begin
			sd_wr[fdn] <= 1;
			sd_lba     <= sd_lba_comb;
			sd_state   <= SD_WRITE;
		end
	end

	SD_READ:
	if (sd_ackD & ~sd_ack) begin
		if (s_odd || SECTOR_SIZE_CODE != 3) begin
			sd_state    <= SD_IDLE;
			sd_done_tgl <= ~sd_done_tgl;
		end else begin
			s_odd      <= 1;
			sd_rd[fdn] <= 1;
			sd_lba     <= sd_lba_comb;   // re-sample for the odd half-sector (size code 3)
		end
	end

	SD_WRITE:
	if (sd_ackD & ~sd_ack) begin
		if (s_odd || SECTOR_SIZE_CODE != 3) begin
			sd_state    <= SD_IDLE;
			sd_done_tgl <= ~sd_done_tgl;
		end else begin
			s_odd      <= 1;
			sd_wr[fdn] <= 1;
			sd_lba     <= sd_lba_comb;
		end
	end

	default: ;
	endcase

	if (!floppy_reset_s) begin
		sd_rd    <= 0;
		sd_wr    <= 0;
		sd_state <= SD_IDLE;
		s_odd    <= 1'b0;
	end
end

// -------------------- CPU data read/write -----------------------
reg data_in_valid;

function [15:0] crc;
	input [15:0] curcrc;
	input  [7:0] val;
	reg    [3:0] i;
	begin
		crc = {curcrc[15:8] ^ val, 8'h00};
		for (i = 0; i < 8; i=i+1'd1) begin
			if(crc[15]) begin
				crc = crc << 1;
				crc = crc ^ 16'h1021;
			end
			else crc = crc << 1;
		end
		crc = {curcrc[7:0] ^ crc[15:8], crc[7:0]};
	end
endfunction

always @(posedge clkcpu) begin : label4
	reg        data_transfer_startD;
	reg [10:0] data_transfer_cnt;
	reg [15:0] crcval;
	reg        crc_en;

	crc_en <= 0;
	if(crc_en) crcval <= crc(crcval, data_out);

	// MEGA65 (#90 bring-up): mirror the WRITTEN value into the readback register.
	// This used to be `data_out <= data_in`, but data_in is assigned from cpu_din
	// in the same clock edge (in the cpu-register-write block), so the readback
	// was one write behind. The real WD1772 has ONE data register: reading it
	// back returns what was just written. The 1581 ROM's power-up self test
	// ($C347: write $FF..$01 to track/sector/data, verify each readback) fails
	// on the stale value, aborts controller init with error $0D and leaves the
	// drive permanently misbehaving (observed on hardware: track register stuck
	// at $FF, no seeks, instant FILE NOT FOUND).
	if (cpu_we && cpu_addr == FDC_REG_DATA) begin
		data_out <= cpu_din;
		data_in_valid <= 1;
	end

	// reset fifo read pointer on reception of a new command or 
	// when multi-sector transfer increments the sector number
	if(cmd_rx || sector_inc_strobe) begin
		data_in_valid <= 0;
		data_transfer_cnt <= 0;
		fifo_cpuptr <= 0;
	end

	drq_set <= 1'b0;

	// MEGA65 (#90): physical read -- CRC-GATED, DISK-PACED byte presentation.
	// Every PHYS_PACE_TICKS clk8m ticks (one DD MFM byte-time, ~32 us) the head
	// of the external read FIFO is popped into the data register and DRQ is
	// raised -- exactly like the real WD1772, which moves a byte from its shift
	// register into the data register at disk pace no matter what the host does.
	// The physical controller must first report a clean complete-field result;
	// until then the FIFO is quarantine and phys_present_now remains false.
	// If the previous byte is still unconsumed (DRQ high) it is OVERWRITTEN and
	// the phys LOST DATA flag is set (status bit 2). Presentation never waits
	// for consumption, so completion (the label2 finalize) is bounded by disk
	// time alone. The pace counter starts expired at op start (phys_rd_start),
	// so the first byte presents as soon as it is available; if the FIFO is
	// empty at an expired tick, the byte presents as soon as it arrives and the
	// pace re-arms from that actual presentation. phys_byte_data/phys_byte_empty
	// are native clkcpu (FIFO read side) so no sync is needed; the pop strobe is
	// one clkcpu wide (clk8m_en is one clkcpu wide). Reuses the same data_out
	// register + drq_set strobe as the image path (inert with phys_mode=0).
	phys_byte_rd_en <= 1'b0;
	if (phys_cmd_clear || phys_rd_start) phys_lost_l <= 1'b0;
	if (phys_rd_start) begin
		phys_pace_cnt <= 8'd0;
		phys_pres_cnt <= 11'd0;
	end else if (clk8m_en) begin
		if (phys_present_now) begin
			data_out        <= phys_byte_data;
			phys_byte_rd_en <= 1'b1;
			drq_set         <= 1'b1;
			phys_pace_cnt   <= PHYS_PACE_TICKS - 8'd1;
			if (phys_pres_cnt != 11'h7FF)
				phys_pres_cnt <= phys_pres_cnt + 11'd1;
			if (drq) begin
				// previous byte never consumed -> real-WD1772 LOST DATA
				phys_lost_l       <= 1'b1;
				phys_dbg_lost_tgl <= ~phys_dbg_lost_tgl;
			end
		end else if (phys_pace_cnt != 8'd0)
			phys_pace_cnt <= phys_pace_cnt - 8'd1;
	end

	// MEGA65 (#90 review): whenever NO read op is delivering data, pop-and-discard
	// any bytes still sitting in the external read FIFO -- residue from an aborted
	// or cancelled operation (Force Interrupt, disk change, core reset while the
	// 50 MHz controller finished a sector into a stalled FIFO). The controller only
	// pushes while an operation is in flight (spanned by phys_reading here), so this
	// can never eat committed clean data. It also discards a completed CRC/RNF error
	// after its done handler drops phys_reading, and guarantees that every new
	// operation starts from an empty FIFO instead of delivering a stale-shifted,
	// CRC-clean-looking stream.
	// Presentation and drain are mutually exclusive on phys_reading, so they can
	// never pop in the same cycle. Each drain EPISODE (first drained byte after a
	// non-draining cycle) toggles phys_dbg_drain_tgl for the diag counters.
	// MEGA65 (#90 round 10 hardening): the gate deliberately does NOT exclude
	// the done-latched window. Read ops are protected by !phys_reading until
	// finalize, so a !phys_done_latched term would only ever have gated the
	// verify/zero-push done window (phys_reading=0 for their whole life) --
	// where no legitimate FIFO content can exist, but where a stray byte would
	// have NO pop path at all (presentation requires phys_reading; finalize
	// requires empty; no watchdog exists, by design) and would wedge busy
	// forever. Draining there is safe and removes that wedge class structurally.
	if (phys_mode && !phys_reading && !phys_byte_empty) begin
		phys_byte_rd_en <= 1'b1;
		if (!phys_draining) phys_dbg_drain_tgl <= ~phys_dbg_drain_tgl;
		phys_draining <= 1'b1;
	end else
		phys_draining <= 1'b0;

	if (clk8m_en) data_transfer_done <= 0;
	data_transfer_startD <= data_transfer_start;
	// received request to read data
	if(~data_transfer_startD & data_transfer_start) begin

		// read_address command has 6 data bytes
		if(cmd[7:4] == 4'b1100) begin
			crcval <= 16'hB230;
			data_transfer_cnt <= 11'd6+11'd1;
		end

		// read/write sector has SECTOR_SIZE data bytes
		if(cmd[7:6] == 2'b10)
			data_transfer_cnt <= SECTOR_SIZE + 1'd1;

		// write sector asserts drq earlier to fill up the data register in time
		if(cmd[7:5] == 3'b101) drq_set <= !data_in_valid;
	end

	// advance fifo pointer when the write sector data consumed
	data_in_strobe <= 1'b0;
	if(cmd[7:5] == 3'b101 && data_in_strobe) fifo_cpuptr <= fifo_cpuptr + 1'd1;

	if(fd_dclk_en) begin
		if(data_transfer_cnt != 0) begin
			if(data_transfer_cnt != 1) begin
				data_lost <= 1'b0;
				if (drq) data_lost <= 1'b1;
				// raise drq, except when the last byte is already taken from the CPU for write
				if (cmd[7:5] != 3'b101 || data_transfer_cnt != 2) drq_set <= 1'b1;

				// read_address
				if(cmd[7:4] == 4'b1100) begin
					case(data_transfer_cnt)
						7: begin data_out <= fd_track; crc_en <= 1; end
						6: begin data_out <= { 7'b0000000, (INVERT_HEAD_RA != 0) ^ floppy_side }; crc_en <= 1; end
						5: begin data_out <= fd_sector; crc_en <= 1; end
						4: begin data_out <= SECTOR_SIZE_CODE[1:0]; crc_en <= 1; end // TODO: sec size 0=128, 1=256, 2=512, 3=1024
						3: data_out <= crcval[15:8];
						2: data_out <= crcval[7:0];
					endcase // case (data_read_cnt)
				end

				// read sector
				if(cmd[7:5] == 3'b100 && fifo_cpuptr != SECTOR_SIZE) begin
					data_out <= fifo_q;
					fifo_cpuptr <= fifo_cpuptr + 1'd1;
				end
				// write sector
				if(cmd[7:5] == 3'b101 && fifo_cpuptr != SECTOR_SIZE) begin
					data_in_strobe <= 1;
					data_in_valid <= 0;
				end

			end

			// count down and stop after last byte
			data_transfer_cnt <= data_transfer_cnt - 11'd1;
			if(data_transfer_cnt == 1)
				data_transfer_done <= 1'b1;
		end
	end
end

// the status byte
// MEGA65 (#90): in phys_mode the mechanical/result bits come from the physical
// controller (synced levels + latched op flags). With phys_mode=0 every bit is
// byte-identical to upstream. WD1772 (MODEL==2) b7 = motor. Write commands force
// the write-protect bit (physical drive is read-only in this milestone).
wire [7:0] status = { (MODEL == 1 || MODEL == 3) ? !floppy_ready : (phys_mode ? phys_motor_on_c : motor_on),
		      phys_mode ? ( ((cmd[7:5] == 3'b101) || (cmd[7:4] == 4'b1111)) ? 1'b1
		                    : (cmd_type_1 ? phys_wprot_c : 1'b0) )
		                : ((cmd[7:5] == 3'b101 || cmd[7:4] == 4'b1111 || cmd_type_1) && fd_writeprot), // wrprot (only for write!)
		      cmd_type_1 ? motor_spin_up_done : ((phys_mode && cmd[7:5] == 3'b100) ? phys_del_l : 1'b0), // data mark / deleted
		      // seek error / record not found. MEGA65 (#90 delivery v2): in phys
		      // mode RNF is SUPPRESSED while the CRC bit is set -- the genuine
		      // 318045-02 DOS job epilogue at $CD3F indexes table $CD5A with
		      // (status>>3)&$0B, and CRC+RNF together hits a $00 hole = job
		      // SUCCESS (the DOS would silently ACCEPT the corrupt sector).
		      // CRC-only maps to job error 5 (DOS 23, retried) as intended.
		      phys_mode ? (RNF & ~phys_crc_l) : RNF,
		      phys_mode ? phys_crc_l : 1'b0,       // crc error (real in phys_mode)
		      // track0 (Type I) / lost data. In phys mode LOST DATA is the real
		      // WD1772 flag: a disk-paced presentation overwrote an unconsumed byte.
		      cmd_type_1 ? (phys_mode ? phys_track0_c : fd_track0)
		                 : (phys_mode ? phys_lost_l   : data_lost),
		      cmd_type_1 ? (phys_mode ? ~phys_index_c : ~fd_index) : drq,       // index mark/drq
		      busy } /* synthesis keep */;

// MEGA65 (#90): track, sector, data_out, step_dir, data_lost, cmd, cmd_rx and the
// FDC_REG_* localparams are forward-declared at the top. cmd_type_* are wires
// forward-declared there and driven here.
assign cmd_type_1 = (cmd[7] == 1'b0);
assign cmd_type_2 = (cmd[7:6] == 2'b10);
assign cmd_type_3 = (cmd[7:5] == 3'b111) || (cmd[7:4] == 4'b1100);
assign cmd_type_4 = (cmd[7:4] == 4'b1101);

// CPU register read
always @(*) begin
	cpu_dout = 8'h00;

	if(cpu_sel && cpu_rw) begin
		case(cpu_addr)
			FDC_REG_CMDSTATUS: cpu_dout = status;
			FDC_REG_TRACK:     cpu_dout = track;
			FDC_REG_SECTOR:    cpu_dout = sector;
			FDC_REG_DATA:      cpu_dout = data_out;
		endcase
	end
end

// cpu register write
// MEGA65 (#90): cmd_rx forward-declared above.
reg cmd_rx_i;

always @(posedge clkcpu) begin
	if(!floppy_reset) begin
		// clear internal registers
		cmd <= 8'h00;
		track <= 8'h00;
		sector <= 8'h00;

		// reset state machines and counters
		cmd_rx_i <= 1'b0;
		cmd_rx <= 1'b0;
	end else begin

		// cmd_rx is delayed to make sure all signals (the cmd!) are stable when
		// cmd_rx is evaluated
		cmd_rx <= cmd_rx_i;

		// command reception is ack'd by fdc going busy
		if((!cmd_type_4 && busy) || (clk8m_en && cmd_type_4 && !busy)) cmd_rx_i <= 1'b0;

		// only react if stb just raised
		if(cpu_we) begin
			if(cpu_addr == FDC_REG_CMDSTATUS) begin       // command register
				// MEGA65 (#90 delivery v2): the real WD1772 IGNORES a command
				// write while busy unless it is Force Interrupt. In phys mode we
				// do the same (upstream image behavior is kept as-is): the write
				// is dropped wholesale -- no cmd reload, no cmd_rx, no register
				// side effects -- and the busycmd diag toggle marks the event.
				if (phys_mode && busy && cpu_din[7:4] != 4'b1101) begin
					phys_dbg_busycmd_tgl <= ~phys_dbg_busycmd_tgl;
				end else begin
				cmd <= cpu_din;
				cmd_rx_i <= 1'b1;
				// ------------- TYPE I commands -------------
				if(cpu_din[7:4] == 4'b0000) begin               // RESTORE
					step_to <= 8'd0;
					track <= 8'hff;
				end

				if(cpu_din[7:4] == 4'b0001) begin               // SEEK
					step_to <= data_in;
				end

				if(cpu_din[7:5] == 3'b001) begin                // STEP
				end

				if(cpu_din[7:5] == 3'b010) begin                // STEP-IN
				end

				if(cpu_din[7:5] == 3'b011) begin                // STEP-OUT
				end

				// ------------- TYPE II commands -------------
				if(cpu_din[7:5] == 3'b100) begin                // read sector
				end

				if(cpu_din[7:5] == 3'b101) begin                // write sector
				end

				// ------------- TYPE III commands ------------
				if(cpu_din[7:4] == 4'b1100) begin               // read address
				end

				if(cpu_din[7:4] == 4'b1110) begin               // read track
				end

				if(cpu_din[7:4] == 4'b1111) begin               // write track
				end

				// ------------- TYPE IV commands -------------
				if(cpu_din[7:4] == 4'b1101) begin               // force intrerupt
				end
				end // MEGA65 (#90 delivery v2): end of the not-ignored branch
			end

			if(cpu_addr == FDC_REG_TRACK)                    // track register
				track <= cpu_din;

			if(cpu_addr == FDC_REG_SECTOR)                   // sector register
				sector <= cpu_din;

			if(cpu_addr == FDC_REG_DATA) begin               // data register
				data_in <= cpu_din;
			end
		end

		if (sector_inc_strobe) sector <= sector + 1'd1;
		if (track_inc_strobe) track <= track + 1'd1;
		if (track_dec_strobe) track <= track - 1'd1;
		if (track_clear_strobe) track <= 8'd0;
		// MEGA65 (#90): Read Address writes the returned ID track byte (C) into the
		// sector register, per the WD1772 datasheet.
		if (phys_set_sector) sector <= phys_c_l;
	end
end

endmodule

// MEGA65 (D81 enable): true DUAL-CLOCK dual-port RAM (was a single `clock`). Port A is
// the SD/io side on clk_sys (QNICE-vdrives domain); port B is the drive/cpu side on clkcpu.
// Standard CDC structure for the WD1772 sector FIFO; Vivado infers a true-dual-port BRAM.
module fdc1772_dpram #(parameter DATAWIDTH=8, ADDRWIDTH=9)
(
	input                   clock_a,
	input   [ADDRWIDTH-1:0] address_a,
	input   [DATAWIDTH-1:0] data_a,
	input                   wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input                   clock_b,
	input   [ADDRWIDTH-1:0] address_b,
	input   [DATAWIDTH-1:0] data_b,
	input                   wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

reg [DATAWIDTH-1:0] ram[0:(1<<ADDRWIDTH)-1];

always @(posedge clock_a) begin
	if(wren_a) begin
		ram[address_a] <= data_a;
		q_a <= data_a;
	end else begin
		q_a <= ram[address_a];
	end
end

always @(posedge clock_b) begin
	if(wren_b) begin
		ram[address_b] <= data_b;
		q_b <= data_b;
	end else begin
		q_b <= ram[address_b];
	end
end

endmodule
