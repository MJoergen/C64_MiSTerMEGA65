//
// tb_fdc1772_physical.sv
//
// Self-checking Icarus Verilog (-g2012) testbench for the MEGA65 physical
// internal 1581 read branch of fdc1772.v (issue #90, delivery v2).
//
// It instantiates fdc1772 with phys_mode=1 and stands in for the VHDL
// physical_1581_controller + physical_1581_rdfifo with:
//   * an empty `floppy` STUB (image-mode mechanics; must be inert here),
//   * a mock backend on a separate 50 MHz-ish clock (clk_be) that answers the
//     flat toggle ABI with the delivery-v2 discipline: it acks Type-I steps,
//     samples the request SEQUENCE TAG at acceptance, latches a request that
//     arrives while it is busy (pending latch, latest edge wins, cleared by
//     cancel), streams Read Sector / Read Address bytes into a real Gray-code
//     async FIFO (an SV port of physical_1581_rdfifo, so the data path
//     genuinely crosses clk_be -> clkcpu) and toggles rd_done with the
//     accepted op's tag on phys_rd_done_seq -- aborted ops complete with
//     THEIR OWN tag,
//   * the real iecdrv_sync (from iecdrv_misc.sv) inside the DUT for the CDC.
//
// Verified delivery-v2 behavior (all with the REAL 1581 ROM's polling idiom,
// $CD17: poll BUSY FIRST, DRQ second, exit the moment busy reads 0, and with
// realistic multi-clkcpu CPU access cycles):
//   (1) register-init readback: the ROM power-up self test pattern ($C343,
//       $FF..$01 into track/sector/data, verify every readback);
//   (2) Read Address reply 6 bytes BYTE-EXACT incl. the ROM's software
//       CCITT-CRC over the 6 bytes ($DA63, preset $B230, residue 0), and a
//       Read Sector 512 bytes BYTE-EXACT;
//   (3) stalled CPU mid-sector (> 2 byte-times): completion still occurs
//       within a disk-time bound, LOST DATA appears in status bit 2, busy
//       drops, and the NEXT op is byte-exact;
//   (4) FIFO residue: stray bytes ahead of an op terminate it in bounded
//       time; idle residue is eaten by the drain (drain-episode diag); the
//       next op is byte-exact;
//   (5) a non-Force-Interrupt command write while busy is IGNORED
//       (phys_dbg_busycmd_tgl toggles; no register side effects);
//   (6) after Force Interrupt mid-op, the cancelled op's late done is
//       ignored via the sequence tag (phys_dbg_staledone_tgl toggles) and a
//       fresh op completes byte-exact;
//   (7) status NEVER shows the CRC and RNF bits together (continuous
//       monitor; plus a forced crc+rnf completion from the backend proves
//       the suppression belt);
//   (8) busy-tail: after the last DRQ of a read, busy stays set for at
//       least ~1 byte-time (polled and measured);
//   (9) round 10 F3: a paced presentation that becomes due while the drive
//       CPU has an OPEN 16-clkcpu read access to the data register is
//       DEFERRED until the access closes -- no corrupted byte, no
//       duplicate, no false LOST DATA (plus a continuous monitor: the data
//       register never changes during any open CPU read of it);
//   (10) round 10 F1: a Force Interrupt whose command strobe lands on the
//       same clk8m tick as a deferred multi-sector reissue produces NO
//       phantom operation (seq tag unchanged, the controller sees no new
//       request) and the WD stays fully operational.
//   (11) round 11 fix A: a ZERO-STEP Type-I command (SEEK or RESTORE whose
//       target track is already current, no step pulses) must hold busy for
//       the WD1772 minimum Type-I busy time (~1.5 ms) so the 1581 DOS command
//       writer's wait-busy-SET poll at $CBFA (~3.5 us cadence) can see it --
//       driven with the exact ROM idiom: busy is observed within 3 polls,
//       stays set 1..3 ms (measured), completes with INTRQ (no hang), and a
//       following Read Address is byte-exact. (Before the fix busy dropped
//       after ~380 ns, invisible to the poll -> DOS error-recovery hang.)
//   (12) CRC quarantine: a complete 512-byte backend capture whose final
//       result is DATA CRC ERROR is drained without presenting a single byte
//       or DRQ to the ROM; a following clean operation remains byte-exact.
//   (13) short-select collision: if the WD data-register select is visible for
//       only one clkcpu, its registered DRQ-clear pulse still defers a byte
//       that becomes due on the following clk8m tick. No swallowed DRQ and no
//       false LOST DATA are permitted.
//
// Run:
//   iverilog -g2012 -o tb.vvp tb_fdc1772_physical.sv fdc1772.v iecdrv_misc.sv
//   vvp tb.vvp        (nonzero exit / $fatal on any mismatch)
//
// C64MEGA65 project, GPLv3.
//

`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// floppy STUB: image-mode mechanics only. In phys_mode it must never drive the
// data path, so a dead stub (all outputs low) is exactly right.
// ---------------------------------------------------------------------------
module floppy #(parameter CLK_EN = 8000) (
	input        clk,
	input        clk8m_en,
	input        select,
	input        motor_on,
	input        step_in,
	input        step_out,
	input [10:0] sector_len,
	input        sector_base,
	input  [4:0] spt,
	input  [9:0] sector_gap_len,
	input        hd,
	input        fm,
	output       dclk_en,
	output [6:0] track,
	output [4:0] sector,
	output       sector_hdr,
	output       sector_data,
	output       ready,
	output reg   index
);
	assign dclk_en = 1'b0;
	assign track = 7'd0;
	assign sector = 5'd0;
	assign sector_hdr = 1'b0;
	assign sector_data = 1'b0;
	assign ready = 1'b0;
	initial index = 1'b0;
endmodule

// ---------------------------------------------------------------------------
// Gray-code async FWFT FIFO (SV port of physical_1581_rdfifo). rd side native
// clkcpu, wr side clk_be. rd_data always shows the head; rd_en pops it.
// ---------------------------------------------------------------------------
module tb_rdfifo #(parameter AW = 10) (
	input            wr_clk, input wr_rst, input wr_en, input [7:0] wr_data, output wr_full,
	input            rd_clk, input rd_rst, input rd_en, output [7:0] rd_data, output rd_empty
);
	function [AW:0] b2g(input [AW:0] b); b2g = (b >> 1) ^ b; endfunction

	reg  [7:0]  mem [0:(1<<AW)-1];
	reg  [AW:0] wbin=0, wgray=0, rbin=0, rgray=0;
	reg  [AW:0] rq1_wg=0, rq2_wg=0, wq1_rg=0, wq2_rg=0;
	reg         full_q=0, empty_q=1;

	wire        wdo   = wr_en & ~full_q;
	wire [AW:0] wbin_n  = wbin + (wdo ? 1'b1 : 1'b0);
	wire [AW:0] wgray_n = b2g(wbin_n);
	wire        rdo   = rd_en & ~empty_q;
	wire [AW:0] rbin_n  = rbin + (rdo ? 1'b1 : 1'b0);
	wire [AW:0] rgray_n = b2g(rbin_n);

	assign wr_full  = full_q;
	assign rd_empty = empty_q;
	assign rd_data  = mem[rbin[AW-1:0]];

	always @(posedge wr_clk) begin
		wq1_rg <= rgray; wq2_rg <= wq1_rg;
		if (wr_rst) begin wbin<=0; wgray<=0; full_q<=0; end
		else begin
			if (wdo) mem[wbin[AW-1:0]] <= wr_data;
			wbin  <= wbin_n;
			wgray <= wgray_n;
			full_q <= (wgray_n == {~wq2_rg[AW], ~wq2_rg[AW-1], wq2_rg[AW-2:0]});
		end
	end

	always @(posedge rd_clk) begin
		rq1_wg <= wgray; rq2_wg <= rq1_wg;
		if (rd_rst) begin rbin<=0; rgray<=0; empty_q<=1; end
		else begin
			rbin  <= rbin_n;
			rgray <= rgray_n;
			empty_q <= (rgray_n == rq2_wg);
		end
	end
endmodule

// ---------------------------------------------------------------------------
module tb_fdc1772_physical;

	localparam [2:0] RDOP_READ_SECTOR  = 3'b000;
	localparam [2:0] RDOP_READ_ADDRESS = 3'b001;
	localparam [2:0] RDOP_VERIFY       = 3'b010;
	localparam [4:0] RES_OK            = 5'b00000;
	localparam [4:0] RES_CANCELLED     = 5'b00111;

	localparam [1:0] REG_CMDSTATUS = 2'd0;
	localparam [1:0] REG_TRACK     = 2'd1;
	localparam [1:0] REG_SECTOR    = 2'd2;
	localparam [1:0] REG_DATA      = 2'd3;

	// one DD MFM byte-time in tb time: PHYS_PACE_TICKS(252) clk8m ticks; the tb
	// clk8m_en fires every 2nd clkcpu (60 ns) -> 252 * 120 ns = 30240 ns
	localparam integer BYTE_NS = 252 * 2 * 60;

	// deterministic sector pattern (both producer + checker use this)
	function [7:0] pat(input [9:0] i); pat = (i * 13 + 5); endfunction

	// CCITT CRC-16 (x^16+x^12+x^5+1, MSB first) -- same algorithm as the
	// fdc1772-internal function and the 1581 ROM's software check at $DA63
	function [15:0] crc16(input [15:0] c, input [7:0] val);
		integer i;
		reg [15:0] x;
		begin
			x = {c[15:8] ^ val, 8'h00};
			for (i = 0; i < 8; i = i + 1)
				x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
			crc16 = {c[7:0] ^ x[15:8], x[7:0]};
		end
	endfunction

	integer errors = 0;
	task expect_eq(input [63:0] got, input [63:0] exp, input [255:0] what);
	begin
		if (got !== exp) begin
			errors = errors + 1;
			$display("FAIL: %0s  got=%0d (0x%0h) expected=%0d (0x%0h)  @%0t",
			         what, got, got, exp, exp, $time);
		end else
			$display("ok  : %0s = 0x%0h", what, got);
	end
	endtask

	// -----------------------------------------------------------------------
	// clocks
	// -----------------------------------------------------------------------
	reg clkcpu = 0;   always #30 clkcpu = ~clkcpu;   // ~16.6 MHz
	reg clk_be = 0;   always #10 clk_be = ~clk_be;   // ~50 MHz backend/controller

	reg clk8m_en = 0;
	always @(posedge clkcpu) clk8m_en <= ~clk8m_en;  // ~8 MHz command-timer enable

	// -----------------------------------------------------------------------
	// CPU bus + control
	// -----------------------------------------------------------------------
	reg        floppy_reset = 0;
	reg  [1:0] cpu_addr = 0;
	reg        cpu_sel = 0;
	reg        cpu_rw = 1;
	reg  [7:0] cpu_din = 0;
	wire [7:0] cpu_dout;
	wire       irq, drq, floppy_step, floppy_ready, fdc_busy;

	// phys ABI: DUT -> backend
	wire       phys_active, phys_cia_motor_on, phys_cia_side;
	wire       phys_step_req_tgl, phys_step_outward;
	wire       phys_rd_req_tgl;
	wire [2:0] phys_rd_op;
	wire [7:0] phys_rd_track, phys_rd_sector;
	wire       phys_rd_side;
	wire       phys_rd_cancel_tgl;
	wire [1:0] phys_rd_seq;
	wire       phys_byte_ovf;
	wire       phys_byte_rd_en;

	// phys ABI: backend -> DUT
	reg        phys_step_ack_tgl = 0;
	reg        phys_rd_done_tgl = 0;
	reg  [1:0] phys_rd_done_seq = 0;
	reg  [4:0] phys_rd_result = RES_OK;
	reg        phys_rd_crc_err = 0, phys_rd_rnf = 0, phys_rd_deleted = 0;
	reg  [7:0] phys_rd_c = 0, phys_rd_h = 0, phys_rd_r = 0, phys_rd_n = 0;
	reg        phys_media_ready = 0, phys_index = 0, phys_track0 = 0;
	reg        phys_wprot = 0, phys_change = 0, phys_motor_on = 0, phys_head_settled = 0;

	// diagnostics
	wire        phys_dbg_lost_tgl, phys_dbg_drain_tgl, phys_dbg_staledone_tgl;
	wire        phys_dbg_busycmd_tgl, phys_dbg_fin_tgl;
	wire [10:0] phys_dbg_pres_cnt;

	// FIFO nets
	wire [7:0] fifo_rd_data;
	wire       fifo_rd_empty, fifo_wr_full;
	reg        fifo_wr_en = 0;
	reg  [7:0] fifo_wr_data = 0;

	// -----------------------------------------------------------------------
	// DUT
	// -----------------------------------------------------------------------
	fdc1772 #(
		.CLK_EN(16'd8000), .FD_NUM(1), .MODEL(2),
		.SECTOR_SIZE_CODE(2'd2), .SECTOR_BASE(1'b1), .EXT_MOTOR(1'b1)
	) dut (
		.clkcpu(clkcpu), .clk_sys(clk_be), .clk8m_en(clk8m_en),
		.floppy_drive(1'b0), .floppy_side(1'b0), .floppy_reset(floppy_reset),
		.floppy_step(floppy_step), .floppy_motor(1'b1),
		.floppy_ready(floppy_ready), .fdc_busy(fdc_busy),
		.irq(irq), .drq(drq),
		.cpu_addr(cpu_addr), .cpu_sel(cpu_sel), .cpu_rw(cpu_rw),
		.cpu_din(cpu_din), .cpu_dout(cpu_dout),
		.img_mounted(1'b0), .img_wp(1'b0), .img_ds(1'b0), .img_size(32'd0),
		.sd_lba(), .sd_rd(), .sd_wr(), .sd_ack(1'b0),
		.sd_buff_addr(9'd0), .sd_dout(8'd0), .sd_din(), .sd_dout_strobe(1'b0),

		.phys_mode(1'b1),
		.phys_active(phys_active), .phys_cia_motor_on(phys_cia_motor_on),
		.phys_cia_side(phys_cia_side),
		.phys_step_req_tgl(phys_step_req_tgl), .phys_step_outward(phys_step_outward),
		.phys_rd_req_tgl(phys_rd_req_tgl), .phys_rd_op(phys_rd_op),
		.phys_rd_track(phys_rd_track), .phys_rd_side(phys_rd_side),
		.phys_rd_sector(phys_rd_sector), .phys_rd_cancel_tgl(phys_rd_cancel_tgl),
		.phys_rd_seq(phys_rd_seq),
		.phys_byte_ovf(phys_byte_ovf),
		.phys_step_ack_tgl(phys_step_ack_tgl), .phys_rd_done_tgl(phys_rd_done_tgl),
		.phys_rd_done_seq(phys_rd_done_seq),
		.phys_rd_result(phys_rd_result), .phys_rd_crc_err(phys_rd_crc_err),
		.phys_rd_rnf(phys_rd_rnf), .phys_rd_deleted(phys_rd_deleted),
		.phys_rd_c(phys_rd_c), .phys_rd_h(phys_rd_h),
		.phys_rd_r(phys_rd_r), .phys_rd_n(phys_rd_n),
		.phys_byte_rd_en(phys_byte_rd_en), .phys_byte_data(fifo_rd_data),
		.phys_byte_empty(fifo_rd_empty),
		.phys_media_ready(phys_media_ready), .phys_index(phys_index),
		.phys_track0(phys_track0), .phys_wprot(phys_wprot),
		.phys_change(phys_change), .phys_motor_on(phys_motor_on),
		.phys_head_settled(phys_head_settled),

		.phys_dbg_lost_tgl(phys_dbg_lost_tgl),
		.phys_dbg_drain_tgl(phys_dbg_drain_tgl),
		.phys_dbg_staledone_tgl(phys_dbg_staledone_tgl),
		.phys_dbg_busycmd_tgl(phys_dbg_busycmd_tgl),
		.phys_dbg_fin_tgl(phys_dbg_fin_tgl),
		.phys_dbg_pres_cnt(phys_dbg_pres_cnt)
	);

	// Match production exactly: one complete 512-byte sector is the quarantine
	// capacity. The 512th write is accepted and makes the FIFO full.
	tb_rdfifo #(.AW(9)) rdfifo (
		.wr_clk(clk_be), .wr_rst(~floppy_reset), .wr_en(fifo_wr_en),
		.wr_data(fifo_wr_data), .wr_full(fifo_wr_full),
		.rd_clk(clkcpu), .rd_rst(~floppy_reset), .rd_en(phys_byte_rd_en),
		.rd_data(fifo_rd_data), .rd_empty(fifo_rd_empty)
	);

	// -----------------------------------------------------------------------
	// diagnostic toggle edge counters (all dbg toggles are clkcpu-domain)
	// -----------------------------------------------------------------------
	integer cnt_lost = 0, cnt_drain = 0, cnt_stale = 0, cnt_busycmd = 0, cnt_fin = 0;
	reg d_lost = 0, d_drain = 0, d_stale = 0, d_busycmd = 0, d_fin = 0;
	always @(posedge clkcpu) begin
		d_lost    <= phys_dbg_lost_tgl;
		d_drain   <= phys_dbg_drain_tgl;
		d_stale   <= phys_dbg_staledone_tgl;
		d_busycmd <= phys_dbg_busycmd_tgl;
		d_fin     <= phys_dbg_fin_tgl;
		if (phys_dbg_lost_tgl    ^ d_lost)    cnt_lost    = cnt_lost + 1;
		if (phys_dbg_drain_tgl   ^ d_drain)   cnt_drain   = cnt_drain + 1;
		if (phys_dbg_staledone_tgl ^ d_stale) cnt_stale   = cnt_stale + 1;
		if (phys_dbg_busycmd_tgl ^ d_busycmd) cnt_busycmd = cnt_busycmd + 1;
		if (phys_dbg_fin_tgl     ^ d_fin)     cnt_fin     = cnt_fin + 1;
	end

	// -----------------------------------------------------------------------
	// (7) continuous invariant: the status register must NEVER show the CRC
	// (bit 3) and RNF (bit 4) bits together -- the genuine 1581 DOS maps that
	// combination to job SUCCESS via the $CD5A table hole.
	// -----------------------------------------------------------------------
	integer crc_rnf_viol = 0;
	always @(posedge clkcpu)
		if (dut.status[3] === 1'b1 && dut.status[4] === 1'b1) begin
			crc_rnf_viol = crc_rnf_viol + 1;
			if (crc_rnf_viol < 5)
				$display("FAIL: status shows CRC and RNF together @%0t", $time);
		end

	// -----------------------------------------------------------------------
	// (9) continuous invariant (round 10 F3): the WD data register must NEVER
	// change while the drive CPU has an OPEN READ access to it -- the T65
	// latches cpu_dout at the CLOSING enable tick, so a mid-access change
	// corrupts the byte read. (In this bench data_out only changes through the
	// phys presentation or a CPU WRITE access, so any hit is a real violation.)
	// -----------------------------------------------------------------------
	wire mon_rd_open = cpu_sel && cpu_rw && (cpu_addr == REG_DATA);
	integer data_mid_rd_viol = 0;
	reg  [7:0] mon_data_q = 0;
	reg        mon_open_q = 0;
	always @(posedge clkcpu) begin
		if (mon_open_q && (dut.data_out !== mon_data_q)) begin
			data_mid_rd_viol = data_mid_rd_viol + 1;
			if (data_mid_rd_viol < 5)
				$display("FAIL: data register changed during an open CPU read @%0t", $time);
		end
		mon_open_q <= mon_rd_open;
		mon_data_q <= dut.data_out;
	end

	// (12) Before a tag-matched, clean controller completion, physical bytes
	// are speculative. They must remain quarantined in the async FIFO instead
	// of being exposed as WD data-register presentations.
	integer precommit_present_viol = 0;
	always @(posedge clkcpu)
		if (dut.phys_present_now && !dut.phys_done_latched)
			precommit_present_viol = precommit_present_viol + 1;

	// -----------------------------------------------------------------------
	// (10) round 10 F1 instrumentation: count clk8m ticks on which a deferred
	// multi-sector reissue is pending in the SAME tick a Force Interrupt
	// command strobe is active (the exact collision the reissue gate must
	// suppress -- proves the test alignment really happened), and count the
	// request edges the mock controller actually sees.
	// -----------------------------------------------------------------------
	integer fi_reissue_coll = 0;
	always @(posedge clkcpu)
		if (clk8m_en && dut.phys_reissue && dut.cmd_rx && dut.cmd_type_4)
			fi_reissue_coll = fi_reissue_coll + 1;

	// -----------------------------------------------------------------------
	// MOCK CONTROLLER BACKEND (clk_be). Delivery-v2 discipline: samples the
	// request tag at acceptance, latches a request that arrives while busy
	// (cleared by cancel), completes every op -- including cancelled ones --
	// with a done toggle carrying the ACCEPTED op's tag on phys_rd_done_seq.
	// Models a track-0 sensor via a head cylinder so Restore terminates.
	// -----------------------------------------------------------------------
	wire be_stepreq_s, be_rdreq_s, be_cancel_s;
	iecdrv_sync be_step_sync (clk_be, phys_step_req_tgl,  be_stepreq_s);
	iecdrv_sync be_rd_sync   (clk_be, phys_rd_req_tgl,    be_rdreq_s);
	iecdrv_sync be_can_sync  (clk_be, phys_rd_cancel_tgl, be_cancel_s);

	reg        be_stepreq_sd = 0, be_rdreq_sd = 0, be_cancel_sd = 0;
	integer    head = 3;             // head cylinder; Restore steps to 0
	reg [3:0]  step_dly = 0;
	reg        step_busy_be = 0;
	reg [3:0]  be_state = 0;
	reg        be_pend = 0;          // request-pending latch (req while busy)
	reg [1:0]  be_seq = 0;           // request tag sampled at acceptance
	reg [15:0] be_dly = 0;
	reg [15:0] be_start_delay = 0;   // stimulus knob: clk_be cycles before serving
	reg [15:0] be_cancel_delay = 0;  // stimulus knob: clk_be cycles before the cancel done
	reg        stray_req = 0;        // stimulus: ONE stray byte enters the FIFO at op start
	reg [3:0]  poison_req = 0;       // stimulus: push N stray bytes while idle (drain food)
	reg        err_req = 0;          // stimulus: complete next op with crc=1 AND rnf=1, 0 bytes
	reg        crc_payload_req = 0;  // next sector pushes 512 bytes, then reports CRC error
	reg        be_crc_bad = 0;       // crc_payload_req latched with the accepted operation
	reg        manual_req = 0;       // stimulus: next op parks in a manual state (tb-paced
	                                 //           pushes via push_req; done on manual_done_req)
	reg        manual_done_req = 0;  // stimulus: complete the manual op now (clean)
	reg        push_req = 0;         // stimulus: push ONE byte (push_val) into the FIFO
	reg  [7:0] push_val = 0;
	reg [9:0]  be_idx = 0;
	reg [2:0]  be_op = 0;
	reg [15:0] be_racrc = 0;
	reg [7:0]  ra_c = 8'h03, ra_h = 8'h00, ra_r = 8'h01, ra_n = 8'h02;

	// backend done helper values are driven quasi-static BEFORE the done
	// toggle flips (registered one clk_be earlier than the toggle would be
	// enough; same-edge is fine through the 2-cycle-delayed consume in the DUT)
	task be_done(input [1:0] seq, input rnf, input crcerr);
	begin
		phys_rd_result   <= (rnf && crcerr) ? RES_CANCELLED : RES_OK;
		phys_rd_rnf      <= rnf;
		phys_rd_crc_err  <= crcerr;
		phys_rd_deleted  <= 1'b0;
		phys_rd_c <= ra_c; phys_rd_h <= ra_h; phys_rd_r <= ra_r; phys_rd_n <= ra_n;
		phys_rd_done_seq <= seq;
		phys_rd_done_tgl <= ~phys_rd_done_tgl;
	end
	endtask

	always @(posedge clk_be) begin
		fifo_wr_en   <= 1'b0;
		be_stepreq_sd <= be_stepreq_s;
		be_rdreq_sd   <= be_rdreq_s;
		be_cancel_sd  <= be_cancel_s;

		// live state
		phys_track0       <= (head == 0);
		phys_media_ready  <= 1'b1;
		phys_motor_on     <= 1'b1;
		phys_head_settled <= 1'b1;
		phys_wprot        <= 1'b0;
		phys_change       <= 1'b0;

		// ---- Type-I step: move the head, then ack after a few cycles ----
		if (be_stepreq_s ^ be_stepreq_sd) begin
			if (phys_step_outward) head <= (head == 0) ? 0 : head - 1;
			else                   head <= head + 1;
			step_busy_be <= 1'b1;
			step_dly     <= 4'd6;
		end
		if (step_busy_be) begin
			if (step_dly != 0) step_dly <= step_dly - 4'd1;
			else begin
				phys_step_ack_tgl <= ~phys_step_ack_tgl;
				step_busy_be <= 1'b0;
			end
		end

		// ---- request/cancel edges (any state): pending latch, cancel abort ----
		if (be_rdreq_s ^ be_rdreq_sd)
			be_pend <= 1'b1;                     // served by the idle state
		if (be_cancel_s ^ be_cancel_sd) begin
			be_pend <= 1'b0;                     // cancel clears the pending latch
			if (be_state != 4'd0) begin
				// abort the op in flight; complete it later with ITS OWN tag
				be_state <= 4'd7;
				be_dly   <= be_cancel_delay;
			end
		end

		// ---- idle-poison stimulus: stray bytes while no op runs ----
		if (poison_req != 0 && be_state == 4'd0 && !be_pend) begin
			fifo_wr_en   <= 1'b1;
			fifo_wr_data <= 8'hDD;
			poison_req   <= poison_req - 4'd1;
		end

		// ---- tb-paced single push (test 9: presentation/CPU-read collision) ----
		if (push_req) begin
			fifo_wr_en   <= 1'b1;
			fifo_wr_data <= push_val;
			push_req     <= 1'b0;
		end

		// ---- read operation FSM ----
		case (be_state)
		4'd0: begin // idle: serve a (possibly latched) request
			if (be_pend) begin
				be_pend <= 1'b0;
				be_seq  <= phys_rd_seq;   // sample tag + params (quasi-static)
				be_op   <= phys_rd_op;
				be_idx  <= 10'd0;
				be_dly  <= be_start_delay;
				be_crc_bad <= crc_payload_req;
				crc_payload_req <= 1'b0;
				if (stray_req) begin
					// residue regression: one spurious byte enters the FIFO
					// after the op started (drain closed), BEFORE the real
					// reply bytes -- models residue slipping into an operation
					fifo_wr_en   <= 1'b1;
					fifo_wr_data <= 8'hEE;
					stray_req    <= 1'b0;
				end
				be_state <= 4'd6;
			end
		end
		4'd6: begin // optional start delay (models seek/rotational latency)
			if (be_dly != 0) be_dly <= be_dly - 16'd1;
			else if (err_req) begin
				// forced ERROR completion with crc=1 AND rnf=1 and no bytes:
				// exactly what a pre-round-10 controller emitted; the DUT
				// status must show CRC only (RNF suppressed)
				err_req <= 1'b0;
				be_done(be_seq, 1'b1, 1'b1);
				be_state <= 4'd0;
			end
			else if (manual_req) be_state <= 4'd8;
			else if (be_op == RDOP_READ_SECTOR)  be_state <= 4'd1;
			else if (be_op == RDOP_READ_ADDRESS) begin
				be_racrc <= 16'hB230;    // CRC state after A1 A1 A1 FE
				be_state <= 4'd4;
			end else begin
				// verify: report OK immediately (matching track -> no seek error)
				be_done(be_seq, 1'b0, 1'b0);
				be_state <= 4'd0;
			end
		end
		4'd1: begin // push 512 data bytes, one per clk_be cycle
			fifo_wr_en   <= 1'b1;
			fifo_wr_data <= pat(be_idx);
			be_idx       <= be_idx + 10'd1;
			if (be_idx == 10'd511) be_state <= 4'd2;
		end
		4'd2: begin // report the captured sector result (with the accepted tag)
			be_done(be_seq, 1'b0, be_crc_bad);
			be_crc_bad <= 1'b0;
			be_state <= 4'd0;
		end
		4'd4: begin // Read Address: push C,H,R,N + true CCITT CRC (hi,lo)
			fifo_wr_en <= 1'b1;
			case (be_idx)
				10'd0: begin fifo_wr_data <= ra_c; be_racrc <= crc16(be_racrc, ra_c); end
				10'd1: begin fifo_wr_data <= ra_h; be_racrc <= crc16(be_racrc, ra_h); end
				10'd2: begin fifo_wr_data <= ra_r; be_racrc <= crc16(be_racrc, ra_r); end
				10'd3: begin fifo_wr_data <= ra_n; be_racrc <= crc16(be_racrc, ra_n); end
				10'd4: fifo_wr_data <= be_racrc[15:8];
				default: fifo_wr_data <= be_racrc[7:0];
			endcase
			be_idx <= be_idx + 10'd1;
			if (be_idx == 10'd5) begin
				// Match physical_1581_controller exactly: its sixth FIFO
				// write and tagged done toggle occur in the same 50 MHz edge.
				// The done synchronizer must not outrun the FIFO write pointer.
				be_done(be_seq, 1'b0, 1'b0);
				be_state <= 4'd0;
			end
		end
		4'd7: begin // cancelled: complete the ABORTED op with ITS OWN tag
			if (be_dly != 0) be_dly <= be_dly - 16'd1;
			else begin
				be_done(be_seq, 1'b1, 1'b0);
				be_state <= 4'd0;
			end
		end
		4'd8: begin // manual op (test 9): tb pushes bytes; done on request
			if (manual_done_req) begin
				manual_done_req <= 1'b0;
				manual_req      <= 1'b0;
				be_done(be_seq, 1'b0, 1'b0);
				be_state        <= 4'd0;
			end
		end
		default: be_state <= 4'd0;
		endcase
	end

	// (10) count the request edges the mock controller actually sees (a
	// phantom reissue would add an extra one)
	integer be_req_edges = 0;
	reg     be_rdreq_pd = 0;
	always @(posedge clk_be) begin
		be_rdreq_pd <= be_rdreq_s;
		if (be_rdreq_s ^ be_rdreq_pd) be_req_edges = be_req_edges + 1;
	end

	// -----------------------------------------------------------------------
	// CPU bus tasks
	// -----------------------------------------------------------------------
	task cpu_write(input [1:0] a, input [7:0] d);
	begin
		@(posedge clkcpu); #1;
		cpu_sel = 1'b1; cpu_rw = 1'b0; cpu_addr = a; cpu_din = d;
		@(posedge clkcpu); #1;
		cpu_sel = 1'b0; cpu_rw = 1'b1;
		@(posedge clkcpu); #1;
	end
	endtask

	// model the REAL drive CPU bus cycle: the 1581's T65 keeps the address (and
	// thus cpu_sel) asserted for a full 2 MHz cycle (~16 clkcpu) and latches the
	// read data at the CLOSING enable tick -- i.e. at the END of the access.
	task cpu_read(input [1:0] a, output [7:0] d);
		integer k;
	begin
		@(posedge clkcpu); #1;
		cpu_sel = 1'b1; cpu_rw = 1'b1; cpu_addr = a;
		for (k = 0; k < 15; k = k + 1) @(posedge clkcpu);
		#1;
		d = cpu_dout;          // the T65 captures din at the closing enable tick
		@(posedge clkcpu); #1;
		cpu_sel = 1'b0;
		@(posedge clkcpu); #1;
	end
	endtask

	// wait until the fdc_busy output reaches `val`. Uses the dedicated busy output
	// (not a CMDSTATUS read) so the pending INTRQ is NOT cleared and can be checked.
	task wait_busy(input val, input [255:0] what);
		integer n;
	begin
		n = 0;
		while (fdc_busy !== val) begin
			@(posedge clkcpu);
			n = n + 1;
			if (n > 4000000) begin
				errors = errors + 1;
				$display("FAIL: timeout waiting busy=%0d (%0s) @%0t", val, what, $time);
				disable wait_busy;
			end
		end
	end
	endtask

	// -----------------------------------------------------------------------
	// ROM-faithful transfer loop ($CD17 idiom): poll BUSY FIRST, DRQ second,
	// take a byte only while busy=1 && drq=1, exit the moment busy reads 0.
	// Collects into rxbuf/rxcnt (caller resets rxcnt); records the time of the
	// last data read and of the first busy=0 status sample (busy-tail measure).
	// -----------------------------------------------------------------------
	reg [7:0] rxbuf [0:1023];
	integer   rxcnt;
	time      last_data_time, busy_clear_time;

	task rom_drain(input integer maxpoll, input [255:0] what);
		integer g;
		reg [7:0] st, d;
	begin
		g = 0;
		forever begin
			cpu_read(REG_CMDSTATUS, st);       // ROM: LDA $6000 / AND #$03 / LSR
			if (st[0] !== 1'b1) begin
				busy_clear_time = $time;
				disable rom_drain;             // busy gone -> ROM exits its loop
			end
			if (st[1] === 1'b1) begin
				cpu_read(REG_DATA, d);
				last_data_time = $time;
				if (rxcnt < 1024) rxbuf[rxcnt] = d;
				rxcnt = rxcnt + 1;
			end
			g = g + 1;
			if (g > maxpoll) begin
				errors = errors + 1;
				$display("FAIL: rom_drain guard exceeded (%0s, got %0d bytes) @%0t",
				         what, rxcnt, $time);
				disable rom_drain;
			end
		end
	end
	endtask

	// take exactly n bytes with the same idiom, then return with the op running
	task rom_take(input integer n, input integer maxpoll, input [255:0] what);
		integer g;
		reg [7:0] st, d;
	begin
		g = 0;
		while (rxcnt < n) begin
			cpu_read(REG_CMDSTATUS, st);
			if (st[0] !== 1'b1) begin
				errors = errors + 1;
				$display("FAIL: busy dropped early during rom_take (%0s) @%0t", what, $time);
				disable rom_take;
			end
			if (st[1] === 1'b1) begin
				cpu_read(REG_DATA, d);
				last_data_time = $time;
				if (rxcnt < 1024) rxbuf[rxcnt] = d;
				rxcnt = rxcnt + 1;
			end
			g = g + 1;
			if (g > maxpoll) begin
				errors = errors + 1;
				$display("FAIL: rom_take guard exceeded (%0s) @%0t", what, $time);
				disable rom_take;
			end
		end
	end
	endtask

	// -----------------------------------------------------------------------
	// (11) ROM-faithful "wait busy SET" poll ($CBFA idiom): after writing a
	// Type-I command, spin reading the STATUS register (busy = bit 0) at a
	// ~3.5 us cadence, looping while busy reads CLEAR -- exactly what the 1581
	// DOS command writer does at $CBFA (BIT $6000 / BEQ). Records the poll count
	// to first busy=SET (polls11) and the time it was first seen (t_set11) into
	// module globals. The REAL ROM loop is UNBOUNDED (a never-set busy hangs the
	// drive forever -- the round-11 bug); this bench bounds it at maxpoll so the
	// failure surfaces as a loud "busy never observed" instead of a wall-clock
	// hang. cpu_read is ~1.08 us, padded with #2400 -> ~3.5 us per poll.
	integer polls11;
	time    t_set11, t_clear11;
	task rom_wait_busy_set(input integer maxpoll, input [255:0] what);
		integer g;
		reg [7:0] st;
	begin
		g = 0; t_set11 = 0; polls11 = 0;
		forever begin
			cpu_read(REG_CMDSTATUS, st);      // ROM: BIT $6000 (reads WD status)
			g = g + 1;
			if (st[0] === 1'b1) begin
				t_set11 = $time; polls11 = g;
				disable rom_wait_busy_set;    // busy seen SET -> ROM leaves the poll
			end
			#2400;                            // pad to ~3.5 us per poll iteration
			if (g >= maxpoll) begin
				errors = errors + 1;
				polls11 = g;
				$display("FAIL: busy never observed SET after %0d polls (%0s) @%0t",
				         g, what, $time);
				disable rom_wait_busy_set;
			end
		end
	end
	endtask

	// -----------------------------------------------------------------------
	// stimulus
	// -----------------------------------------------------------------------
	reg  [7:0]  rbyte, status;
	reg  [15:0] swcrc;
	integer     i, k, exp_fin, base_stale, base_busycmd, base_drain;
	integer     base_lost, base_req;
	reg  [1:0]  seq_before;
	time        t_start;

	initial begin
		// global watchdog (the paced RS ops alone are ~15.5 ms each; tests
		// 2a, 3 and 10 each stream a full 512-byte sector at pace)
		#200_000_000;
		$display("FAIL: global timeout");
		$fatal(1, "global timeout");
	end

	initial begin
		// CRC function self-check against the known MFM constants:
		// CRC(A1,A1,A1) from FFFF = CDB4; +FE = B230 (the $DA63 preset)
		if (crc16(crc16(crc16(16'hFFFF, 8'hA1), 8'hA1), 8'hA1) !== 16'hCDB4 ||
		    crc16(16'hCDB4, 8'hFE) !== 16'hB230) begin
			$display("FAIL: tb crc16 self-check");
			$fatal(1, "tb crc16 self-check");
		end

		// reset
		floppy_reset = 1'b0;
		// Emulate FPGA power-up-to-0 for the image-path timers that the WD model's
		// reset clause does not clear (they are 0 at config time on hardware, but
		// iverilog starts regs at X, which would wedge the !delaying / !step_busy
		// execution gate). Pure sim init -- no effect on the RTL.
		dut.delay_cnt = 0;
		dut.step_rate_cnt = 0;
		dut.step_pulse_cnt = 0;
		dut.motor_spin_up_sequence = 0;
		dut.motor_timeout_index = 0;
		dut.data_lost = 0;
		repeat (8) @(posedge clkcpu);
		floppy_reset = 1'b1;
		repeat (8) @(posedge clkcpu);
		exp_fin = 0;

		// ------------- (1) REGISTER-INIT READBACK (ROM $C343 pattern) -------------
		// The DOS power-up self test writes $FF..$01 to the WD track/sector/data
		// registers and verifies EVERY readback; one mismatch aborts controller
		// init with error $0D (track register stuck at $FF -- seen on hardware).
		$display("--- (1) REGISTER INIT READBACK ($C343 pattern) ---");
		for (i = 255; i >= 1; i = i - 1) begin
			cpu_write(REG_TRACK,  i[7:0]);
			cpu_write(REG_SECTOR, i[7:0]);
			cpu_write(REG_DATA,   i[7:0]);
			cpu_read(REG_TRACK, rbyte);
			if (rbyte !== i[7:0]) begin errors = errors + 1; $display("FAIL: track readback wrote %02h got %02h", i[7:0], rbyte); end
			cpu_read(REG_SECTOR, rbyte);
			if (rbyte !== i[7:0]) begin errors = errors + 1; $display("FAIL: sector readback wrote %02h got %02h", i[7:0], rbyte); end
			cpu_read(REG_DATA, rbyte);
			if (rbyte !== i[7:0]) begin errors = errors + 1; $display("FAIL: data readback wrote %02h got %02h", i[7:0], rbyte); end
		end
		$display("ok  : 255 x3 register readbacks byte-exact");

		// -------------------- RESTORE (0x00) --------------------
		$display("--- RESTORE (head starts at %0d) ---", head);
		cpu_write(REG_CMDSTATUS, 8'h00);
		wait_busy(1'b1, "restore accepted");
		wait_busy(1'b0, "restore done");
		cpu_read(REG_TRACK, rbyte);
		expect_eq(rbyte, 8'h00, "track register after restore");
		expect_eq(head,   0,     "physical head at track0 after restore");
		expect_eq(irq,    1'b1,  "INTRQ asserted after restore");

		// -------------------- SEEK to track 3 (0x10) --------------------
		$display("--- SEEK to 3 ---");
		cpu_write(REG_DATA, 8'h03);     // seek target -> data register
		cpu_write(REG_CMDSTATUS, 8'h10);
		wait_busy(1'b1, "seek accepted");
		wait_busy(1'b0, "seek done");
		cpu_read(REG_TRACK, rbyte);
		expect_eq(rbyte, 8'h03, "track register after seek");
		expect_eq(head,   3,     "physical head at cyl 3 after seek");

		// -------------------- SEEK with VERIFY (0x14, same track) --------------------
		$display("--- SEEK 3 with V flag (verify op) ---");
		cpu_write(REG_DATA, 8'h03);
		cpu_write(REG_CMDSTATUS, 8'h14);
		wait_busy(1'b1, "seek+V accepted");
		wait_busy(1'b0, "seek+V done");
		exp_fin = exp_fin + 1;
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[4], 1'b0, "no seek error after verify");

		// ------------- (2a) READ SECTOR, ROM loop, byte-exact + (8) busy tail -------------
		$display("--- (2a) READ SECTOR 1 (ROM busy-first loop, 512 bytes, ~15.5 ms) ---");
		cpu_write(REG_TRACK,  8'h03);
		cpu_write(REG_SECTOR, 8'h01);
		cpu_write(REG_CMDSTATUS, 8'h80);
		wait_busy(1'b1, "read-sector accepted");
		rxcnt = 0;
		rom_drain(40000, "read sector");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 512, "ROM loop took exactly 512 sector bytes");
		for (i = 0; i < 512; i = i + 1)
			if (rxbuf[i] !== pat(i[9:0])) begin
				errors = errors + 1;
				if (errors < 12)
					$display("FAIL: data[%0d] got=0x%0h exp=0x%0h", i, rxbuf[i], pat(i[9:0]));
			end
		$display("ok  : 512 sector bytes byte-exact");
		// (8) busy-tail: busy must outlive the last DRQ/data byte by >= ~1 byte-time
		if (busy_clear_time - last_data_time < (BYTE_NS * 8) / 10 ||
		    busy_clear_time - last_data_time > BYTE_NS * 4) begin
			errors = errors + 1;
			$display("FAIL: busy tail after last byte = %0d ns (expected ~%0d ns)",
			         busy_clear_time - last_data_time, BYTE_NS);
		end else
			$display("ok  : busy tail after last data byte = %0d ns (~1 byte-time)",
			         busy_clear_time - last_data_time);
		expect_eq(phys_dbg_pres_cnt, 11'd512, "pres_cnt diagnostic after read sector");
		// final status: motor(b7)=1, all error bits clear => 0x80
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status, 8'h80, "status word after clean read");

		// ------------- (2b) READ ADDRESS, ROM loop, byte-exact + software CRC -------------
		$display("--- (2b) READ ADDRESS (ROM loop + $DA63 software CRC) ---");
		cpu_write(REG_SECTOR, 8'hEE);          // WD must overwrite this with C
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "read-address accepted");
		rxcnt = 0;
		rom_drain(5000, "read address");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "ROM loop took exactly 6 RA bytes");
		expect_eq(rxbuf[0], 8'h03, "read-address byte0 (C)");
		expect_eq(rxbuf[1], 8'h00, "read-address byte1 (H)");
		expect_eq(rxbuf[2], 8'h01, "read-address byte2 (R)");
		expect_eq(rxbuf[3], 8'h02, "read-address byte3 (N)");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1)
			swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "ROM software CRC residue over the 6-byte reply");
		expect_eq(phys_dbg_pres_cnt, 11'd6, "pres_cnt diagnostic after read address");
		cpu_read(REG_SECTOR, rbyte);
		expect_eq(rbyte, 8'h03, "sector register = found C after read address");
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[0], 1'b0, "ra status busy clear");
		expect_eq(status[4], 1'b0, "ra status RNF clear");
		expect_eq(status[1], 1'b0, "ra status DRQ clear");

		// ------------- (4a) IDLE RESIDUE -> between-ops drain -------------
		$display("--- (4a) IDLE FIFO POISON -> drain episode ---");
		base_drain = cnt_drain;
		poison_req = 4'd5;                 // 5 stray bytes while no op runs
		repeat (200) @(posedge clkcpu);    // give the drain time to eat them
		expect_eq(fifo_rd_empty, 1'b1, "FIFO empty again after idle poison");
		if (cnt_drain <= base_drain) begin
			errors = errors + 1;
			$display("FAIL: no drain episode counted for idle poison");
		end else
			$display("ok  : drain episode counted (cnt_drain=%0d)", cnt_drain);
		cpu_write(REG_CMDSTATUS, 8'hC0);   // and the next op is byte-exact
		wait_busy(1'b1, "post-poison RA accepted");
		rxcnt = 0;
		rom_drain(5000, "post-poison RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "post-poison RA took 6 bytes");
		expect_eq(rxbuf[0], 8'h03, "post-poison RA byte0 (C)");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "post-poison RA software CRC residue");

		// ------------- (4b) RESIDUE INSIDE AN OP -> bounded, next op clean -------------
		// One spurious byte enters the FIFO just after a Read Address starts
		// (drain closed). Delivery v2 presents ALL FIFO bytes at pace -- the
		// reply is shifted (the real ROM rejects it via its software CRC and
		// retries) -- and the op MUST terminate one byte-time after the last
		// presentation. Nothing is left over, so the follow-up is byte-exact.
		$display("--- (4b) STRAY BYTE INSIDE AN OP ---");
		stray_req = 1'b1;
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "stray-RA accepted");
		rxcnt = 0;
		rom_drain(5000, "stray RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 7, "stray-RA terminated after 7 bytes (no wedge)");
		expect_eq(rxbuf[0], 8'hEE, "stray-RA byte0 is the stray (shifted reply)");
		expect_eq(rxbuf[1], 8'h03, "stray-RA byte1 is the real C");
		expect_eq(phys_dbg_pres_cnt, 11'd7, "pres_cnt diagnostic counts the stray");
		swcrc = 16'hB230;                  // the ROM's check MUST fail on the shift
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		if (swcrc === 16'h0000) begin
			errors = errors + 1;
			$display("FAIL: shifted RA reply unexpectedly passed the software CRC");
		end
		repeat (100) @(posedge clkcpu);
		cpu_write(REG_CMDSTATUS, 8'hC0);   // follow-up RA must be clean again
		wait_busy(1'b1, "post-stray RA accepted");
		rxcnt = 0;
		rom_drain(5000, "post-stray RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "post-stray RA got all 6 bytes");
		expect_eq(rxbuf[0], 8'h03, "post-stray byte0 (C) clean again");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "post-stray RA software CRC residue");

		// ------------- (5) NON-FI COMMAND WRITE WHILE BUSY IS IGNORED -------------
		$display("--- (5) COMMAND WRITE WHILE BUSY (must be ignored) ---");
		base_busycmd   = cnt_busycmd;
		be_start_delay = 16'd5000;         // ~100 us of extra busy time
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "busy-cmd RA accepted");
		repeat (300) @(posedge clkcpu);    // well inside the op
		cpu_write(REG_CMDSTATUS, 8'h00);   // RESTORE while busy -> must be IGNORED
		repeat (10) @(posedge clkcpu);
		expect_eq(fdc_busy, 1'b1, "still busy after ignored command write");
		expect_eq(cnt_busycmd, base_busycmd + 1, "busycmd diag toggled once");
		cpu_read(REG_TRACK, rbyte);
		expect_eq(rbyte, 8'h03, "track register untouched by ignored RESTORE");
		rxcnt = 0;
		rom_drain(8000, "busy-cmd RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "op undisturbed by ignored command write");
		expect_eq(rxbuf[0], 8'h03, "byte0 (C) still byte-exact");
		expect_eq(head, 3, "head never moved (RESTORE really ignored)");
		be_start_delay = 16'd0;

		// ------------- (3) STALLED CPU MID-SECTOR -> LOST DATA, bounded completion -------------
		$display("--- (3) STALLED CPU MID-SECTOR (~15.5 ms paced op) ---");
		t_start = $time;
		cpu_write(REG_TRACK,  8'h03);
		cpu_write(REG_SECTOR, 8'h01);
		cpu_write(REG_CMDSTATUS, 8'h80);
		wait_busy(1'b1, "stall-RS accepted");
		rxcnt = 0;
		rom_take(100, 10000, "first 100 bytes before the stall");
		#(5 * BYTE_NS);                    // CPU stops consuming for 5 byte-times
		rom_drain(40000, "stall RS resume");
		exp_fin = exp_fin + 1;
		if (rxcnt >= 512) begin
			errors = errors + 1;
			$display("FAIL: stalled read lost no bytes (rxcnt=%0d)", rxcnt);
		end else
			$display("ok  : stalled read consumed %0d/512 (bytes lost as expected)", rxcnt);
		if (cnt_lost == 0) begin
			errors = errors + 1;
			$display("FAIL: no LOST DATA diag event during the stall");
		end else
			$display("ok  : LOST DATA diag events = %0d", cnt_lost);
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[0], 1'b0, "stall: busy dropped");
		expect_eq(status[2], 1'b1, "stall: LOST DATA visible in status bit 2");
		expect_eq(status[4], 1'b0, "stall: no RNF");
		expect_eq(status[3], 1'b0, "stall: no CRC error");
		if ($time - t_start > 520 * BYTE_NS) begin
			errors = errors + 1;
			$display("FAIL: stalled op exceeded 520 byte-times (%0d ns)", $time - t_start);
		end else
			$display("ok  : stalled op completed in %0d ns (< 520 byte-times)", $time - t_start);
		// the NEXT op must be byte-exact (also proves LOST DATA clears at accept)
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "post-stall RA accepted");
		rxcnt = 0;
		rom_drain(5000, "post-stall RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "post-stall RA got all 6 bytes");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "post-stall RA software CRC residue");
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[2], 1'b0, "LOST DATA cleared by the next command");

		// ------------- (6) FORCE INTERRUPT -> stale done ignored via seq tag -------------
		$display("--- (6) FORCE INTERRUPT MID-OP + STALE DONE ---");
		base_stale      = cnt_stale;
		be_start_delay  = 16'd3000;        // op idles ~60 us before streaming
		be_cancel_delay = 16'd2000;        // cancelled op completes ~40 us later
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "FI-victim RA accepted");
		repeat (200) @(posedge clkcpu);    // mid-op (backend still in start delay)
		cpu_write(REG_CMDSTATUS, 8'hD0);   // Force Interrupt (no INTRQ flavor)
		wait_busy(1'b0, "busy drops on Force Interrupt");
		be_start_delay = 16'd0;
		// fresh op BEFORE the cancelled op's late done arrives: its request is
		// latched by the backend (pending), its seq differs from the stale done
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "fresh RA accepted");
		rxcnt = 0;
		rom_drain(8000, "fresh RA after FI");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "fresh RA after FI got all 6 bytes");
		expect_eq(rxbuf[0], 8'h03, "fresh RA byte0 (C) byte-exact");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "fresh RA software CRC residue");
		expect_eq(cnt_stale, base_stale + 1, "stale done ignored exactly once (seq tag)");
		be_cancel_delay = 16'd0;

		// ------------- (7b) FORCED crc+rnf COMPLETION -> RNF suppressed -------------
		// A pre-round-10 controller emitted crc=1 AND rnf=1 (RES_DATA_CRC_ERROR
		// family); the fdc-side belt must report CRC only, because the genuine
		// DOS maps CRC+RNF to job SUCCESS via the $CD5A table hole.
		$display("--- (7b) FORCED CRC+RNF COMPLETION (suppression belt) ---");
		err_req = 1'b1;
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "err RA accepted");
		rxcnt = 0;
		rom_drain(5000, "err RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 0, "err completion streamed no bytes");
		expect_eq(phys_dbg_pres_cnt, 11'd0, "pres_cnt diagnostic is 0 for the err op");
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[3], 1'b1, "err completion: CRC error set");
		expect_eq(status[4], 1'b0, "err completion: RNF SUPPRESSED by CRC");

		// ------------- (12) CRC-BAD CAPTURE IS NEVER EXPOSED TO THE ROM -------------
		// The backend fills the production-sized 512-byte FIFO, then reports a
		// data CRC error. Those bytes are not a committed sector. The WD must
		// discard them without asserting DRQ; otherwise splice garbage reaches
		// the genuine ROM before the CRC result exists.
		$display("--- (12) CRC-BAD 512-BYTE CAPTURE QUARANTINE ---");
		base_drain = cnt_drain;
		base_lost  = cnt_lost;
		crc_payload_req = 1'b1;
		cpu_write(REG_TRACK,  8'h03);
		cpu_write(REG_SECTOR, 8'h01);
		cpu_write(REG_CMDSTATUS, 8'h80);
		wait_busy(1'b1, "crc-bad RS accepted");
		rxcnt = 0;
		rom_drain(10000, "crc-bad RS quarantine");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 0, "CRC-bad sector exposed zero bytes to ROM");
		expect_eq(phys_dbg_pres_cnt, 11'd0, "CRC-bad sector presentation count");
		expect_eq(fifo_rd_empty, 1'b1, "CRC-bad sector FIFO drained");
		expect_eq(cnt_lost, base_lost, "CRC-bad discard generated no LOST DATA");
		if (cnt_drain <= base_drain) begin
			errors = errors + 1;
			$display("FAIL: CRC-bad payload was not discarded by a drain episode");
		end else
			$display("ok  : CRC-bad payload discarded by the idle/error drain");
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[3], 1'b1, "CRC-bad sector status CRC set");
		expect_eq(status[4], 1'b0, "CRC-bad sector status RNF clear");
		// Recovery proof: quarantine/drain must not disturb the next operation.
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "post-CRC RA accepted");
		rxcnt = 0;
		rom_drain(5000, "post-CRC RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "post-CRC RA got all 6 bytes");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "post-CRC RA software CRC residue");

		// ------------- (9) PRESENTATION DUE DURING AN OPEN DATA-REGISTER READ -------------
		// Round 10 F3: byte A is presented and left unconsumed until the next
		// presentation is about to become due. The CPU
		// then opens a 16-clkcpu read access on the data register (consuming A).
		// Byte B is already quarantined and its presentation becomes due INSIDE
		// the open access. It must be DEFERRED until the access closes: the CPU
		// latches A uncorrupted at its closing tick, B presents right after the
		// access (fresh DRQ, no duplicate of A), and because the deferred
		// presentation samples a stable (already cleared) drq, no false LOST
		// DATA is flagged.
		$display("--- (9) PACED PRESENTATION vs OPEN CPU DATA-REGISTER READ ---");
		base_lost  = cnt_lost;
		manual_req = 1'b1;
		cpu_write(REG_TRACK,  8'h03);
		cpu_write(REG_SECTOR, 8'h01);
		cpu_write(REG_CMDSTATUS, 8'h80);   // read sector; backend parks in manual state
		wait_busy(1'b1, "manual RS accepted");
		wait (be_state == 4'd8);           // backend accepted the op
		push_val = 8'h5A; push_req = 1'b1;
		wait (push_req == 1'b0);
		push_val = 8'hA5; push_req = 1'b1;
		wait (push_req == 1'b0);
		repeat (20) @(posedge clkcpu);
		expect_eq(drq, 1'b0, "(9) bytes remain quarantined before clean done");
		manual_done_req = 1'b1;            // clean result releases the two bytes
		wait (drq === 1'b1);
		wait (dut.phys_pace_cnt <= 8'd4);  // next presentation becomes due during the access
		// open the 16-clkcpu data-register read access (same shape as cpu_read)
		@(posedge clkcpu); #1;
		cpu_sel = 1'b1; cpu_rw = 1'b1; cpu_addr = REG_DATA;
		for (k = 0; k < 15; k = k + 1) @(posedge clkcpu);
		#1;
		rbyte = cpu_dout;                  // the T65 latch at the closing tick
		// prove B's presentation was DUE during the access ... and was deferred
		expect_eq(dut.phys_pace_cnt, 8'd0, "(9) pace expired during the access");
		expect_eq(fifo_rd_empty, 1'b0, "(9) byte B FIFO-visible before the close");
		expect_eq(drq, 1'b0, "(9) presentation DEFERRED while the read is open");
		@(posedge clkcpu); #1;
		cpu_sel = 1'b0;
		@(posedge clkcpu); #1;
		expect_eq(rbyte, 8'h5A, "(9) CPU latched byte A uncorrupted at the closing tick");
		wait (drq === 1'b1);               // B presents right after the access closes
		cpu_read(REG_DATA, rbyte);
		expect_eq(rbyte, 8'hA5, "(9) byte B presented after the access (no duplicate)");
		expect_eq(cnt_lost, base_lost, "(9) no false LOST DATA from the collision");
		rxcnt = 0;
		rom_drain(8000, "manual RS completion");
		exp_fin = exp_fin + 1;
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[2], 1'b0, "(9) status LOST DATA clear");
		expect_eq(phys_dbg_pres_cnt, 11'd2, "(9) exactly 2 bytes presented");

		// ------------- (13) ONE-CYCLE SELECT / REGISTERED DRQ-CLEAR COLLISION -------------
		// A short select can disappear before cpu_rw_data (the registered WD data
		// read strobe) clears DRQ. Arrange byte D to become due on that clear tick.
		// Guarding only the combinational open-select level pops D, clears its new
		// DRQ in the same edge, and falsely samples C's old DRQ as LOST.
		$display("--- (13) SHORT DATA-SELECT vs REGISTERED DRQ CLEAR ---");
		base_lost  = cnt_lost;
		manual_req = 1'b1;
		cpu_write(REG_CMDSTATUS, 8'h80);
		wait_busy(1'b1, "short-select manual RS accepted");
		wait (be_state == 4'd8);
		push_val = 8'h3C; push_req = 1'b1;
		wait (push_req == 1'b0);
		push_val = 8'hC3; push_req = 1'b1;
		wait (push_req == 1'b0);
		manual_done_req = 1'b1;
		wait (drq === 1'b1);               // C is presented; D remains in FIFO
		wait (dut.phys_pace_cnt == 8'd0);  // next byte is due on the next clk8m tick
		// Select is visible to the DUT for exactly one clkcpu edge.
		cpu_sel = 1'b1; cpu_rw = 1'b1; cpu_addr = REG_DATA;
		@(posedge clkcpu); #1;
		rbyte = cpu_dout;
		cpu_sel = 1'b0;
		@(posedge clkcpu); #1;             // registered clear and due presentation collide here
		expect_eq(rbyte, 8'h3C, "(13) short-select CPU latched byte C");
		expect_eq(drq, 1'b0, "(13) old DRQ cleared on the collision tick");
		expect_eq(fifo_rd_empty, 1'b0, "(13) byte D deferred across registered clear");
		expect_eq(cnt_lost, base_lost, "(13) registered clear caused no false LOST DATA");
		if (fifo_rd_empty !== 1'b1) begin
			wait (drq === 1'b1);
			cpu_read(REG_DATA, rbyte);
			expect_eq(rbyte, 8'hC3, "(13) byte D has a fresh DRQ after deferral");
		end
		rxcnt = 0;
		rom_drain(8000, "short-select manual RS completion");
		exp_fin = exp_fin + 1;
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[2], 1'b0, "(13) status LOST DATA clear");
		expect_eq(phys_dbg_pres_cnt, 11'd2, "(13) exactly 2 bytes presented");

		// ------------- (10) FORCE INTERRUPT vs DEFERRED MULTI-SECTOR REISSUE -------------
		// Round 10 F1: a multi-sector read (m=1) is issued and NOT consumed; the
		// op paces through all 512 presentations. Its finalize tick T sets
		// phys_reissue, and the deferred reissue would execute at tick T+1. The
		// Force Interrupt below is timed so its write edge lands exactly on T:
		// the finalize still reads cmd=$90 (old value) and arms the reissue,
		// while cmd_rx first reads 1 at tick T+1 -- together with the armed
		// phys_reissue. Without the !cmd_rx gate this launches a phantom op
		// (req toggle + seq bump) on the same edge as the cancel toggle.
		$display("--- (10) FI COLLIDES WITH THE DEFERRED MULTI-SECTOR REISSUE TICK ---");
		base_stale = cnt_stale;
		base_req   = be_req_edges;
		cpu_write(REG_TRACK,  8'h03);
		cpu_write(REG_SECTOR, 8'h01);
		cpu_write(REG_CMDSTATUS, 8'h90);   // read sector, multiple-sector flag
		wait_busy(1'b1, "multi-sector RS accepted");
		wait (dut.phys_pres_cnt == 11'd512);  // the 512th presentation tick P
		expect_eq(be_req_edges, base_req + 1, "(10) sector-1 request reached the controller");
		seq_before = phys_rd_seq;             // tag of the sector-1 op
		// finalize tick T = P + 252 clk8m ticks = t_P + BYTE_NS. Aim the FI
		// write edge exactly at T (cpu_write consumes one posedge + one cycle):
		// call it 90 ns before T so its write-processing edge IS T.
		#(BYTE_NS - 90);
		cpu_write(REG_CMDSTATUS, 8'hD0);   // Force Interrupt (no INTRQ flavor)
		repeat (30) @(posedge clkcpu);
		if (fi_reissue_coll == 0) begin
			errors = errors + 1;
			$display("FAIL: (10) alignment missed -- no FI/reissue collision tick observed");
		end else
			$display("ok  : (10) FI/reissue collision tick observed (%0d)", fi_reissue_coll);
		expect_eq(fdc_busy, 1'b0, "(10) busy clear after FI");
		expect_eq(phys_rd_seq, seq_before, "(10) NO phantom reissue: seq tag unchanged");
		expect_eq(dut.phys_rd_pending, 1'b0, "(10) NO phantom reissue: no op pending");
		repeat (400) @(posedge clkcpu);    // give any phantom request time to cross
		expect_eq(be_req_edges, base_req + 1, "(10) controller saw only the ONE real request");
		expect_eq(cnt_stale, base_stale, "(10) no stale done (sector-1 done was consumed)");
		// the WD must remain fully operational: follow-up RA byte-exact
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "post-collision RA accepted");
		rxcnt = 0;
		rom_drain(8000, "post-collision RA");
		exp_fin = exp_fin + 2;             // sector-1 finalize + this RA
		expect_eq(rxcnt, 6, "(10) post-collision RA got all 6 bytes");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "(10) post-collision RA software CRC residue");

		// ------------- (11) ZERO-STEP TYPE-I BUSY VISIBILITY (round 11 fix A) -------------
		// A zero-step SEEK / RESTORE (the target track is ALREADY current) needs
		// no step pulses and, before round 11, dropped busy after ~380 ns -- far
		// too short for the 1581 DOS command writer's wait-busy-SET poll at $CBFA
		// (BIT $6000 / BEQ, one read every ~3.5 us). That sub-microsecond busy
		// pulse was INVISIBLE to the ROM, so the DOS error-recovery job's
		// re-positioning zero-step seek at $CB0F hung forever (motor frozen on,
		// LED frozen off, observed on hardware 2026-07-14). The real WD1772 keeps
		// busy on the order of a millisecond even for zero steps; fix A restores
		// that (PHYS_T1_MIN_TICKS ~= 1.5 ms). This test drives the exact ROM
		// idiom and asserts: (a) busy is SEEN within the first 3 polls, (b) it
		// stays set for 1..3 ms (measured), (c) the command completes with INTRQ
		// (no hang), and (d) a following Read Address is byte-exact (engine sane).
		$display("--- (11) ZERO-STEP TYPE-I BUSY VISIBILITY (round 11 fix A) ---");

		// ---- 11a: zero-step SEEK -- data register == track register, no step ----
		cpu_write(REG_TRACK, 8'h03);          // current track = N
		cpu_write(REG_DATA,  8'h03);          // seek target  = N -> zero step
		cpu_write(REG_CMDSTATUS, 8'h18);      // SEEK, h=1 (no spinup), V=0
		rom_wait_busy_set(200, "(11a) SEEK busy-set poll");
		expect_eq((polls11 <= 3), 1'b1, "(11a) busy=1 seen within first 3 ROM polls");
		wait_busy(1'b0, "(11a) zero-step SEEK completes (no hang)");
		t_clear11 = $time;
		repeat (2) @(posedge clkcpu);         // INTRQ latches one tick after busy drops
		expect_eq(irq, 1'b1, "(11a) INTRQ asserted after zero-step SEEK");
		$display("ok  : (11a) zero-step SEEK busy held %0d ns (%0d poll(s) to first SET)",
		         t_clear11 - t_set11, polls11);
		if (t_clear11 - t_set11 < 1_000_000 || t_clear11 - t_set11 > 3_000_000) begin
			errors = errors + 1;
			$display("FAIL: (11a) zero-step SEEK busy %0d ns outside [1ms,3ms]",
			         t_clear11 - t_set11);
		end else
			$display("ok  : (11a) zero-step SEEK busy duration within [1ms,3ms]");
		cpu_read(REG_TRACK, rbyte);
		expect_eq(rbyte, 8'h03, "(11a) track register unchanged by zero-step SEEK");
		expect_eq(head, 3, "(11a) head never moved (truly zero-step)");
		// (d) engine health: a Read Address right after must be byte-exact
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "(11a) post-SEEK RA accepted");
		rxcnt = 0;
		rom_drain(5000, "(11a) post-SEEK RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "(11a) post-SEEK RA got all 6 bytes");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "(11a) post-SEEK RA CRC residue (engine healthy)");

		// ---- 11b: zero-step RESTORE at track0 -- mock exposes track0 via head ----
		// A first RESTORE steps the head from 3 to 0 (multi-step, not measured).
		// The SECOND RESTORE is zero-step because phys_track0_c is already
		// asserted, so it is held busy SOLELY by the round-11 minimum Type-I
		// timer -- the pure test of fix A on the RESTORE path.
		$display("--- (11b) zero-step RESTORE at track0 ---");
		cpu_write(REG_CMDSTATUS, 8'h08);      // RESTORE h=1: steps head 3 -> 0
		wait_busy(1'b1, "(11b) priming RESTORE accepted");
		wait_busy(1'b0, "(11b) priming RESTORE done");
		expect_eq(head, 0, "(11b) head at track0 after priming RESTORE");
		cpu_write(REG_CMDSTATUS, 8'h08);      // RESTORE h=1, head already 0 -> zero step
		rom_wait_busy_set(200, "(11b) RESTORE busy-set poll");
		expect_eq((polls11 <= 3), 1'b1, "(11b) busy=1 seen within first 3 ROM polls");
		wait_busy(1'b0, "(11b) zero-step RESTORE completes (no hang)");
		t_clear11 = $time;
		repeat (2) @(posedge clkcpu);         // INTRQ latches one tick after busy drops
		expect_eq(irq, 1'b1, "(11b) INTRQ asserted after zero-step RESTORE");
		$display("ok  : (11b) zero-step RESTORE busy held %0d ns (%0d poll(s) to first SET)",
		         t_clear11 - t_set11, polls11);
		if (t_clear11 - t_set11 < 1_000_000 || t_clear11 - t_set11 > 3_000_000) begin
			errors = errors + 1;
			$display("FAIL: (11b) zero-step RESTORE busy %0d ns outside [1ms,3ms]",
			         t_clear11 - t_set11);
		end else
			$display("ok  : (11b) zero-step RESTORE busy duration within [1ms,3ms]");
		expect_eq(head, 0, "(11b) head still track0 after zero-step RESTORE");
		// (d) engine health again
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "(11b) post-RESTORE RA accepted");
		rxcnt = 0;
		rom_drain(5000, "(11b) post-RESTORE RA");
		exp_fin = exp_fin + 1;
		expect_eq(rxcnt, 6, "(11b) post-RESTORE RA got all 6 bytes");
		swcrc = 16'hB230;
		for (i = 0; i < 6 && i < rxcnt; i = i + 1) swcrc = crc16(swcrc, rxbuf[i]);
		expect_eq(swcrc, 16'h0000, "(11b) post-RESTORE RA CRC residue (engine healthy)");

		// -------------------- verdict --------------------
		repeat (10) @(posedge clkcpu);
		expect_eq(cnt_fin, exp_fin, "finalize diag toggled once per completed op");
		expect_eq(crc_rnf_viol, 0, "(7) status never showed CRC and RNF together");
		expect_eq(data_mid_rd_viol, 0, "(9) data register never changed during an open CPU read");
		expect_eq(precommit_present_viol, 0, "(12) no byte presented before clean completion");
		if (errors == 0) begin
			$display("==== PASS: all CRC-gated physical-mode delivery checks passed ====");
			$finish;
		end else begin
			$display("==== FAIL: %0d error(s) ====", errors);
			$fatal(1, "physical-mode testbench failed");
		end
	end

endmodule
