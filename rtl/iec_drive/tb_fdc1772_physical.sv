//
// tb_fdc1772_physical.sv
//
// Self-checking Icarus Verilog (-g2012) testbench for the MEGA65 physical
// internal 1581 read branch of fdc1772.v (issue #90).
//
// It instantiates fdc1772 with phys_mode=1 and stands in for the VHDL
// physical_1581_controller + physical_1581_rdfifo with:
//   * an empty `floppy` STUB (image-mode mechanics; must be inert here),
//   * a mock backend on a separate 50 MHz-ish clock (clk_be) that answers the
//     flat toggle ABI: it acks Type-I steps, and for a Read Sector it streams a
//     known 512-byte pattern into a real Gray-code async FIFO (a SystemVerilog
//     port of physical_1581_rdfifo, so the data path genuinely crosses
//     clk_be -> clkcpu) and then toggles rd_done with RES_OK,
//   * the real iecdrv_sync (from iecdrv_misc.sv) inside the DUT for the CDC.
//
// Scenario: Restore -> Seek(3) -> Read Sector(1). Checks: busy set/clear,
// DRQ pulses, 512 data-register bytes == the pattern, status bits, INTRQ.
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

	localparam [1:0] REG_CMDSTATUS = 2'd0;
	localparam [1:0] REG_TRACK     = 2'd1;
	localparam [1:0] REG_SECTOR    = 2'd2;
	localparam [1:0] REG_DATA      = 2'd3;

	// deterministic sector pattern (both producer + checker use this)
	function [7:0] pat(input [9:0] i); pat = (i * 13 + 5); endfunction

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
	wire       phys_byte_ovf;
	wire       phys_byte_rd_en;

	// phys ABI: backend -> DUT
	reg        phys_step_ack_tgl = 0;
	reg        phys_rd_done_tgl = 0;
	reg  [4:0] phys_rd_result = RES_OK;
	reg        phys_rd_crc_err = 0, phys_rd_rnf = 0, phys_rd_deleted = 0;
	reg  [7:0] phys_rd_c = 0, phys_rd_h = 0, phys_rd_r = 0, phys_rd_n = 0;
	reg        phys_media_ready = 0, phys_index = 0, phys_track0 = 0;
	reg        phys_wprot = 0, phys_change = 0, phys_motor_on = 0, phys_head_settled = 0;

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
		.phys_byte_ovf(phys_byte_ovf),
		.phys_step_ack_tgl(phys_step_ack_tgl), .phys_rd_done_tgl(phys_rd_done_tgl),
		.phys_rd_result(phys_rd_result), .phys_rd_crc_err(phys_rd_crc_err),
		.phys_rd_rnf(phys_rd_rnf), .phys_rd_deleted(phys_rd_deleted),
		.phys_rd_c(phys_rd_c), .phys_rd_h(phys_rd_h),
		.phys_rd_r(phys_rd_r), .phys_rd_n(phys_rd_n),
		.phys_byte_rd_en(phys_byte_rd_en), .phys_byte_data(fifo_rd_data),
		.phys_byte_empty(fifo_rd_empty),
		.phys_media_ready(phys_media_ready), .phys_index(phys_index),
		.phys_track0(phys_track0), .phys_wprot(phys_wprot),
		.phys_change(phys_change), .phys_motor_on(phys_motor_on),
		.phys_head_settled(phys_head_settled)
	);

	tb_rdfifo #(.AW(10)) rdfifo (
		.wr_clk(clk_be), .wr_rst(~floppy_reset), .wr_en(fifo_wr_en),
		.wr_data(fifo_wr_data), .wr_full(fifo_wr_full),
		.rd_clk(clkcpu), .rd_rst(~floppy_reset), .rd_en(phys_byte_rd_en),
		.rd_data(fifo_rd_data), .rd_empty(fifo_rd_empty)
	);

	// -----------------------------------------------------------------------
	// MOCK CONTROLLER BACKEND (clk_be). Syncs the DUT request toggles in, acks
	// steps, and streams a sector on Read Sector. Models a track-0 sensor via a
	// head cylinder so Restore terminates realistically.
	// -----------------------------------------------------------------------
	wire be_stepreq_s, be_rdreq_s, be_cancel_s;
	iecdrv_sync be_step_sync (clk_be, phys_step_req_tgl,  be_stepreq_s);
	iecdrv_sync be_rd_sync   (clk_be, phys_rd_req_tgl,    be_rdreq_s);
	iecdrv_sync be_can_sync  (clk_be, phys_rd_cancel_tgl, be_cancel_s);

	reg        be_stepreq_sd = 0, be_rdreq_sd = 0, be_cancel_sd = 0;
	integer    head = 3;             // head cylinder; Restore steps to 0
	reg [3:0]  step_dly = 0;
	reg        step_busy_be = 0;
	reg [2:0]  be_state = 0;
	reg [9:0]  be_idx = 0;
	reg [2:0]  be_op = 0;

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

		// ---- read operation ----
		case (be_state)
		0: begin
			if (be_rdreq_s ^ be_rdreq_sd) begin
				be_op  <= phys_rd_op;
				be_idx <= 10'd0;
				if (phys_rd_op == RDOP_READ_SECTOR)       be_state <= 3'd1; // stream 512
				else if (phys_rd_op == RDOP_READ_ADDRESS) be_state <= 3'd4; // stream 6
				else begin
					// verify: report OK immediately (matching track -> no seek error)
					phys_rd_result  <= RES_OK; phys_rd_rnf <= 1'b0;
					phys_rd_crc_err <= 1'b0;   phys_rd_deleted <= 1'b0;
					phys_rd_done_tgl <= ~phys_rd_done_tgl;
				end
			end
		end
		1: begin // push 512 data bytes, one per clk_be cycle
			fifo_wr_en   <= 1'b1;
			fifo_wr_data <= pat(be_idx);
			be_idx       <= be_idx + 10'd1;
			if (be_idx == 10'd511) be_state <= 3'd2;
		end
		2: begin // report done OK
			phys_rd_result  <= RES_OK; phys_rd_rnf <= 1'b0;
			phys_rd_crc_err <= 1'b0;   phys_rd_deleted <= 1'b0;
			phys_rd_c <= 8'h03; phys_rd_h <= 8'h00; phys_rd_r <= 8'h01; phys_rd_n <= 8'h02;
			phys_rd_done_tgl <= ~phys_rd_done_tgl;
			be_state <= 3'd0;
		end
		4: begin // Read Address: push C,H,R,N,CRC-hi,CRC-lo
			fifo_wr_en <= 1'b1;
			case (be_idx)
				0: fifo_wr_data <= 8'h03;
				1: fifo_wr_data <= 8'h00;
				2: fifo_wr_data <= 8'h01;
				3: fifo_wr_data <= 8'h02;
				4: fifo_wr_data <= 8'hAA;
				default: fifo_wr_data <= 8'h55;
			endcase
			be_idx <= be_idx + 10'd1;
			if (be_idx == 10'd5) be_state <= 3'd5;
		end
		5: begin
			phys_rd_result <= RES_OK; phys_rd_rnf <= 1'b0;
			phys_rd_crc_err <= 1'b0;  phys_rd_deleted <= 1'b0;
			phys_rd_c <= 8'h03; phys_rd_h <= 8'h00; phys_rd_r <= 8'h01; phys_rd_n <= 8'h02;
			phys_rd_done_tgl <= ~phys_rd_done_tgl;
			be_state <= 3'd0;
		end
		default: be_state <= 3'd0;
		endcase
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

	// MEGA65 (#90 review): model the REAL drive CPU bus cycle. The 1581's T65 keeps
	// the address (and thus cpu_sel) asserted for a full 2 MHz cycle (~16 clkcpu)
	// and latches the read data at the CLOSING enable tick -- i.e. at the END of
	// the access, not one clkcpu after cpu_sel rises. Sampling early masked a real
	// bug where the physical read path replaced data_out mid-access with the next
	// buffered FIFO byte. Hold cpu_sel for 16 clkcpu and sample on the last tick.
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
			if (n > 2000000) begin
				errors = errors + 1;
				$display("FAIL: timeout waiting busy=%0d (%0s) @%0t", val, what, $time);
				disable wait_busy;
			end
		end
	end
	endtask

	// -----------------------------------------------------------------------
	// stimulus
	// -----------------------------------------------------------------------
	reg [7:0] rbyte, status;
	integer   got, guard;

	initial begin
		// global watchdog
		#4_000_000;
		$display("FAIL: global timeout");
		$fatal(1, "global timeout");
	end

	initial begin
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

		// -------------------- 1) RESTORE (0x00) --------------------
		$display("--- RESTORE (head starts at %0d) ---", head);
		cpu_write(REG_CMDSTATUS, 8'h00);
		wait_busy(1'b1, "restore accepted");
		wait_busy(1'b0, "restore done");
		cpu_read(REG_TRACK, rbyte);
		expect_eq(rbyte, 8'h00, "track register after restore");
		expect_eq(head,   0,     "physical head at track0 after restore");
		expect_eq(irq,    1'b1,  "INTRQ asserted after restore");

		// -------------------- 2) SEEK to track 3 (0x10) --------------------
		$display("--- SEEK to 3 ---");
		cpu_write(REG_DATA, 8'h03);     // seek target -> data register
		cpu_write(REG_CMDSTATUS, 8'h10);
		wait_busy(1'b1, "seek accepted");
		wait_busy(1'b0, "seek done");
		cpu_read(REG_TRACK, rbyte);
		expect_eq(rbyte, 8'h03, "track register after seek");
		expect_eq(head,   3,     "physical head at cyl 3 after seek");

		// -------------------- 3) READ SECTOR 1 (0x80) --------------------
		$display("--- READ SECTOR 1 ---");
		cpu_write(REG_TRACK,  8'h03);
		cpu_write(REG_SECTOR, 8'h01);
		cpu_write(REG_CMDSTATUS, 8'h80);
		wait_busy(1'b1, "read-sector accepted");

		// drain 512 DRQ-paced bytes and check them against the pattern
		got   = 0;
		guard = 0;
		while (got < 512) begin
			@(posedge clkcpu);
			guard = guard + 1;
			if (guard > 4_000_000) begin
				errors = errors + 1;
				$display("FAIL: timeout draining sector (got %0d/512)", got);
				got = 512;
			end
			else if (drq === 1'b1) begin
				cpu_read(REG_DATA, rbyte);
				if (rbyte !== pat(got[9:0])) begin
					errors = errors + 1;
					if (errors < 12)
						$display("FAIL: data[%0d] got=0x%0h exp=0x%0h", got, rbyte, pat(got[9:0]));
				end
				got = got + 1;
			end
		end
		$display("drained %0d bytes", got);

		wait_busy(1'b0, "read-sector done");
		expect_eq(irq, 1'b1, "INTRQ asserted after read sector");

		// final status: motor(b7)=1, wp(b6)=0, deleted(b5)=0, RNF(b4)=0,
		// CRC(b3)=0, lost(b2)=0, DRQ(b1)=0, busy(b0)=0  => 0x80
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[0], 1'b0, "status busy clear");
		expect_eq(status[4], 1'b0, "status RNF clear");
		expect_eq(status[3], 1'b0, "status CRC clear");
		expect_eq(status[1], 1'b0, "status DRQ clear");
		expect_eq(status[7], 1'b1, "status motor set");
		expect_eq(status,    8'h80, "status word after clean read");

		// -------------------- 4) READ ADDRESS (0xC0) --------------------
		// The 1581 DOS uses Read Address to locate the head, so the 6 reply
		// bytes (C,H,R,N,CRC-hi,CRC-lo) must arrive byte-exact and in order.
		// The backend queues all 6 instantly, so this exercises exactly the
		// prebuffered-FIFO case where a too-early pop would shift the stream.
		$display("--- READ ADDRESS ---");
		cpu_write(REG_SECTOR, 8'hEE);          // WD must overwrite this with C
		cpu_write(REG_CMDSTATUS, 8'hC0);
		wait_busy(1'b1, "read-address accepted");
		got   = 0;
		guard = 0;
		while (got < 6) begin
			@(posedge clkcpu);
			guard = guard + 1;
			if (guard > 4_000_000) begin
				errors = errors + 1;
				$display("FAIL: timeout draining read-address (got %0d/6)", got);
				got = 6;
			end
			else if (drq === 1'b1) begin
				cpu_read(REG_DATA, rbyte);
				case (got)
					0: expect_eq(rbyte, 8'h03, "read-address byte0 (C)");
					1: expect_eq(rbyte, 8'h00, "read-address byte1 (H)");
					2: expect_eq(rbyte, 8'h01, "read-address byte2 (R)");
					3: expect_eq(rbyte, 8'h02, "read-address byte3 (N)");
					4: expect_eq(rbyte, 8'hAA, "read-address byte4 (CRC hi)");
					5: expect_eq(rbyte, 8'h55, "read-address byte5 (CRC lo)");
				endcase
				got = got + 1;
			end
		end
		wait_busy(1'b0, "read-address done");
		expect_eq(irq, 1'b1, "INTRQ asserted after read address");
		cpu_read(REG_SECTOR, rbyte);
		expect_eq(rbyte, 8'h03, "sector register = found C after read address");
		cpu_read(REG_CMDSTATUS, status);
		expect_eq(status[0], 1'b0, "ra status busy clear");
		expect_eq(status[4], 1'b0, "ra status RNF clear");
		expect_eq(status[1], 1'b0, "ra status DRQ clear");

		// -------------------- verdict --------------------
		repeat (10) @(posedge clkcpu);
		if (errors == 0) begin
			$display("==== PASS: all physical-mode checks passed ====");
			$finish;
		end else begin
			$display("==== FAIL: %0d error(s) ====", errors);
			$fatal(1, "physical-mode testbench failed");
		end
	end

endmodule
