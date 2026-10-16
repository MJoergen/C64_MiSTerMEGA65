//
// tb_fdc1772_image_golden.sv
//
// Image-mode GOLDEN TRACE for fdc1772.v (C64MEGA65 issue #244, milestone M2).
//
// The physical write milestone adds a phys_mode-gated Write Sector / Write
// Track branch to fdc1772.v. Image mode (phys_mode = 0: D81 images through
// vdrives) must stay expression-for-expression the shipped code. This bench
// proves it by behavior: it drives a SAVE-like register sequence with the
// genuine 1581 ROM's idioms through fdc1772 in IMAGE mode, with the REAL
// floppy.v mechanics model (via tb_floppy_iverilog.py, a verified pure
// declaration reorder for iverilog) and a mock vdrives/SD backend that acks
// sd_rd/sd_wr exactly like vdrives (fdc1772 label3 protocol), and writes a
// register-level TRANSCRIPT:
//   * every CPU register read and write with value and clkcpu cycle stamp
//     (consecutive identical reads of the same register are coalesced into
//     one line "first-stamp R<reg> <val> x<count> ..<last-stamp>"),
//   * every sd_rd / sd_wr request with its LBA and every ack edge,
//   * every DRQ and INTRQ edge,
//   * the 512 bytes the backend received for each SD write.
// The transcript of the pre-change fdc1772.v (submodule commit aa353e1) is
// recorded next to this bench as tb_fdc1772_image_golden.txt; the transcript
// of the changed fdc1772.v must be BYTE-IDENTICAL (cmp). With -DFDC_WR_PORTS
// the bench also connects the #244 write ports of the changed fdc1772.v and,
// with +wrnoise, drives them with pseudo-random levels and strobes the whole
// time (in a multi-drive core the controller-driven inputs reach every drive
// un-gated): the transcript must still be identical.
//
// Sequence (cylinder 1, side 0; the ROM idioms of $CBF4 / $CBEC / $CD17 /
// $C667 / $CD3F at one 6502 cycle = 16 clkcpu, clkcpu = 31.25 MHz):
//   mount (img_mounted pulse, img_size = 819200: D81, 2 sides x 10 x 512),
//   Restore $08, Seek $18 to cylinder 1, Read Address $C8 (6 bytes),
//   Write Sector $A8 to sector R+1 with 512 bytes at DRQ pace, status,
//   Write Track $FA attempt (the image path's fake: BUSY is too short for the
//   $CBF4 poll -- the bench bounds that poll and records it), Force
//   Interrupt $D0, Read Sector $88 of the written sector (512 bytes, checked
//   against the written data).
//
// Run (from a scratch directory; ID = this directory):
//   python3 -I $ID/tb_floppy_iverilog.py $ID/floppy.v > floppy_iverilog.v
//   iverilog -g2012 -o g_orig.vvp $ID/tb_fdc1772_image_golden.sv fdc1772_orig.v \
//            floppy_iverilog.v $ID/iecdrv_misc.sv
//   vvp g_orig.vvp +trace=orig.txt
//   iverilog -g2012 -DFDC_WR_PORTS -o g_new.vvp $ID/tb_fdc1772_image_golden.sv \
//            $ID/fdc1772.v floppy_iverilog.v $ID/iecdrv_misc.sv
//   vvp g_new.vvp +trace=new.txt ; vvp g_new.vvp +trace=noise.txt +wrnoise
//   cmp $ID/tb_fdc1772_image_golden.txt orig.txt
//   cmp orig.txt new.txt ; cmp orig.txt noise.txt
// (fdc1772_orig.v = `git show aa353e1:rtl/iec_drive/fdc1772.v` in the submodule.)
//
// C64MEGA65 project, GPLv3.
//

`timescale 1ns/1ps

module tb_fdc1772_image_golden;

	localparam [1:0] REG_CMDSTATUS = 2'd0;
	localparam [1:0] REG_TRACK     = 2'd1;
	localparam [1:0] REG_SECTOR    = 2'd2;
	localparam [1:0] REG_DATA      = 2'd3;
	localparam integer CYC = 16;      // clkcpu per 6502 cycle (T65 bus cycle)

	// clocks: clkcpu 31.25 MHz (core clock), clk8m_en every 4th clkcpu, clk_sys 50 MHz
	reg clkcpu = 0;   always #16 clkcpu = ~clkcpu;
	reg clk_sys = 0;  always #10 clk_sys = ~clk_sys;
	reg [1:0] div = 0;
	reg clk8m_en = 0;
	always @(posedge clkcpu) begin div <= div + 2'd1; clk8m_en <= (div == 2'd3); end

	integer cycn = 0;                  // clkcpu cycle stamp of the transcript
	always @(posedge clkcpu) cycn = cycn + 1;

	integer errors = 0;
	integer tf;                        // transcript file

	// -------------------------------------------------------------------
	// DUT
	// -------------------------------------------------------------------
	reg        floppy_reset = 0;
	reg  [1:0] cpu_addr = 0;
	reg        cpu_sel = 0;
	reg        cpu_rw = 1;
	reg  [7:0] cpu_din = 0;
	wire [7:0] cpu_dout;
	wire       irq, drq, floppy_step, floppy_ready, fdc_busy;
	reg        img_mounted = 0;
	wire [31:0] sd_lba;
	wire       sd_rd, sd_wr;
	reg        sd_ack = 0;
	reg  [8:0] sd_buff_addr = 0;
	reg  [7:0] sd_dout = 0;
	wire [7:0] sd_din;
	reg        sd_dout_strobe = 0;

	// #244 write ports (connected with -DFDC_WR_PORTS; driven by noise with +wrnoise)
	reg        n_full = 0, n_feed = 0, n_lost = 0, n_busy = 0;
	reg  [2:0] n_level = 0;
	reg        n_done_tgl = 0;
	reg  [1:0] n_done_seq = 0;
	reg  [4:0] n_result = 0;
	reg        n_rnf = 0, n_crc = 0;

	fdc1772 #(
		.CLK_EN(16'd8000), .FD_NUM(1), .MODEL(2),
		.SECTOR_SIZE_CODE(2'd2), .SECTOR_BASE(1'b1), .EXT_MOTOR(1'b1)
	) dut (
		.clkcpu(clkcpu), .clk_sys(clk_sys), .clk8m_en(clk8m_en),
		.floppy_drive(1'b0), .floppy_side(1'b1), .floppy_reset(floppy_reset),
		.floppy_step(floppy_step), .floppy_motor(1'b1),
		.floppy_ready(floppy_ready), .fdc_busy(fdc_busy),
		.irq(irq), .drq(drq),
		.cpu_addr(cpu_addr), .cpu_sel(cpu_sel), .cpu_rw(cpu_rw),
		.cpu_din(cpu_din), .cpu_dout(cpu_dout),
		.img_mounted(img_mounted), .img_wp(1'b0), .img_ds(1'b0), .img_size(32'd819200),
		.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
		.sd_buff_addr(sd_buff_addr), .sd_dout(sd_dout), .sd_din(sd_din),
		.sd_dout_strobe(sd_dout_strobe),

		// image mode: the physical ABI is idle (the read-path inputs are shared
		// controller outputs in the core; here they are simply held)
		.phys_mode(1'b0),
		.phys_active(), .phys_cia_motor_on(), .phys_cia_side(),
		.phys_step_req_tgl(), .phys_step_outward(),
		.phys_rd_req_tgl(), .phys_rd_op(), .phys_rd_track(), .phys_rd_side(),
		.phys_rd_sector(), .phys_rd_cancel_tgl(), .phys_rd_seq(), .phys_byte_ovf(),
		.phys_step_ack_tgl(1'b0), .phys_rd_done_tgl(n_done_tgl), .phys_rd_done_seq(n_done_seq),
		.phys_rd_result(n_result), .phys_rd_crc_err(n_crc), .phys_rd_rnf(n_rnf),
		.phys_rd_deleted(1'b0),
		.phys_rd_c(8'd0), .phys_rd_h(8'd0), .phys_rd_r(8'd0), .phys_rd_n(8'd0),
		.phys_byte_rd_en(), .phys_byte_data(8'd0), .phys_byte_empty(1'b1),
		.phys_media_ready(1'b0), .phys_index(1'b0), .phys_track0(1'b0),
		.phys_wprot(1'b0), .phys_change(1'b0), .phys_motor_on(1'b0),
		.phys_head_settled(1'b0),
`ifdef FDC_WR_PORTS
		.phys_wr_deleted(), .phys_wr_precomp(), .phys_wr_byte_wr(), .phys_wr_byte_data(),
		.phys_wr_fifo_full(n_full), .phys_wr_fifo_level(n_level),
		.phys_wr_feed(n_feed), .phys_wr_lost(n_lost), .phys_wr_busy(n_busy),
		.phys_dbg_wr_fin_tgl(), .phys_dbg_wr_wp_tgl(), .phys_dbg_wr_drop_tgl(),
`endif
		.phys_dbg_lost_tgl(), .phys_dbg_drain_tgl(), .phys_dbg_staledone_tgl(),
		.phys_dbg_busycmd_tgl(), .phys_dbg_fin_tgl(), .phys_dbg_pres_cnt()
	);

	// +wrnoise: pseudo-random activity on every controller-driven input the
	// image-mode drive must ignore (the #244 write levels and the shared done
	// channel), from a fixed seed -- the trace must not change
	reg     noise_on = 0;
	integer seed = 32'h1581_0244;
	always @(posedge clk_sys) if (noise_on) begin
		n_full  <= $random(seed);
		n_feed  <= $random(seed);
		n_lost  <= $random(seed);
		n_busy  <= $random(seed);
		n_level <= $random(seed);
		n_rnf   <= $random(seed);
		n_crc   <= $random(seed);
		n_result <= $random(seed);
		n_done_seq <= $random(seed);
		if (($random(seed) & 255) == 0) n_done_tgl <= ~n_done_tgl;
	end

	// -------------------------------------------------------------------
	// transcript: DRQ / INTRQ edges, SD requests and acks
	// -------------------------------------------------------------------
	reg drq_d = 0, irq_d = 0, sdrd_d = 0, sdwr_d = 0;
	reg  [1:0] last_ra = 2'd0;          // read coalescing: last register, value, count, stamps
	reg  [7:0] last_rv = 8'd0;
	integer    last_rn = 0, last_rt0 = 0, last_rt1 = 0;
	reg        tr_on = 0;

	task tr_flush;
	begin
		if (last_rn == 1) $fwrite(tf, "%0d R%0d %02x\n", last_rt0, last_ra, last_rv);
		else if (last_rn > 1) $fwrite(tf, "%0d R%0d %02x x%0d ..%0d\n", last_rt0, last_ra, last_rv, last_rn, last_rt1);
		last_rn = 0;
	end
	endtask

	always @(posedge clkcpu) if (tr_on) begin
		drq_d <= drq; irq_d <= irq;
		if (drq !== drq_d) begin tr_flush; $fwrite(tf, "%0d DRQ %0d\n", cycn, drq); end
		if (irq !== irq_d) begin tr_flush; $fwrite(tf, "%0d IRQ %0d\n", cycn, irq); end
	end

	// -------------------------------------------------------------------
	// mock vdrives / SD backend (clk_sys): image store, request/ack per label3
	// -------------------------------------------------------------------
	reg  [7:0] img [0:819199];
	reg  [7:0] wrcap [0:511];
	integer    sd_i, sd_dly;
	integer    sd_lba_l;
	reg  [2:0] sd_st = 0;
	always @(posedge clk_sys) begin
		sdrd_d <= sd_rd; sdwr_d <= sd_wr;
		sd_dout_strobe <= 1'b0;
		case (sd_st)
		0: begin
			if (sd_rd && !sdrd_d) begin
				sd_lba_l = sd_lba; sd_dly = 200; sd_st <= 1;
				if (tr_on) begin tr_flush; $fwrite(tf, "%0d SDRD lba=%0d\n", cycn, sd_lba); end
			end else if (sd_wr && !sdwr_d) begin
				sd_lba_l = sd_lba; sd_dly = 200; sd_st <= 4;
				if (tr_on) begin tr_flush; $fwrite(tf, "%0d SDWR lba=%0d\n", cycn, sd_lba); end
			end
		end
		1: if (sd_dly > 0) sd_dly = sd_dly - 1;
		   else begin sd_ack <= 1'b1; sd_i = 0; sd_st <= 2; end
		2: begin // read: stream 512 bytes into the fdc FIFO (port A)
			sd_buff_addr   <= sd_i;
			sd_dout        <= img[sd_lba_l * 512 + sd_i];
			sd_dout_strobe <= 1'b1;
			sd_i = sd_i + 1;
			if (sd_i == 512) sd_st <= 3;
		end
		3: begin
			sd_ack <= 1'b0; sd_st <= 0;
			if (tr_on) begin tr_flush; $fwrite(tf, "%0d SDACK rd lba=%0d\n", cycn, sd_lba_l); end
		end
		4: if (sd_dly > 0) sd_dly = sd_dly - 1;
		   else begin sd_ack <= 1'b1; sd_i = 0; sd_st <= 5; end
		5: begin // write: the address set on edge j is read by the RAM on edge j+1
		         // and its registered q_a (sd_din) is sampled here on edge j+2
			if (sd_i >= 2) wrcap[sd_i - 2] = sd_din;
			if (sd_i < 512) sd_buff_addr <= sd_i;
			sd_i = sd_i + 1;
			if (sd_i == 514) sd_st <= 6;
		end
		6: begin
			for (sd_i = 0; sd_i < 512; sd_i = sd_i + 1) img[sd_lba_l * 512 + sd_i] = wrcap[sd_i];
			sd_ack <= 1'b0; sd_st <= 0;
			if (tr_on) begin
				tr_flush;
				$fwrite(tf, "%0d SDACK wr lba=%0d\n", cycn, sd_lba_l);
				for (sd_i = 0; sd_i < 512; sd_i = sd_i + 16)
					$fwrite(tf, "  wr %03x: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", sd_i[11:0],
					        wrcap[sd_i], wrcap[sd_i+1], wrcap[sd_i+2], wrcap[sd_i+3], wrcap[sd_i+4], wrcap[sd_i+5],
					        wrcap[sd_i+6], wrcap[sd_i+7], wrcap[sd_i+8], wrcap[sd_i+9], wrcap[sd_i+10],
					        wrcap[sd_i+11], wrcap[sd_i+12], wrcap[sd_i+13], wrcap[sd_i+14], wrcap[sd_i+15]);
			end
		end
		default: sd_st <= 0;
		endcase
	end

	// -------------------------------------------------------------------
	// CPU bus cycles (one 6502 cycle = CYC clkcpu; R/W low only in the last
	// clkcpu of a write, as the T65 strobe in c1581_drv) and the ROM idioms
	// -------------------------------------------------------------------
	task cyc(input integer n);
		integer k;
	begin
		for (k = 0; k < n * CYC; k = k + 1) @(posedge clkcpu);
		#1;
	end
	endtask

	task bus_rd(input [1:0] a, output [7:0] d);
		integer k;
	begin
		cpu_sel = 1'b1; cpu_rw = 1'b1; cpu_addr = a;
		for (k = 0; k < CYC - 1; k = k + 1) @(posedge clkcpu);
		#1; d = cpu_dout;
		@(posedge clkcpu); #1;
		cpu_sel = 1'b0;
		if (tr_on) begin
			if (last_rn > 0 && last_ra == a && last_rv == d) begin last_rn = last_rn + 1; last_rt1 = cycn; end
			else begin tr_flush; last_ra = a; last_rv = d; last_rn = 1; last_rt0 = cycn; last_rt1 = cycn; end
		end
	end
	endtask

	task bus_wr(input [1:0] a, input [7:0] d);
		integer k;
	begin
		cpu_sel = 1'b1; cpu_rw = 1'b1; cpu_addr = a; cpu_din = d;
		for (k = 0; k < CYC - 1; k = k + 1) @(posedge clkcpu);
		#1; cpu_rw = 1'b0;
		@(posedge clkcpu); #1;
		cpu_sel = 1'b0; cpu_rw = 1'b1;
		if (tr_on) begin tr_flush; $fwrite(tf, "%0d W%0d %02x\n", cycn, a, d); end
	end
	endtask

	// $CBF4: NOP; STA $6000; LDA #1; BIT $6000; BEQ -- bounded here (the ROM is not)
	task rom_cmd(input [7:0] c, input integer maxpoll);
		reg [7:0] st;
		integer g;
		reg seen;
	begin
		cyc(2); cyc(3); bus_wr(REG_CMDSTATUS, c); cyc(2);
		g = 0; seen = 0;
		while (!seen) begin
			cyc(3); bus_rd(REG_CMDSTATUS, st); g = g + 1;
			if (st[0] === 1'b1) begin seen = 1; cyc(2); end
			else begin
				cyc(3);
				if (g >= maxpoll) begin
					seen = 1;
					tr_flush; $fwrite(tf, "%0d NOTE busy-set poll gave up after %0d polls\n", cycn, g);
				end
			end
		end
		cyc(26);
	end
	endtask

	// $CBEC + $CD3F epilogue: wait BUSY = 0, then the status the job evaluates
	task rom_status(output [7:0] st_out);
		reg [7:0] st;
		integer g;
		reg idle;
	begin
		cyc(12);
		g = 0; idle = 0;
		while (!idle) begin
			cyc(3); bus_rd(REG_CMDSTATUS, st); g = g + 1;
			if (st[0] !== 1'b1) begin idle = 1; cyc(2); end
			else begin
				cyc(3);
				if (g > 2000000) begin idle = 1; errors = errors + 1; $display("FAIL: BUSY never cleared @%0t", $time); end
			end
		end
		cyc(12); bus_rd(REG_CMDSTATUS, st_out); cyc(29);
	end
	endtask

	// $CD17-style read loop: BUSY first, DRQ second, one data read per DRQ
	reg  [7:0] rbuf [0:1023];
	integer    rcnt;
	task rom_rloop(input integer maxbytes);
		reg [7:0] st, d;
		reg done_l;
		integer g;
	begin
		rcnt = 0; done_l = 0; g = 0;
		while (!done_l) begin
			cyc(3); bus_rd(REG_CMDSTATUS, st); cyc(4); g = g + 1;
			if (st[0] !== 1'b1) begin cyc(3); done_l = 1; end
			else begin
				cyc(2);
				if (st[1] !== 1'b1) cyc(3);
				else begin
					cyc(2 + 3 + 3); bus_rd(REG_DATA, d);   // ... LDA $6003
					if (rcnt < 1024) rbuf[rcnt] = d;
					rcnt = rcnt + 1;
					cyc(6 + 5 + 3);
					if (rcnt >= maxbytes) done_l = 1;
				end
			end
			if (g > 4000000) begin done_l = 1; errors = errors + 1; $display("FAIL: rom_rloop guard @%0t", $time); end
		end
	end
	endtask

	// $C667 write loop: BUSY first, DRQ second, STA $6003 per DRQ, two pages
	reg  [7:0] wbuf [0:511];
	integer    wcnt;
	task rom_wloop;
		reg [7:0] st;
		reg done_l;
		integer g;
	begin
		wcnt = 0; done_l = 0; g = 0;
		while (!done_l) begin
			cyc(3); bus_rd(REG_CMDSTATUS, st); cyc(4); g = g + 1;
			if (st[0] !== 1'b1) begin cyc(3); done_l = 1; end
			else begin
				cyc(2);
				if (st[1] !== 1'b1) cyc(3);
				else begin
					cyc(2 + 3 + 2 + 5 + 3); bus_wr(REG_DATA, wbuf[wcnt]); wcnt = wcnt + 1;
					cyc(3 + 5);
					if ((wcnt % 256) != 0) cyc(3);
					else begin cyc(2 + 5 + 2); if (wcnt >= 512) begin cyc(2); done_l = 1; end else cyc(3); end
				end
			end
			if (g > 4000000) begin done_l = 1; errors = errors + 1; $display("FAIL: rom_wloop guard @%0t", $time); end
		end
	end
	endtask

	// -------------------------------------------------------------------
	// stimulus
	// -------------------------------------------------------------------
	reg  [7:0] st, rr;
	reg  [7:0] ra_c, ra_r;
	integer    i, bad;
	reg [1023:0] tname;

	initial begin
		#400_000_000;
		$display("FAIL: global timeout");
		$fatal(1, "global timeout");
	end

	initial begin
		if (!$value$plusargs("trace=%s", tname)) tname = "golden_trace.txt";
		tf = $fopen(tname, "w");
		if (tf == 0) $fatal(1, "cannot open the transcript file");
		if ($test$plusargs("wrnoise")) noise_on = 1;

		for (i = 0; i < 819200; i = i + 1) img[i] = (i * 5 + (i >> 9)) & 8'hFF;
		for (i = 0; i < 512; i = i + 1) wbuf[i] = (i * 11 + 7) ^ (i >> 4);

		// FPGA power-up-to-0 for timers/state the RTL reset clauses do not clear
		// (iverilog starts regs at X; hardware configures them to 0). Pure sim
		// init, identical for every fdc1772.v version under test. floppy.rate is
		// started at full speed: the 1581 motor has been spinning (PA2 on).
		dut.delay_cnt = 0; dut.step_rate_cnt = 0; dut.step_pulse_cnt = 0;
		dut.motor_spin_up_sequence = 0; dut.motor_timeout_index = 0; dut.data_lost = 0;
		dut.fdd[0].floppy.index_pulse_cnt = 0;  dut.fdd[0].floppy.index = 0;
		dut.fdd[0].floppy.step_busy = 0;        dut.fdd[0].floppy.step_inD = 0;
		dut.fdd[0].floppy.step_outD = 0;        dut.fdd[0].floppy.sec_state = 0;
		dut.fdd[0].floppy.sec_byte_cnt = 0;     dut.fdd[0].floppy.byte_cnt = 0;
		dut.fdd[0].floppy.index_pulse_start = 0; dut.fdd[0].floppy.byte_clk_en = 0;
		dut.fdd[0].floppy.clk_cnt2 = 0;         dut.fdd[0].floppy.spin_up_counter = 0;
		dut.fdd[0].floppy.rate = 250000;        dut.fdd[0].floppy.motor_onD = 1;
		dut.fdd[0].floppy.data_clk = 0;         dut.fdd[0].floppy.data_clk_en = 0;
		dut.fdd[0].floppy.clk_cnt = 0;

		floppy_reset = 1'b0;
		repeat (16) @(posedge clkcpu);
		floppy_reset = 1'b1;
		repeat (16) @(posedge clkcpu); #1;
		img_mounted = 1'b1; repeat (8) @(posedge clkcpu); #1; img_mounted = 1'b0;
		repeat (64) @(posedge clkcpu); #1;
		tr_on = 1;
		$fwrite(tf, "%0d MOUNT img_size=819200\n", cycn);

		// Restore $08 (head is already on cylinder 0)
		rom_cmd(8'h08, 20); rom_status(st);
		// Seek $18 to cylinder 1: data register = target
		cyc(3); bus_wr(REG_DATA, 8'd1); cyc(4);
		rom_cmd(8'h18, 20); rom_status(st);
		cyc(36600);                                    // $CBD5 18.3 ms settle
		// Read Address $C8
		rom_cmd(8'hC8, 20);
		rom_rloop(6);
		rom_status(st);
		ra_c = rbuf[0]; ra_r = rbuf[2];
		$display("info: Read Address C=%0d H=%0d R=%0d N=%0d status=%02x", rbuf[0], rbuf[1], rbuf[2], rbuf[3], st);
		// Write Sector $A8 at R+1 (wrapping 10 -> 1)
		cyc(3); bus_wr(REG_TRACK, ra_c);
		cyc(3); bus_wr(REG_SECTOR, (ra_r == 8'd10) ? 8'd1 : ra_r + 8'd1);
		cyc(40);
		rom_cmd(8'hA8, 20);
		rom_wloop;
		rom_status(st);
		$display("info: Write Sector wrote %0d bytes, status=%02x", wcnt, st);
		// Write Track $FA attempt (the image path fakes it)
		cyc(200);
		rom_cmd(8'hFA, 20);
		rom_status(st);
		// Force Interrupt $D0 ($CFD1) and its 126 us delay
		cyc(2); cyc(3); bus_wr(REG_CMDSTATUS, 8'hD0); cyc(251);
		rom_status(st);
		// Read Sector $88 of the written sector
		cyc(3); bus_wr(REG_SECTOR, (ra_r == 8'd10) ? 8'd1 : ra_r + 8'd1);
		cyc(40);
		rom_cmd(8'h88, 20);
		rom_rloop(512);
		rom_status(st);
		tr_flush;
		bad = 0;
		for (i = 0; i < 512; i = i + 1) if (rbuf[i] !== wbuf[i]) bad = bad + 1;
		$display("info: Read Sector got %0d bytes, %0d mismatching the written data, status=%02x", rcnt, bad, st);
		$fwrite(tf, "%0d END read-back mismatches=%0d\n", cycn, bad);
		$fclose(tf);
		if (rcnt != 512 || bad != 0) errors = errors + 1;
		if (wcnt != 512) errors = errors + 1;
		if (errors == 0) begin
			$display("==== golden trace written to %0s (sequence completed) ====", tname);
			$finish;
		end else begin
			$display("==== FAIL: %0d error(s) ====", errors);
			$fatal(1, "image golden bench failed");
		end
	end

endmodule
