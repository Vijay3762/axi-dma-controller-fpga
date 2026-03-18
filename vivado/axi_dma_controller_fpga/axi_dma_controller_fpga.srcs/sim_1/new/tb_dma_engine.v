// =============================================================================
// Module      : tb_dma_engine.v
// Description : Comprehensive testbench for dma_engine.v
//
// Tests performed:
//   Test 1  - Reset behavior: all outputs de-asserted after reset
//   Test 2  - Single-word transfer (4 bytes): full AR→R→AW→W→B cycle
//   Test 3  - Multi-word transfer (16 bytes): 4 iterations of the FSM loop
//   Test 4  - Read error (RRESP=SLVERR): engine asserts error, goes to ERROR state
//   Test 5  - Write error (BRESP=SLVERR): engine asserts error, goes to ERROR state
//   Test 6  - ACK clears done and returns to IDLE
//   Test 7  - ACK clears error and returns to IDLE
//   Test 8  - No re-trigger while busy (start ignored when busy=1)
//   Test 9  - Address pointer increments correctly each iteration (src+4, dst+4)
//   Test 10 - ARVALID de-asserted after AR handshake (no ghost AR pulses)
//   Test 11 - AWVALID de-asserted after AW handshake
//   Test 12 - WLAST is high during write beat, de-asserted after W handshake
//   Test 13 - WSTRB is 4'hF on every write beat
//   Test 14 - ARLEN/ARSIZE/ARBURST and AWLEN/AWSIZE/AWBURST are correct
//   Test 15 - Back-to-back transfers: second start after first ack
//   Test 16 - Slow subordinate: ARREADY delayed by 5 cycles (stall test)
//   Test 17 - Slow subordinate: AWREADY delayed by 5 cycles
//   Test 18 - Slow subordinate: WREADY delayed by 5 cycles
//   Test 19 - Slow subordinate: BVALID delayed by 5 cycles
//   Test 20 - data_buf integrity: RDATA is correctly forwarded to WDATA
// =============================================================================

`timescale 1ns / 1ps

module tb_dma_engine;

// =============================================================================
// Parameters
// =============================================================================
parameter DATA_WIDTH = 32;
parameter ADDR_WIDTH = 32;
parameter CLK_PERIOD = 10;   // 10 ns = 100 MHz

// =============================================================================
// DUT Signal Declarations
// =============================================================================

// Global
reg clk;
reg rst_n;

// Control Interface
reg  [ADDR_WIDTH-1:0] src_addr;
reg  [ADDR_WIDTH-1:0] dst_addr;
reg  [DATA_WIDTH-1:0] transfer_len;
reg                   start;

// Status Interface
wire busy;
wire done;
wire error;

// AXI4 Full Master - Read Address Channel (AR)
wire [ADDR_WIDTH-1:0] M_AXI_ARADDR;
wire                  M_AXI_ARVALID;
wire [           7:0] M_AXI_ARLEN;
wire [           2:0] M_AXI_ARSIZE;
wire [           1:0] M_AXI_ARBURST;
reg                   M_AXI_ARREADY;

// AXI4 Full Master - Read Data Channel (R)
reg  [DATA_WIDTH-1:0] M_AXI_RDATA;
reg  [           1:0] M_AXI_RRESP;
reg                   M_AXI_RVALID;
reg                   M_AXI_RLAST;
wire                  M_AXI_RREADY;

// AXI4 Full Master - Write Address Channel (AW)
wire [ADDR_WIDTH-1:0] M_AXI_AWADDR;
wire                  M_AXI_AWVALID;
wire [           7:0] M_AXI_AWLEN;
wire [           2:0] M_AXI_AWSIZE;
wire [           1:0] M_AXI_AWBURST;
reg                   M_AXI_AWREADY;

// AXI4 Full Master - Write Data Channel (W)
wire [    DATA_WIDTH-1:0] M_AXI_WDATA;
wire [(DATA_WIDTH/8)-1:0] M_AXI_WSTRB;
wire                      M_AXI_WVALID;
wire                      M_AXI_WLAST;
reg                       M_AXI_WREADY;

// AXI4 Full Master - Write Response Channel (B)
reg  [1:0] M_AXI_BRESP;
reg        M_AXI_BVALID;
wire       M_AXI_BREADY;

// ACK
reg ack;

// =============================================================================
// Test Tracking
// =============================================================================
integer pass_count;
integer fail_count;

// =============================================================================
// DUT Instantiation
// =============================================================================
dma_engine #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) DUT (
    .clk              (clk),
    .rst_n            (rst_n),
    .src_addr         (src_addr),
    .dst_addr         (dst_addr),
    .transfer_len     (transfer_len),
    .start            (start),
    .busy             (busy),
    .done             (done),
    .error            (error),
    .M_AXI_ARADDR     (M_AXI_ARADDR),
    .M_AXI_ARVALID    (M_AXI_ARVALID),
    .M_AXI_ARLEN      (M_AXI_ARLEN),
    .M_AXI_ARSIZE     (M_AXI_ARSIZE),
    .M_AXI_ARBURST    (M_AXI_ARBURST),
    .M_AXI_ARREADY    (M_AXI_ARREADY),
    .M_AXI_RDATA      (M_AXI_RDATA),
    .M_AXI_RRESP      (M_AXI_RRESP),
    .M_AXI_RVALID     (M_AXI_RVALID),
    .M_AXI_RLAST      (M_AXI_RLAST),
    .M_AXI_RREADY     (M_AXI_RREADY),
    .M_AXI_AWADDR     (M_AXI_AWADDR),
    .M_AXI_AWVALID    (M_AXI_AWVALID),
    .M_AXI_AWLEN      (M_AXI_AWLEN),
    .M_AXI_AWSIZE     (M_AXI_AWSIZE),
    .M_AXI_AWBURST    (M_AXI_AWBURST),
    .M_AXI_AWREADY    (M_AXI_AWREADY),
    .M_AXI_WDATA      (M_AXI_WDATA),
    .M_AXI_WSTRB      (M_AXI_WSTRB),
    .M_AXI_WVALID     (M_AXI_WVALID),
    .M_AXI_WLAST      (M_AXI_WLAST),
    .M_AXI_WREADY     (M_AXI_WREADY),
    .M_AXI_BRESP      (M_AXI_BRESP),
    .M_AXI_BVALID     (M_AXI_BVALID),
    .M_AXI_BREADY     (M_AXI_BREADY),
    .ack              (ack)
);

// =============================================================================
// Clock Generation
// =============================================================================
initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// =============================================================================
// Waveform Dump
// =============================================================================
initial begin
    $dumpfile("tb_dma_engine.vcd");
    $dumpvars(0, tb_dma_engine);
end

// =============================================================================
// Task: check
// =============================================================================
task check;
    input integer    test_id;
    input reg [255:0] test_name;
    input reg [63:0]  actual;
    input reg [63:0]  expected;
    begin
        if (actual === expected) begin
            $display("  [PASS] Test %0d - %s | got 0x%0X", test_id, test_name, actual);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] Test %0d - %s | got 0x%0X  expected 0x%0X",
                     test_id, test_name, actual, expected);
            fail_count = fail_count + 1;
        end
    end
endtask

// =============================================================================
// Task: do_reset
// Applies active-low reset for N cycles then releases.
// All AXI subordinate signals are put to idle inside here.
// =============================================================================
task do_reset;
    begin
        rst_n         = 1'b0;
        // Idle all subordinate-side signals
        M_AXI_ARREADY = 1'b0;
        M_AXI_RDATA   = 32'h0;
        M_AXI_RRESP   = 2'b00;
        M_AXI_RVALID  = 1'b0;
        M_AXI_RLAST   = 1'b0;
        M_AXI_AWREADY = 1'b0;
        M_AXI_WREADY  = 1'b0;
        M_AXI_BRESP   = 2'b00;
        M_AXI_BVALID  = 1'b0;
        ack           = 1'b0;
        start         = 1'b0;
        repeat(5) @(posedge clk);
        rst_n = 1'b1;
        repeat(2) @(posedge clk);
    end
endtask

// =============================================================================
// Task: axi_ar_handshake
// Waits for ARVALID, then asserts ARREADY for 1 cycle to complete the handshake.
// delay_cycles: how many cycles to wait before asserting ARREADY (tests stalls).
// =============================================================================
task axi_ar_handshake;
    input integer delay_cycles;
    begin
        // Wait for engine to assert ARVALID
        @(posedge clk);
        while (!M_AXI_ARVALID) @(posedge clk);
        // Apply optional stall delay
        repeat(delay_cycles) @(posedge clk);
        // Assert ARREADY for 1 cycle
        @(negedge clk); M_AXI_ARREADY = 1'b1;
        @(posedge clk);
        @(negedge clk); M_AXI_ARREADY = 1'b0;
    end
endtask

// =============================================================================
// Task: axi_r_handshake
// Drives RDATA/RRESP/RVALID/RLAST so the engine can capture read data.
// =============================================================================
task axi_r_handshake;
    input [DATA_WIDTH-1:0] rdata;
    input [1:0]            rresp;
    begin
        // Wait until RREADY from engine is asserted
        @(posedge clk);
        while (!M_AXI_RREADY) @(posedge clk);
        @(negedge clk);
        M_AXI_RDATA  = rdata;
        M_AXI_RRESP  = rresp;
        M_AXI_RVALID = 1'b1;
        M_AXI_RLAST  = 1'b1;   // Single-beat burst, LAST is always with data
        @(posedge clk);
        @(negedge clk);
        M_AXI_RVALID = 1'b0;
        M_AXI_RLAST  = 1'b0;
    end
endtask

// =============================================================================
// Task: axi_aw_handshake
// =============================================================================
task axi_aw_handshake;
    input integer delay_cycles;
    begin
        @(posedge clk);
        while (!M_AXI_AWVALID) @(posedge clk);
        repeat(delay_cycles) @(posedge clk);
        @(negedge clk); M_AXI_AWREADY = 1'b1;
        @(posedge clk);
        @(negedge clk); M_AXI_AWREADY = 1'b0;
    end
endtask

// =============================================================================
// Task: axi_w_handshake
// =============================================================================
task axi_w_handshake;
    input integer delay_cycles;
    begin
        @(posedge clk);
        while (!M_AXI_WVALID) @(posedge clk);
        repeat(delay_cycles) @(posedge clk);
        @(negedge clk); M_AXI_WREADY = 1'b1;
        @(posedge clk);
        @(negedge clk); M_AXI_WREADY = 1'b0;
    end
endtask

// =============================================================================
// Task: axi_b_handshake
// =============================================================================
task axi_b_handshake;
    input [1:0]   bresp;
    input integer delay_cycles;
    begin
        @(posedge clk);
        while (!M_AXI_BREADY) @(posedge clk);
        repeat(delay_cycles) @(posedge clk);
        @(negedge clk);
        M_AXI_BRESP  = bresp;
        M_AXI_BVALID = 1'b1;
        @(posedge clk);
        @(negedge clk);
        M_AXI_BVALID = 1'b0;
    end
endtask

// =============================================================================
// Task: do_one_beat
// Drives a single full AR→R→AW→W→B sequence (1 word = 4 bytes).
// Used inside multi-iteration tests.
// rdata   : the payload the subordinate returns on the R channel
// rresp   : response code for R  (2'b00=OKAY, 2'b10=SLVERR)
// bresp   : response code for B  (2'b00=OKAY, 2'b10=SLVERR)
// ar_dly  : stall cycles before ARREADY
// aw_dly  : stall cycles before AWREADY
// w_dly   : stall cycles before WREADY
// b_dly   : stall cycles before BVALID
// =============================================================================
task do_one_beat;
    input [DATA_WIDTH-1:0] rdata;
    input [1:0]            rresp;
    input [1:0]            bresp;
    input integer          ar_dly;
    input integer          aw_dly;
    input integer          w_dly;
    input integer          b_dly;
    begin
        fork
            axi_ar_handshake(ar_dly);
            axi_r_handshake(rdata, rresp);
        join
        // After R completes FSM moves to WRITE_ADDR only if RRESP==OKAY
        if (rresp == 2'b00) begin
            fork
                axi_aw_handshake(aw_dly);
                axi_w_handshake(w_dly);
                axi_b_handshake(bresp, b_dly);
            join
        end
    end
endtask

// =============================================================================
// Task: pulse_start
// Drives start HIGH for exactly 1 cycle (matching the AXI-lite slave behavior).
// =============================================================================
task pulse_start;
    begin
        @(negedge clk);
        start = 1'b1;
        @(posedge clk);
        @(negedge clk);
        start = 1'b0;
    end
endtask

// =============================================================================
// Task: send_ack
// Drives ack HIGH for 1 cycle, which clears DONE/ERROR and returns FSM to IDLE.
// =============================================================================
task send_ack;
    begin
        @(negedge clk);
        ack = 1'b1;
        @(posedge clk);
        @(negedge clk);
        ack = 1'b0;
    end
endtask

// =============================================================================
// Task: wait_for_done_or_error
// Blocks until done or error goes HIGH; times out after N cycles.
// =============================================================================
task wait_for_done_or_error;
    input integer timeout;
    integer i;
    begin
        i = 0;
        while (!done && !error && i < timeout) begin
            @(posedge clk);
            i = i + 1;
        end
        if (i >= timeout) begin
            $display("  [TIMEOUT] wait_for_done_or_error exceeded %0d cycles", timeout);
            fail_count = fail_count + 1;
        end
    end
endtask

// =============================================================================
// Captured waveform helper registers (set by monitor blocks in specific tests)
// =============================================================================
reg arvalid_after_handshake;   // sampled 2 cycles after AR handshake
reg awvalid_after_handshake;
reg wlast_during_beat;         // sampled while WVALID is high
reg wlast_after_beat;          // sampled 1 cycle after W handshake

// =============================================================================
// MAIN TEST SEQUENCE
// =============================================================================
initial begin
    pass_count = 0;
    fail_count = 0;

    // Initialise control inputs
    src_addr     = 32'h0;
    dst_addr     = 32'h0;
    transfer_len = 32'h0;
    start        = 1'b0;
    ack          = 1'b0;

    $display("=============================================================");
    $display("  DMA Engine Testbench Starting");
    $display("=============================================================");

    // =========================================================================
    // TEST 1 - Reset: all status outputs LOW, all AXI master outputs LOW
    // =========================================================================
    $display("\n--- Test 1: Reset behavior ---");
    do_reset();

    check(1, "busy after reset",        {31'b0, busy},           32'h0);
    check(1, "done after reset",        {31'b0, done},           32'h0);
    check(1, "error after reset",       {31'b0, error},          32'h0);
    check(1, "ARVALID after reset",     {31'b0, M_AXI_ARVALID},  32'h0);
    check(1, "AWVALID after reset",     {31'b0, M_AXI_AWVALID},  32'h0);
    check(1, "WVALID after reset",      {31'b0, M_AXI_WVALID},   32'h0);
    check(1, "BREADY after reset",      {31'b0, M_AXI_BREADY},   32'h0);
    check(1, "RREADY after reset",      {31'b0, M_AXI_RREADY},   32'h0);
    check(1, "ARSIZE after reset",      {29'b0, M_AXI_ARSIZE},   32'h2);  // 4 bytes
    check(1, "ARBURST after reset",     {30'b0, M_AXI_ARBURST},  32'h1);  // INCR

    // =========================================================================
    // TEST 2 - Single-word transfer (4 bytes): complete AR→R→AW→W→B cycle
    // =========================================================================
    $display("\n--- Test 2: Single-word transfer (4 bytes) ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;          // exactly 1 word

    fork
        // Stimulus thread
        begin
            pulse_start();
            do_one_beat(32'hCAFEBABE, 2'b00, 2'b00, 0, 0, 0, 0);
            wait_for_done_or_error(100);
        end
        // Checker thread (runs in parallel, samples at the right moments)
        begin
            // Wait until busy goes high (FSM left IDLE)
            @(posedge clk); while (!busy) @(posedge clk);
            check(2, "busy asserted", {31'b0, busy}, 32'h1);
        end
    join

    check(2, "done after 4-byte xfer",  {31'b0, done},  32'h1);
    check(2, "error stays low",         {31'b0, error}, 32'h0);
    check(2, "busy de-asserted",        {31'b0, busy},  32'h0);
    // Verify ARADDR and AWADDR pointed to the right locations
    // (can only sample static wires; last valid values held until next transfer)
    // WDATA must have been the RDATA payload
    check(2, "WDATA == RDATA forwarded", M_AXI_WDATA, 32'hCAFEBABE);

    send_ack();
    @(posedge clk);
    check(2, "done cleared after ack",  {31'b0, done}, 32'h0);

    // =========================================================================
    // TEST 3 - Multi-word transfer (16 bytes = 4 iterations)
    // =========================================================================
    $display("\n--- Test 3: Multi-word transfer (16 bytes, 4 iterations) ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd16;

    fork
        begin
            pulse_start();
            // Drive 4 AR→R→AW→W→B beats, each with incrementing RDATA
            do_one_beat(32'hDEAD_0001, 2'b00, 2'b00, 0, 0, 0, 0);
            do_one_beat(32'hDEAD_0002, 2'b00, 2'b00, 0, 0, 0, 0);
            do_one_beat(32'hDEAD_0003, 2'b00, 2'b00, 0, 0, 0, 0);
            do_one_beat(32'hDEAD_0004, 2'b00, 2'b00, 0, 0, 0, 0);
            wait_for_done_or_error(200);
        end
        begin : busy_check3
            @(posedge clk); while (!busy) @(posedge clk);
            check(3, "busy during multi-word xfer", {31'b0, busy}, 32'h1);
        end
    join

    check(3, "done after 16-byte xfer", {31'b0, done},  32'h1);
    check(3, "error stays low",         {31'b0, error}, 32'h0);
    send_ack();

    // =========================================================================
    // TEST 4 - Read error (RRESP = SLVERR = 2'b10): engine must latch error
    // =========================================================================
    $display("\n--- Test 4: Read error (RRESP=SLVERR) ---");
    do_reset();
    src_addr     = 32'hDEAD_0000;
    dst_addr     = 32'hBEEF_0000;
    transfer_len = 32'd4;

    fork
        begin
            pulse_start();
            // AR handshake completes normally
            axi_ar_handshake(0);
            // R channel returns SLVERR
            axi_r_handshake(32'hBAD1_BAD1, 2'b10);
            // Write channels must NOT be driven (FSM should have jumped to ERROR)
            wait_for_done_or_error(50);
        end
        begin : dummy4
            // nothing parallel needed
        end
    join

    check(4, "error asserted on RRESP=SLVERR", {31'b0, error}, 32'h1);
    check(4, "done stays low",                 {31'b0, done},  32'h0);
    check(4, "busy de-asserted on error",      {31'b0, busy},  32'h0);
    send_ack();
    @(posedge clk);
    check(4, "error cleared after ack",        {31'b0, error}, 32'h0);

    // =========================================================================
    // TEST 5 - Write error (BRESP = SLVERR = 2'b10)
    // =========================================================================
    $display("\n--- Test 5: Write error (BRESP=SLVERR) ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    fork
        begin
            pulse_start();
            axi_ar_handshake(0);
            axi_r_handshake(32'h1234_5678, 2'b00);   // read is OK
            fork
                axi_aw_handshake(0);
                axi_w_handshake(0);
                axi_b_handshake(2'b10, 0);            // BRESP = SLVERR
            join
            wait_for_done_or_error(50);
        end
        begin : dummy5
        end
    join

    check(5, "error asserted on BRESP=SLVERR", {31'b0, error}, 32'h1);
    check(5, "done stays low",                 {31'b0, done},  32'h0);
    send_ack();
    @(posedge clk);
    check(5, "error cleared after ack",        {31'b0, error}, 32'h0);

    // =========================================================================
    // TEST 6 - ACK clears done and engine returns to IDLE (verified by
    //          accepting a fresh transfer immediately after ack)
    // =========================================================================
    $display("\n--- Test 6: ACK clears done and engine returns to IDLE ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    fork
        begin
            pulse_start();
            do_one_beat(32'hABCD_1234, 2'b00, 2'b00, 0, 0, 0, 0);
            wait_for_done_or_error(100);
        end
        begin : dummy6 end
    join

    check(6, "done before ack",  {31'b0, done}, 32'h1);
    send_ack();
    repeat(2) @(posedge clk);
    check(6, "done cleared",     {31'b0, done}, 32'h0);
    check(6, "busy cleared",     {31'b0, busy}, 32'h0);

    // Issue a second transfer to confirm FSM is really back in IDLE
    fork
        begin
            pulse_start();
            do_one_beat(32'hFFFF_FFFF, 2'b00, 2'b00, 0, 0, 0, 0);
            wait_for_done_or_error(100);
        end
        begin : dummy6b end
    join
    check(6, "second transfer completes after ack", {31'b0, done}, 32'h1);
    send_ack();

    // =========================================================================
    // TEST 7 - ACK clears error
    // =========================================================================
    $display("\n--- Test 7: ACK clears error and engine returns to IDLE ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    fork
        begin
            pulse_start();
            axi_ar_handshake(0);
            axi_r_handshake(32'h0, 2'b10);   // force error
            wait_for_done_or_error(50);
        end
        begin : dummy7 end
    join

    check(7, "error set",         {31'b0, error}, 32'h1);
    send_ack();
    repeat(2) @(posedge clk);
    check(7, "error cleared",     {31'b0, error}, 32'h0);

    // Confirm FSM is back in IDLE - issue a clean transfer
    fork
        begin
            pulse_start();
            do_one_beat(32'h0000_0001, 2'b00, 2'b00, 0, 0, 0, 0);
            wait_for_done_or_error(100);
        end
        begin : dummy7b end
    join
    check(7, "transfer succeeds after error+ack", {31'b0, done}, 32'h1);
    send_ack();

    // =========================================================================
    // TEST 8 - Start is ignored while engine is busy
    //          We stall all subordinate channels so engine stays busy,
    //          pulse start again, and verify done fires only once.
    // =========================================================================
    $display("\n--- Test 8: Start ignored while busy ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    // Start first transfer
    pulse_start();

    // Wait until engine is definitely busy (in READ_ADDR state)
    @(posedge clk); while (!busy) @(posedge clk);
    check(8, "busy set", {31'b0, busy}, 32'h1);

    // Fire start AGAIN while busy - this should be completely ignored
    pulse_start();

    // Now complete the first transfer normally
    fork
        begin
            axi_ar_handshake(0);
            axi_r_handshake(32'hDEAD_BEEF, 2'b00);
            fork
                axi_aw_handshake(0);
                axi_w_handshake(0);
                axi_b_handshake(2'b00, 0);
            join
            wait_for_done_or_error(100);
        end
        begin : dummy8 end
    join

    check(8, "done fires exactly once", {31'b0, done}, 32'h1);
    // done must be 1, not some glitched state due to double-start
    send_ack();
    @(posedge clk);
    check(8, "done cleared",            {31'b0, done}, 32'h0);

    // =========================================================================
    // TEST 9 - Address pointer increments by 4 each iteration
    //          We do a 12-byte transfer (3 words) and capture ARADDR each beat.
    // =========================================================================
    $display("\n--- Test 9: Address pointer increments correctly ---");
    do_reset();
    src_addr     = 32'h1000_0000;
    dst_addr     = 32'h2000_0000;
    transfer_len = 32'd12;   // 3 words

    // We need to sample ARADDR and AWADDR at each beat.
    // Use a fork-join approach with local storage.
    begin : test9_block
        reg [ADDR_WIDTH-1:0] captured_araddr [0:2];
        reg [ADDR_WIDTH-1:0] captured_awaddr [0:2];
        integer beat;

        fork
            // Stimulus
            begin
                pulse_start();
                // Beat 0
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                captured_araddr[0] = M_AXI_ARADDR;
                @(posedge clk); while (!M_AXI_AWVALID && !M_AXI_ARVALID) @(posedge clk);
                // Complete beat 0 - AR handshake
                @(negedge clk); M_AXI_ARREADY = 1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY = 1'b0;
                // R
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=32'hAABB_CC01; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                // AW
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                captured_awaddr[0] = M_AXI_AWADDR;
                @(negedge clk); M_AXI_AWREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY=1'b0;
                // W
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                @(negedge clk); M_AXI_WREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY=1'b0;
                // B
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;

                // Beat 1
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                captured_araddr[1] = M_AXI_ARADDR;
                @(negedge clk); M_AXI_ARREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY=1'b0;
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=32'hAABB_CC02; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                captured_awaddr[1] = M_AXI_AWADDR;
                @(negedge clk); M_AXI_AWREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY=1'b0;
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                @(negedge clk); M_AXI_WREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY=1'b0;
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;

                // Beat 2
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                captured_araddr[2] = M_AXI_ARADDR;
                @(negedge clk); M_AXI_ARREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY=1'b0;
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=32'hAABB_CC03; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                captured_awaddr[2] = M_AXI_AWADDR;
                @(negedge clk); M_AXI_AWREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY=1'b0;
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                @(negedge clk); M_AXI_WREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY=1'b0;
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;

                wait_for_done_or_error(50);
            end
        join

        // Now check all captured addresses
        check(9, "ARADDR beat0", captured_araddr[0], 32'h1000_0000);
        check(9, "ARADDR beat1", captured_araddr[1], 32'h1000_0004);
        check(9, "ARADDR beat2", captured_araddr[2], 32'h1000_0008);
        check(9, "AWADDR beat0", captured_awaddr[0], 32'h2000_0000);
        check(9, "AWADDR beat1", captured_awaddr[1], 32'h2000_0004);
        check(9, "AWADDR beat2", captured_awaddr[2], 32'h2000_0008);
    end
    check(9, "done after 12-byte xfer", {31'b0, done}, 32'h1);
    send_ack();

    // =========================================================================
    // TEST 10 - ARVALID de-asserted after AR handshake (no ghost AR pulses)
    //           Monitor ARVALID for 5 cycles after the handshake completes.
    // =========================================================================
    $display("\n--- Test 10: ARVALID de-asserted after AR handshake ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    begin : test10_block
        integer arvalid_count;
        arvalid_count = 0;

        pulse_start();

        // Wait for ARVALID to go high
        @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);

        // Complete handshake
        @(negedge clk); M_AXI_ARREADY = 1'b1;
        @(posedge clk); @(negedge clk); M_AXI_ARREADY = 1'b0;

        // Monitor ARVALID for 5 cycles - should be 0
        repeat(5) begin
            @(posedge clk);
            if (M_AXI_ARVALID) arvalid_count = arvalid_count + 1;
        end

        check(10, "ARVALID ghost pulses after handshake", arvalid_count, 0);

        // Complete the rest of the transfer to leave clean state
        fork
            begin
                axi_r_handshake(32'h1234_0000, 2'b00);
                fork
                    axi_aw_handshake(0);
                    axi_w_handshake(0);
                    axi_b_handshake(2'b00, 0);
                join
                wait_for_done_or_error(50);
            end
        join
    end
    send_ack();

    // =========================================================================
    // TEST 11 - AWVALID de-asserted after AW handshake
    // =========================================================================
    $display("\n--- Test 11: AWVALID de-asserted after AW handshake ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    begin : test11_block
        integer awvalid_count;
        awvalid_count = 0;

        fork
            begin
                pulse_start();
                axi_ar_handshake(0);
                axi_r_handshake(32'hABCD_0000, 2'b00);
                // Wait for AWVALID
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                // Complete AW handshake
                @(negedge clk); M_AXI_AWREADY = 1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY = 1'b0;
                // Monitor AWVALID for 5 cycles
                repeat(5) begin
                    @(posedge clk);
                    if (M_AXI_AWVALID) awvalid_count = awvalid_count + 1;
                end
                // Drain W and B
                axi_w_handshake(0);
                axi_b_handshake(2'b00, 0);
                wait_for_done_or_error(50);
            end
        join

        check(11, "AWVALID ghost pulses after handshake", awvalid_count, 0);
    end
    send_ack();

    // =========================================================================
    // TEST 12 - WLAST HIGH during write beat, LOW after W handshake
    // =========================================================================
    $display("\n--- Test 12: WLAST asserted exactly during write beat ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    begin : test12_block
        reg wlast_sampled_during;
        reg wlast_sampled_after;
        wlast_sampled_during = 1'b0;
        wlast_sampled_after  = 1'b0;

        fork
            begin
                pulse_start();
                axi_ar_handshake(0);
                axi_r_handshake(32'h1111_2222, 2'b00);
                axi_aw_handshake(0);
                // Wait for WVALID, sample WLAST before asserting WREADY
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                wlast_sampled_during = M_AXI_WLAST;
                // Complete W handshake
                @(negedge clk); M_AXI_WREADY = 1'b1;
                @(posedge clk);
                // Sample WLAST one cycle after handshake
                @(negedge clk); M_AXI_WREADY = 1'b0;
                @(posedge clk);
                wlast_sampled_after = M_AXI_WLAST;
                axi_b_handshake(2'b00, 0);
                wait_for_done_or_error(50);
            end
        join

        check(12, "WLAST=1 during write beat",   {31'b0, wlast_sampled_during}, 32'h1);
        check(12, "WLAST=0 after W handshake",   {31'b0, wlast_sampled_after},  32'h0);
    end
    send_ack();

    // =========================================================================
    // TEST 13 - WSTRB is 4'hF on every write beat
    // =========================================================================
    $display("\n--- Test 13: WSTRB = 4'hF on every write beat ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd8;   // 2 beats

    begin : test13_block
        reg [3:0] captured_wstrb [0:1];
        integer   beat;
        captured_wstrb[0] = 4'h0;
        captured_wstrb[1] = 4'h0;

        fork
            begin
                pulse_start();
                // Beat 0
                axi_ar_handshake(0);
                axi_r_handshake(32'hAAAA_AAAA, 2'b00);
                axi_aw_handshake(0);
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                captured_wstrb[0] = M_AXI_WSTRB;
                axi_w_handshake(0);
                axi_b_handshake(2'b00, 0);
                // Beat 1
                axi_ar_handshake(0);
                axi_r_handshake(32'hBBBB_BBBB, 2'b00);
                axi_aw_handshake(0);
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                captured_wstrb[1] = M_AXI_WSTRB;
                axi_w_handshake(0);
                axi_b_handshake(2'b00, 0);
                wait_for_done_or_error(100);
            end
        join

        check(13, "WSTRB beat0 = 4'hF", {28'b0, captured_wstrb[0]}, 32'hF);
        check(13, "WSTRB beat1 = 4'hF", {28'b0, captured_wstrb[1]}, 32'hF);
    end
    send_ack();

    // =========================================================================
    // TEST 14 - AXI channel attributes: ARLEN=0, ARSIZE=3'b010, ARBURST=2'b01
    //                                   AWLEN=0, AWSIZE=3'b010, AWBURST=2'b01
    // =========================================================================
    $display("\n--- Test 14: AXI channel attributes (LEN/SIZE/BURST) ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    begin : test14_block
        reg [7:0] cap_arlen;  reg [2:0] cap_arsize;  reg [1:0] cap_arburst;
        reg [7:0] cap_awlen;  reg [2:0] cap_awsize;  reg [1:0] cap_awburst;

        fork
            begin
                pulse_start();
                // Capture AR attrs when ARVALID is high
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                cap_arlen   = M_AXI_ARLEN;
                cap_arsize  = M_AXI_ARSIZE;
                cap_arburst = M_AXI_ARBURST;
                axi_ar_handshake(0);
                axi_r_handshake(32'h5A5A_5A5A, 2'b00);
                // Capture AW attrs
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                cap_awlen   = M_AXI_AWLEN;
                cap_awsize  = M_AXI_AWSIZE;
                cap_awburst = M_AXI_AWBURST;
                axi_aw_handshake(0);
                axi_w_handshake(0);
                axi_b_handshake(2'b00, 0);
                wait_for_done_or_error(50);
            end
        join

        check(14, "ARLEN  = 0",         {24'b0, cap_arlen},   32'h0);
        check(14, "ARSIZE = 3'b010",    {29'b0, cap_arsize},  32'h2);
        check(14, "ARBURST = INCR",     {30'b0, cap_arburst}, 32'h1);
        check(14, "AWLEN  = 0",         {24'b0, cap_awlen},   32'h0);
        check(14, "AWSIZE = 3'b010",    {29'b0, cap_awsize},  32'h2);
        check(14, "AWBURST = INCR",     {30'b0, cap_awburst}, 32'h1);
    end
    send_ack();

    // =========================================================================
    // TEST 15 - Back-to-back transfers: second start fires immediately after ack
    // =========================================================================
    $display("\n--- Test 15: Back-to-back transfers ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    // First transfer
    fork
        begin
            pulse_start();
            do_one_beat(32'h1111_1111, 2'b00, 2'b00, 0, 0, 0, 0);
            wait_for_done_or_error(100);
        end
        begin : dummy15a end
    join
    check(15, "first xfer done",  {31'b0, done},  32'h1);
    send_ack();
    repeat(2) @(posedge clk);
    check(15, "done cleared after ack", {31'b0, done}, 32'h0);

    // Immediately start second transfer (different src/dst)
    src_addr = 32'hC000_0000;
    dst_addr = 32'hD000_0000;
    fork
        begin
            pulse_start();
            do_one_beat(32'h2222_2222, 2'b00, 2'b00, 0, 0, 0, 0);
            wait_for_done_or_error(100);
        end
        begin : dummy15b end
    join
    check(15, "second xfer done", {31'b0, done},  32'h1);
    check(15, "WDATA second xfer", M_AXI_WDATA, 32'h2222_2222);
    send_ack();

    // =========================================================================
    // TEST 16 - Slow subordinate: ARREADY delayed 5 cycles
    // =========================================================================
    $display("\n--- Test 16: Slow subordinate - ARREADY delayed 5 cycles ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    fork
        begin
            pulse_start();
            do_one_beat(32'hDEAD_CAFE, 2'b00, 2'b00, 5, 0, 0, 0);
            wait_for_done_or_error(200);
        end
        begin : dummy16 end
    join
    check(16, "done with AR stall (5 cy)", {31'b0, done},  32'h1);
    check(16, "no error with AR stall",    {31'b0, error}, 32'h0);
    send_ack();

    // =========================================================================
    // TEST 17 - Slow subordinate: AWREADY delayed 5 cycles
    // =========================================================================
    $display("\n--- Test 17: Slow subordinate - AWREADY delayed 5 cycles ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    fork
        begin
            pulse_start();
            do_one_beat(32'hCAFE_0001, 2'b00, 2'b00, 0, 5, 0, 0);
            wait_for_done_or_error(200);
        end
        begin : dummy17 end
    join
    check(17, "done with AW stall (5 cy)", {31'b0, done},  32'h1);
    check(17, "no error with AW stall",    {31'b0, error}, 32'h0);
    send_ack();

    // =========================================================================
    // TEST 18 - Slow subordinate: WREADY delayed 5 cycles
    // =========================================================================
    $display("\n--- Test 18: Slow subordinate - WREADY delayed 5 cycles ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    fork
        begin
            pulse_start();
            do_one_beat(32'hCAFE_0002, 2'b00, 2'b00, 0, 0, 5, 0);
            wait_for_done_or_error(200);
        end
        begin : dummy18 end
    join
    check(18, "done with W stall (5 cy)",  {31'b0, done},  32'h1);
    check(18, "no error with W stall",     {31'b0, error}, 32'h0);
    send_ack();

    // =========================================================================
    // TEST 19 - Slow subordinate: BVALID delayed 5 cycles
    // =========================================================================
    $display("\n--- Test 19: Slow subordinate - BVALID delayed 5 cycles ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd4;

    fork
        begin
            pulse_start();
            do_one_beat(32'hCAFE_0003, 2'b00, 2'b00, 0, 0, 0, 5);
            wait_for_done_or_error(200);
        end
        begin : dummy19 end
    join
    check(19, "done with B stall (5 cy)",  {31'b0, done},  32'h1);
    check(19, "no error with B stall",     {31'b0, error}, 32'h0);
    send_ack();

    // =========================================================================
    // TEST 20 - data_buf integrity: each RDATA payload is forwarded to WDATA
    //           We check 3 beats with distinct RDATA patterns.
    // =========================================================================
    $display("\n--- Test 20: data_buf integrity (RDATA -> WDATA) ---");
    do_reset();
    src_addr     = 32'hA000_0000;
    dst_addr     = 32'hB000_0000;
    transfer_len = 32'd12;   // 3 beats

    begin : test20_block
        reg [DATA_WIDTH-1:0] captured_wdata [0:2];
        reg [DATA_WIDTH-1:0] rdata_pattern  [0:2];

        rdata_pattern[0] = 32'h1234_5678;
        rdata_pattern[1] = 32'hABCD_EF01;
        rdata_pattern[2] = 32'hDEAD_BEEF;

        captured_wdata[0] = 0;
        captured_wdata[1] = 0;
        captured_wdata[2] = 0;

        fork
            begin
                pulse_start();

                // Beat 0
                axi_ar_handshake(0);
                axi_r_handshake(rdata_pattern[0], 2'b00);
                axi_aw_handshake(0);
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                captured_wdata[0] = M_AXI_WDATA;
                axi_w_handshake(0);
                axi_b_handshake(2'b00, 0);

                // Beat 1
                axi_ar_handshake(0);
                axi_r_handshake(rdata_pattern[1], 2'b00);
                axi_aw_handshake(0);
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                captured_wdata[1] = M_AXI_WDATA;
                axi_w_handshake(0);
                axi_b_handshake(2'b00, 0);

                // Beat 2
                axi_ar_handshake(0);
                axi_r_handshake(rdata_pattern[2], 2'b00);
                axi_aw_handshake(0);
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                captured_wdata[2] = M_AXI_WDATA;
                axi_w_handshake(0);
                axi_b_handshake(2'b00, 0);

                wait_for_done_or_error(100);
            end
        join

        check(20, "WDATA beat0 == RDATA[0]", captured_wdata[0], rdata_pattern[0]);
        check(20, "WDATA beat1 == RDATA[1]", captured_wdata[1], rdata_pattern[1]);
        check(20, "WDATA beat2 == RDATA[2]", captured_wdata[2], rdata_pattern[2]);
    end
    send_ack();

    // =========================================================================
    // FINAL REPORT
    // =========================================================================
    $display("\n=============================================================");
    $display("  DMA ENGINE TESTBENCH COMPLETE");
    $display("  PASSED : %0d", pass_count);
    $display("  FAILED : %0d", fail_count);
    if (fail_count == 0)
        $display("  RESULT : ALL TESTS PASSED");
    else
        $display("  RESULT : SOME TESTS FAILED - review above");
    $display("=============================================================\n");

    $finish;
end

// =============================================================================
// Timeout Watchdog
// =============================================================================
initial begin
    #500000;
    $display("[TIMEOUT] Simulation exceeded 500us limit - possible handshake deadlock");
    $finish;
end

endmodule