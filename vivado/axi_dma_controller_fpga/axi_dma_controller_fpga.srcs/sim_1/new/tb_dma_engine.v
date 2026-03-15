`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/14/2026 07:39:30 PM
// Design Name: 
// Module Name: tb_dma_engine
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


// =============================================================================
// Module      : tb_dma_engine.v
// Description : Testbench for dma_engine.v
//
// Includes a simple AXI4 memory model that responds to master transactions.
//
// Tests:
//   Test 1  - Single word transfer (4 bytes), verify data copied correctly
//   Test 2  - Multi word transfer (16 bytes = 4 words)
//   Test 3  - Verify busy goes HIGH on start, LOW on done
//   Test 4  - Verify done goes HIGH after transfer, clears on ACK
//   Test 5  - Verify ARVALID deasserts after ARREADY handshake
//   Test 6  - Verify AWVALID deasserts after AWREADY handshake
//   Test 7  - Verify WLAST is 1 during WRITE_DATA and 0 after
//   Test 8  - Simulate RRESP error, verify error flag set
//   Test 9  - Simulate BRESP error, verify error flag set
//   Test 10 - Verify error clears after ACK
// =============================================================================


module tb_dma_engine;

// =============================================================================
// Parameters
// =============================================================================
parameter DATA_WIDTH  = 32;
parameter ADDR_WIDTH  = 32;
parameter CLK_PERIOD  = 10;         // 10ns = 100MHz
parameter MEM_SIZE    = 1024;       // 1024 words = 4KB memory model

// =============================================================================
// DUT Signal Declarations
// =============================================================================

// Global
reg  clk;
reg  rst_n;

// Control interface
reg  [ADDR_WIDTH-1:0] src_addr;
reg  [ADDR_WIDTH-1:0] dst_addr;
reg  [DATA_WIDTH-1:0] transfer_len;
reg                   start;
reg                   ack;

// Status interface
wire busy;
wire done;
wire error;

// AXI4 Read Address Channel
wire [ADDR_WIDTH-1:0] M_AXI_ARADDR;
wire                  M_AXI_ARVALID;
wire [7:0]            M_AXI_ARLEN;
wire [2:0]            M_AXI_ARSIZE;
wire [1:0]            M_AXI_ARBURST;
reg                   M_AXI_ARREADY;

// AXI4 Read Data Channel
reg  [DATA_WIDTH-1:0] M_AXI_RDATA;
reg  [1:0]            M_AXI_RRESP;
reg                   M_AXI_RVALID;
reg                   M_AXI_RLAST;
wire                  M_AXI_RREADY;

// AXI4 Write Address Channel
wire [ADDR_WIDTH-1:0] M_AXI_AWADDR;
wire                  M_AXI_AWVALID;
wire [7:0]            M_AXI_AWLEN;
wire [2:0]            M_AXI_AWSIZE;
wire [1:0]            M_AXI_AWBURST;
reg                   M_AXI_AWREADY;

// AXI4 Write Data Channel
wire [DATA_WIDTH-1:0] M_AXI_WDATA;
wire [3:0]            M_AXI_WSTRB;
wire                  M_AXI_WVALID;
wire                  M_AXI_WLAST;
reg                   M_AXI_WREADY;

// AXI4 Write Response Channel
reg  [1:0]            M_AXI_BRESP;
reg                   M_AXI_BVALID;
wire                  M_AXI_BREADY;

// =============================================================================
// Simple AXI Memory Model
// Two separate arrays - source and destination
// Source is pre-filled with test data
// Destination is checked after transfer
// =============================================================================
reg [DATA_WIDTH-1:0] src_mem [0:MEM_SIZE-1];   // source memory
reg [DATA_WIDTH-1:0] dst_mem [0:MEM_SIZE-1];   // destination memory

// Memory base addresses used in tests
localparam SRC_BASE = 32'h10000000;
localparam DST_BASE = 32'h20000000;

// =============================================================================
// Testbench internals
// =============================================================================
integer pass_count;
integer fail_count;
integer i;

// Force error flag for Tests 8 and 9
reg inject_rresp_error;     // forces RRESP = 2'b10 (SLVERR)
reg inject_bresp_error;     // forces BRESP = 2'b10 (SLVERR)

// =============================================================================
// DUT Instantiation
// =============================================================================
dma_engine #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) DUT (
    .clk            (clk),
    .rst_n          (rst_n),
    .src_addr       (src_addr),
    .dst_addr       (dst_addr),
    .transfer_len   (transfer_len),
    .start          (start),
    .ack            (ack),
    .busy           (busy),
    .done           (done),
    .error          (error),
    .M_AXI_ARADDR   (M_AXI_ARADDR),
    .M_AXI_ARVALID  (M_AXI_ARVALID),
    .M_AXI_ARLEN    (M_AXI_ARLEN),
    .M_AXI_ARSIZE   (M_AXI_ARSIZE),
    .M_AXI_ARBURST  (M_AXI_ARBURST),
    .M_AXI_ARREADY  (M_AXI_ARREADY),
    .M_AXI_RDATA    (M_AXI_RDATA),
    .M_AXI_RRESP    (M_AXI_RRESP),
    .M_AXI_RVALID   (M_AXI_RVALID),
    .M_AXI_RLAST    (M_AXI_RLAST),
    .M_AXI_RREADY   (M_AXI_RREADY),
    .M_AXI_AWADDR   (M_AXI_AWADDR),
    .M_AXI_AWVALID  (M_AXI_AWVALID),
    .M_AXI_AWLEN    (M_AXI_AWLEN),
    .M_AXI_AWSIZE   (M_AXI_AWSIZE),
    .M_AXI_AWBURST  (M_AXI_AWBURST),
    .M_AXI_AWREADY  (M_AXI_AWREADY),
    .M_AXI_WDATA    (M_AXI_WDATA),
    .M_AXI_WSTRB    (M_AXI_WSTRB),
    .M_AXI_WVALID   (M_AXI_WVALID),
    .M_AXI_WLAST    (M_AXI_WLAST),
    .M_AXI_WREADY   (M_AXI_WREADY),
    .M_AXI_BRESP    (M_AXI_BRESP),
    .M_AXI_BVALID   (M_AXI_BVALID),
    .M_AXI_BREADY   (M_AXI_BREADY)
);

// =============================================================================
// Clock Generation
// =============================================================================
initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// =============================================================================
// AXI Memory Model - Read Side
// Responds to ARVALID by asserting ARREADY, then serves RDATA from src_mem
// Word index = (ARADDR - SRC_BASE) / 4
// =============================================================================
reg [ADDR_WIDTH-1:0] rd_addr_latch;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        M_AXI_ARREADY <= 1'b0;
        M_AXI_RVALID  <= 1'b0;
        M_AXI_RDATA   <= 32'h0;
        M_AXI_RRESP   <= 2'b00;
        M_AXI_RLAST   <= 1'b0;
        rd_addr_latch <= 32'h0;
    end else begin

        // Default deassert
        M_AXI_ARREADY <= 1'b0;
        M_AXI_RVALID  <= 1'b0;
        M_AXI_RLAST   <= 1'b0;

        // Accept read address
        if (M_AXI_ARVALID && !M_AXI_ARREADY) begin
            M_AXI_ARREADY <= 1'b1;
            rd_addr_latch <= M_AXI_ARADDR;
        end

        // Serve read data one cycle after accepting address
        if (M_AXI_ARREADY) begin
            M_AXI_RVALID <= 1'b1;
            M_AXI_RLAST  <= 1'b1;   // always last for single beat

            if (inject_rresp_error) begin
                M_AXI_RRESP <= 2'b10;   // SLVERR - for Test 8
                M_AXI_RDATA <= 32'hDEADBEEF;
            end else begin
                M_AXI_RRESP <= 2'b00;   // OKAY
                // Word index into src_mem array
                M_AXI_RDATA <= src_mem[(rd_addr_latch - SRC_BASE) >> 2];
            end
        end

        // Deassert after handshake
        if (M_AXI_RVALID && M_AXI_RREADY) begin
            M_AXI_RVALID <= 1'b0;
            M_AXI_RLAST  <= 1'b0;
        end

    end
end

// =============================================================================
// AXI Memory Model - Write Side
// Responds to AWVALID + WVALID, writes data into dst_mem, sends BRESP
// Word index = (AWADDR - DST_BASE) / 4
// =============================================================================
reg [ADDR_WIDTH-1:0] wr_addr_latch;
reg                  wr_addr_valid;
reg                  wr_data_pending;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        M_AXI_AWREADY   <= 1'b0;
        M_AXI_WREADY    <= 1'b0;
        M_AXI_BVALID    <= 1'b0;
        M_AXI_BRESP     <= 2'b00;
        wr_addr_latch   <= 32'h0;
        wr_addr_valid   <= 1'b0;
        wr_data_pending <= 1'b0;
    end else begin

        // Defaults
        M_AXI_AWREADY <= 1'b0;
        M_AXI_WREADY  <= 1'b0;

        // Accept write address
        if (M_AXI_AWVALID && !M_AXI_AWREADY && !wr_addr_valid) begin
            M_AXI_AWREADY <= 1'b1;
            wr_addr_latch <= M_AXI_AWADDR;
            wr_addr_valid <= 1'b1;
        end

        // Accept write data - only after address is latched
        if (wr_addr_valid && M_AXI_WVALID && !M_AXI_WREADY) begin
            M_AXI_WREADY <= 1'b1;
            // Write to destination memory
            dst_mem[(wr_addr_latch - DST_BASE) >> 2] <= M_AXI_WDATA;
            wr_data_pending <= 1'b1;
        end

        // Send write response after data is accepted
        if (wr_data_pending && !M_AXI_BVALID) begin
            M_AXI_BVALID    <= 1'b1;
            M_AXI_BRESP     <= inject_bresp_error ? 2'b10 : 2'b00;
            wr_addr_valid   <= 1'b0;
            wr_data_pending <= 1'b0;
        end

        // Deassert after response handshake
        if (M_AXI_BVALID && M_AXI_BREADY) begin
            M_AXI_BVALID <= 1'b0;
        end

    end
end

// =============================================================================
// Task: check
// =============================================================================
task check;
    input integer     test_id;
    input reg [255:0] test_name;
    input [31:0]      actual;
    input [31:0]      expected;
    begin
        if (actual === expected) begin
            $display("  [PASS] Test %0d  %s", test_id, test_name);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] Test %0d  %s | got 0x%08X  expected 0x%08X",
                     test_id, test_name, actual, expected);
            fail_count = fail_count + 1;
        end
    end
endtask

// =============================================================================
// Task: trigger_dma
// Asserts start for 1 cycle then waits for done or error
// =============================================================================
task trigger_dma;
    input [ADDR_WIDTH-1:0] s_addr;
    input [ADDR_WIDTH-1:0] d_addr;
    input [DATA_WIDTH-1:0] len;
    begin
        src_addr     = s_addr;
        dst_addr     = d_addr;
        transfer_len = len;

        @(posedge clk); #1;
        start = 1'b1;
        @(posedge clk); #1;
        start = 1'b0;

        // Wait for done or error (with timeout)
        fork
            begin : wait_done
                @(posedge done or posedge error);
            end
            begin : timeout
                #100000;
                $display("  [TIMEOUT] DMA did not complete in time");
                disable wait_done;
            end
        join

        @(posedge clk); #1;
    end
endtask

// =============================================================================
// Task: send_ack
// Pulses ack for 1 cycle to let DMA return to IDLE
// =============================================================================
task send_ack;
    begin
        @(posedge clk); #1;
        ack = 1'b1;
        @(posedge clk); #1;
        ack = 1'b0;
        @(posedge clk);
    end
endtask

// =============================================================================
// Task: init_src_memory
// Fills source memory with known pattern: src_mem[i] = 0xA0000000 + i
// =============================================================================
task init_src_memory;
    input integer num_words;
    integer j;
    begin
        for (j = 0; j < num_words; j = j + 1) begin
            src_mem[j] = 32'hA0000000 + j;
            dst_mem[j] = 32'h00000000;    // clear destination
        end
    end
endtask

// =============================================================================
// Main Test Sequence
// =============================================================================
initial begin

    // Initialise
    rst_n              = 1'b0;
    start              = 1'b0;
    ack                = 1'b0;
    src_addr           = 32'h0;
    dst_addr           = 32'h0;
    transfer_len       = 32'h0;
    inject_rresp_error = 1'b0;
    inject_bresp_error = 1'b0;
    pass_count         = 0;
    fail_count         = 0;

    // Clear memory
    for (i = 0; i < MEM_SIZE; i = i + 1) begin
        src_mem[i] = 32'h0;
        dst_mem[i] = 32'h0;
    end

    $display("=============================================================");
    $display("   DMA Engine Testbench");
    $display("=============================================================");

    // Apply reset
    repeat(5) @(posedge clk);
    rst_n = 1'b1;
    repeat(3) @(posedge clk);
    $display("\n  Reset released. Starting tests...\n");

    // =========================================================================
    // TEST 1 - Single word transfer (4 bytes = 1 word)
    // src_mem[0] = 0xA0000000 must appear in dst_mem[0] after transfer
    // =========================================================================
    $display("--- Test 1: Single word transfer (4 bytes) ---");
    init_src_memory(1);
    src_mem[0] = 32'hDEADC0DE;

    trigger_dma(SRC_BASE, DST_BASE, 32'h4);   // 4 bytes

    check(1, "dst_mem[0] matches src_mem[0]", dst_mem[0], 32'hDEADC0DE);
    check(1, "busy is 0 after transfer", busy, 1'b0);

    send_ack;
    check(1, "done clears after ack", done, 1'b0);

    // =========================================================================
    // TEST 2 - Multi word transfer (16 bytes = 4 words)
    // Each word must be copied to the correct destination address
    // =========================================================================
    $display("--- Test 2: Multi word transfer (16 bytes = 4 words) ---");
    init_src_memory(4);

    trigger_dma(SRC_BASE, DST_BASE, 32'h10);  // 16 bytes = 4 words

    check(2, "dst_mem[0] = 0xA0000000", dst_mem[0], 32'hA0000000);
    check(2, "dst_mem[1] = 0xA0000001", dst_mem[1], 32'hA0000001);
    check(2, "dst_mem[2] = 0xA0000002", dst_mem[2], 32'hA0000002);
    check(2, "dst_mem[3] = 0xA0000003", dst_mem[3], 32'hA0000003);

    send_ack;

    // =========================================================================
    // TEST 3 - busy goes HIGH on start, LOW after done
    // =========================================================================
    $display("--- Test 3: busy signal behaviour ---");
    init_src_memory(1);

    // Before start - busy must be 0
    @(posedge clk);
    check(3, "busy = 0 before start", busy, 1'b0);

    // Pulse start and immediately check busy
    @(posedge clk); #1;
    start = 1'b1;
    @(posedge clk); #1;
    start = 1'b0;
    // Give 1 cycle for Block 3 to register
    @(posedge clk);
    check(3, "busy = 1 after start pulse", busy, 1'b1);

    // Wait for done
    @(posedge done or posedge error);
    @(posedge clk);
    check(3, "busy = 0 when done", busy, 1'b0);
    send_ack;

    // =========================================================================
    // TEST 4 - done goes HIGH after transfer, clears after ACK
    // =========================================================================
    $display("--- Test 4: done flag behaviour ---");
    init_src_memory(1);

    trigger_dma(SRC_BASE, DST_BASE, 32'h4);

    check(4, "done = 1 after transfer", done, 1'b1);
    check(4, "error = 0 on clean transfer", error, 1'b0);

    send_ack;
    check(4, "done = 0 after ack", done, 1'b0);

    // =========================================================================
    // TEST 5 - ARVALID deasserts after ARREADY handshake
    // =========================================================================
    $display("--- Test 5: ARVALID deasserts after handshake ---");
    init_src_memory(1);

    @(posedge clk); #1;
    start = 1'b1;
    @(posedge clk); #1;
    start = 1'b0;

    // Wait until ARVALID goes high
    @(posedge M_AXI_ARVALID);
    // Wait until ARREADY handshake completes
    @(posedge M_AXI_ARREADY);
    // Next cycle ARVALID should deassert
    @(posedge clk);
    check(5, "ARVALID = 0 after ARREADY handshake", M_AXI_ARVALID, 1'b0);

    @(posedge done or posedge error);
    send_ack;

    // =========================================================================
    // TEST 6 - AWVALID deasserts after AWREADY handshake
    // =========================================================================
    $display("--- Test 6: AWVALID deasserts after handshake ---");
    init_src_memory(1);

    @(posedge clk); #1;
    start = 1'b1;
    @(posedge clk); #1;
    start = 1'b0;

    @(posedge M_AXI_AWVALID);
    @(posedge M_AXI_AWREADY);
    @(posedge clk);
    check(6, "AWVALID = 0 after AWREADY handshake", M_AXI_AWVALID, 1'b0);

    @(posedge done or posedge error);
    send_ack;

    // =========================================================================
    // TEST 7 - WLAST = 1 during WRITE_DATA, 0 after handshake
    // =========================================================================
    $display("--- Test 7: WLAST asserts during WRITE_DATA ---");
    init_src_memory(1);

    @(posedge clk); #1;
    start = 1'b1;
    @(posedge clk); #1;
    start = 1'b0;

    // Wait for WVALID (WRITE_DATA state)
    @(posedge M_AXI_WVALID);
    @(posedge clk);
    check(7, "WLAST = 1 during WRITE_DATA", M_AXI_WLAST, 1'b1);

    // Wait for WREADY handshake
    @(posedge M_AXI_WREADY);
    @(posedge clk);
    check(7, "WLAST = 0 after WREADY handshake", M_AXI_WLAST, 1'b0);

    @(posedge done or posedge error);
    send_ack;

    // =========================================================================
    // TEST 8 - Inject RRESP error, verify error flag is set
    // =========================================================================
    $display("--- Test 8: RRESP error sets error flag ---");
    init_src_memory(1);
    inject_rresp_error = 1'b1;    // memory model will return RRESP=2'b10

    trigger_dma(SRC_BASE, DST_BASE, 32'h4);

    check(8, "error = 1 after RRESP error", error, 1'b1);
    check(8, "done = 0 on error", done, 1'b0);
    check(8, "busy = 0 in ERROR state", busy, 1'b0);

    inject_rresp_error = 1'b0;    // clear injection
    send_ack;

    // =========================================================================
    // TEST 9 - Inject BRESP error, verify error flag is set
    // =========================================================================
    $display("--- Test 9: BRESP error sets error flag ---");
    init_src_memory(1);
    inject_bresp_error = 1'b1;    // memory model will return BRESP=2'b10

    trigger_dma(SRC_BASE, DST_BASE, 32'h4);

    check(9, "error = 1 after BRESP error", error, 1'b1);
    check(9, "done = 0 on error", done, 1'b0);
    check(9, "busy = 0 in ERROR state", busy, 1'b0);

    inject_bresp_error = 1'b0;
    send_ack;

    // =========================================================================
    // TEST 10 - Error clears after ACK, DMA returns to IDLE
    // =========================================================================
    $display("--- Test 10: error clears after ACK ---");
    init_src_memory(1);
    inject_rresp_error = 1'b1;

    trigger_dma(SRC_BASE, DST_BASE, 32'h4);
    check(10, "error = 1 before ACK", error, 1'b1);

    inject_rresp_error = 1'b0;
    send_ack;
    check(10, "error = 0 after ACK", error, 1'b0);
    check(10, "busy = 0 after ACK", busy, 1'b0);

    // Verify DMA works again after error+ack (returned to IDLE)
    @(posedge clk);
    init_src_memory(1);
    src_mem[0] = 32'hBEEFCAFE;
    trigger_dma(SRC_BASE, DST_BASE, 32'h4);
    check(10, "DMA works after error recovery", dst_mem[0], 32'hBEEFCAFE);
    send_ack;

    // =========================================================================
    // FINAL REPORT
    // =========================================================================
    repeat(5) @(posedge clk);
    $display("\n=============================================================");
    $display("   DMA ENGINE TESTBENCH COMPLETE");
    $display("   PASSED  : %0d", pass_count);
    $display("   FAILED  : %0d", fail_count);
    $display("   TOTAL   : %0d", pass_count + fail_count);
    if (fail_count == 0)
        $display("   RESULT  : ALL TESTS PASSED -- ready for dma_top integration");
    else
        $display("   RESULT  : %0d TESTS FAILED -- open waveform and debug", fail_count);
    $display("=============================================================\n");

    $finish;
end

// =============================================================================
// Timeout Watchdog
// =============================================================================
initial begin
    #2000000;
    $display("[TIMEOUT] Simulation exceeded time limit -- FSM may be stuck");
    $finish;
end

// =============================================================================
// Waveform Dump - for GTKWave
// Remove if using Vivado XSIM
// =============================================================================
initial begin
    $dumpfile("tb_dma_engine.vcd");
    $dumpvars(0, tb_dma_engine);
end

endmodule