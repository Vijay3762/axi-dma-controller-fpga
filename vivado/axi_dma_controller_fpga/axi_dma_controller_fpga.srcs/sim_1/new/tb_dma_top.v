`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/18/2026 12:45:56 PM
// Design Name: 
// Module Name: tb_dma_top
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
// Module      : tb_dma_top.v
// Description : Integration testbench for dma_top.v
//               Exercises the full PS→AXI-Lite→DMA Engine→DDR path end-to-end.
//               The testbench behaves as two actors simultaneously:
//                 1. PS Master  - drives AXI-Lite writes/reads to control registers
//                 2. DDR Memory - responds to AXI4 Full master transactions
//
// Tests performed:
//   Test 1  - Reset: all outputs idle, STATUS = 0x0
//   Test 2  - Write all three config registers, read them back via AXI-Lite
//   Test 3  - Single-word transfer (4 bytes): write START, poll STATUS=DONE, write ACK
//   Test 4  - Multi-word transfer (16 bytes = 4 iterations), verify done
//   Test 5  - Read error path: DDR returns RRESP=SLVERR, STATUS must show ERROR
//   Test 6  - Write error path: DDR returns BRESP=SLVERR, STATUS must show ERROR
//   Test 7  - ACK clears DONE, STATUS returns to idle
//   Test 8  - ACK clears ERROR, STATUS returns to idle
//   Test 9  - BUSY bit set immediately after START, before transfer completes
//   Test 10 - Back-to-back transfers: second START after first ACK
//   Test 11 - STATUS register is read-only (write attempt is silently ignored)
//   Test 12 - Slow DDR: ARREADY/AWREADY/WREADY/BVALID each delayed 5 cycles
//   Test 13 - data integrity: RDATA payload reaches WDATA correctly (3 patterns)
//   Test 14 - AXI4 Full channel attributes: ARLEN=0, ARSIZE=4B, ARBURST=INCR
//   Test 15 - WSTRB=4'hF and WLAST=1 on every write beat
// =============================================================================

`timescale 1ns / 1ps

module tb_dma_top;

// =============================================================================
// Parameters
// =============================================================================
parameter DATA_WIDTH = 32;
parameter ADDR_WIDTH = 32;
parameter CLK_PERIOD = 10;   // 10 ns = 100 MHz

// =============================================================================
// Register Map (must match axi_lite_slave)
// =============================================================================
localparam REG_SRC_ADDR     = 32'h00;
localparam REG_DST_ADDR     = 32'h04;
localparam REG_TRANSFER_LEN = 32'h08;
localparam REG_CONTROL      = 32'h0C;
localparam REG_STATUS       = 32'h10;

localparam CTRL_START = 32'h0000_0001;  // CONTROL[0]
localparam CTRL_ACK   = 32'h0000_0004;  // CONTROL[2]

localparam STATUS_BUSY  = 32'h0000_0001;
localparam STATUS_DONE  = 32'h0000_0002;
localparam STATUS_ERROR = 32'h0000_0004;

// =============================================================================
// DUT Signal Declarations
// =============================================================================

// Global
reg clk;
reg rst_n;

// AXI-Lite Slave (PS side)
reg  [ADDR_WIDTH-1:0]     S_AXI_AWADDR;
reg                       S_AXI_AWVALID;
wire                      S_AXI_AWREADY;
reg  [DATA_WIDTH-1:0]     S_AXI_WDATA;
reg  [(DATA_WIDTH/8)-1:0] S_AXI_WSTRB;
reg                       S_AXI_WVALID;
wire                      S_AXI_WREADY;
wire [1:0]                S_AXI_BRESP;
wire                      S_AXI_BVALID;
reg                       S_AXI_BREADY;
reg  [ADDR_WIDTH-1:0]     S_AXI_ARADDR;
reg                       S_AXI_ARVALID;
wire                      S_AXI_ARREADY;
wire [DATA_WIDTH-1:0]     S_AXI_RDATA;
wire [1:0]                S_AXI_RRESP;
wire                      S_AXI_RVALID;
reg                       S_AXI_RREADY;

// AXI4 Full Master (DDR side)
wire [ADDR_WIDTH-1:0]     M_AXI_ARADDR;
wire                      M_AXI_ARVALID;
wire [7:0]                M_AXI_ARLEN;
wire [2:0]                M_AXI_ARSIZE;
wire [1:0]                M_AXI_ARBURST;
reg                       M_AXI_ARREADY;
reg  [DATA_WIDTH-1:0]     M_AXI_RDATA;
reg  [1:0]                M_AXI_RRESP;
reg                       M_AXI_RVALID;
reg                       M_AXI_RLAST;
wire                      M_AXI_RREADY;
wire [ADDR_WIDTH-1:0]     M_AXI_AWADDR;
wire                      M_AXI_AWVALID;
wire [7:0]                M_AXI_AWLEN;
wire [2:0]                M_AXI_AWSIZE;
wire [1:0]                M_AXI_AWBURST;
reg                       M_AXI_AWREADY;
wire [DATA_WIDTH-1:0]     M_AXI_WDATA;
wire [(DATA_WIDTH/8)-1:0] M_AXI_WSTRB;
wire                      M_AXI_WVALID;
wire                      M_AXI_WLAST;
reg                       M_AXI_WREADY;
reg  [1:0]                M_AXI_BRESP;
reg                       M_AXI_BVALID;
wire                      M_AXI_BREADY;

// =============================================================================
// Test Tracking
// =============================================================================
integer pass_count;
integer fail_count;
reg [DATA_WIDTH-1:0] read_data;   // result of each AXI-Lite read

// =============================================================================
// DUT Instantiation
// =============================================================================
dma_top #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) DUT (
    .clk            (clk),
    .rst_n          (rst_n),
    .S_AXI_AWADDR   (S_AXI_AWADDR),
    .S_AXI_AWVALID  (S_AXI_AWVALID),
    .S_AXI_AWREADY  (S_AXI_AWREADY),
    .S_AXI_WDATA    (S_AXI_WDATA),
    .S_AXI_WSTRB    (S_AXI_WSTRB),
    .S_AXI_WVALID   (S_AXI_WVALID),
    .S_AXI_WREADY   (S_AXI_WREADY),
    .S_AXI_BRESP    (S_AXI_BRESP),
    .S_AXI_BVALID   (S_AXI_BVALID),
    .S_AXI_BREADY   (S_AXI_BREADY),
    .S_AXI_ARADDR   (S_AXI_ARADDR),
    .S_AXI_ARVALID  (S_AXI_ARVALID),
    .S_AXI_ARREADY  (S_AXI_ARREADY),
    .S_AXI_RDATA    (S_AXI_RDATA),
    .S_AXI_RRESP    (S_AXI_RRESP),
    .S_AXI_RVALID   (S_AXI_RVALID),
    .S_AXI_RREADY   (S_AXI_RREADY),
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
// Clock
// =============================================================================
initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// =============================================================================
// Waveform Dump
// =============================================================================
initial begin
    $dumpfile("tb_dma_top.vcd");
    $dumpvars(0, tb_dma_top);
end

// =============================================================================
// Task: check
// =============================================================================
task check;
    input integer     test_id;
    input reg [255:0] test_name;
    input reg [63:0]  actual;
    input reg [63:0]  expected;
    begin
        if (actual === expected) begin
            $display("  [PASS] T%0d - %s | got 0x%0X", test_id, test_name, actual);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] T%0d - %s | got 0x%0X  expected 0x%0X",
                     test_id, test_name, actual, expected);
            fail_count = fail_count + 1;
        end
    end
endtask

// =============================================================================
// Task: do_reset
// =============================================================================
task do_reset;
    begin
        rst_n          = 1'b0;
        S_AXI_AWADDR   = 0; S_AXI_AWVALID = 0;
        S_AXI_WDATA    = 0; S_AXI_WSTRB   = 4'hF; S_AXI_WVALID = 0;
        S_AXI_BREADY   = 0;
        S_AXI_ARADDR   = 0; S_AXI_ARVALID = 0;
        S_AXI_RREADY   = 0;
        M_AXI_ARREADY  = 0;
        M_AXI_RDATA    = 0; M_AXI_RRESP = 2'b00; M_AXI_RVALID = 0; M_AXI_RLAST = 0;
        M_AXI_AWREADY  = 0;
        M_AXI_WREADY   = 0;
        M_AXI_BRESP    = 2'b00; M_AXI_BVALID = 0;
        repeat(5) @(posedge clk);
        rst_n = 1'b1;
        repeat(2) @(posedge clk);
    end
endtask

// =============================================================================
// Task: ps_write  - PS writes one register via AXI-Lite
// =============================================================================
task ps_write;
    input [ADDR_WIDTH-1:0]     addr;
    input [DATA_WIDTH-1:0]     data;
    input [(DATA_WIDTH/8)-1:0] strb;
    begin
        @(posedge clk); #1;
        S_AXI_AWADDR  = addr;
        S_AXI_AWVALID = 1'b1;
        S_AXI_WDATA   = data;
        S_AXI_WSTRB   = strb;
        S_AXI_WVALID  = 1'b1;
        @(posedge clk); #1;
        S_AXI_AWVALID = 1'b0;
        S_AXI_WVALID  = 1'b0;
        S_AXI_BREADY  = 1'b1;
        @(posedge clk);
        while (!S_AXI_BVALID) @(posedge clk);
        @(posedge clk); #1;
        S_AXI_BREADY = 1'b0;
    end
endtask

// =============================================================================
// Task: ps_read  - PS reads one register via AXI-Lite
// Result stored in read_data
// =============================================================================
task ps_read;
    input [ADDR_WIDTH-1:0] addr;
    begin
        @(posedge clk); #1;
        S_AXI_ARADDR  = addr;
        S_AXI_ARVALID = 1'b1;
        @(posedge clk);
        while (!S_AXI_ARREADY) @(posedge clk);
        #1; S_AXI_ARVALID = 1'b0;
        S_AXI_RREADY = 1'b1;
        @(posedge clk);
        while (!S_AXI_RVALID) @(posedge clk);
        read_data = S_AXI_RDATA;
        @(posedge clk); #1;
        S_AXI_RREADY = 1'b0;
    end
endtask

// =============================================================================
// Task: ps_poll_status
// Keeps reading STATUS until the expected bits are set, or times out.
// =============================================================================
task ps_poll_status;
    input [DATA_WIDTH-1:0] expected_mask;
    input integer          timeout;
    integer                i;
    begin
        i = 0;
        read_data = 0;
        while (((read_data & expected_mask) !== expected_mask) && i < timeout) begin
            ps_read(REG_STATUS);
            i = i + 1;
        end
        if (i >= timeout) begin
            $display("  [TIMEOUT] ps_poll_status: 0x%0X not seen after %0d polls",
                     expected_mask, timeout);
            fail_count = fail_count + 1;
        end
    end
endtask

// =============================================================================
// Task: ddr_serve_one_beat
// Drives a single AR→R→AW→W→B sequence from the DDR memory side.
// rdata    : payload to return on R channel
// rresp    : 2'b00=OKAY  2'b10=SLVERR
// bresp    : 2'b00=OKAY  2'b10=SLVERR
// ar_dly, aw_dly, w_dly, b_dly : stall cycles before each handshake
// =============================================================================
task ddr_serve_one_beat;
    input [DATA_WIDTH-1:0] rdata;
    input [1:0]            rresp;
    input [1:0]            bresp;
    input integer          ar_dly;
    input integer          aw_dly;
    input integer          w_dly;
    input integer          b_dly;
    begin
        // ---- AR handshake ----
        @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
        repeat(ar_dly) @(posedge clk);
        @(negedge clk); M_AXI_ARREADY = 1'b1;
        @(posedge clk); @(negedge clk); M_AXI_ARREADY = 1'b0;

        // ---- R handshake ----
        @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
        @(negedge clk);
        M_AXI_RDATA  = rdata;
        M_AXI_RRESP  = rresp;
        M_AXI_RVALID = 1'b1;
        M_AXI_RLAST  = 1'b1;
        @(posedge clk); @(negedge clk);
        M_AXI_RVALID = 1'b0;
        M_AXI_RLAST  = 1'b0;

        // ---- Only continue to write channels if read was OK ----
        if (rresp == 2'b00) begin
            // ---- AW handshake ----
            @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
            repeat(aw_dly) @(posedge clk);
            @(negedge clk); M_AXI_AWREADY = 1'b1;
            @(posedge clk); @(negedge clk); M_AXI_AWREADY = 1'b0;

            // ---- W handshake ----
            @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
            repeat(w_dly) @(posedge clk);
            @(negedge clk); M_AXI_WREADY = 1'b1;
            @(posedge clk); @(negedge clk); M_AXI_WREADY = 1'b0;

            // ---- B handshake ----
            @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
            repeat(b_dly) @(posedge clk);
            @(negedge clk);
            M_AXI_BRESP  = bresp;
            M_AXI_BVALID = 1'b1;
            @(posedge clk); @(negedge clk);
            M_AXI_BVALID = 1'b0;
        end
    end
endtask

// =============================================================================
// Task: setup_transfer
// Writes SRC, DST, LEN registers via AXI-Lite - no START yet.
// =============================================================================
task setup_transfer;
    input [ADDR_WIDTH-1:0] src;
    input [ADDR_WIDTH-1:0] dst;
    input [DATA_WIDTH-1:0] len;
    begin
        ps_write(REG_SRC_ADDR,     src, 4'hF);
        ps_write(REG_DST_ADDR,     dst, 4'hF);
        ps_write(REG_TRANSFER_LEN, len, 4'hF);
    end
endtask

// =============================================================================
// Task: fire_start
// Writes CONTROL[0]=1 via AXI-Lite.
// =============================================================================
task fire_start;
    begin
        ps_write(REG_CONTROL, CTRL_START, 4'hF);
    end
endtask

// =============================================================================
// Task: fire_ack
// Writes CONTROL[2]=1 via AXI-Lite.
// =============================================================================
task fire_ack;
    begin
        ps_write(REG_CONTROL, CTRL_ACK, 4'hF);
    end
endtask

// =============================================================================
// MAIN TEST SEQUENCE
// =============================================================================
initial begin
    pass_count = 0;
    fail_count = 0;

    $display("=============================================================");
    $display("  dma_top Integration Testbench Starting");
    $display("=============================================================");

    // =========================================================================
    // TEST 1 - Reset: STATUS = 0, all AXI4 master outputs idle
    // =========================================================================
    $display("\n--- Test 1: Reset behavior ---");
    do_reset();

    ps_read(REG_STATUS);
    check(1, "STATUS=0 after reset",   read_data,                 32'h0);
    check(1, "M_AXI_ARVALID=0",        {31'b0, M_AXI_ARVALID},   32'h0);
    check(1, "M_AXI_AWVALID=0",        {31'b0, M_AXI_AWVALID},   32'h0);
    check(1, "M_AXI_WVALID=0",         {31'b0, M_AXI_WVALID},    32'h0);
    check(1, "M_AXI_BREADY=0",         {31'b0, M_AXI_BREADY},    32'h0);
    check(1, "M_AXI_RREADY=0",         {31'b0, M_AXI_RREADY},    32'h0);

    // =========================================================================
    // TEST 2 - Write and read back all config registers via AXI-Lite
    // =========================================================================
    $display("\n--- Test 2: Config register write/readback ---");
    do_reset();

    ps_write(REG_SRC_ADDR,     32'hA000_0000, 4'hF);
    ps_write(REG_DST_ADDR,     32'hB000_0000, 4'hF);
    ps_write(REG_TRANSFER_LEN, 32'h0000_0400, 4'hF);  // 1024 bytes

    ps_read(REG_SRC_ADDR);
    check(2, "SRC_ADDR readback",      read_data, 32'hA000_0000);
    ps_read(REG_DST_ADDR);
    check(2, "DST_ADDR readback",      read_data, 32'hB000_0000);
    ps_read(REG_TRANSFER_LEN);
    check(2, "TRANSFER_LEN readback",  read_data, 32'h0000_0400);

    // =========================================================================
    // TEST 3 - Single-word transfer (4 bytes):
    //          PS writes START → DDR serves one beat → PS polls DONE → PS writes ACK
    // =========================================================================
    $display("\n--- Test 3: Single-word transfer (4 bytes) end-to-end ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd4);

    fork
        // PS thread
        begin
            fire_start();
            ps_poll_status(STATUS_DONE, 200);
            check(3, "STATUS=DONE after 4-byte xfer", read_data & STATUS_DONE, STATUS_DONE);
            check(3, "BUSY cleared at DONE",           read_data & STATUS_BUSY, 32'h0);
            check(3, "ERROR not set",                  read_data & STATUS_ERROR, 32'h0);
            fire_ack();
            repeat(4) @(posedge clk);
            ps_read(REG_STATUS);
            check(3, "STATUS=0 after ACK", read_data, 32'h0);
        end
        // DDR thread
        begin
            ddr_serve_one_beat(32'hCAFE_BABE, 2'b00, 2'b00, 0, 0, 0, 0);
        end
    join

    // =========================================================================
    // TEST 4 - Multi-word transfer (16 bytes = 4 iterations)
    // =========================================================================
    $display("\n--- Test 4: Multi-word transfer (16 bytes) ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd16);

    fork
        begin
            fire_start();
            ps_poll_status(STATUS_DONE, 500);
            check(4, "STATUS=DONE after 16-byte xfer", read_data & STATUS_DONE,  STATUS_DONE);
            check(4, "ERROR not set",                   read_data & STATUS_ERROR, 32'h0);
            fire_ack();
        end
        begin
            ddr_serve_one_beat(32'hDEAD_0001, 2'b00, 2'b00, 0, 0, 0, 0);
            ddr_serve_one_beat(32'hDEAD_0002, 2'b00, 2'b00, 0, 0, 0, 0);
            ddr_serve_one_beat(32'hDEAD_0003, 2'b00, 2'b00, 0, 0, 0, 0);
            ddr_serve_one_beat(32'hDEAD_0004, 2'b00, 2'b00, 0, 0, 0, 0);
        end
    join

    // =========================================================================
    // TEST 5 - Read error: DDR returns RRESP=SLVERR → STATUS must show ERROR
    // =========================================================================
    $display("\n--- Test 5: Read error (RRESP=SLVERR) ---");
    do_reset();
    setup_transfer(32'hDEAD_0000, 32'hBEEF_0000, 32'd4);

    fork
        begin
            fire_start();
            ps_poll_status(STATUS_ERROR, 200);
            check(5, "STATUS=ERROR on RRESP=SLVERR", read_data & STATUS_ERROR, STATUS_ERROR);
            check(5, "DONE not set on read error",   read_data & STATUS_DONE,  32'h0);
            check(5, "BUSY cleared on error",        read_data & STATUS_BUSY,  32'h0);
            fire_ack();
            repeat(4) @(posedge clk);
            ps_read(REG_STATUS);
            check(5, "STATUS=0 after ERROR+ACK", read_data, 32'h0);
        end
        begin
            // AR handshake
            @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
            @(negedge clk); M_AXI_ARREADY = 1'b1;
            @(posedge clk); @(negedge clk); M_AXI_ARREADY = 1'b0;
            // R with SLVERR
            @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
            @(negedge clk);
            M_AXI_RDATA = 32'hBAD1_BAD1; M_AXI_RRESP = 2'b10;
            M_AXI_RVALID = 1'b1; M_AXI_RLAST = 1'b1;
            @(posedge clk); @(negedge clk);
            M_AXI_RVALID = 1'b0; M_AXI_RLAST = 1'b0;
        end
    join

    // =========================================================================
    // TEST 6 - Write error: DDR returns BRESP=SLVERR → STATUS must show ERROR
    // =========================================================================
    $display("\n--- Test 6: Write error (BRESP=SLVERR) ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd4);

    fork
        begin
            fire_start();
            ps_poll_status(STATUS_ERROR, 200);
            check(6, "STATUS=ERROR on BRESP=SLVERR", read_data & STATUS_ERROR, STATUS_ERROR);
            check(6, "DONE not set on write error",  read_data & STATUS_DONE,  32'h0);
            fire_ack();
        end
        begin
            ddr_serve_one_beat(32'h1234_5678, 2'b00, 2'b10, 0, 0, 0, 0);
        end
    join

    // =========================================================================
    // TEST 7 - ACK clears DONE: second ps_read(STATUS) must return 0
    // =========================================================================
    $display("\n--- Test 7: ACK clears DONE ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd4);

    fork
        begin
            fire_start();
            ps_poll_status(STATUS_DONE, 200);
            check(7, "DONE set before ACK", read_data & STATUS_DONE, STATUS_DONE);
            fire_ack();
            repeat(4) @(posedge clk);
            ps_read(REG_STATUS);
            check(7, "STATUS=0 after DONE+ACK", read_data, 32'h0);
        end
        begin ddr_serve_one_beat(32'hABCD_1234, 2'b00, 2'b00, 0, 0, 0, 0); end
    join

    // =========================================================================
    // TEST 8 - ACK clears ERROR: second ps_read(STATUS) must return 0
    // =========================================================================
    $display("\n--- Test 8: ACK clears ERROR ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd4);

    fork
        begin
            fire_start();
            ps_poll_status(STATUS_ERROR, 200);
            check(8, "ERROR set before ACK", read_data & STATUS_ERROR, STATUS_ERROR);
            fire_ack();
            repeat(4) @(posedge clk);
            ps_read(REG_STATUS);
            check(8, "STATUS=0 after ERROR+ACK", read_data, 32'h0);
        end
        begin
            @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
            @(negedge clk); M_AXI_ARREADY = 1'b1;
            @(posedge clk); @(negedge clk); M_AXI_ARREADY = 1'b0;
            @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
            @(negedge clk);
            M_AXI_RDATA = 32'h0; M_AXI_RRESP = 2'b10;
            M_AXI_RVALID = 1'b1; M_AXI_RLAST = 1'b1;
            @(posedge clk); @(negedge clk);
            M_AXI_RVALID = 1'b0; M_AXI_RLAST = 1'b0;
        end
    join

    // =========================================================================
    // TEST 9 - BUSY bit asserted immediately after START, before transfer ends
    // =========================================================================
    $display("\n--- Test 9: BUSY bit set immediately after START ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd4);

    // Write START but stall DDR so engine stays busy
    fork
        begin
            fire_start();
            // Read STATUS while DDR is still stalling - BUSY must be set
            ps_read(REG_STATUS);
            check(9, "BUSY=1 while DDR stalling", read_data & STATUS_BUSY, STATUS_BUSY);
            // Now let DDR respond
            repeat(2) @(posedge clk);  // signal DDR thread to proceed
        end
        begin
            // Stall AR for 20 cycles to guarantee PS can read STATUS
            @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
            repeat(20) @(posedge clk);
            @(negedge clk); M_AXI_ARREADY = 1'b1;
            @(posedge clk); @(negedge clk); M_AXI_ARREADY = 1'b0;
            // R
            @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
            @(negedge clk);
            M_AXI_RDATA = 32'hBEEF_0001; M_AXI_RRESP = 2'b00;
            M_AXI_RVALID = 1'b1; M_AXI_RLAST = 1'b1;
            @(posedge clk); @(negedge clk);
            M_AXI_RVALID = 1'b0; M_AXI_RLAST = 1'b0;
            // AW / W / B
            @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
            @(negedge clk); M_AXI_AWREADY = 1'b1;
            @(posedge clk); @(negedge clk); M_AXI_AWREADY = 1'b0;
            @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
            @(negedge clk); M_AXI_WREADY = 1'b1;
            @(posedge clk); @(negedge clk); M_AXI_WREADY = 1'b0;
            @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
            @(negedge clk); M_AXI_BRESP = 2'b00; M_AXI_BVALID = 1'b1;
            @(posedge clk); @(negedge clk); M_AXI_BVALID = 1'b0;
        end
    join

    // Poll done and ack
    ps_poll_status(STATUS_DONE, 200);
    fire_ack();

    // =========================================================================
    // TEST 10 - Back-to-back: second transfer starts immediately after first ACK
    // =========================================================================
    $display("\n--- Test 10: Back-to-back transfers ---");
    do_reset();

    // First transfer
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd4);
    fork
        begin
            fire_start();
            ps_poll_status(STATUS_DONE, 200);
            check(10, "first xfer DONE",  read_data & STATUS_DONE,  STATUS_DONE);
            fire_ack();
            repeat(4) @(posedge clk);
            ps_read(REG_STATUS);
            check(10, "STATUS=0 after first ACK", read_data, 32'h0);
        end
        begin ddr_serve_one_beat(32'h1111_1111, 2'b00, 2'b00, 0, 0, 0, 0); end
    join

    // Second transfer with different addresses
    setup_transfer(32'hC000_0000, 32'hD000_0000, 32'd4);
    fork
        begin
            fire_start();
            ps_poll_status(STATUS_DONE, 200);
            check(10, "second xfer DONE", read_data & STATUS_DONE,  STATUS_DONE);
            check(10, "no ERROR second",  read_data & STATUS_ERROR, 32'h0);
            fire_ack();
        end
        begin ddr_serve_one_beat(32'h2222_2222, 2'b00, 2'b00, 0, 0, 0, 0); end
    join

    // =========================================================================
    // TEST 11 - STATUS is read-only: write attempt must not change it
    // =========================================================================
    $display("\n--- Test 11: STATUS register is read-only ---");
    do_reset();

    // Try to write all 1s to STATUS
    ps_write(REG_STATUS, 32'hFFFF_FFFF, 4'hF);
    // STATUS is assembled from busy/done/error wires - all 0 at idle
    ps_read(REG_STATUS);
    check(11, "STATUS ignores PS write", read_data, 32'h0);

    // =========================================================================
    // TEST 12 - Slow DDR: each channel stalled 5 cycles independently
    // =========================================================================
    $display("\n--- Test 12: Slow DDR (5-cycle stall on every channel) ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd4);

    fork
        begin
            fire_start();
            ps_poll_status(STATUS_DONE, 500);
            check(12, "DONE with all-channel stalls", read_data & STATUS_DONE,  STATUS_DONE);
            check(12, "no ERROR with stalls",         read_data & STATUS_ERROR, 32'h0);
            fire_ack();
        end
        begin
            ddr_serve_one_beat(32'hCAFE_0001, 2'b00, 2'b00, 5, 5, 5, 5);
        end
    join

    // =========================================================================
    // TEST 13 - data integrity: RDATA forwarded to WDATA for 3 different patterns
    // =========================================================================
    $display("\n--- Test 13: data_buf integrity (RDATA -> WDATA, 3 patterns) ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd12);  // 3 words

    begin : test13_block
        reg [DATA_WIDTH-1:0] pattern [0:2];
        reg [DATA_WIDTH-1:0] cap_wdata [0:2];
        integer i;

        pattern[0]  = 32'h1234_5678;
        pattern[1]  = 32'hABCD_EF01;
        pattern[2]  = 32'hDEAD_BEEF;
        cap_wdata[0] = 0; cap_wdata[1] = 0; cap_wdata[2] = 0;

        fork
            begin
                fire_start();
                ps_poll_status(STATUS_DONE, 500);
                check(13, "DONE after 3-pattern xfer", read_data & STATUS_DONE, STATUS_DONE);
                fire_ack();
            end
            begin
                // Beat 0
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                @(negedge clk); M_AXI_ARREADY = 1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY = 1'b0;
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=pattern[0]; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                @(negedge clk); M_AXI_AWREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY=1'b0;
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                cap_wdata[0] = M_AXI_WDATA;
                @(negedge clk); M_AXI_WREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY=1'b0;
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;

                // Beat 1
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                @(negedge clk); M_AXI_ARREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY=1'b0;
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=pattern[1]; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                @(negedge clk); M_AXI_AWREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY=1'b0;
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                cap_wdata[1] = M_AXI_WDATA;
                @(negedge clk); M_AXI_WREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY=1'b0;
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;

                // Beat 2
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                @(negedge clk); M_AXI_ARREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY=1'b0;
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=pattern[2]; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                @(negedge clk); M_AXI_AWREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY=1'b0;
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                cap_wdata[2] = M_AXI_WDATA;
                @(negedge clk); M_AXI_WREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY=1'b0;
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;
            end
        join

        check(13, "WDATA[0]==RDATA[0]", cap_wdata[0], pattern[0]);
        check(13, "WDATA[1]==RDATA[1]", cap_wdata[1], pattern[1]);
        check(13, "WDATA[2]==RDATA[2]", cap_wdata[2], pattern[2]);
    end

    // =========================================================================
    // TEST 14 - AXI4 Full channel attributes: ARLEN=0, ARSIZE=2, ARBURST=INCR
    //                                          AWLEN=0, AWSIZE=2, AWBURST=INCR
    // =========================================================================
    $display("\n--- Test 14: AXI4 Full channel attributes ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd4);

    begin : test14_block
        reg [7:0] cap_arlen;  reg [2:0] cap_arsize;  reg [1:0] cap_arburst;
        reg [7:0] cap_awlen;  reg [2:0] cap_awsize;  reg [1:0] cap_awburst;

        fork
            begin
                fire_start();
                ps_poll_status(STATUS_DONE, 200);
                fire_ack();
            end
            begin
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                cap_arlen   = M_AXI_ARLEN;
                cap_arsize  = M_AXI_ARSIZE;
                cap_arburst = M_AXI_ARBURST;
                @(negedge clk); M_AXI_ARREADY = 1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY = 1'b0;
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=32'h5A5A_5A5A; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                cap_awlen   = M_AXI_AWLEN;
                cap_awsize  = M_AXI_AWSIZE;
                cap_awburst = M_AXI_AWBURST;
                @(negedge clk); M_AXI_AWREADY = 1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY = 1'b0;
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                @(negedge clk); M_AXI_WREADY = 1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY = 1'b0;
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;
            end
        join

        check(14, "ARLEN=0",         {24'b0, cap_arlen},   32'h0);
        check(14, "ARSIZE=4B",       {29'b0, cap_arsize},  32'h2);
        check(14, "ARBURST=INCR",    {30'b0, cap_arburst}, 32'h1);
        check(14, "AWLEN=0",         {24'b0, cap_awlen},   32'h0);
        check(14, "AWSIZE=4B",       {29'b0, cap_awsize},  32'h2);
        check(14, "AWBURST=INCR",    {30'b0, cap_awburst}, 32'h1);
    end

    // =========================================================================
    // TEST 15 - WSTRB=4'hF and WLAST=1 on every write beat (2-word transfer)
    // =========================================================================
    $display("\n--- Test 15: WSTRB=4'hF and WLAST=1 on write beats ---");
    do_reset();
    setup_transfer(32'hA000_0000, 32'hB000_0000, 32'd8);  // 2 words

    begin : test15_block
        reg [3:0] cap_wstrb [0:1];
        reg       cap_wlast [0:1];

        fork
            begin
                fire_start();
                ps_poll_status(STATUS_DONE, 300);
                check(15, "DONE after 2-word xfer", read_data & STATUS_DONE, STATUS_DONE);
                fire_ack();
            end
            begin
                // Beat 0
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                @(negedge clk); M_AXI_ARREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY=1'b0;
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=32'hAAAA_AAAA; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                @(negedge clk); M_AXI_AWREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY=1'b0;
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                cap_wstrb[0] = M_AXI_WSTRB;
                cap_wlast[0] = M_AXI_WLAST;
                @(negedge clk); M_AXI_WREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY=1'b0;
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;

                // Beat 1
                @(posedge clk); while (!M_AXI_ARVALID) @(posedge clk);
                @(negedge clk); M_AXI_ARREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_ARREADY=1'b0;
                @(posedge clk); while (!M_AXI_RREADY) @(posedge clk);
                @(negedge clk); M_AXI_RDATA=32'hBBBB_BBBB; M_AXI_RRESP=2'b00;
                                M_AXI_RVALID=1'b1; M_AXI_RLAST=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_RVALID=1'b0; M_AXI_RLAST=1'b0;
                @(posedge clk); while (!M_AXI_AWVALID) @(posedge clk);
                @(negedge clk); M_AXI_AWREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_AWREADY=1'b0;
                @(posedge clk); while (!M_AXI_WVALID) @(posedge clk);
                cap_wstrb[1] = M_AXI_WSTRB;
                cap_wlast[1] = M_AXI_WLAST;
                @(negedge clk); M_AXI_WREADY=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_WREADY=1'b0;
                @(posedge clk); while (!M_AXI_BREADY) @(posedge clk);
                @(negedge clk); M_AXI_BRESP=2'b00; M_AXI_BVALID=1'b1;
                @(posedge clk); @(negedge clk); M_AXI_BVALID=1'b0;
            end
        join

        check(15, "WSTRB=F beat0",  {28'b0, cap_wstrb[0]}, 32'hF);
        check(15, "WLAST=1 beat0",  {31'b0, cap_wlast[0]}, 32'h1);
        check(15, "WSTRB=F beat1",  {28'b0, cap_wstrb[1]}, 32'hF);
        check(15, "WLAST=1 beat1",  {31'b0, cap_wlast[1]}, 32'h1);
    end

    // =========================================================================
    // FINAL REPORT
    // =========================================================================
    $display("\n=============================================================");
    $display("  dma_top INTEGRATION TESTBENCH COMPLETE");
    $display("  PASSED : %0d", pass_count);
    $display("  FAILED : %0d", fail_count);
    if (fail_count == 0)
        $display("  RESULT : ALL TESTS PASSED - ready for FPGA");
    else
        $display("  RESULT : SOME TESTS FAILED - review above before taping out");
    $display("=============================================================\n");

    $finish;
end

// =============================================================================
// Timeout Watchdog
// =============================================================================
initial begin
    #2000000;
    $display("[TIMEOUT] Simulation exceeded 2ms - possible handshake deadlock");
    $finish;
end

endmodule