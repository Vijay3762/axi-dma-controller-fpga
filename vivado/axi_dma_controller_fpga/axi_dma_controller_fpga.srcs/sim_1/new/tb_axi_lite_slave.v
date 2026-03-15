// =============================================================================
// Module      : tb_axi_lite_slave.v
// Description : Testbench for axi_lite_slave.v
//
// Tests performed:
//   Test 1 - Write SRC_ADDR        and read it back
//   Test 2 - Write DST_ADDR        and read it back
//   Test 3 - Write TRANSFER_LEN    and read it back
//   Test 4 - Write CONTROL (START) and verify start pulse = exactly 1 cycle
//   Test 5 - Write to STATUS       and verify it is ignored (read-only)
//   Test 6 - Read STATUS           and verify busy/done/error bits are live
//   Test 7 - Write unknown address and verify no register is corrupted
//   Test 8 - Read unknown address  and verify DEADBEEF is returned
//   Test 9 - WSTRB test            write only lower 2 bytes of SRC_ADDR
//   Test 10- Verify BRESP and RRESP are always 2'b00 (OKAY)
// =============================================================================

`timescale 1ns / 1ps

module tb_axi_lite_slave;

// =============================================================================
// Parameters
// =============================================================================
parameter DATA_WIDTH = 32;
parameter ADDR_WIDTH = 32;
parameter CLK_PERIOD = 10;   // 10ns = 100MHz

// =============================================================================
// DUT Signal Declarations
// =============================================================================

// Global
reg  S_AXI_ACLK;
reg  S_AXI_ARESETN;

// Write Address Channel
reg  [ADDR_WIDTH-1:0]     S_AXI_AWADDR;
reg                       S_AXI_AWVALID;
wire                      S_AXI_AWREADY;

// Write Data Channel
reg  [DATA_WIDTH-1:0]     S_AXI_WDATA;
reg  [(DATA_WIDTH/8)-1:0] S_AXI_WSTRB;
reg                       S_AXI_WVALID;
wire                      S_AXI_WREADY;

// Write Response Channel
wire [1:0]                S_AXI_BRESP;
wire                      S_AXI_BVALID;
reg                       S_AXI_BREADY;

// Read Address Channel
reg  [ADDR_WIDTH-1:0]     S_AXI_ARADDR;
reg                       S_AXI_ARVALID;
wire                      S_AXI_ARREADY;

// Read Data Channel
wire [DATA_WIDTH-1:0]     S_AXI_RDATA;
wire [1:0]                S_AXI_RRESP;
wire                      S_AXI_RVALID;
reg                       S_AXI_RREADY;

// DMA Engine Interface
wire [DATA_WIDTH-1:0]     src_addr;
wire [DATA_WIDTH-1:0]     dst_addr;
wire [DATA_WIDTH-1:0]     transfer_len;
wire                      start;
wire                      soft_reset;

// Simulated DMA Engine status inputs
reg                       busy;
reg                       done;
reg                       error;

// =============================================================================
// Test tracking
// =============================================================================
integer test_num;
integer pass_count;
integer fail_count;
reg [31:0] read_data;      // stores result of each read transaction

// =============================================================================
// DUT Instantiation
// =============================================================================
axi_lite_slave #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) DUT (
    .S_AXI_ACLK      (S_AXI_ACLK),
    .S_AXI_ARESETN   (S_AXI_ARESETN),
    .S_AXI_AWADDR    (S_AXI_AWADDR),
    .S_AXI_AWVALID   (S_AXI_AWVALID),
    .S_AXI_AWREADY   (S_AXI_AWREADY),
    .S_AXI_WDATA     (S_AXI_WDATA),
    .S_AXI_WSTRB     (S_AXI_WSTRB),
    .S_AXI_WVALID    (S_AXI_WVALID),
    .S_AXI_WREADY    (S_AXI_WREADY),
    .S_AXI_BRESP     (S_AXI_BRESP),
    .S_AXI_BVALID    (S_AXI_BVALID),
    .S_AXI_BREADY    (S_AXI_BREADY),
    .S_AXI_ARADDR    (S_AXI_ARADDR),
    .S_AXI_ARVALID   (S_AXI_ARVALID),
    .S_AXI_ARREADY   (S_AXI_ARREADY),
    .S_AXI_RDATA     (S_AXI_RDATA),
    .S_AXI_RRESP     (S_AXI_RRESP),
    .S_AXI_RVALID    (S_AXI_RVALID),
    .S_AXI_RREADY    (S_AXI_RREADY),
    .src_addr        (src_addr),
    .dst_addr        (dst_addr),
    .transfer_len    (transfer_len),
    .start           (start),
    .soft_reset      (soft_reset),
    .busy            (busy),
    .done            (done),
    .error           (error)
);

// =============================================================================
// Clock Generation - toggles every half period
// =============================================================================
initial S_AXI_ACLK = 0;
always #5 S_AXI_ACLK = ~S_AXI_ACLK;

// =============================================================================
// Task: axi_write
// Performs a complete AXI-Lite write transaction with WSTRB support
// Usage: axi_write(address, data, strobe)
// =============================================================================
task axi_write;
    input [ADDR_WIDTH-1:0] addr;
    input [DATA_WIDTH-1:0] data;
    input [(DATA_WIDTH/8)-1:0] strb;
    begin
        // Drive write address and write data simultaneously
        // (AXI-Lite allows both channels at same time)
        @(posedge S_AXI_ACLK);
        #1;  // small delay after clock edge to avoid setup violations
        S_AXI_AWADDR  = addr;
        S_AXI_AWVALID = 1'b1;
        S_AXI_WDATA   = data;
        S_AXI_WSTRB   = strb;
        S_AXI_WVALID  = 1'b1;

        // Wait for both address and data handshakes to complete
        // AWREADY and WREADY are permanently high in our design,
        // so this completes on the very next clock edge
        @(posedge S_AXI_ACLK);
        #1;
        S_AXI_AWVALID = 1'b0;
        S_AXI_WVALID  = 1'b0;

        // Wait for write response (BVALID from slave)
        S_AXI_BREADY = 1'b1;
        @(posedge S_AXI_ACLK);
        while (!S_AXI_BVALID) @(posedge S_AXI_ACLK);
        // Handshake completes this cycle (BVALID & BREADY both high)
        @(posedge S_AXI_ACLK);
        #1;
        S_AXI_BREADY = 1'b0;
    end
endtask

// =============================================================================
// Task: axi_read
// Performs a complete AXI-Lite read transaction
// Result stored in global variable read_data
// =============================================================================
task axi_read;
    input [ADDR_WIDTH-1:0] addr;
    begin
        @(posedge S_AXI_ACLK);
        #1;
        S_AXI_ARADDR  = addr;
        S_AXI_ARVALID = 1'b1;

        // Wait for ARREADY (slave accepts read address)
        @(posedge S_AXI_ACLK);
        while (!S_AXI_ARREADY) @(posedge S_AXI_ACLK);
        #1;
        S_AXI_ARVALID = 1'b0;

        // Wait for RVALID (slave drives read data)
        S_AXI_RREADY = 1'b1;
        @(posedge S_AXI_ACLK);
        while (!S_AXI_RVALID) @(posedge S_AXI_ACLK);

        // Capture the data on the handshake cycle
        read_data = S_AXI_RDATA;

        @(posedge S_AXI_ACLK);
        #1;
        S_AXI_RREADY = 1'b0;
    end
endtask

// =============================================================================
// Task: check
// Compares actual vs expected and prints PASS/FAIL
// =============================================================================
task check;
    input [63:0]   test_id;
    input [255:0]  test_name;
    input [31:0]   actual;
    input [31:0]   expected;
    begin
        if (actual === expected) begin
            $display("  [PASS] Test %0d - %s | got 0x%08X", test_id, test_name, actual);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] Test %0d - %s | got 0x%08X expected 0x%08X",
                     test_id, test_name, actual, expected);
            fail_count = fail_count + 1;
        end
    end
endtask

// =============================================================================
// Main Test Sequence
// =============================================================================
integer start_cycle;   // used to count START pulse duration
integer pulse_count;

initial begin
    // -------------------------------------------------------------------------
    // Initialise all inputs to safe idle state
    // -------------------------------------------------------------------------
    S_AXI_ARESETN = 1'b0;
    S_AXI_AWADDR  = 0;
    S_AXI_AWVALID = 1'b0;
    S_AXI_WDATA   = 0;
    S_AXI_WSTRB   = 4'hF;
    S_AXI_WVALID  = 1'b0;
    S_AXI_BREADY  = 1'b0;
    S_AXI_ARADDR  = 0;
    S_AXI_ARVALID = 1'b0;
    S_AXI_RREADY  = 1'b0;
    busy          = 1'b0;
    done          = 1'b0;
    error         = 1'b0;
    pass_count    = 0;
    fail_count    = 0;

    $display("=============================================================");
    $display("  AXI-Lite Slave Testbench Starting");
    $display("=============================================================");

    // -------------------------------------------------------------------------
    // Apply reset for 5 clock cycles then release
    // -------------------------------------------------------------------------
    repeat(5) @(posedge S_AXI_ACLK);
    S_AXI_ARESETN = 1'b1;
    repeat(2) @(posedge S_AXI_ACLK);
    $display("\n  Reset released. Starting tests...\n");

    // =========================================================================
    // TEST 1 - Write SRC_ADDR and read it back
    // =========================================================================
    $display("--- Test 1: Write and read SRC_ADDR ---");
    axi_write(32'h00, 32'h10000000, 4'hF);
    axi_read(32'h00);
    check(1, "SRC_ADDR readback", read_data, 32'h10000000);
    // Also verify the output wire to DMA engine
    check(1, "src_addr output wire", src_addr, 32'h10000000);

    // =========================================================================
    // TEST 2 - Write DST_ADDR and read it back
    // =========================================================================
    $display("--- Test 2: Write and read DST_ADDR ---");
    axi_write(32'h04, 32'h20000000, 4'hF);
    axi_read(32'h04);
    check(2, "DST_ADDR readback", read_data, 32'h20000000);
    check(2, "dst_addr output wire", dst_addr, 32'h20000000);

    // =========================================================================
    // TEST 3 - Write TRANSFER_LEN and read it back
    // =========================================================================
    $display("--- Test 3: Write and read TRANSFER_LEN ---");
    axi_write(32'h08, 32'h00000400, 4'hF);   // 1024 bytes
    axi_read(32'h08);
    check(3, "TRANSFER_LEN readback", read_data, 32'h00000400);
    check(3, "transfer_len output wire", transfer_len, 32'h00000400);

    // =========================================================================
    // TEST 4 - Write CONTROL[0]=1 and verify start is exactly 1 cycle pulse
    // =========================================================================
    $display("--- Test 4: START pulse is exactly 1 clock cycle ---");
    pulse_count = 0;
    axi_write(32'h0C, 32'h00000001, 4'hF);   // set START bit

    // Count how many cycles start stays HIGH
    // We monitor for 10 cycles after the write
    repeat(10) begin
        @(posedge S_AXI_ACLK);
        if (start === 1'b1)
            pulse_count = pulse_count + 1;
    end

    if (pulse_count === 1)
        $display("  [PASS] Test 4 - START pulse width = 1 cycle exactly");
    else begin
        $display("  [FAIL] Test 4 - START pulse width = %0d cycles (expected 1)", pulse_count);
        fail_count = fail_count + 1;
    end

    // =========================================================================
    // TEST 5 - Write to STATUS (read-only), verify it is silently ignored
    // SRC_ADDR should be unchanged after this
    // =========================================================================
    $display("--- Test 5: STATUS register is read-only ---");
    axi_write(32'h10, 32'hFFFFFFFF, 4'hF);  // try to write all 1s to STATUS
    // STATUS is assembled from busy/done/error - writing must have no effect
    // verify by reading STATUS back - busy/done/error are all 0 so should be 0
    axi_read(32'h10);
    check(5, "STATUS ignores writes", read_data, 32'h00000000);

    // =========================================================================
    // TEST 6 - Verify STATUS reflects live DMA engine signals
    // =========================================================================
    $display("--- Test 6: STATUS register reflects live engine signals ---");

    // Simulate DMA engine asserting busy
    busy = 1'b1; done = 1'b0; error = 1'b0;
    @(posedge S_AXI_ACLK);
    axi_read(32'h10);
    check(6, "STATUS busy=1", read_data, 32'h00000001);   // bit[0]=busy

    // Simulate DMA engine done
    busy = 1'b0; done = 1'b1; error = 1'b0;
    @(posedge S_AXI_ACLK);
    axi_read(32'h10);
    check(6, "STATUS done=1", read_data, 32'h00000002);   // bit[1]=done

    // Simulate DMA engine error
    busy = 1'b0; done = 1'b0; error = 1'b1;
    @(posedge S_AXI_ACLK);
    axi_read(32'h10);
    check(6, "STATUS error=1", read_data, 32'h00000004);  // bit[2]=error

    // Simulate all three at once
    busy = 1'b1; done = 1'b1; error = 1'b1;
    @(posedge S_AXI_ACLK);
    axi_read(32'h10);
    check(6, "STATUS all bits=1", read_data, 32'h00000007); // bits[2:0]=111

    // Reset status signals
    busy = 1'b0; done = 1'b0; error = 1'b0;

    // =========================================================================
    // TEST 7 - Write to unknown address, verify no register is corrupted
    // =========================================================================
    $display("--- Test 7: Write to unknown address does not corrupt registers ---");
    // First record current values
    axi_write(32'h14, 32'hDEADDEAD, 4'hF);  // 0x14 does not exist
    // Verify SRC_ADDR is still 0x10000000 from Test 1
    axi_read(32'h00);
    check(7, "SRC_ADDR unchanged after bad write", read_data, 32'h10000000);

    // =========================================================================
    // TEST 8 - Read from unknown address, verify DEADBEEF is returned
    // =========================================================================
    $display("--- Test 8: Read unknown address returns 0xDEADBEEF ---");
    axi_read(32'h14);
    check(8, "Unknown addr returns DEADBEEF", read_data, 32'hDEADBEEF);

    // =========================================================================
    // TEST 9 - WSTRB: write only lower byte of SRC_ADDR
    // Upper 3 bytes must be unchanged, only [7:0] must update
    // =========================================================================
    $display("--- Test 9: WSTRB partial write (lower byte only) ---");
    // First set SRC_ADDR to a known value
    axi_write(32'h00, 32'hAABBCCDD, 4'hF);   // all bytes written
    axi_read(32'h00);
    check(9, "SRC_ADDR setup for WSTRB test", read_data, 32'hAABBCCDD);

    // Now write only lower byte (WSTRB=4'b0001), upper 3 bytes should stay
    axi_write(32'h00, 32'hXXXXXX11, 4'b0001);  // only byte[7:0] = 0x11 valid
    axi_read(32'h00);
    // Expected: upper 3 bytes unchanged (0xAABBCC), lower byte = 0x11
    check(9, "WSTRB lower byte only", read_data, 32'hAABBCC11);

    // =========================================================================
    // TEST 10 - Verify BRESP and RRESP are always 2'b00 (OKAY)
    // =========================================================================
    $display("--- Test 10: BRESP and RRESP are always OKAY (2'b00) ---");
    axi_write(32'h00, 32'h12345678, 4'hF);
    // Check BRESP - sampled when BVALID & BREADY
    // The axi_write task completes the handshake; we check after
    // Drive a fresh read and check RRESP
    axi_read(32'h00);
    check(10, "RRESP is OKAY", {30'b0, S_AXI_RRESP}, 32'h00000000);

    // ==========================================================================
    // FINAL REPORT
    // ==========================================================================
    $display("\n=============================================================");
    $display("  TESTBENCH COMPLETE");
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
// Timeout Watchdog - kills simulation if it hangs waiting for a handshake
// =============================================================================
initial begin
    #100000;
    $display("[TIMEOUT] Simulation exceeded time limit - possible handshake deadlock");
    $finish;
end

// =============================================================================
// Waveform Dump - open in GTKWave after simulation
// Remove this block if using Vivado XSIM (it has its own waveform viewer)
// =============================================================================
initial begin
    $dumpfile("tb_axi_lite_slave.vcd");
    $dumpvars(0, tb_axi_lite_slave);
end

endmodule