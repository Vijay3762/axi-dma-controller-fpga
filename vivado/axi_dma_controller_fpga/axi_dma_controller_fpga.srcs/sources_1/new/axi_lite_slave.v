`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/06/2026 03:46:18 PM
// Design Name: 
// Module Name: axi_lite_slave
// Project Name: DMA
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


module axi_lite_slave #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32   // address width of zedboard = 32-bits
) (
    // Global Signals
    input wire S_AXI_ACLK,
    input wire S_AXI_ARESETN, // Active LOW sync reset

    // Write Address Channel (AW)
    input  wire [ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire                  S_AXI_AWVALID,
    output reg                   S_AXI_AWREADY,

    // Write Data Channel (W)
    input  wire [    DATA_WIDTH-1:0] S_AXI_WDATA,
    input  wire [(DATA_WIDTH/8)-1:0] S_AXI_WSTRB,   // Byte-lane enable strobes
    input  wire                      S_AXI_WVALID,
    output reg                       S_AXI_WREADY,

    // Write Response Channel (B)
    output reg  [1:0] S_AXI_BRESP,
    output reg        S_AXI_BVALID,
    input  wire       S_AXI_BREADY,

    // Read Address Channel (AR)
    input  wire [ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire                  S_AXI_ARVALID,
    output reg                   S_AXI_ARREADY,

    // Read Data Channel (R)
    output reg  [DATA_WIDTH-1:0] S_AXI_RDATA,
    output reg  [           1:0] S_AXI_RRESP,
    output reg                   S_AXI_RVALID,
    input  wire                  S_AXI_RREADY,

    // Outputs to DMA Engine
    output wire [DATA_WIDTH-1:0] src_addr,
    output wire [DATA_WIDTH-1:0] dst_addr,
    output wire [DATA_WIDTH-1:0] transfer_len,
    output wire                  start,         // 1-cycle pulse
    //output wire                  soft_reset,

    // Inputs from DMA Engine (Status)
    input wire busy,
    input wire done,
    input wire error,

    output wire ack
);

  // Internal Register Storage (the actual flip-flops)
  // These are the physical registers. The PS reads/writes these via AXI-Lite.

  reg [    DATA_WIDTH-1:0] reg_src_addr;
  reg [    DATA_WIDTH-1:0] reg_dst_addr;
  reg [    DATA_WIDTH-1:0] reg_transfer_len;
  reg [    DATA_WIDTH-1:0] reg_control;

  // Internal Handshake and Flow-Control Flags
  // these registers are used to track the write_address and write_data signals

  reg [    ADDR_WIDTH-1:0] aw_addr_latch;  // Latched write address
  reg                      aw_addr_valid;  // Flag: write address has been captured

  reg [    DATA_WIDTH-1:0] w_data_latch;  // Latched write data
  reg [(DATA_WIDTH/8)-1:0] w_strb_latch;  // Latched write strobes
  reg                      w_data_valid;  // Flag: write data has been captured

  reg [    ADDR_WIDTH-1:0] ar_addr_latch;  // Latched read address
  reg                      read_pending;  // Flag: read is in progress

  // For START pulse generation - we keep a 1-cycle delayed copy of CONTROL[0]
  // --> (2)
  reg                      start_prev;

  // Write Address Channel (AW)
  // AWREADY is always kept at HIGH because we are always ready to accept an address.
  // when AWVALID & AWREADY are both 1, we latch the address.

  always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
      S_AXI_AWREADY <= 1'b1;  // Ready immediately after reset
      aw_addr_latch <= {ADDR_WIDTH{1'b0}};
      aw_addr_valid <= 1'b0;
    end else begin
      if (S_AXI_AWVALID && S_AXI_AWREADY) begin
        aw_addr_latch <= S_AXI_AWADDR;  // Capture the address
        aw_addr_valid <= 1'b1;  // indicate that we have the address
      end

      // Once we've processed both address AND data (write decode done),
      // clear the flag so we're ready for the next transaction.
      // This is cleared in the write-decode block below using a wire signal.
      if (aw_addr_valid && w_data_valid) begin
        aw_addr_valid <= 1'b0;
      end
    end
  end

  // Write Data Channel (W)
  // WREADY is always kept at HIGH because we are always ready to accept the
  // write data.(even if we the AWADDR is not arrived)
  // as soon as WVALID & WREADY are both 1, we latch data and strobes.

  always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
      S_AXI_WREADY <= 1'b1;
      w_data_latch <= {DATA_WIDTH{1'b0}};
      w_strb_latch <= {(DATA_WIDTH / 8) {1'b0}};
      w_data_valid <= 1'b0;
    end else begin
      // Handshake completes
      if (S_AXI_WVALID && S_AXI_WREADY) begin
        w_data_latch <= S_AXI_WDATA;
        w_strb_latch <= S_AXI_WSTRB;
        w_data_valid <= 1'b1;
      end

      // Clear valid once write decode consumes both address and data
      if (aw_addr_valid && w_data_valid) begin
        w_data_valid <= 1'b0;
      end
    end
  end

  // Write Decode and Register Update
  // Only runs when BOTH address and data have been latched.
  // Applies byte strobes - only updates bytes where strobe bit = 1.

  always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
      reg_src_addr     <= 32'h0;
      reg_dst_addr     <= 32'h0;
      reg_transfer_len <= 32'h0;
      reg_control      <= 32'h0;
    end else begin
      if (aw_addr_valid && w_data_valid) begin

        // Auto-clear CONTROL[0] (START) every cycle so it does not persist
        // The start pulse logic (Step 8) will handle the single-cycle pulse.
        reg_control[0] <= 1'b0;

        case (aw_addr_latch[4:0])  // lower 5 bits select the register

          5'h00: begin  // SRC_ADDR
            if (w_strb_latch[0]) reg_src_addr[7:0] <= w_data_latch[7:0];
            if (w_strb_latch[1]) reg_src_addr[15:8] <= w_data_latch[15:8];
            if (w_strb_latch[2]) reg_src_addr[23:16] <= w_data_latch[23:16];
            if (w_strb_latch[3]) reg_src_addr[31:24] <= w_data_latch[31:24];
          end

          5'h04: begin  // DST_ADDR
            if (w_strb_latch[0]) reg_dst_addr[7:0] <= w_data_latch[7:0];
            if (w_strb_latch[1]) reg_dst_addr[15:8] <= w_data_latch[15:8];
            if (w_strb_latch[2]) reg_dst_addr[23:16] <= w_data_latch[23:16];
            if (w_strb_latch[3]) reg_dst_addr[31:24] <= w_data_latch[31:24];
          end

          5'h08: begin  // TRANSFER_LEN
            if (w_strb_latch[0]) reg_transfer_len[7:0] <= w_data_latch[7:0];
            if (w_strb_latch[1]) reg_transfer_len[15:8] <= w_data_latch[15:8];
            if (w_strb_latch[2]) reg_transfer_len[23:16] <= w_data_latch[23:16];
            if (w_strb_latch[3]) reg_transfer_len[31:24] <= w_data_latch[31:24];
          end

          5'h0C: begin
            if (w_strb_latch[0]) reg_control[7:0] <= w_data_latch[7:0];
            if (w_strb_latch[1]) reg_control[15:8] <= w_data_latch[15:8];
            if (w_strb_latch[2]) reg_control[23:16] <= w_data_latch[23:16];
            if (w_strb_latch[3]) reg_control[31:24] <= w_data_latch[31:24];
          end

          5'h10: begin
            // STATUS is READ-ONLY. Silently ignore any write attempt.
          end

          default: begin
            // Unknown address - ignore silently
          end

        endcase
      end
    end
  end

  // After a successful register write, we must send a response back to the PS.
  // BVALID is asserted on the cycle after the write decode completes.
  // We hold BVALID HIGH until the PS sends BREADY to acknowledge it.
  // BRESP = 2'b00 means OKAY - we have no error conditions on writes.

  always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
      S_AXI_BVALID <= 1'b0;
      S_AXI_BRESP  <= 2'b00;
    end else begin
      if (aw_addr_valid && w_data_valid && !S_AXI_BVALID) begin
        // Write decode just fired - send response
        S_AXI_BVALID <= 1'b1;
        S_AXI_BRESP  <= 2'b00;  // OKAY
      end else if (S_AXI_BVALID && S_AXI_BREADY) begin
        // PS acknowledged the response - deassert
        S_AXI_BVALID <= 1'b0;
      end
    end
  end

  // Read Address Channel (AR)
  // ARREADY is HIGH when no read is pending.
  // When handshake completes, latch address and set read_pending flag.

  always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
      S_AXI_ARREADY <= 1'b1;  // Ready to accept read address at reset
      ar_addr_latch <= {ADDR_WIDTH{1'b0}};
      read_pending  <= 1'b0;
    end else begin
      if (S_AXI_ARVALID && S_AXI_ARREADY) begin
        ar_addr_latch <= S_AXI_ARADDR;  // Latch read address
        read_pending  <= 1'b1;  // Signal read data channel to respond
        S_AXI_ARREADY <= 1'b0;  // Stop accepting new read addresses
      end

      // Once PS accepts the read data, clear read_pending and become ready again
      if (S_AXI_RVALID && S_AXI_RREADY) begin
        read_pending  <= 1'b0;
        S_AXI_ARREADY <= 1'b1;
      end
    end
  end

  // Read Data Channel (R)
  // STATUS register is assembled live here from DMA engine inputs.
  // Hold RVALID HIGH until PS asserts RREADY.
  // For unknown addresses, return 32'hDEADBEEF - useful for debugging.

  always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
      S_AXI_RVALID <= 1'b0;
      S_AXI_RDATA  <= 32'h0;
      S_AXI_RRESP  <= 2'b00;
    end else begin
      if (read_pending && !S_AXI_RVALID) begin
        S_AXI_RVALID <= 1'b1;
        S_AXI_RRESP  <= 2'b00;  // OKAY

        case (ar_addr_latch[4:0])
          5'h00:   S_AXI_RDATA <= reg_src_addr;
          5'h04:   S_AXI_RDATA <= reg_dst_addr;
          5'h08:   S_AXI_RDATA <= reg_transfer_len;
          5'h0C:   S_AXI_RDATA <= reg_control;
          5'h10:   S_AXI_RDATA <= {29'b0, error, done, busy};  // STATUS - live
          default: S_AXI_RDATA <= 32'hDEADBEEF;  // Debug marker
        endcase

      end else if (S_AXI_RVALID && S_AXI_RREADY) begin
        // PS accepted the data - deassert RVALID
        S_AXI_RVALID <= 1'b0;
      end
    end
  end

  always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) reg_control[0] <= 1'b0;
    else if (reg_control[0])  // if START was written
      reg_control[0] <= 1'b0;  // clear it next cycle
  end

  // START Pulse Generation (Edge Detection)
  // Problem: if we wire start = reg_control[0] directly, it stays HIGH until
  //          PS manually clears it. The DMA FSM would re-trigger every cycle.
  // Solution: detect the RISING EDGE of reg_control[0] using a delayed copy.
  //           start = reg_control[0] AND NOT(delayed copy) = only HIGH for 1 cycle.

  always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) start_prev <= 1'b0;
    else start_prev <= reg_control[0];
  end

  // Rising edge detect: was 0 last cycle, is 1 this cycle
  assign start        = reg_control[0] & ~start_prev;
  //assign soft_reset   = reg_control[1];

  // Connect Internal Registers to Output Ports

  assign src_addr     = reg_src_addr;
  assign dst_addr     = reg_dst_addr;
  assign transfer_len = reg_transfer_len;

  assign ack          = reg_control[2];

endmodule