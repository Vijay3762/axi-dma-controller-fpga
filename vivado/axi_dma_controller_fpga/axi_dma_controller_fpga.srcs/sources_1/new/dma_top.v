`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/18/2026 12:38:56 PM
// Design Name: 
// Module Name: dma_top
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
// Module      : dma_top.v
// Description : Top-level wrapper that integrates axi_lite_slave and dma_engine
//
// This module has NO logic of its own.
// It only instantiates the two submodules and wires them together.
//
// Port Structure:
//   S_AXI_*  - AXI-Lite Slave port  (PS writes control registers here)
//   M_AXI_*  - AXI4 Full Master port (DMA reads/writes DDR memory here)
//
// Internal Wire Flow:
//   axi_lite_slave → dma_engine : src_addr, dst_addr, transfer_len, start, ack
//   dma_engine → axi_lite_slave : busy, done, error
//
// Register Map (seen by PS at base address 0x43000000):
//   0x00  SRC_ADDR      (R/W)
//   0x04  DST_ADDR      (R/W)
//   0x08  TRANSFER_LEN  (R/W)
//   0x0C  CONTROL       bit[0]=START  bit[1]=RESERVED  bit[2]=ACK
//   0x10  STATUS        bit[0]=BUSY   bit[1]=DONE       bit[2]=ERROR
// =============================================================================

module dma_top #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
)(
    // =========================================================================
    // Global Signals
    // =========================================================================
    input  wire clk,
    input  wire rst_n,      // Active LOW reset

    // =========================================================================
    // AXI-Lite Slave Port - PS controls the DMA through these signals
    // =========================================================================

    // Write Address Channel
    input  wire [ADDR_WIDTH-1:0]      S_AXI_AWADDR,
    input  wire                       S_AXI_AWVALID,
    output wire                       S_AXI_AWREADY,

    // Write Data Channel
    input  wire [DATA_WIDTH-1:0]      S_AXI_WDATA,
    input  wire [(DATA_WIDTH/8)-1:0]  S_AXI_WSTRB,
    input  wire                       S_AXI_WVALID,
    output wire                       S_AXI_WREADY,

    // Write Response Channel
    output wire [1:0]                 S_AXI_BRESP,
    output wire                       S_AXI_BVALID,
    input  wire                       S_AXI_BREADY,

    // Read Address Channel
    input  wire [ADDR_WIDTH-1:0]      S_AXI_ARADDR,
    input  wire                       S_AXI_ARVALID,
    output wire                       S_AXI_ARREADY,

    // Read Data Channel
    output wire [DATA_WIDTH-1:0]      S_AXI_RDATA,
    output wire [1:0]                 S_AXI_RRESP,
    output wire                       S_AXI_RVALID,
    input  wire                       S_AXI_RREADY,

    // =========================================================================
    // AXI4 Full Master Port - DMA accesses DDR memory through these signals
    // =========================================================================

    // Read Address Channel
    output wire [ADDR_WIDTH-1:0]      M_AXI_ARADDR,
    output wire                       M_AXI_ARVALID,
    output wire [7:0]                 M_AXI_ARLEN,
    output wire [2:0]                 M_AXI_ARSIZE,
    output wire [1:0]                 M_AXI_ARBURST,
    input  wire                       M_AXI_ARREADY,

    // Read Data Channel
    input  wire [DATA_WIDTH-1:0]      M_AXI_RDATA,
    input  wire [1:0]                 M_AXI_RRESP,
    input  wire                       M_AXI_RVALID,
    input  wire                       M_AXI_RLAST,
    output wire                       M_AXI_RREADY,

    // Write Address Channel
    output wire [ADDR_WIDTH-1:0]      M_AXI_AWADDR,
    output wire                       M_AXI_AWVALID,
    output wire [7:0]                 M_AXI_AWLEN,
    output wire [2:0]                 M_AXI_AWSIZE,
    output wire [1:0]                 M_AXI_AWBURST,
    input  wire                       M_AXI_AWREADY,

    // Write Data Channel
    output wire [DATA_WIDTH-1:0]      M_AXI_WDATA,
    output wire [(DATA_WIDTH/8)-1:0]  M_AXI_WSTRB,
    output wire                       M_AXI_WVALID,
    output wire                       M_AXI_WLAST,
    input  wire                       M_AXI_WREADY,

    // Write Response Channel
    input  wire [1:0]                 M_AXI_BRESP,
    input  wire                       M_AXI_BVALID,
    output wire                       M_AXI_BREADY
);

// =============================================================================
// Internal Wires - connecting axi_lite_slave outputs to dma_engine inputs
// and dma_engine outputs back to axi_lite_slave inputs
//
// Naming convention: w_ prefix = internal wire
// =============================================================================

// axi_lite_slave → dma_engine
wire [DATA_WIDTH-1:0] w_src_addr;       // source memory address
wire [DATA_WIDTH-1:0] w_dst_addr;       // destination memory address
wire [DATA_WIDTH-1:0] w_transfer_len;   // number of bytes to transfer
wire                  w_start;          // 1-cycle start pulse
wire                  w_ack;            // PS acknowledges done/error

// dma_engine → axi_lite_slave
wire                  w_busy;           // transfer in progress → STATUS[0]
wire                  w_done;           // transfer complete   → STATUS[1]
wire                  w_error;          // AXI error occurred  → STATUS[2]

// =============================================================================
// Submodule 1 - AXI-Lite Slave (Register Map)
//
// Receives AXI-Lite transactions from PS
// Stores register values and exposes them as plain wires to dma_engine
// Assembles STATUS register from dma_engine status outputs
// =============================================================================
axi_lite_slave #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) u_axi_lite_slave (

    // Global
    .S_AXI_ACLK    (clk),
    .S_AXI_ARESETN (rst_n),

    // Write Address Channel
    .S_AXI_AWADDR  (S_AXI_AWADDR),
    .S_AXI_AWVALID (S_AXI_AWVALID),
    .S_AXI_AWREADY (S_AXI_AWREADY),

    // Write Data Channel
    .S_AXI_WDATA   (S_AXI_WDATA),
    .S_AXI_WSTRB   (S_AXI_WSTRB),
    .S_AXI_WVALID  (S_AXI_WVALID),
    .S_AXI_WREADY  (S_AXI_WREADY),

    // Write Response Channel
    .S_AXI_BRESP   (S_AXI_BRESP),
    .S_AXI_BVALID  (S_AXI_BVALID),
    .S_AXI_BREADY  (S_AXI_BREADY),

    // Read Address Channel
    .S_AXI_ARADDR  (S_AXI_ARADDR),
    .S_AXI_ARVALID (S_AXI_ARVALID),
    .S_AXI_ARREADY (S_AXI_ARREADY),

    // Read Data Channel
    .S_AXI_RDATA   (S_AXI_RDATA),
    .S_AXI_RRESP   (S_AXI_RRESP),
    .S_AXI_RVALID  (S_AXI_RVALID),
    .S_AXI_RREADY  (S_AXI_RREADY),

    // Outputs to DMA Engine
    .src_addr      (w_src_addr),
    .dst_addr      (w_dst_addr),
    .transfer_len  (w_transfer_len),
    .start         (w_start),
    .ack           (w_ack),

    // Inputs from DMA Engine
    .busy          (w_busy),
    .done          (w_done),
    .error         (w_error)
);

// =============================================================================
// Submodule 2 - DMA Engine (FSM)
//
// Receives plain control signals from axi_lite_slave
// Drives AXI4 Full Master transactions to DDR memory
// Reports status back to axi_lite_slave
// =============================================================================
dma_engine #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) u_dma_engine (

    // Global
    .clk           (clk),
    .rst_n         (rst_n),

    // Control from axi_lite_slave
    .src_addr      (w_src_addr),
    .dst_addr      (w_dst_addr),
    .transfer_len  (w_transfer_len),
    .start         (w_start),
    .ack           (w_ack),

    // Status back to axi_lite_slave
    .busy          (w_busy),
    .done          (w_done),
    .error         (w_error),

    // AXI4 Full Master - Read Address Channel
    .M_AXI_ARADDR  (M_AXI_ARADDR),
    .M_AXI_ARVALID (M_AXI_ARVALID),
    .M_AXI_ARLEN   (M_AXI_ARLEN),
    .M_AXI_ARSIZE  (M_AXI_ARSIZE),
    .M_AXI_ARBURST (M_AXI_ARBURST),
    .M_AXI_ARREADY (M_AXI_ARREADY),

    // AXI4 Full Master - Read Data Channel
    .M_AXI_RDATA   (M_AXI_RDATA),
    .M_AXI_RRESP   (M_AXI_RRESP),
    .M_AXI_RVALID  (M_AXI_RVALID),
    .M_AXI_RLAST   (M_AXI_RLAST),
    .M_AXI_RREADY  (M_AXI_RREADY),

    // AXI4 Full Master - Write Address Channel
    .M_AXI_AWADDR  (M_AXI_AWADDR),
    .M_AXI_AWVALID (M_AXI_AWVALID),
    .M_AXI_AWLEN   (M_AXI_AWLEN),
    .M_AXI_AWSIZE  (M_AXI_AWSIZE),
    .M_AXI_AWBURST (M_AXI_AWBURST),
    .M_AXI_AWREADY (M_AXI_AWREADY),

    // AXI4 Full Master - Write Data Channel
    .M_AXI_WDATA   (M_AXI_WDATA),
    .M_AXI_WSTRB   (M_AXI_WSTRB),
    .M_AXI_WVALID  (M_AXI_WVALID),
    .M_AXI_WLAST   (M_AXI_WLAST),
    .M_AXI_WREADY  (M_AXI_WREADY),

    // AXI4 Full Master - Write Response Channel
    .M_AXI_BRESP   (M_AXI_BRESP),
    .M_AXI_BVALID  (M_AXI_BVALID),
    .M_AXI_BREADY  (M_AXI_BREADY)
);

endmodule