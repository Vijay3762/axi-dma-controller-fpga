`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/10/2026 07:29:07 PM
// Design Name: 
// Module Name: dma_engine
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

module dma_engine #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
) (

    // Global Signals
    input wire clk,
    input wire rst_n, // active LOW reset

    // Control Interface - from axi_lite_slave registers
    input wire [ADDR_WIDTH-1:0] src_addr,      // source memory address
    input wire [ADDR_WIDTH-1:0] dst_addr,      // destination memory address
    input wire [DATA_WIDTH-1:0] transfer_len,  // number of BYTES to transfer
    input wire                  start,         // 1-cycle pulse - begin transfer
    //input wire                  soft_reset,    // PS can reset engine mid-transfer

    // Status Interface - back to axi_lite_slave STATUS register
    output reg busy,  // transfer in progress
    output reg done,  // transfer completed successfully
    output reg error, // AXI error occurred (RESP != OKAY)

    // AXI4 Full Master - Read Address Channel (AR)
    output reg  [ADDR_WIDTH-1:0] M_AXI_ARADDR,   // address to read from
    output reg                   M_AXI_ARVALID,  // we are presenting a valid address
    output reg  [           7:0] M_AXI_ARLEN,    // burst length (0 = 1 beat)
    output reg  [           2:0] M_AXI_ARSIZE,   // beat size (2 = 4 bytes)
    output reg  [           1:0] M_AXI_ARBURST,  // burst type (01 = INCR)
    input  wire                  M_AXI_ARREADY,  // memory is ready to accept address

    // AXI4 Full Master - Read Data Channel (R)
    input  wire [DATA_WIDTH-1:0] M_AXI_RDATA,   // data coming back from memory
    input  wire [           1:0] M_AXI_RRESP,   // response (00=OKAY, 10=SLVERR)
    input  wire                  M_AXI_RVALID,  // memory is presenting valid data
    input  wire                  M_AXI_RLAST,   // last beat of burst
    output reg                   M_AXI_RREADY,  // we are ready to accept data

    // AXI4 Full Master - Write Address Channel (AW)
    output reg  [ADDR_WIDTH-1:0] M_AXI_AWADDR,   // address to write to
    output reg                   M_AXI_AWVALID,  // we are presenting a valid address
    output reg  [           7:0] M_AXI_AWLEN,    // burst length (0 = 1 beat)
    output reg  [           2:0] M_AXI_AWSIZE,   // beat size (2 = 4 bytes)
    output reg  [           1:0] M_AXI_AWBURST,  // burst type (01 = INCR)
    input  wire                  M_AXI_AWREADY,  // memory is ready to accept address

    // AXI4 Full Master - Write Data Channel (W)
    output reg  [    DATA_WIDTH-1:0] M_AXI_WDATA,   // data to write to memory
    output reg  [(DATA_WIDTH/8)-1:0] M_AXI_WSTRB,   // byte enables (always 4'hF here)
    output reg                       M_AXI_WVALID,  // we are presenting valid data
    output reg                       M_AXI_WLAST,   // this is the last beat of burst
    input  wire                      M_AXI_WREADY,  // memory is ready to accept data

    // AXI4 Full Master - Write Response Channel (B)
    input  wire [1:0] M_AXI_BRESP,   // write response (00=OKAY)
    input  wire       M_AXI_BVALID,  // memory is presenting valid response
    output reg        M_AXI_BREADY,  // we are ready to accept response

    input wire ack
);

  localparam  IDLE        = 4'd0,
              READ_ADDR   = 4'd1,
              READ_DATA   = 4'd2,
              WRITE_ADDR  = 4'd3,
              WRITE_DATA  = 4'd4,
              WRITE_RESP  = 4'd5,
              UPDATE      = 4'd6,
              DONE        = 4'd7,
              ERROR       = 4'd8;

  reg [ADDR_WIDTH-1:0] cur_src;
  reg [ADDR_WIDTH-1:0] cur_dst;
  reg [DATA_WIDTH-1:0] count;
  reg [DATA_WIDTH-1:0] data_buf;

  reg [3:0] state, next_state;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= IDLE;
    end else begin
      state <= next_state;
    end
  end

  always @(*) begin
    next_state = state;
    case (state)
      IDLE: begin
        if (start) next_state = READ_ADDR;
        else next_state = IDLE;
      end

      READ_ADDR: begin
        if (M_AXI_ARVALID && M_AXI_ARREADY) next_state = READ_DATA;
        else next_state = READ_ADDR;
      end

      READ_DATA: begin
        if (M_AXI_RVALID && M_AXI_RREADY) begin
          if (M_AXI_RRESP != 2'b00) next_state = ERROR;
          else next_state = WRITE_ADDR;
        end else next_state = READ_DATA;
      end

      WRITE_ADDR: begin
        if (M_AXI_AWVALID && M_AXI_AWREADY) next_state = WRITE_DATA;
        else next_state = WRITE_ADDR;
      end

      WRITE_DATA: begin
        if (M_AXI_WVALID && M_AXI_WREADY) next_state = WRITE_RESP;
        else next_state = WRITE_DATA;
      end

      WRITE_RESP: begin
        if (M_AXI_BVALID && M_AXI_BREADY) begin
          if (M_AXI_BRESP != 2'b00) next_state = ERROR;
          else next_state = UPDATE;
        end else next_state = WRITE_RESP;
      end

      UPDATE: begin
        if (count <= 4) next_state = DONE;
        else next_state = READ_ADDR;
      end

      DONE: begin
        if (ack) next_state = IDLE;
        else next_state = DONE;
      end

      ERROR: begin
        if (ack) next_state = IDLE;
        else next_state = ERROR;
      end
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin

      done          <= 1'b0;
      busy          <= 1'b0;
      error         <= 1'b0;

      cur_src       <= {ADDR_WIDTH{1'b0}};
      cur_dst       <= {ADDR_WIDTH{1'b0}};
      count         <= {DATA_WIDTH{1'b0}};
      data_buf      <= {DATA_WIDTH{1'b0}};

      M_AXI_ARADDR  <= {ADDR_WIDTH{1'b0}};
      M_AXI_ARVALID <= 1'b0;
      M_AXI_ARLEN   <= 8'd0;
      M_AXI_ARSIZE  <= 3'b010;  //size = 4 bytes
      M_AXI_ARBURST <= 2'b01;  //INCR burst type

      M_AXI_RREADY  <= 1'b0;

      M_AXI_AWADDR  <= {ADDR_WIDTH{1'b0}};
      M_AXI_AWVALID <= 1'b0;
      M_AXI_AWLEN   <= 8'd0;
      M_AXI_AWSIZE  <= 3'b010;
      M_AXI_AWBURST <= 2'b01;

      M_AXI_WDATA   <= {DATA_WIDTH{1'b0}};
      M_AXI_WSTRB   <= {(DATA_WIDTH / 8) {1'b0}};
      M_AXI_WVALID  <= 1'b0;
      M_AXI_WLAST   <= 1'b0;

      M_AXI_BREADY  <= 1'b0;
    end else begin
      case (state)
        IDLE: begin
          busy  <= 1'b0;
          done  <= 1'b0;
          error <= 1'b0;
          if (start) begin
            cur_src <= src_addr;
            cur_dst <= dst_addr;
            count   <= transfer_len;
            busy    <= 1'b1;
          end
        end

        READ_ADDR: begin
          M_AXI_ARADDR  <= cur_src;
          M_AXI_ARVALID <= 1'b1;
          M_AXI_ARLEN   <= 8'd0;
          M_AXI_ARSIZE  <= 3'b010;
          M_AXI_ARBURST <= 2'b01;
          if (M_AXI_ARVALID && M_AXI_ARREADY) M_AXI_ARVALID <= 1'b0;
          else M_AXI_ARVALID <= 1'b1;
        end

        READ_DATA: begin
          M_AXI_RREADY <= 1'b1;
          if (M_AXI_RREADY && M_AXI_RVALID) begin
            M_AXI_RREADY <= 1'b0;
            if (M_AXI_RRESP == 2'b00) begin
              data_buf <= M_AXI_RDATA;
            end
          end
        end

        WRITE_ADDR: begin
          M_AXI_AWADDR  <= cur_dst;
          M_AXI_AWVALID <= 1'b1;
          M_AXI_AWLEN   <= 8'd0;
          M_AXI_AWSIZE  <= 3'b010;
          M_AXI_AWBURST <= 2'b01;
          if (M_AXI_AWVALID && M_AXI_AWREADY) M_AXI_AWVALID <= 1'b0;
          else M_AXI_AWVALID <= 1'b1;
        end

        WRITE_DATA: begin
          M_AXI_WDATA  <= data_buf;
          M_AXI_WSTRB  <= 4'hF;  //enabling all 4 bytes
          M_AXI_WVALID <= 1'b1;
          M_AXI_WLAST  <= 1'b1;  // ← always last because AWLEN=0 means 1 beat
          if (M_AXI_WVALID && M_AXI_WREADY) begin
            M_AXI_WVALID <= 1'b0;
            M_AXI_WLAST  <= 1'b0;
          end
        end

        WRITE_RESP: begin
          M_AXI_BREADY <= 1'b1;
          if (M_AXI_BVALID && M_AXI_BREADY) M_AXI_BREADY <= 1'b0;
          else M_AXI_BREADY <= 1'b1;
        end

        UPDATE: begin
          cur_src <= cur_src + 4;
          cur_dst <= cur_dst + 4;
          count   <= count - 4;
        end

        DONE: begin
          done <= 1'b1;
          busy <= 1'b0;
          if (ack) done <= 1'b0;
          else done <= 1'b1;
        end

        ERROR: begin
          error <= 1'b1;
          busy  <= 1'b0;
          if (ack) error <= 1'b0;
          else error <= 1'b1;
        end
      endcase
    end
  end

endmodule