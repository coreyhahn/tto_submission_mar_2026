/*
 * Bernstein-Yang modular inverse accelerator for secp256k1
 * Tiny Tapeout interface wrapper
 *
 * Pin Map:
 *   ui_in[7:0]   = data byte in  (MSB-first, 32 bytes = 256 bits)
 *   uo_out[7:0]  = data byte out (MSB-first)
 *   uio_in[0]    = wr:    rising edge shifts ui_in into input register
 *   uio_in[1]    = rd:    rising edge shifts next output byte onto uo_out
 *   uio_out[0]   = ready: high when accepting input bytes (LOAD state)
 *   uio_out[1]   = valid: high when result available (READ state)
 *
 * Protocol:
 *   1. Write 32 bytes (pulse wr with each byte on ui_in, MSB first)
 *   2. Computation auto-starts after 32nd byte (~742 cycles)
 *   3. Poll valid until high
 *   4. Read MSB byte from uo_out, then pulse rd 31 times for rest
 *   5. Next wr pulse returns to step 1
 *
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`include "byang_pkg.vh"

module tt_um_corey (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // =========================================================================
    // Edge detection on control inputs
    // =========================================================================

    reg wr_prev, rd_prev;
    wire wr_pulse = uio_in[0] & ~wr_prev;
    wire rd_pulse = uio_in[1] & ~rd_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_prev <= 1'b0;
            rd_prev <= 1'b0;
        end else begin
            wr_prev <= uio_in[0];
            rd_prev <= uio_in[1];
        end
    end

    // =========================================================================
    // FSM
    // =========================================================================

    localparam [1:0] S_LOAD = 2'd0,
                     S_BUSY = 2'd1,
                     S_READ = 2'd2;

    reg [1:0]   state;
    reg [4:0]   byte_cnt;
    reg [255:0] shift_reg;
    reg         inv_go;

    // =========================================================================
    // Inverter signals
    // =========================================================================

    wire        inv_ready;
    wire        inv_done;
    wire [255:0] inv_result;

    // =========================================================================
    // Outputs
    // =========================================================================

    assign uio_oe  = 8'h03;
    assign uio_out = {6'b0, (state == S_READ), (state == S_LOAD)};
    assign uo_out  = shift_reg[255:248];

    // =========================================================================
    // Main FSM
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_LOAD;
            byte_cnt <= 5'd0;
            inv_go   <= 1'b0;
        end else begin
            inv_go <= 1'b0;

            case (state)
                S_LOAD: begin
                    if (wr_pulse) begin
                        shift_reg <= {shift_reg[247:0], ui_in};
                        if (byte_cnt == 5'd31) begin
                            byte_cnt <= 5'd0;
                            inv_go   <= 1'b1;
                            state    <= S_BUSY;
                        end else begin
                            byte_cnt <= byte_cnt + 5'd1;
                        end
                    end
                end

                S_BUSY: begin
                    if (inv_done) begin
                        shift_reg <= inv_result;
                        state     <= S_READ;
                    end
                end

                S_READ: begin
                    if (wr_pulse) begin
                        shift_reg <= {shift_reg[247:0], ui_in};
                        byte_cnt  <= 5'd1;
                        state     <= S_LOAD;
                    end else if (rd_pulse) begin
                        shift_reg <= {shift_reg[247:0], 8'b0};
                    end
                end

                default: state <= S_LOAD;
            endcase
        end
    end

    // =========================================================================
    // Inverter instance
    // =========================================================================

    byang_inv u_inv (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (inv_go),
        .ready_in  (inv_ready),
        .a_in      (shift_reg),
        .valid_out (inv_done),
        .ready_out (state == S_BUSY),
        .result    (inv_result)
    );

    wire _unused = &{ena, inv_ready, uio_in[7:2], 1'b0};

endmodule
