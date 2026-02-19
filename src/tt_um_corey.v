/*
 * Bernstein-Yang modular inverse accelerator for secp256k1
 * Tiny Tapeout interface wrapper
 *
 * Pin Map:
 *   ui_in[7:0]   = data byte in  (MSB-first, 34 bytes = 272 bits)
 *   uo_out[7:0]  = data byte out (MSB-first)
 *   uio_in[2]    = wr:    rising edge shifts ui_in into input register
 *   uio_in[3]    = rd:    rising edge shifts next output byte onto uo_out
 *   uio_in[5:4]  = mode:  output mode select (sampled during S_READ)
 *                    00 = Normal inverse result (34 bytes)
 *                    01 = Status + perf counter readout
 *                    10 = TRNG random bytes
 *                    11 = Reserved
 *   uio_out[0]   = ready: high when accepting input bytes
 *   uio_out[1]   = valid: high when result available (READ state)
 *   uio_out[4]   = sec_corrected: SECDED single-bit correction occurred
 *   uio_out[5]   = ded_detected:  SECDED uncorrectable error detected
 *   uio_out[6]   = trng_ready:    TRNG byte available
 *   ena          = enables TRNG ring oscillators (active high)
 *
 * Input layout (34 bytes, MSB-first):
 *   shift_reg[271:16] = data[255:0]     (bytes 0-31)
 *   shift_reg[15:7]   = check[8:0]      (SECDED check bits)
 *   shift_reg[6:0]    = padding          (ignored)
 *
 * Output layout (34 bytes, MSB-first):
 *   shift_reg[271:16] = result[255:0]   (bytes 0-31)
 *   shift_reg[15:7]   = check[8:0]      (SECDED check, computed from result)
 *   shift_reg[6:0]    = {sec, ded, 5'b0}
 *
 * Copyright (c) 2024 Corey Hahn
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
    wire wr_pulse = uio_in[2] & ~wr_prev;
    wire rd_pulse = uio_in[3] & ~rd_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_prev <= 1'b0;
            rd_prev <= 1'b0;
        end else begin
            wr_prev <= uio_in[2];
            rd_prev <= uio_in[3];
        end
    end

    // =========================================================================
    // FSM
    // =========================================================================

    localparam [1:0] S_LOAD = 2'd0,
                     S_BUSY = 2'd1,
                     S_READ = 2'd2;

    reg [1:0]   state;
    reg [5:0]   byte_cnt;
    reg [271:0] shift_reg;
    reg         inv_go;
    reg         next_loaded;
    reg         pipe_pending;

    // SECDED error flags (latched on inv_go, updated each computation)
    reg sec_corrected;
    reg ded_detected;

    // =========================================================================
    // Inverter signals
    // =========================================================================

    wire        inv_ready;
    wire        inv_done;
    wire [255:0] inv_result;
    wire [`CTR_WIDTH-1:0] inv_cycles;
    wire [`CTR_WIDTH-1:0] perf_total;
    wire [`CTR_WIDTH-1:0] perf_double;
    wire [`CTR_WIDTH-1:0] perf_triple;

    // =========================================================================
    // SECDED: shared parity calculator (muxed for encode/decode)
    // =========================================================================

    // Mux: encode (output path) when inv_done, decode (input path) otherwise
    wire [255:0] parity_data = inv_done ? inv_result : shift_reg[271:16];
    wire [8:0]   parity_check;

    secded_parity u_parity (
        .data  (parity_data),
        .check (parity_check)
    );

    // SECDED decode: correction on input path
    // Valid when parity_data = shift_reg[271:16] (i.e., inv_done == 0)
    wire [255:0] corrected_data;
    wire         sec_raw, ded_raw;

    secded_correct u_correct (
        .data_in    (shift_reg[271:16]),
        .check_in   (shift_reg[15:7]),
        .check_calc (parity_check),
        .data_out   (corrected_data),
        .sec_flag   (sec_raw),
        .ded_flag   (ded_raw)
    );

    // SECDED encode: check bits for output
    // Valid when parity_data = inv_result (i.e., inv_done == 1)
    wire [8:0] output_check = parity_check;

    // =========================================================================
    // Mode select (active during S_READ)
    // =========================================================================

    wire [1:0] mode = uio_in[5:4];

    // =========================================================================
    // TRNG
    // =========================================================================

    wire [7:0] trng_data;
    wire       trng_ready;
    wire       trng_mode = (state == S_READ) && (mode == 2'b10);
    wire       trng_consume = rd_pulse && trng_mode;

    trng u_trng (
        .clk      (clk),
        .rst_n    (rst_n),
        .enable   (ena),
        .consume  (trng_consume),
        .data_out (trng_data),
        .ready    (trng_ready)
    );

    // =========================================================================
    // Outputs
    // =========================================================================

    assign uio_oe  = 8'h73;  // bits 0,1,4,5,6 are outputs
    wire accepting = (state == S_LOAD) || (state == S_BUSY && !next_loaded);
    assign uio_out = {1'b0, trng_ready, ded_detected, sec_corrected, 2'b0, (state == S_READ), accepting};
    assign uo_out  = trng_mode ? trng_data : shift_reg[271:264];

    // =========================================================================
    // Main FSM
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_LOAD;
            byte_cnt      <= 6'd0;
            inv_go        <= 1'b0;
            shift_reg     <= 272'd0;
            next_loaded   <= 1'b0;
            pipe_pending  <= 1'b0;
            sec_corrected <= 1'b0;
            ded_detected  <= 1'b0;
        end else begin
            inv_go <= 1'b0;

            // Latch SECDED flags when data is fully loaded (inv_go pulse cycle)
            // At this point shift_reg has all 34 bytes and SECDED decode is valid
            if (inv_go) begin
                sec_corrected <= sec_raw;
                ded_detected  <= ded_raw;
            end

            case (state)
                S_LOAD: begin
                    if (wr_pulse) begin
                        shift_reg <= {shift_reg[263:0], ui_in};
                        if (byte_cnt == 6'd33) begin
                            byte_cnt <= 6'd0;
                            inv_go   <= 1'b1;
                            state    <= S_BUSY;
                        end else begin
                            byte_cnt <= byte_cnt + 6'd1;
                        end
                    end
                end

                S_BUSY: begin
                    if (inv_done) begin
                        if (mode == 2'b01) begin
                            // Mode 01: perf counter readout
                            // {perf_total[9:0], perf_double[9:0], perf_triple[9:0],
                            //  cycle_count[9:0], status[7:0], 224'b0}
                            shift_reg <= {perf_total, perf_double, perf_triple,
                                          inv_cycles,
                                          sec_corrected, ded_detected, trng_ready, 5'b0,
                                          224'b0};
                        end else begin
                            // Mode 00: normal result with SECDED check
                            shift_reg <= {inv_result, output_check, sec_corrected, ded_detected, 5'b0};
                        end
                        byte_cnt    <= 6'd0;
                        next_loaded <= 1'b0;
                        state       <= S_READ;
                    end else if (wr_pulse && !next_loaded) begin
                        shift_reg <= {shift_reg[263:0], ui_in};
                        if (byte_cnt == 6'd33) begin
                            byte_cnt     <= 6'd0;
                            inv_go       <= 1'b1;
                            next_loaded  <= 1'b1;
                            pipe_pending <= 1'b1;
                        end else begin
                            byte_cnt <= byte_cnt + 6'd1;
                        end
                    end
                end

                S_READ: begin
                    if (wr_pulse && pipe_pending) begin
                        byte_cnt     <= 6'd0;
                        pipe_pending <= 1'b0;
                        state        <= S_BUSY;
                    end else if (wr_pulse) begin
                        shift_reg <= {shift_reg[263:0], ui_in};
                        byte_cnt  <= 6'd1;
                        state     <= S_LOAD;
                    end else if (rd_pulse && mode != 2'b10) begin
                        shift_reg <= {shift_reg[263:0], 8'b0};
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
        .clk             (clk),
        .rst_n           (rst_n),
        .valid_in        (inv_go),
        .ready_in        (inv_ready),
        .a_in            (corrected_data),
        .valid_out       (inv_done),
        .ready_out       (state == S_BUSY),
        .result          (inv_result),
        .cycle_count     (inv_cycles),
        .perf_total_cycles (perf_total),
        .perf_double_steps (perf_double),
        .perf_triple_steps (perf_triple)
    );

    wire _unused = &{inv_ready, uio_in[7], uio_in[1:0], 1'b0};

endmodule
