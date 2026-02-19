`default_nettype none

// Ring oscillator TRNG with Von Neumann debiasing
// In simulation (SIM defined), uses LFSR for deterministic testability
// In hardware, uses 3 ring oscillators (5, 7, 9 inverters) XOR'd
// Ring oscillators are gated by 'enable' — stopped when low, oscillating when high

module trng (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       enable,    // gates ring oscillators (active high)
    input  wire       consume,   // pulse: consume current byte, start filling next
    output wire [7:0] data_out,
    output wire       ready      // high when 8 debiased bits available
);

    // =========================================================================
    // Entropy source
    // =========================================================================

`ifdef SIM
    // Simulation: LFSR-based pseudo-random for testability
    reg [15:0] lfsr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            lfsr <= 16'hACE1;
        else if (enable)
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[14] ^ lfsr[12] ^ lfsr[3]};
    end
    wire entropy_bit = lfsr[0];
`else
    // Hardware: 3 ring oscillators (5, 7, 9 inverters) built from standard cells
    // Direct cell instantiation avoids Yosys loop detection and unmapped cell errors
    wire ro5_out, ro7_out, ro9_out;

    ring_osc5 u_ro5 (.enable(enable), .out(ro5_out));
    ring_osc7 u_ro7 (.enable(enable), .out(ro7_out));
    ring_osc9 u_ro9 (.enable(enable), .out(ro9_out));

    // XOR all ring outputs for raw entropy
    wire entropy_raw = ro5_out ^ ro7_out ^ ro9_out;

    // Double-flop synchronizer into clock domain
    reg entropy_ff1, entropy_ff2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            entropy_ff1 <= 1'b0;
            entropy_ff2 <= 1'b0;
        end else begin
            entropy_ff1 <= entropy_raw;
            entropy_ff2 <= entropy_ff1;
        end
    end
    wire entropy_bit = entropy_ff2;
`endif

    // =========================================================================
    // Von Neumann debiasing
    // =========================================================================
    // Sample pairs of bits:
    //   (0,1) → output 1
    //   (1,0) → output 0
    //   (0,0) or (1,1) → discard

    reg        prev_sample;
    reg        have_prev;

    // =========================================================================
    // Output shift register
    // =========================================================================

    reg [7:0]  out_reg;
    reg [2:0]  bit_cnt;
    reg        byte_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_sample <= 1'b0;
            have_prev   <= 1'b0;
            out_reg     <= 8'b0;
            bit_cnt     <= 3'b0;
            byte_ready  <= 1'b0;
        end else begin
            if (consume && byte_ready)
                byte_ready <= 1'b0;

            if (!byte_ready) begin
                if (have_prev) begin
                    if (prev_sample != entropy_bit) begin
                        // Valid pair: shift in debiased bit
                        out_reg <= {out_reg[6:0], entropy_bit};
                        if (bit_cnt == 3'd7) begin
                            byte_ready <= 1'b1;
                            bit_cnt    <= 3'd0;
                        end else begin
                            bit_cnt <= bit_cnt + 3'd1;
                        end
                    end
                    have_prev <= 1'b0;
                end else begin
                    prev_sample <= entropy_bit;
                    have_prev   <= 1'b1;
                end
            end
        end
    end

    assign data_out = out_reg;
    assign ready    = byte_ready;

endmodule
