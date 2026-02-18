`default_nettype none
`include "byang_pkg.vh"

module byang_inv (
    input  wire                      clk,
    input  wire                      rst_n,

    // Input interface (valid/ready)
    input  wire                      valid_in,
    output wire                      ready_in,
    input  wire [`PRIME_BITS-1:0]    a_in,

    // Output interface (valid/ready)
    output wire                      valid_out,
    input  wire                      ready_out,
    output wire [`PRIME_BITS-1:0]    result,
    output wire [`CTR_WIDTH-1:0]     cycle_count
);

    // =========================================================================
    // State declarations
    // =========================================================================

    localparam [1:0] IDLE    = 2'd0,
                     COMPUTE = 2'd1,
                     DONE    = 2'd2;

    reg [1:0] state;

    // Input register
    reg [`PRIME_BITS-1:0] input_reg;
    reg                   input_valid;

    // Working registers
    reg signed [`OP_WIDTH-1:0]    f_reg, g_reg, d_reg, e_reg;
    reg signed [`DELTA_WIDTH-1:0] delta_reg;

    // Iteration counter
    reg [`CTR_WIDTH-1:0] counter;

    // Output register
    reg [`PRIME_BITS-1:0] output_reg;
    reg                   output_valid;
    reg [`CTR_WIDTH-1:0]  cycle_count_reg;

    // =========================================================================
    // Divstep instance (combinatorial)
    // =========================================================================

    wire signed [`OP_WIDTH-1:0]    f_next, g_next, d_next, e_next;
    wire signed [`DELTA_WIDTH-1:0] delta_next;

    byang_divstep u_divstep (
        .delta_in  (delta_reg),
        .f_in      (f_reg),
        .g_in      (g_reg),
        .d_in      (d_reg),
        .e_in      (e_reg),
        .delta_out (delta_next),
        .f_out     (f_next),
        .g_out     (g_next),
        .d_out     (d_next),
        .e_out     (e_next)
    );

    // =========================================================================
    // Opportunistic double-step logic
    // When g_next is even, the next divstep is trivial (just shift g,e; bump delta)
    // so we fold it into the same clock cycle
    // =========================================================================

    wire g_next_even = ~g_next[0];

    // Second g shift (arithmetic right shift by 1 more)
    wire signed [`OP_WIDTH-1:0] g_double = g_next >>> 1;

    // Second e correction + shift (same pattern as divstep e_corrected)
    reg signed [`OP_WIDTH:0] e_double_raw;
    always @(*) begin
        if (e_next[0]) begin
            if (e_next[`OP_WIDTH-1])
                e_double_raw = {e_next[`OP_WIDTH-1], e_next} + $signed({{2{1'b0}}, `SECP256K1_P});
            else
                e_double_raw = {e_next[`OP_WIDTH-1], e_next} - $signed({{2{1'b0}}, `SECP256K1_P});
        end else begin
            e_double_raw = {e_next[`OP_WIDTH-1], e_next};
        end
    end
    wire signed [`OP_WIDTH-1:0] e_double = e_double_raw >>> 1;

    // Second delta increment
    wire signed [`DELTA_WIDTH-1:0] delta_double = delta_next + 1;

    // =========================================================================
    // Sign correction (combinatorial, used in DONE state)
    // =========================================================================

    reg [`PRIME_BITS-1:0] corrected_result;

    reg signed [`OP_WIDTH-1:0] d_signed;

    always @(*) begin
        d_signed = f_reg[`OP_WIDTH-1] ? (-d_reg) : d_reg;

        if (d_signed[`OP_WIDTH-1])
            corrected_result = d_signed[`PRIME_BITS-1:0] + `SECP256K1_P;
        else if (d_signed >= $signed({1'b0, `SECP256K1_P}))
            corrected_result = d_signed[`PRIME_BITS-1:0] - `SECP256K1_P;
        else
            corrected_result = d_signed[`PRIME_BITS-1:0];
    end

    // =========================================================================
    // Input register (independent of FSM)
    // =========================================================================

    assign ready_in = ~input_valid;

    reg load_input;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_reg   <= 'b0;
            input_valid <= 1'b0;
        end else if (load_input) begin
            input_valid <= 1'b0;
        end else if (valid_in && ready_in) begin
            input_reg   <= a_in;
            input_valid <= 1'b1;
        end
    end

    // =========================================================================
    // Output interface
    // =========================================================================

    assign valid_out = output_valid;
    assign result      = output_reg;
    assign cycle_count = cycle_count_reg;

    // =========================================================================
    // FSM + working registers + output register
    // =========================================================================

    wire output_free;
    assign output_free = ~output_valid | (valid_out & ready_out);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            load_input   <= 1'b0;
            output_valid   <= 1'b0;
            counter        <= {`CTR_WIDTH{1'b0}};
            output_reg     <= {`PRIME_BITS{1'b0}};
            cycle_count_reg <= {`CTR_WIDTH{1'b0}};
        end else begin
            load_input   <= 1'b0;

            // Output handshake: clear when consumed
            if (valid_out && ready_out)
                output_valid <= 1'b0;

            case (state)
                IDLE: begin
                    if (input_valid) begin
                        f_reg      <= $signed({1'b0, `SECP256K1_P});
                        g_reg      <= $signed({1'b0, input_reg});
                        d_reg      <= {`OP_WIDTH{1'b0}};
                        e_reg      <= {{(`OP_WIDTH-1){1'b0}}, 1'b1};
                        delta_reg  <= {{(`DELTA_WIDTH-1){1'b0}}, 1'b1};
                        counter    <= {`CTR_WIDTH{1'b0}};
                        load_input <= 1'b1;
                        state      <= COMPUTE;
                    end
                end

                COMPUTE: begin
                    if (g_next == 0) begin
                        // Early termination: g reached zero
                        f_reg     <= f_next;
                        g_reg     <= g_next;
                        d_reg     <= d_next;
                        e_reg     <= e_next;
                        delta_reg <= delta_next;
                        counter   <= counter + 1'b1;
                        state     <= DONE;
                    end else if (g_next_even && counter < `NUM_ITERS - 1) begin
                        // Double step: divstep + one trivial even shift
                        f_reg     <= f_next;
                        g_reg     <= g_double;
                        d_reg     <= d_next;
                        e_reg     <= e_double;
                        delta_reg <= delta_double;
                        counter   <= counter + 2'd2;

                        if (counter >= `NUM_ITERS - 2)
                            state <= DONE;
                    end else begin
                        // Single step
                        f_reg     <= f_next;
                        g_reg     <= g_next;
                        d_reg     <= d_next;
                        e_reg     <= e_next;
                        delta_reg <= delta_next;
                        counter   <= counter + 1'b1;

                        if (counter == `NUM_ITERS - 1)
                            state <= DONE;
                    end
                end

                DONE: begin
                    if (output_free) begin
                        output_reg      <= corrected_result;
                        cycle_count_reg <= counter;
                        output_valid    <= 1'b1;
                        if (input_valid) begin
                            f_reg      <= $signed({1'b0, `SECP256K1_P});
                            g_reg      <= $signed({1'b0, input_reg});
                            d_reg      <= {`OP_WIDTH{1'b0}};
                            e_reg      <= {{(`OP_WIDTH-1){1'b0}}, 1'b1};
                            delta_reg  <= {{(`DELTA_WIDTH-1){1'b0}}, 1'b1};
                            counter    <= {`CTR_WIDTH{1'b0}};
                            load_input <= 1'b1;
                            state      <= COMPUTE;
                        end else begin
                            state <= IDLE;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
