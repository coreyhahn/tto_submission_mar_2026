`default_nettype none
`include "byang_pkg.vh"

module byang_divstep (
    input  wire signed [`DELTA_WIDTH-1:0] delta_in,
    input  wire signed [`OP_WIDTH-1:0]    f_in, g_in, d_in, e_in,

    output wire signed [`DELTA_WIDTH-1:0] delta_out,
    output wire signed [`OP_WIDTH-1:0]    f_out, g_out, d_out, e_out
);

    wire swap;
    wire g_odd;

    assign g_odd = g_in[0];
    assign swap  = (delta_in > 0) && g_odd;

    // --- g computation (258-bit intermediate) ---
    reg signed [`OP_WIDTH:0] g_raw;

    always @(*) begin
        if (swap)       g_raw = g_in - f_in;
        else if (g_odd) g_raw = g_in + f_in;
        else            g_raw = g_in;
    end
    assign g_out = g_raw >>> 1;

    // --- f passthrough or swap ---
    assign f_out = swap ? g_in : f_in;

    // --- e computation (258-bit intermediate + p correction) ---
    reg signed [`OP_WIDTH:0] e_raw;

    always @(*) begin
        if (swap)       e_raw = e_in - d_in;
        else if (g_odd) e_raw = e_in + d_in;
        else            e_raw = e_in;
    end

    reg signed [`OP_WIDTH:0] e_corrected;
    always @(*) begin
        if (e_raw[0]) begin
            if (e_raw[`OP_WIDTH])
                e_corrected = e_raw + $signed({{2{1'b0}}, `SECP256K1_P});
            else
                e_corrected = e_raw - $signed({{2{1'b0}}, `SECP256K1_P});
        end else begin
            e_corrected = e_raw;
        end
    end
    assign e_out = e_corrected >>> 1;

    // --- d passthrough or swap ---
    assign d_out = swap ? e_in : d_in;

    // --- delta update ---
    assign delta_out = swap ? (1 - delta_in) : (delta_in + 1);

endmodule
