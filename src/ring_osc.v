`default_nettype none

// Gated ring oscillator modules built from standard cells.
// Using direct sg13g2 cell instantiation so Yosys treats them as
// leaf cells — no combinational loop detection, no unmapped cell errors.
//
// When enable=0, the AND gate forces chain[0]=0 and the ring stops.
// When enable=1, the ring oscillates at a frequency set by gate delays.
//
// For simulation (SIM defined), stubs output constant 0 since the
// TRNG module uses its own LFSR-based entropy source.

`ifdef SIM

module ring_osc5 (input wire enable, output wire out);
    assign out = 1'b0;
endmodule

module ring_osc7 (input wire enable, output wire out);
    assign out = 1'b0;
endmodule

module ring_osc9 (input wire enable, output wire out);
    assign out = 1'b0;
endmodule

`else

// 5-inverter gated ring oscillator
module ring_osc5 (
    input  wire enable,
    output wire out
);
    // AND gate + 5 inverters = 5 inversions (odd) → oscillates
    (* keep *) wire [4:0] c;
    wire and_out;
    sg13g2_and2_1 u_and (.A(enable), .B(c[4]),  .X(and_out));
    sg13g2_inv_1  u_i0  (.A(and_out), .Y(c[0]));
    sg13g2_inv_1  u_i1  (.A(c[0]),    .Y(c[1]));
    sg13g2_inv_1  u_i2  (.A(c[1]),    .Y(c[2]));
    sg13g2_inv_1  u_i3  (.A(c[2]),    .Y(c[3]));
    sg13g2_inv_1  u_i4  (.A(c[3]),    .Y(c[4]));

    assign out = c[4];
endmodule

// 7-inverter gated ring oscillator
module ring_osc7 (
    input  wire enable,
    output wire out
);
    (* keep *) wire [6:0] c;

    wire and_out;
    sg13g2_and2_1 u_and (.A(enable), .B(c[6]),  .X(and_out));
    sg13g2_inv_1  u_i0  (.A(and_out), .Y(c[0]));
    sg13g2_inv_1  u_i1  (.A(c[0]),    .Y(c[1]));
    sg13g2_inv_1  u_i2  (.A(c[1]),    .Y(c[2]));
    sg13g2_inv_1  u_i3  (.A(c[2]),    .Y(c[3]));
    sg13g2_inv_1  u_i4  (.A(c[3]),    .Y(c[4]));
    sg13g2_inv_1  u_i5  (.A(c[4]),    .Y(c[5]));
    sg13g2_inv_1  u_i6  (.A(c[5]),    .Y(c[6]));

    assign out = c[6];
endmodule

// 9-inverter gated ring oscillator
module ring_osc9 (
    input  wire enable,
    output wire out
);
    (* keep *) wire [8:0] c;

    wire and_out;
    sg13g2_and2_1 u_and (.A(enable), .B(c[8]),  .X(and_out));
    sg13g2_inv_1  u_i0  (.A(and_out), .Y(c[0]));
    sg13g2_inv_1  u_i1  (.A(c[0]),    .Y(c[1]));
    sg13g2_inv_1  u_i2  (.A(c[1]),    .Y(c[2]));
    sg13g2_inv_1  u_i3  (.A(c[2]),    .Y(c[3]));
    sg13g2_inv_1  u_i4  (.A(c[3]),    .Y(c[4]));
    sg13g2_inv_1  u_i5  (.A(c[4]),    .Y(c[5]));
    sg13g2_inv_1  u_i6  (.A(c[5]),    .Y(c[6]));
    sg13g2_inv_1  u_i7  (.A(c[6]),    .Y(c[7]));
    sg13g2_inv_1  u_i8  (.A(c[7]),    .Y(c[8]));

    assign out = c[8];
endmodule

`endif
