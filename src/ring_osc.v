`default_nettype none

// Parameterized gated ring oscillator
// Blackboxed to prevent Yosys from flagging the intentional combinational loop.
// In hardware, this is an odd-length inverter chain with enable gating.
// When enable=0, output is driven low and the ring does not oscillate.
// When enable=1, the ring oscillates at a frequency determined by gate delays.
//
// For simulation (SIM defined), a stub is used — the TRNG module provides
// its own LFSR-based entropy source instead.

`ifdef SIM

// Simulation stub: output is constant 0 (TRNG uses LFSR path instead)
module ring_osc #(
    parameter NUM_INV = 5
) (
    input  wire enable,
    output wire out
);
    assign out = 1'b0;
endmodule

`else

(* blackbox *)
module ring_osc #(
    parameter NUM_INV = 5
) (
    input  wire enable,
    output wire out
);

    (* keep *) wire [NUM_INV-1:0] chain;

    // Gated feedback: first inverter AND'd with enable
    assign chain[0] = enable & ~chain[NUM_INV-1];

    genvar i;
    generate
        for (i = 1; i < NUM_INV; i = i + 1) begin : gen_inv
            assign chain[i] = ~chain[i-1];
        end
    endgenerate

    assign out = chain[NUM_INV-1];

endmodule

`endif
