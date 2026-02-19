# Area Fill Ideas — secp256k1 Bernstein-Yang Inverse

## Current Design Metrics

| Metric            | Value                        |
|-------------------|------------------------------|
| Cells (synthesis) | ~20,300                      |
| Flip-flops        | 1,831                        |
| Core area         | 526,140 um2                  |
| Utilization       | 66.89%                       |
| Remaining budget  | ~174,000 um2 (~10,000 cells) |
| Technology        | IHP sg13g2 (130nm)           |
| Tiles             | 8x2 (Tiny Tapeout)          |
| Clock             | 50 MHz                       |

---

## Speed

### 1. Opportunistic Shift-by-2 [IMPLEMENTED]

When g_next is even after a divstep, the next iteration is trivial (no swap, no
add/sub — just a right shift of g and e, plus delta increment). We fold that
second shift into the same clock cycle.

- **Cells:** ~1,800 (second e-correction adder/subtractor + muxes)
- **FFs:** 0
- **Budget:** ~18%
- **Speedup:** ~33% fewer cycles (measured 497 vs 742 for inv(2))
- **Timing risk:** Low — adds one 258-bit adder in parallel with existing path
- **Status:** Implemented and verified. Test passes, result unchanged.

### 2. Dual Divstep (Deterministic 2x)

Instantiate a second combinational `byang_divstep` unit chained after the first,
executing two full divsteps per cycle unconditionally. Halves iteration count to
371 cycles.

- **Cells:** ~6,000 (full second divstep with 2x 258-bit add/sub, correction, muxes)
- **FFs:** 0
- **Budget:** ~60%
- **Speedup:** 2x (371 cycles fixed)
- **Timing risk:** High — doubles combinational depth on critical path, may not close at 50 MHz
- **Note:** Mutually exclusive with opportunistic shift-by-2 (pick one or the other)

### 3. Wider I/O (16-bit)

Load/unload 16 bits per cycle instead of 8. Halves the 32-cycle serial transfer
overhead to 16 cycles in each direction.

- **Cells:** ~100 (wider shift mux in tt_um_corey.v)
- **FFs:** 0
- **Budget:** ~1%
- **Speedup:** Marginal (saves ~32 cycles total on I/O, <5% of compute time)
- **Timing risk:** None
- **Note:** Requires using more TT I/O pins or multiplexing the existing 8-bit bus

---

## Reliability

### 4. Parity on Datapath Registers (f, g, d, e)

Add a parity bit per 257-bit working register. Compute expected parity from the
divstep outputs, compare against stored parity each cycle. Flag if a single-bit
flip corrupts state mid-computation.

- **Cells:** ~1,500 (4x 257-input XOR reduction trees + comparison logic)
- **FFs:** 4
- **Budget:** ~15%
- **Benefit:** Detects single-event upsets (SEU) on the datapath every cycle
- **Timing risk:** Low — XOR tree is fast, runs in parallel
- **Note:** Detection only, not correction. Could halt computation and flag error.

### 5. TMR on Control Logic (FSM + Counter)

Triple the FSM state register (2-bit x 3) and iteration counter (10-bit x 3)
with majority voting. Protects the most critical control state against bit flips.

- **Cells:** ~150 (majority voters: ~3 gates per bit)
- **FFs:** 30 (2x extra copies of FSM + counter)
- **Budget:** ~2%
- **Benefit:** SEU tolerance on control path — prevents stuck/skipped states
- **Timing risk:** None

### 6. Input Range Check

Verify the input value is in the valid range [1, p-1] before starting
computation. A 256-bit comparator checks a > 0 and a < p, raises an error flag
on invalid input.

- **Cells:** ~600 (256-bit comparator + OR reduction for zero check)
- **FFs:** 1 (error flag)
- **Budget:** ~6%
- **Benefit:** Rejects garbage inputs, prevents undefined behavior
- **Timing risk:** None — runs during IDLE/LOAD, not on critical compute path

---

## Diagnostics

### 7. Built-In Self-Test (BIST)

Hardcode a known test vector (e.g., a=2) and expected answer in ROM. On a BIST
trigger signal, run the full computation and compare output against expected.
Provides a post-fabrication go/no-go signal.

- **Cells:** ~1,200 (256-bit test vector ROM + 256-bit expected ROM + comparator + mux)
- **FFs:** ~10 (BIST state + pass/fail flag)
- **Budget:** ~12%
- **Benefit:** Post-fab silicon validation without external test equipment
- **Timing risk:** None
- **Note:** BIST trigger could be a specific uio pin or a magic input pattern

### 8. Cycle Counter Exposure

Expose the internal iteration counter value through the output interface so
software can verify the computation took the expected number of divsteps.

- **Cells:** ~20 (mux counter onto output bus)
- **FFs:** 0
- **Budget:** <1%
- **Benefit:** Detects stuck/skipped states from software side

### 9. Error/Status Register

Aggregate all error flags (parity errors, range check failures, BIST pass/fail)
into a readable 8-bit status byte on uo_out, accessible via a status read mode.

- **Cells:** ~50 (mux + flag aggregation)
- **FFs:** 8
- **Budget:** ~1%
- **Benefit:** Single read to check overall health
- **Note:** Depends on features 4, 6, 7 being implemented

---

## Bonus — Complementary Crypto

### 10. Modular Multiplier (256-bit iterative Montgomery)

A bit-serial Montgomery multiplier: accumulate over 256 cycles using a single
256-bit adder + conditional subtraction. Enables on-chip verification that
a * inv(a) mod p == 1.

- **Cells:** ~3,000 (256-bit adder + conditional subtractor + control)
- **FFs:** ~600 (operand registers + accumulator)
- **Budget:** ~30%
- **Speedup:** N/A (new capability, 256 cycles per multiply)
- **Timing risk:** Low — simple iterative datapath
- **Note:** Useful for ECDSA point multiplication if paired with point add/double logic

### 11. Ring Oscillator TRNG

A few ring oscillators (3-5 inverter chains) sampled asynchronously for entropy.
XOR outputs together, accumulate into a shift register. Provides a hardware
random number source for nonce generation.

- **Cells:** ~80 (ring oscillators + sampling logic)
- **FFs:** ~35 (entropy shift register)
- **Budget:** ~1%
- **Benefit:** On-chip entropy source for cryptographic nonce generation
- **Timing risk:** None — asynchronous by design
- **Note:** Quality depends on jitter; may need post-processing (von Neumann debiasing)

---

## Suggested Combos

### Combo A — Balanced (speed + reliability + diagnostics)

| Feature                  | Cells | FFs |
|--------------------------|------:|----:|
| Opportunistic shift-by-2 | 1,800 |   0 |
| Parity on datapath       | 1,500 |   4 |
| TMR on control           |   150 |  30 |
| Input range check        |   600 |   1 |
| BIST                     | 1,200 |  10 |
| Status register          |    50 |   8 |
| **Total**                | **5,300** | **53** |

**Estimated utilization: ~84%**

### Combo B — Max speed + diagnostics

| Feature           | Cells | FFs |
|-------------------|------:|----:|
| Dual divstep      | 6,000 |   0 |
| BIST              | 1,200 |  10 |
| Status register   |    50 |   8 |
| Wider I/O         |   100 |   0 |
| **Total**         | **7,350** | **18** |

**Estimated utilization: ~91%**

### Combo C — Kitchen sink (minus dual divstep)

| Feature                  | Cells | FFs |
|--------------------------|------:|----:|
| Opportunistic shift-by-2 | 1,800 |   0 |
| Parity on datapath       | 1,500 |   4 |
| TMR on control           |   150 |  30 |
| Input range check        |   600 |   1 |
| BIST                     | 1,200 |  10 |
| Status register          |    50 |   8 |
| Cycle counter exposure   |    20 |   0 |
| Montgomery multiplier    | 3,000 | 600 |
| Ring oscillator TRNG     |    80 |  35 |
| Wider I/O                |   100 |   0 |
| **Total**                | **8,500** | **688** |

**Estimated utilization: ~95%**

---

All estimates are pre-synthesis (±30%). Yosys optimization and technology mapping
will shift numbers. Run synthesis after each addition to track actual utilization.
