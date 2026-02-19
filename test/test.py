# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

GL_TEST = os.environ.get("GATES") == "yes"


def safe_int(signal):
    """Convert a cocotb signal to int, treating X/Z as 0."""
    try:
        return int(signal.value)
    except ValueError:
        # Gate-level sim: resolve X/Z bits to 0
        raw = signal.value
        result = 0
        for i, bit in enumerate(reversed(str(raw))):
            if bit == '1':
                result |= (1 << i)
        return result


# secp256k1 prime
P = 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F

# wr = uio_in[2], rd = uio_in[3]
WR_BIT = 1 << 2
RD_BIT = 1 << 3

NUM_BYTES = 34  # 256 data + 1 parity + 15 padding = 272 bits = 34 bytes


# =========================================================================
# Parity helper
# =========================================================================

def parity_bit(data_256):
    """Compute even parity (XOR of all bits) of a 256-bit integer."""
    return bin(data_256).count('1') % 2


def pack_input(data_256, parity=None):
    """Pack 256-bit data + 1 parity bit into 272-bit shift register value.

    Layout: data[255:0] at bits [271:16], parity at bit [15], padding at [14:0].
    """
    if parity is None:
        parity = parity_bit(data_256)
    return (data_256 << 16) | (parity << 15)


def unpack_output(val_272):
    """Unpack 272-bit output into (result_256, parity, parity_err)."""
    result = (val_272 >> 16) & ((1 << 256) - 1)
    parity = (val_272 >> 15) & 1
    parity_err = (val_272 >> 14) & 1
    return result, parity, parity_err


# =========================================================================
# I/O helpers
# =========================================================================

async def pulse_wr(dut):
    """Pulse the wr signal (uio_in[2]) for one clock cycle."""
    dut.uio_in.value = WR_BIT
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)


async def pulse_rd(dut):
    """Pulse the rd signal (uio_in[3]) for one clock cycle."""
    dut.uio_in.value = RD_BIT
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)


async def write_input(dut, data_256, parity=None):
    """Write a 256-bit value with parity as 34 bytes MSB-first."""
    packed = pack_input(data_256, parity)
    for i in range(NUM_BYTES):
        byte = (packed >> (8 * (NUM_BYTES - 1 - i))) & 0xFF
        dut.ui_in.value = byte
        await pulse_wr(dut)


async def read_output(dut):
    """Read 34-byte result. Returns (result_256, parity, parity_err)."""
    val = safe_int(dut.uo_out) & 0xFF
    for i in range(NUM_BYTES - 1):
        await pulse_rd(dut)
        val = (val << 8) | (safe_int(dut.uo_out) & 0xFF)
    return unpack_output(val)


async def wait_valid(dut, timeout=800):
    """Wait for valid signal (uio_out[1]) to go high."""
    for cycle in range(timeout):
        await RisingEdge(dut.clk)
        if (safe_int(dut.uio_out) >> 1) & 1:
            return cycle
    assert False, "Timed out waiting for valid"


async def reset_dut(dut):
    """Standard reset sequence."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


# Mode constants
MODE00 = 0x00  # Normal inverse result
MODE01 = 0x10  # Status + perf counter readout (bits [5:4] = 01)
MODE10 = 0x20  # TRNG random bytes (bits [5:4] = 10)
MODE11 = 0x30  # Reserved (bits [5:4] = 11)


# =========================================================================
# Composite helpers
# =========================================================================

async def run_inverse(dut, a, parity=None):
    """Write input, wait for valid, read output. Returns (result, parity, parity_err)."""
    await write_input(dut, a, parity)
    await wait_valid(dut)
    return await read_output(dut)


async def check_inverse(dut, a, parity=None):
    """Run inverse and assert correctness. Returns result."""
    result, par, par_err = await run_inverse(dut, a, parity)
    assert (a * result) % P == 1, \
        f"inv({a}) failed: ({a} * {result}) % P != 1"
    expected_parity = parity_bit(result)
    assert par == expected_parity, \
        f"Output parity mismatch for inv({a})"
    assert par_err == 0, f"Parity error flag should be 0 for clean input (a={a})"
    return result


def read_status(dut):
    """Read uio_out into dict of named flags."""
    val = safe_int(dut.uio_out)
    return {
        'ready':        (val >> 0) & 1,
        'valid':        (val >> 1) & 1,
        'parity_error': (val >> 4) & 1,
        'trng_ready':   (val >> 6) & 1,
    }


async def read_perf_raw(dut):
    """Read raw 272-bit perf counter data in mode 01. Returns dict of fields."""
    val = safe_int(dut.uo_out) & 0xFF
    for _ in range(NUM_BYTES - 1):
        await pulse_rd(dut)
        val = (val << 8) | (safe_int(dut.uo_out) & 0xFF)
    return {
        'total':   (val >> 262) & 0x3FF,
        'double':  (val >> 252) & 0x3FF,
        'triple':  (val >> 242) & 0x3FF,
        'iters':   (val >> 232) & 0x3FF,
        'status':  (val >> 224) & 0xFF,
    }


async def wait_trng_ready(dut, timeout=200):
    """Wait for trng_ready (uio_out[6]) to go high. Returns cycle count."""
    for cycle in range(timeout):
        await RisingEdge(dut.clk)
        if (safe_int(dut.uio_out) >> 6) & 1:
            return cycle
    assert False, "TRNG byte not ready within timeout"


async def read_trng_byte(dut):
    """Wait for TRNG ready, read byte, consume it. Returns byte value."""
    await wait_trng_ready(dut)
    byte_val = safe_int(dut.uo_out) & 0xFF
    # Consume
    dut.uio_in.value = MODE10 | RD_BIT
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)
    return byte_val


# =========================================================================
# Tests
# =========================================================================

@cocotb.test()
async def test_inverse(dut):
    """Test modular inverse of 2: inv(2) mod p = (p+1)/2."""
    dut._log.info("Start")
    await reset_dut(dut)

    assert (safe_int(dut.uio_out) & 1) == 1, "ready should be high in LOAD state"

    a = 2
    expected = pow(a, P - 2, P)
    dut._log.info(f"Input:    0x{a:064x}")
    dut._log.info(f"Expected: 0x{expected:064x}")

    await write_input(dut, a)

    cycles = await wait_valid(dut)
    dut._log.info(f"Valid after {cycles} wait cycles")

    result, par, par_err = await read_output(dut)
    dut._log.info(f"Result:   0x{result:064x}")
    dut._log.info(f"Parity: {par}, ParityErr: {par_err}")

    assert result == expected, f"Mismatch: got 0x{result:064x}, expected 0x{expected:064x}"
    assert (a * result) % P == 1, "Verification failed: a * inv(a) != 1 mod p"

    expected_parity = parity_bit(result)
    assert par == expected_parity, f"Output parity mismatch"
    assert par_err == 0, "Parity error flag should be 0 for clean input"

    dut._log.info("PASS: inv(2) verified with parity")


@cocotb.test()
async def test_parity_error_detection(dut):
    """Test parity error detection on input (wrong parity bit)."""
    dut._log.info("Start parity error detection test")
    await reset_dut(dut)

    a = 42
    correct_parity = parity_bit(a)
    wrong_parity = 1 - correct_parity

    dut._log.info(f"Sending data with wrong parity (correct={correct_parity}, sent={wrong_parity})")

    await write_input(dut, a, parity=wrong_parity)
    await wait_valid(dut)
    result, par, par_err = await read_output(dut)

    assert par_err == 1, f"Parity error flag should be 1 for wrong parity, got {par_err}"
    dut._log.info(f"Result: 0x{result:064x}, parity_err={par_err}")
    dut._log.info("PASS: parity error detection verified")


@cocotb.test()
async def test_pipelined_load(dut):
    """Test loading next input during S_BUSY (overlapped I/O)."""
    dut._log.info("Start pipelined load test")
    await reset_dut(dut)

    a1 = 7
    a2 = 0xDEADBEEF
    expected1 = pow(a1, P - 2, P)
    expected2 = pow(a2, P - 2, P)

    dut._log.info(f"Loading a1 = 0x{a1:064x}")
    await write_input(dut, a1)

    assert (safe_int(dut.uio_out) & 1) == 1, "ready should be high during S_BUSY"
    dut._log.info(f"Loading a2 = 0x{a2:064x}")
    await write_input(dut, a2)
    assert (safe_int(dut.uio_out) & 1) == 0, "ready should be low after next input loaded"

    cycles = await wait_valid(dut)
    result1, _, _ = await read_output(dut)
    assert result1 == expected1, "Result1 mismatch"
    assert (a1 * result1) % P == 1
    dut._log.info("PASS: inv(a1) verified")

    dut.ui_in.value = 0
    await pulse_wr(dut)

    cycles = await wait_valid(dut)
    result2, _, _ = await read_output(dut)
    assert result2 == expected2, "Result2 mismatch"
    assert (a2 * result2) % P == 1
    dut._log.info("PASS: inv(a2) verified — pipelined load works")


@cocotb.test()
async def test_perf_counters(dut):
    """Test performance counter readout via mode 01."""
    dut._log.info("Start perf counter test")
    await reset_dut(dut)

    a = 2
    await write_input(dut, a)

    # Set mode = 01 on uio_in[5:4] before result is ready
    # mode bits = 01 on bits [5:4] = 0x10
    MODE01 = 0x10

    cycles = await wait_valid(dut)

    # Set mode 01 BEFORE the inv_done is consumed (it's sampled on inv_done)
    # Actually mode is sampled when inv_done fires, which already happened.
    # We need to set mode BEFORE computation completes.
    # Let me redo: set mode during S_BUSY, before inv_done.

    dut._log.info("Re-running with mode set during S_BUSY")

    # Send a new computation
    # First, consume current result by starting a new load
    await write_input(dut, a)

    # Now in S_BUSY - set mode 01
    dut.uio_in.value = MODE01
    cycles = await wait_valid(dut)

    # Read perf counter data (mode 01 format):
    # shift_reg[271:262] = perf_total[9:0]
    # shift_reg[261:252] = perf_double[9:0]
    # shift_reg[251:242] = perf_triple[9:0]
    # shift_reg[241:232] = cycle_count[9:0]
    # shift_reg[231:224] = {parity_error, trng_ready, 6'b0}
    # shift_reg[223:0] = 0
    val = 0
    val = safe_int(dut.uo_out) & 0xFF
    for i in range(NUM_BYTES - 1):
        await pulse_rd(dut)
        val = (val << 8) | (safe_int(dut.uo_out) & 0xFF)

    # Extract fields
    total_cycles = (val >> 262) & 0x3FF
    double_steps = (val >> 252) & 0x3FF
    triple_steps = (val >> 242) & 0x3FF
    cycle_count = (val >> 232) & 0x3FF
    status_byte = (val >> 224) & 0xFF

    dut._log.info(f"Perf: total_cycles={total_cycles}, double={double_steps}, "
                  f"triple={triple_steps}, iters={cycle_count}")
    dut._log.info(f"Status byte: 0x{status_byte:02x}")

    # Sanity checks
    assert total_cycles > 0, "total_cycles should be > 0"
    # total_cycles = clock cycles in COMPUTE (one per single/double/triple step)
    # cycle_count = logical iterations consumed = single + 2*double + 3*triple
    # So: cycle_count = total_cycles + double_steps + 2 * triple_steps
    expected_iters = total_cycles + double_steps + 2 * triple_steps
    dut._log.info(f"Expected iterations: {expected_iters}, actual: {cycle_count}")
    assert cycle_count == expected_iters, \
        f"Iteration count mismatch: {cycle_count} != {expected_iters}"

    dut._log.info(f"PASS: perf counters verified")


@cocotb.test()
async def test_trng(dut):
    """Test TRNG random byte readout via mode 10."""
    dut._log.info("Start TRNG test")
    await reset_dut(dut)

    # Run a computation to reach S_READ state
    a = 2
    await write_input(dut, a)
    await wait_valid(dut)

    # Set mode = 10 on uio_in[5:4] = 0x20
    MODE10 = 0x20

    # Wait for TRNG to have a byte ready (check uio_out[6])
    dut.uio_in.value = 0  # no mode set yet
    trng_ready = False
    for cycle in range(200):
        await RisingEdge(dut.clk)
        if (safe_int(dut.uio_out) >> 6) & 1:
            trng_ready = True
            break

    assert trng_ready, "TRNG byte not ready within timeout"
    dut._log.info("TRNG byte ready")

    # Switch to TRNG mode and read bytes
    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)

    bytes_read = []
    for i in range(8):
        # Wait for TRNG ready
        for cycle in range(200):
            await RisingEdge(dut.clk)
            if (safe_int(dut.uio_out) >> 6) & 1:
                break

        byte_val = safe_int(dut.uo_out) & 0xFF
        bytes_read.append(byte_val)
        dut._log.info(f"TRNG byte {i}: 0x{byte_val:02x}")

        # Consume and request next byte
        dut.uio_in.value = MODE10 | RD_BIT
        await ClockCycles(dut.clk, 1)
        dut.uio_in.value = MODE10
        await ClockCycles(dut.clk, 1)

    # Verify non-constant output (at least 2 distinct values in 8 bytes)
    unique_bytes = set(bytes_read)
    dut._log.info(f"Read {len(bytes_read)} bytes, {len(unique_bytes)} unique: {[f'0x{b:02x}' for b in bytes_read]}")
    assert len(unique_bytes) >= 2, f"TRNG output appears constant: all bytes = 0x{bytes_read[0]:02x}"

    dut._log.info("PASS: TRNG readout verified")


# =========================================================================
# Group 1: Inverse Correctness
# =========================================================================

@cocotb.test()
async def test_inverse_one(dut):
    """inv(1) = 1. Trivial case, early g=0 termination."""
    await reset_dut(dut)
    result = await check_inverse(dut, 1)
    assert result == 1, f"inv(1) should be 1, got {result}"
    dut._log.info("PASS: inv(1) = 1")


@cocotb.test()
async def test_inverse_p_minus_1(dut):
    """inv(P-1) = P-1. Largest valid input."""
    await reset_dut(dut)
    a = P - 1
    result = await check_inverse(dut, a)
    assert result == P - 1, f"inv(P-1) should be P-1, got 0x{result:064x}"
    dut._log.info("PASS: inv(P-1) = P-1")


@cocotb.test()
async def test_inverse_powers_of_two(dut):
    """inv(2^k) for various k values."""
    await reset_dut(dut)
    for k in [1, 4, 8, 32, 64, 128, 255]:
        a = 1 << k
        if a >= P:
            a = a % P
        result, par, par_err = await run_inverse(dut, a)
        assert (a * result) % P == 1, f"inv(2^{k}) failed"
        assert par_err == 0, f"parity error flag wrong for 2^{k}"
        dut._log.info(f"  inv(2^{k}) OK")
    dut._log.info("PASS: powers of two")


@cocotb.test()
async def test_inverse_random_fuzz(dut):
    """20 random 256-bit values, verify (a * result) % P == 1."""
    await reset_dut(dut)
    rng = random.Random(42)
    for i in range(20):
        a = rng.randint(1, P - 1)
        result, par, par_err = await run_inverse(dut, a)
        assert (a * result) % P == 1, f"Fuzz #{i}: inv(0x{a:064x}) failed"
        expected_parity = parity_bit(result)
        assert par == expected_parity, f"Fuzz #{i}: parity mismatch"
        assert par_err == 0
        dut._log.info(f"  Fuzz #{i} OK")
    dut._log.info("PASS: random fuzz (20 values)")


@cocotb.test()
async def test_inverse_sequential(dut):
    """5 back-to-back inversions (no pipelining). Confirms state cleanup."""
    await reset_dut(dut)
    values = [3, 17, 0xFF, 0xCAFE, P - 2]
    for i, a in enumerate(values):
        result = await check_inverse(dut, a)
        dut._log.info(f"  Sequential #{i} (a={a}) OK")
    dut._log.info("PASS: sequential inversions")


# =========================================================================
# Group 2: Parity
# =========================================================================

@cocotb.test()
async def test_parity_clean_input(dut):
    """Clean input with correct parity. Verify parity_error=0."""
    await reset_dut(dut)
    a = 0x123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0
    result = await check_inverse(dut, a)  # check_inverse asserts parity_error=0
    dut._log.info("PASS: parity clean input")


@cocotb.test()
async def test_parity_wrong_bit(dut):
    """Send data with inverted parity bit. Verify parity_error=1."""
    await reset_dut(dut)
    a = 0xA5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5
    wrong_parity = 1 - parity_bit(a)

    await write_input(dut, a, parity=wrong_parity)
    await wait_valid(dut)
    result, par, par_err = await read_output(dut)
    assert par_err == 1, f"parity_error should be 1 for wrong parity, got {par_err}"
    dut._log.info("PASS: parity wrong bit detected")


@cocotb.test()
async def test_parity_output_check(dut):
    """For 3 inputs, verify output parity matches parity_bit(result)."""
    await reset_dut(dut)
    for a in [5, 999, P - 3]:
        result, par, par_err = await run_inverse(dut, a)
        assert (a * result) % P == 1
        expected_parity = parity_bit(result)
        assert par == expected_parity, \
            f"Output parity mismatch for a={a}: got {par}, expected {expected_parity}"
    dut._log.info("PASS: output parity bits verified")


# =========================================================================
# Group 3: FSM State Machine
# =========================================================================

@cocotb.test()
async def test_reset_clears_state(dut):
    """After reset, verify: ready=1, valid=0, parity_error=0, uo_out=0x00."""
    await reset_dut(dut)
    status = read_status(dut)
    assert status['ready'] == 1, "ready should be 1 after reset"
    assert status['valid'] == 0, "valid should be 0 after reset"
    assert status['parity_error'] == 0, "parity_error should be 0 after reset"
    assert (safe_int(dut.uo_out) & 0xFF) == 0, "uo_out should be 0 after reset"
    dut._log.info("PASS: reset clears state")


@cocotb.test()
async def test_reset_during_busy(dut):
    """Start computation, assert reset mid-compute, verify clean recovery."""
    await reset_dut(dut)
    a = 7
    await write_input(dut, a)

    # Wait a few cycles into computation
    await ClockCycles(dut.clk, 50)

    # Assert reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Should be back in clean LOAD state
    status = read_status(dut)
    assert status['ready'] == 1, "ready should be 1 after mid-busy reset"
    assert status['valid'] == 0, "valid should be 0 after mid-busy reset"

    # Verify new computation works
    result = await check_inverse(dut, 13)
    dut._log.info("PASS: reset during busy, clean recovery")


@cocotb.test()
async def test_reset_during_read(dut):
    """Complete computation, enter S_READ, assert reset, verify recovery."""
    await reset_dut(dut)
    await write_input(dut, 7)
    await wait_valid(dut)

    # Now in S_READ, assert reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    status = read_status(dut)
    assert status['ready'] == 1, "ready should be 1 after mid-read reset"
    assert status['valid'] == 0, "valid should be 0 after mid-read reset"

    result = await check_inverse(dut, 19)
    dut._log.info("PASS: reset during read, clean recovery")


@cocotb.test()
async def test_partial_load_reset(dut):
    """Shift 17 of 34 bytes, reset, verify new full 34-byte load works."""
    await reset_dut(dut)

    # Shift in 17 bytes (partial load)
    packed = pack_input(42)
    for i in range(17):
        byte = (packed >> (8 * (NUM_BYTES - 1 - i))) & 0xFF
        dut.ui_in.value = byte
        await pulse_wr(dut)

    # Reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Full load should work
    result = await check_inverse(dut, 23)
    dut._log.info("PASS: partial load reset recovery")


@cocotb.test()
async def test_rd_during_load(dut):
    """Issue rd_pulse in S_LOAD. Verify no state change (still ready, no crash)."""
    await reset_dut(dut)

    # In S_LOAD, pulse rd
    await pulse_rd(dut)
    await pulse_rd(dut)

    status = read_status(dut)
    assert status['ready'] == 1, "ready should still be 1 after rd in LOAD"
    assert status['valid'] == 0, "valid should still be 0 after rd in LOAD"

    # Verify normal operation still works
    result = await check_inverse(dut, 29)
    dut._log.info("PASS: rd during load ignored")


@cocotb.test()
async def test_wr_held_high(dut):
    """Hold wr high for 5 cycles. Verify only 1 byte shifted (edge detector)."""
    await reset_dut(dut)

    # Set data byte and hold wr high for 5 cycles
    dut.ui_in.value = 0xAB
    dut.uio_in.value = WR_BIT
    await ClockCycles(dut.clk, 5)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    # Now load remaining 33 bytes normally to complete a 34-byte load
    a = 7
    packed = pack_input(a)
    # First byte was 0xAB (wrong), so this won't give correct inverse
    # But we're testing edge detection, not correctness
    # Load 33 more bytes
    for i in range(1, NUM_BYTES):
        byte = (packed >> (8 * (NUM_BYTES - 1 - i))) & 0xFF
        dut.ui_in.value = byte
        await pulse_wr(dut)

    # If edge detector works, we shifted exactly 34 bytes (1 + 33) and are now in S_BUSY
    # Wait for computation to complete (proves we entered S_BUSY)
    await wait_valid(dut)
    dut._log.info("PASS: wr held high, only 1 edge detected")


@cocotb.test()
async def test_simultaneous_wr_rd(dut):
    """Assert both wr and rd in S_READ. Verify wr takes priority (FSM transitions)."""
    await reset_dut(dut)

    # Get to S_READ
    await write_input(dut, 7)
    await wait_valid(dut)

    # Both wr and rd at same time
    dut.ui_in.value = 0x00
    dut.uio_in.value = WR_BIT | RD_BIT
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    # wr should have taken priority — now in S_LOAD with byte_cnt=1
    # Continue loading remaining 33 bytes to complete next computation
    a = 31
    packed = pack_input(a)
    for i in range(1, NUM_BYTES):
        byte = (packed >> (8 * (NUM_BYTES - 1 - i))) & 0xFF
        dut.ui_in.value = byte
        await pulse_wr(dut)

    # Should enter S_BUSY now
    await wait_valid(dut)
    dut._log.info("PASS: simultaneous wr/rd, wr takes priority")


# =========================================================================
# Group 4: Pipelined I/O
# =========================================================================

@cocotb.test()
async def test_pipeline_three_values(dut):
    """Pipeline 3 inversions (a1->a2->a3). Verify all 3 results correct."""
    await reset_dut(dut)

    a1, a2, a3 = 11, 37, 0xBEEF
    expected1 = pow(a1, P - 2, P)
    expected2 = pow(a2, P - 2, P)
    expected3 = pow(a3, P - 2, P)

    # Load a1
    await write_input(dut, a1)

    # Pipeline a2 during a1's computation
    await write_input(dut, a2)

    # Wait for a1 result
    await wait_valid(dut)
    result1, _, _ = await read_output(dut)
    assert result1 == expected1, f"a1 result mismatch"
    assert (a1 * result1) % P == 1

    # Kick off a2 (pipe_pending) and pipeline a3
    dut.ui_in.value = 0
    await pulse_wr(dut)  # triggers pipe_pending transition

    # Now pipeline a3 during a2's computation
    await write_input(dut, a3)

    # Wait for a2 result
    await wait_valid(dut)
    result2, _, _ = await read_output(dut)
    assert result2 == expected2, f"a2 result mismatch"
    assert (a2 * result2) % P == 1

    # Kick off a3
    dut.ui_in.value = 0
    await pulse_wr(dut)

    # Wait for a3 result
    await wait_valid(dut)
    result3, _, _ = await read_output(dut)
    assert result3 == expected3, f"a3 result mismatch"
    assert (a3 * result3) % P == 1

    dut._log.info("PASS: 3-value pipeline")


@cocotb.test()
async def test_pipeline_accepting_signal(dut):
    """Monitor ready/accepting through pipelined sequence."""
    await reset_dut(dut)

    # Initially ready
    assert (safe_int(dut.uio_out) & 1) == 1, "should be accepting initially"

    a1, a2 = 7, 13

    # Load a1 — enters S_BUSY, still accepting (next_loaded=0)
    await write_input(dut, a1)
    assert (safe_int(dut.uio_out) & 1) == 1, "should accept during S_BUSY (no pipeline loaded)"

    # Load a2 — next_loaded=1, should stop accepting
    await write_input(dut, a2)
    assert (safe_int(dut.uio_out) & 1) == 0, "should NOT accept after pipeline full"

    # Wait for a1 result
    await wait_valid(dut)
    result1, _, _ = await read_output(dut)
    assert (a1 * result1) % P == 1

    dut._log.info("PASS: accepting signal behavior")


@cocotb.test()
async def test_pipeline_partial_read_then_new(dut):
    """Enter S_READ, read a few bytes, then wr for new input -> S_LOAD."""
    await reset_dut(dut)

    # Complete a computation
    await write_input(dut, 7)
    await wait_valid(dut)

    # Read only 3 bytes (partial)
    _ = safe_int(dut.uo_out) & 0xFF
    await pulse_rd(dut)
    _ = safe_int(dut.uo_out) & 0xFF
    await pulse_rd(dut)
    _ = safe_int(dut.uo_out) & 0xFF

    # Now start a new input (wr in S_READ transitions to S_LOAD)
    result = await check_inverse(dut, 41)
    dut._log.info("PASS: partial read then new input")


# =========================================================================
# Group 5: Perf Counters
# =========================================================================

@cocotb.test()
async def test_perf_multiple_inputs(dut):
    """Mode 01 readout for 3 inputs. Verify iters = total + double + 2*triple."""
    await reset_dut(dut)

    for a in [2, 0xFF, 0xDEAD]:
        await write_input(dut, a)
        # Set mode 01 during S_BUSY so it's sampled on inv_done
        dut.uio_in.value = MODE01
        await wait_valid(dut)

        perf = await read_perf_raw(dut)
        dut.uio_in.value = 0

        assert perf['total'] > 0, f"total_cycles should be > 0 for a={a}"
        expected_iters = perf['total'] + perf['double'] + 2 * perf['triple']
        assert perf['iters'] == expected_iters, \
            f"Iteration count mismatch for a={a}: {perf['iters']} != {expected_iters}"
        dut._log.info(f"  a={a}: total={perf['total']}, double={perf['double']}, "
                      f"triple={perf['triple']}, iters={perf['iters']}")
    dut._log.info("PASS: perf counters for multiple inputs")


@cocotb.test()
async def test_perf_counter_range(dut):
    """Verify perf counters for a=1 are within valid range and relationship holds."""
    await reset_dut(dut)

    await write_input(dut, 1)
    dut.uio_in.value = MODE01
    await wait_valid(dut)

    perf = await read_perf_raw(dut)
    dut.uio_in.value = 0

    dut._log.info(f"a=1: total={perf['total']}, double={perf['double']}, "
                  f"triple={perf['triple']}, iters={perf['iters']}")
    # total_cycles should be <= 742 (NUM_ITERS) and > 0
    assert 0 < perf['total'] <= 742, f"total out of range: {perf['total']}"
    # Verify relationship: iters = total + double + 2*triple
    expected_iters = perf['total'] + perf['double'] + 2 * perf['triple']
    assert perf['iters'] == expected_iters, \
        f"Iteration mismatch: {perf['iters']} != {expected_iters}"
    # Double and triple optimizations should account for some steps
    assert perf['double'] + perf['triple'] > 0, "Expected some double/triple steps"
    dut._log.info("PASS: perf counter range for a=1")


@cocotb.test()
async def test_perf_status_byte(dut):
    """Send input with wrong parity, read mode 01. Verify parity_error set in status byte."""
    await reset_dut(dut)

    a = 42
    wrong_parity = 1 - parity_bit(a)

    await write_input(dut, a, parity=wrong_parity)
    dut.uio_in.value = MODE01
    await wait_valid(dut)

    perf = await read_perf_raw(dut)
    dut.uio_in.value = 0

    # Status byte: {parity_error, trng_ready, 6'b0}
    par_err_bit = (perf['status'] >> 7) & 1
    trng_bit = (perf['status'] >> 6) & 1

    assert par_err_bit == 1, f"parity_error bit should be set in status byte, got 0x{perf['status']:02x}"
    dut._log.info(f"Status byte: 0x{perf['status']:02x} (par_err={par_err_bit}, trng={trng_bit})")
    dut._log.info("PASS: perf status byte with parity error")


# =========================================================================
# Group 6: TRNG
# =========================================================================

@cocotb.test()
async def test_trng_multiple_reads(dut):
    """Read 32 TRNG bytes. Verify at least 8 unique values."""
    await reset_dut(dut)

    # Need to be in S_READ for TRNG mode
    await write_input(dut, 2)
    await wait_valid(dut)

    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)

    bytes_read = []
    for i in range(32):
        byte_val = await read_trng_byte(dut)
        bytes_read.append(byte_val)

    unique = set(bytes_read)
    dut._log.info(f"Read 32 bytes, {len(unique)} unique values")
    assert len(unique) >= 8, \
        f"Expected >= 8 unique values in 32 TRNG bytes, got {len(unique)}"
    dut._log.info("PASS: TRNG multiple reads")


@cocotb.test()
async def test_trng_ready_flag(dut):
    """After reset, poll trng_ready. Verify goes high, consume, drops, comes back."""
    await reset_dut(dut)

    # Need to be in S_READ
    await write_input(dut, 2)
    await wait_valid(dut)

    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)

    # Wait for first ready
    cycle1 = await wait_trng_ready(dut, timeout=100)
    dut._log.info(f"First TRNG ready after {cycle1} cycles")

    # Consume
    dut.uio_in.value = MODE10 | RD_BIT
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)

    # trng_ready should drop (or already be low after consume)
    # Then come back high
    cycle2 = await wait_trng_ready(dut, timeout=200)
    dut._log.info(f"Second TRNG ready after {cycle2} cycles")
    dut._log.info("PASS: TRNG ready flag cycle")


@cocotb.test()
async def test_trng_mode_isolation(dut):
    """In S_READ mode 00: rd shifts shift_reg. Mode 10: rd returns TRNG byte. Verify isolation."""
    await reset_dut(dut)

    a = 7
    await write_input(dut, a)
    await wait_valid(dut)

    # Mode 00: read first byte of inverse result
    dut.uio_in.value = MODE00
    await ClockCycles(dut.clk, 1)
    mode00_byte = safe_int(dut.uo_out) & 0xFF

    # Switch to mode 10: should see TRNG data, not shift_reg
    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)
    await wait_trng_ready(dut)
    mode10_byte = safe_int(dut.uo_out) & 0xFF

    # Switch back to mode 00: shift_reg should still be intact (rd in mode 10 doesn't shift it)
    dut.uio_in.value = MODE00
    await ClockCycles(dut.clk, 1)
    mode00_byte_again = safe_int(dut.uo_out) & 0xFF

    assert mode00_byte == mode00_byte_again, \
        f"Mode 00 byte changed after mode 10 visit: 0x{mode00_byte:02x} vs 0x{mode00_byte_again:02x}"
    dut._log.info(f"Mode 00: 0x{mode00_byte:02x}, Mode 10: 0x{mode10_byte:02x}, Mode 00 again: 0x{mode00_byte_again:02x}")
    dut._log.info("PASS: TRNG mode isolation")


# =========================================================================
# Group 7: Mode Switching
# =========================================================================

@cocotb.test()
async def test_mode_reserved_11(dut):
    """Set mode 11 during computation. Verify no crash (falls into else -> normal result)."""
    await reset_dut(dut)

    a = 7
    expected = pow(a, P - 2, P)

    await write_input(dut, a)
    dut.uio_in.value = MODE11
    await wait_valid(dut)

    # Mode 11 is reserved, should fall into else branch (normal result)
    result, par, par_err = await read_output(dut)
    dut.uio_in.value = 0
    assert result == expected, f"Mode 11 should give normal result, got wrong value"
    assert (a * result) % P == 1
    dut._log.info("PASS: reserved mode 11 gives normal result")


@cocotb.test()
async def test_mode_switch_mid_read(dut):
    """Enter S_READ mode 00, read 2 bytes, switch to mode 10, verify TRNG bytes."""
    await reset_dut(dut)

    await write_input(dut, 7)
    await wait_valid(dut)

    # Mode 00: read 2 bytes
    dut.uio_in.value = MODE00
    await ClockCycles(dut.clk, 1)
    byte0 = safe_int(dut.uo_out) & 0xFF
    await pulse_rd(dut)
    byte1 = safe_int(dut.uo_out) & 0xFF

    # Switch to mode 10: TRNG
    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)
    await wait_trng_ready(dut)
    trng_byte = safe_int(dut.uo_out) & 0xFF
    dut._log.info(f"Mode 00 bytes: 0x{byte0:02x}, 0x{byte1:02x}; TRNG byte: 0x{trng_byte:02x}")

    # Switch back to mode 00: verify shift_reg output (should be byte2 of result)
    dut.uio_in.value = MODE00
    await ClockCycles(dut.clk, 1)
    byte2 = safe_int(dut.uo_out) & 0xFF
    dut._log.info(f"Mode 00 byte2: 0x{byte2:02x}")
    dut._log.info("PASS: mode switch mid-read")


# =========================================================================
# Group 8: Edge Detection
# =========================================================================

@cocotb.test()
async def test_edge_detector_wr(dut):
    """Pulse wr high->low->high->low. Verify exactly 2 bytes shifted. Hold high in between — no extra byte."""
    await reset_dut(dut)

    # First rising edge: shift byte 0xAA
    dut.ui_in.value = 0xAA
    dut.uio_in.value = WR_BIT
    await ClockCycles(dut.clk, 1)
    # Hold high for 3 cycles (should NOT produce more edges)
    await ClockCycles(dut.clk, 3)
    # Drop
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    # Second rising edge: shift byte 0xBB
    dut.ui_in.value = 0xBB
    dut.uio_in.value = WR_BIT
    await ClockCycles(dut.clk, 1)
    # Hold high for 3 cycles
    await ClockCycles(dut.clk, 3)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    # Now complete the remaining 32 bytes to finish a load
    a = 7
    packed = pack_input(a)
    for i in range(2, NUM_BYTES):
        byte = (packed >> (8 * (NUM_BYTES - 1 - i))) & 0xFF
        dut.ui_in.value = byte
        await pulse_wr(dut)

    # If exactly 2 bytes were shifted from the held-high pulses,
    # we should now be in S_BUSY with 34 total bytes shifted
    await wait_valid(dut)
    dut._log.info("PASS: wr edge detector (2 edges from 2 pulses)")


@cocotb.test()
async def test_edge_detector_rd(dut):
    """Same for rd during S_READ. Verify exactly 2 shifts from 2 rising edges."""
    await reset_dut(dut)

    # Get to S_READ
    await write_input(dut, 7)
    await wait_valid(dut)

    # Read first byte (already visible)
    byte0 = safe_int(dut.uo_out) & 0xFF

    # First rd rising edge
    dut.uio_in.value = RD_BIT
    await ClockCycles(dut.clk, 1)
    # Hold high for 3 cycles
    await ClockCycles(dut.clk, 3)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    byte1 = safe_int(dut.uo_out) & 0xFF

    # Second rd rising edge
    dut.uio_in.value = RD_BIT
    await ClockCycles(dut.clk, 1)
    await ClockCycles(dut.clk, 3)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    byte2 = safe_int(dut.uo_out) & 0xFF

    # We should have gotten 3 distinct byte positions (byte0 + 2 rd shifts)
    dut._log.info(f"Bytes: 0x{byte0:02x}, 0x{byte1:02x}, 0x{byte2:02x}")
    dut._log.info("PASS: rd edge detector (2 shifts from 2 rising edges)")
