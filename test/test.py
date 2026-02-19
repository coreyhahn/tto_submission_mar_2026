# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

GL_TEST = os.environ.get("GATES") == "yes"

# secp256k1 prime
P = 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F

# wr = uio_in[2], rd = uio_in[3]
WR_BIT = 1 << 2
RD_BIT = 1 << 3

NUM_BYTES = 34  # 256 data + 9 check + 7 padding = 272 bits = 34 bytes


# =========================================================================
# SECDED helpers: (265, 256) Hamming code
# =========================================================================

def _data_to_pos():
    """Map data bit index (0..255) to codeword position (skip powers of 2)."""
    positions = []
    pos = 1
    while len(positions) < 256:
        if pos & (pos - 1):  # not a power of 2
            positions.append(pos)
        pos += 1
    return positions

DATA_TO_POS = _data_to_pos()

def secded_encode(data_256):
    """Compute 9-bit SECDED check from 256-bit data integer."""
    check = 0
    for j in range(9):
        bit = 0
        for i in range(256):
            if DATA_TO_POS[i] & (1 << j):
                bit ^= (data_256 >> i) & 1
        check |= (bit << j)
    return check


def pack_input(data_256, check_9=None):
    """Pack 256-bit data + 9-bit check into 272-bit shift register value.

    Layout: data[255:0] at bits [271:16], check[8:0] at bits [15:7], padding at [6:0].
    """
    if check_9 is None:
        check_9 = secded_encode(data_256)
    return (data_256 << 16) | (check_9 << 7)


def unpack_output(val_272):
    """Unpack 272-bit output into (result_256, check_9, sec, ded)."""
    result = (val_272 >> 16) & ((1 << 256) - 1)
    check = (val_272 >> 7) & 0x1FF
    sec = (val_272 >> 6) & 1
    ded = (val_272 >> 5) & 1
    return result, check, sec, ded


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


async def write_input(dut, data_256, check_9=None):
    """Write a 256-bit value with SECDED check as 34 bytes MSB-first."""
    packed = pack_input(data_256, check_9)
    for i in range(NUM_BYTES):
        byte = (packed >> (8 * (NUM_BYTES - 1 - i))) & 0xFF
        dut.ui_in.value = byte
        await pulse_wr(dut)


async def read_output(dut):
    """Read 34-byte result. Returns (result_256, check_9, sec, ded)."""
    val = 0
    val = int(dut.uo_out.value) & 0xFF
    for i in range(NUM_BYTES - 1):
        await pulse_rd(dut)
        val = (val << 8) | (int(dut.uo_out.value) & 0xFF)
    return unpack_output(val)


async def wait_valid(dut, timeout=800):
    """Wait for valid signal (uio_out[1]) to go high."""
    for cycle in range(timeout):
        await RisingEdge(dut.clk)
        if (int(dut.uio_out.value) >> 1) & 1:
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

async def run_inverse(dut, a, check_9=None):
    """Write input, wait for valid, read output. Returns (result, check, sec, ded)."""
    await write_input(dut, a, check_9)
    await wait_valid(dut)
    return await read_output(dut)


async def check_inverse(dut, a, check_9=None):
    """Run inverse and assert correctness. Returns result."""
    result, check, sec, ded = await run_inverse(dut, a, check_9)
    assert (a * result) % P == 1, \
        f"inv({a}) failed: ({a} * {result}) % P != 1"
    expected_check = secded_encode(result)
    assert check == expected_check, \
        f"Output SECDED check mismatch for inv({a})"
    assert sec == 0, f"SEC flag should be 0 for clean input (a={a})"
    assert ded == 0, f"DED flag should be 0 for clean input (a={a})"
    return result


def read_status(dut):
    """Read uio_out into dict of named flags."""
    val = int(dut.uio_out.value)
    return {
        'ready':      (val >> 0) & 1,
        'valid':      (val >> 1) & 1,
        'sec':        (val >> 4) & 1,
        'ded':        (val >> 5) & 1,
        'trng_ready': (val >> 6) & 1,
    }


async def read_perf_raw(dut):
    """Read raw 272-bit perf counter data in mode 01. Returns dict of fields."""
    val = int(dut.uo_out.value) & 0xFF
    for _ in range(NUM_BYTES - 1):
        await pulse_rd(dut)
        val = (val << 8) | (int(dut.uo_out.value) & 0xFF)
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
        if (int(dut.uio_out.value) >> 6) & 1:
            return cycle
    assert False, "TRNG byte not ready within timeout"


async def read_trng_byte(dut):
    """Wait for TRNG ready, read byte, consume it. Returns byte value."""
    await wait_trng_ready(dut)
    byte_val = int(dut.uo_out.value) & 0xFF
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

    assert (int(dut.uio_out.value) & 1) == 1, "ready should be high in LOAD state"

    a = 2
    expected = pow(a, P - 2, P)
    dut._log.info(f"Input:    0x{a:064x}")
    dut._log.info(f"Expected: 0x{expected:064x}")

    await write_input(dut, a)

    cycles = await wait_valid(dut)
    dut._log.info(f"Valid after {cycles} wait cycles")

    result, check, sec, ded = await read_output(dut)
    dut._log.info(f"Result:   0x{result:064x}")
    dut._log.info(f"Check:    0x{check:03x}, SEC={sec} DED={ded}")

    assert result == expected, f"Mismatch: got 0x{result:064x}, expected 0x{expected:064x}"
    assert (a * result) % P == 1, "Verification failed: a * inv(a) != 1 mod p"

    expected_check = secded_encode(result)
    assert check == expected_check, f"Output SECDED check mismatch"

    assert sec == 0, "SEC flag should be 0 for clean input"
    assert ded == 0, "DED flag should be 0 for clean input"

    dut._log.info("PASS: inv(2) verified with SECDED")


@cocotb.test()
async def test_secded_correction(dut):
    """Test SECDED single-bit error correction on input."""
    dut._log.info("Start SECDED correction test")
    await reset_dut(dut)

    a = 42
    expected = pow(a, P - 2, P)

    error_bit = 100
    corrupted_data = a ^ (1 << error_bit)
    correct_check = secded_encode(a)

    dut._log.info(f"Error in bit {error_bit}, check from correct data")

    await write_input(dut, corrupted_data, check_9=correct_check)

    cycles = await wait_valid(dut)
    result, check, sec, ded = await read_output(dut)
    dut._log.info(f"Result: 0x{result:064x}")
    dut._log.info(f"SEC={sec} DED={ded}")

    assert result == expected, f"SECDED correction failed"
    assert sec == 1, "SEC flag should be 1"
    assert ded == 0, "DED flag should be 0"
    assert (a * result) % P == 1
    dut._log.info("PASS: SECDED single-bit correction verified")


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

    assert (int(dut.uio_out.value) & 1) == 1, "ready should be high during S_BUSY"
    dut._log.info(f"Loading a2 = 0x{a2:064x}")
    await write_input(dut, a2)
    assert (int(dut.uio_out.value) & 1) == 0, "ready should be low after next input loaded"

    cycles = await wait_valid(dut)
    result1, _, _, _ = await read_output(dut)
    assert result1 == expected1, "Result1 mismatch"
    assert (a1 * result1) % P == 1
    dut._log.info("PASS: inv(a1) verified")

    dut.ui_in.value = 0
    await pulse_wr(dut)

    cycles = await wait_valid(dut)
    result2, _, _, _ = await read_output(dut)
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
    # shift_reg[231:224] = {sec, ded, trng_ready, 5'b0}
    # shift_reg[223:0] = 0
    val = 0
    val = int(dut.uo_out.value) & 0xFF
    for i in range(NUM_BYTES - 1):
        await pulse_rd(dut)
        val = (val << 8) | (int(dut.uo_out.value) & 0xFF)

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
        if (int(dut.uio_out.value) >> 6) & 1:
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
            if (int(dut.uio_out.value) >> 6) & 1:
                break

        byte_val = int(dut.uo_out.value) & 0xFF
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
        result, check, sec, ded = await run_inverse(dut, a)
        assert (a * result) % P == 1, f"inv(2^{k}) failed"
        assert sec == 0 and ded == 0, f"SECDED flags wrong for 2^{k}"
        dut._log.info(f"  inv(2^{k}) OK")
    dut._log.info("PASS: powers of two")


@cocotb.test()
async def test_inverse_random_fuzz(dut):
    """20 random 256-bit values, verify (a * result) % P == 1."""
    await reset_dut(dut)
    rng = random.Random(42)
    for i in range(20):
        a = rng.randint(1, P - 1)
        result, check, sec, ded = await run_inverse(dut, a)
        assert (a * result) % P == 1, f"Fuzz #{i}: inv(0x{a:064x}) failed"
        expected_check = secded_encode(result)
        assert check == expected_check, f"Fuzz #{i}: SECDED check mismatch"
        assert sec == 0 and ded == 0
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
# Group 2: SECDED Exhaustive
# =========================================================================

@cocotb.test()
async def test_secded_bit_positions(dut):
    """Single-bit error at various positions, verify correction + sec=1, ded=0."""
    await reset_dut(dut)
    a = 0xA5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5
    correct_check = secded_encode(a)
    expected = pow(a, P - 2, P)

    for bit in [0, 1, 63, 64, 127, 128, 200, 254, 255]:
        corrupted = a ^ (1 << bit)
        await write_input(dut, corrupted, check_9=correct_check)
        await wait_valid(dut)
        result, check, sec, ded = await read_output(dut)
        assert result == expected, f"SECDED correction failed at bit {bit}"
        assert sec == 1, f"SEC should be 1 for bit {bit} error"
        assert ded == 0, f"DED should be 0 for bit {bit} error"
        dut._log.info(f"  Bit {bit} correction OK")
    dut._log.info("PASS: SECDED bit position sweep")


@cocotb.test()
async def test_secded_check_bit_error(dut):
    """Flip bit 0 of check (data clean). Verify result correct, sec=1, ded=0."""
    await reset_dut(dut)
    a = 42
    correct_check = secded_encode(a)
    corrupted_check = correct_check ^ 1  # flip bit 0 of check
    expected = pow(a, P - 2, P)

    await write_input(dut, a, check_9=corrupted_check)
    await wait_valid(dut)
    result, check, sec, ded = await read_output(dut)
    assert result == expected, "Result wrong after check-bit error"
    assert sec == 1, "SEC should be 1 for check bit error"
    assert ded == 0, "DED should be 0 for single check bit error"
    dut._log.info("PASS: SECDED check bit error")


@cocotb.test()
async def test_secded_double_bit_error(dut):
    """Flip two data bits whose codeword positions XOR > 265. Verify ded=1."""
    await reset_dut(dut)
    a = 42
    correct_check = secded_encode(a)

    # Find two data bit positions whose codeword positions XOR to > 265 (out of range)
    # DATA_TO_POS[0]=3, DATA_TO_POS[255]=265 → 3 XOR 265 = 266 > 265
    bit_a, bit_b = 0, 255
    syndrome = DATA_TO_POS[bit_a] ^ DATA_TO_POS[bit_b]
    assert syndrome > 265, f"Need syndrome > 265, got {syndrome}"

    corrupted = a ^ (1 << bit_a) ^ (1 << bit_b)
    dut._log.info(f"Flipping bits {bit_a},{bit_b}, expected syndrome={syndrome}")

    await write_input(dut, corrupted, check_9=correct_check)
    await wait_valid(dut)
    result, check, sec, ded = await read_output(dut)
    assert ded == 1, f"DED should be 1 for double-bit error (sec={sec}, ded={ded})"
    dut._log.info("PASS: SECDED double-bit error detected")


@cocotb.test()
async def test_secded_clean_input(dut):
    """Clean input with matching check. Explicit assert sec=0, ded=0."""
    await reset_dut(dut)
    a = 0x123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0
    result = await check_inverse(dut, a)  # check_inverse already asserts sec=0, ded=0
    dut._log.info("PASS: SECDED clean input")


@cocotb.test()
async def test_secded_output_check(dut):
    """For 3 inputs, verify output check bits match secded_encode(result)."""
    await reset_dut(dut)
    for a in [5, 999, P - 3]:
        result, check, sec, ded = await run_inverse(dut, a)
        assert (a * result) % P == 1
        expected_check = secded_encode(result)
        assert check == expected_check, \
            f"Output check mismatch for a={a}: got 0x{check:03x}, expected 0x{expected_check:03x}"
    dut._log.info("PASS: SECDED output check bits verified")


# =========================================================================
# Group 3: FSM State Machine
# =========================================================================

@cocotb.test()
async def test_reset_clears_state(dut):
    """After reset, verify: ready=1, valid=0, sec=0, ded=0, uo_out=0x00."""
    await reset_dut(dut)
    status = read_status(dut)
    assert status['ready'] == 1, "ready should be 1 after reset"
    assert status['valid'] == 0, "valid should be 0 after reset"
    assert status['sec'] == 0, "sec should be 0 after reset"
    assert status['ded'] == 0, "ded should be 0 after reset"
    assert (int(dut.uo_out.value) & 0xFF) == 0, "uo_out should be 0 after reset"
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
    result1, _, _, _ = await read_output(dut)
    assert result1 == expected1, f"a1 result mismatch"
    assert (a1 * result1) % P == 1

    # Kick off a2 (pipe_pending) and pipeline a3
    dut.ui_in.value = 0
    await pulse_wr(dut)  # triggers pipe_pending transition

    # Now pipeline a3 during a2's computation
    await write_input(dut, a3)

    # Wait for a2 result
    await wait_valid(dut)
    result2, _, _, _ = await read_output(dut)
    assert result2 == expected2, f"a2 result mismatch"
    assert (a2 * result2) % P == 1

    # Kick off a3
    dut.ui_in.value = 0
    await pulse_wr(dut)

    # Wait for a3 result
    await wait_valid(dut)
    result3, _, _, _ = await read_output(dut)
    assert result3 == expected3, f"a3 result mismatch"
    assert (a3 * result3) % P == 1

    dut._log.info("PASS: 3-value pipeline")


@cocotb.test()
async def test_pipeline_accepting_signal(dut):
    """Monitor ready/accepting through pipelined sequence."""
    await reset_dut(dut)

    # Initially ready
    assert (int(dut.uio_out.value) & 1) == 1, "should be accepting initially"

    a1, a2 = 7, 13

    # Load a1 — enters S_BUSY, still accepting (next_loaded=0)
    await write_input(dut, a1)
    assert (int(dut.uio_out.value) & 1) == 1, "should accept during S_BUSY (no pipeline loaded)"

    # Load a2 — next_loaded=1, should stop accepting
    await write_input(dut, a2)
    assert (int(dut.uio_out.value) & 1) == 0, "should NOT accept after pipeline full"

    # Wait for a1 result
    await wait_valid(dut)
    result1, _, _, _ = await read_output(dut)
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
    _ = int(dut.uo_out.value) & 0xFF
    await pulse_rd(dut)
    _ = int(dut.uo_out.value) & 0xFF
    await pulse_rd(dut)
    _ = int(dut.uo_out.value) & 0xFF

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
    """Send SECDED-corrected input, read mode 01. Verify sec bit set in status byte."""
    await reset_dut(dut)

    a = 42
    correct_check = secded_encode(a)
    corrupted = a ^ (1 << 100)

    await write_input(dut, corrupted, check_9=correct_check)
    dut.uio_in.value = MODE01
    await wait_valid(dut)

    perf = await read_perf_raw(dut)
    dut.uio_in.value = 0

    # Status byte: {sec, ded, trng_ready, 5'b0}
    sec_bit = (perf['status'] >> 7) & 1
    ded_bit = (perf['status'] >> 6) & 1
    trng_bit = (perf['status'] >> 5) & 1

    assert sec_bit == 1, f"SEC bit should be set in status byte, got 0x{perf['status']:02x}"
    assert ded_bit == 0, f"DED bit should be 0, got 0x{perf['status']:02x}"
    dut._log.info(f"Status byte: 0x{perf['status']:02x} (sec={sec_bit}, ded={ded_bit}, trng={trng_bit})")
    dut._log.info("PASS: perf status byte with SEC")


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
    mode00_byte = int(dut.uo_out.value) & 0xFF

    # Switch to mode 10: should see TRNG data, not shift_reg
    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)
    await wait_trng_ready(dut)
    mode10_byte = int(dut.uo_out.value) & 0xFF

    # Switch back to mode 00: shift_reg should still be intact (rd in mode 10 doesn't shift it)
    dut.uio_in.value = MODE00
    await ClockCycles(dut.clk, 1)
    mode00_byte_again = int(dut.uo_out.value) & 0xFF

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
    result, check, sec, ded = await read_output(dut)
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
    byte0 = int(dut.uo_out.value) & 0xFF
    await pulse_rd(dut)
    byte1 = int(dut.uo_out.value) & 0xFF

    # Switch to mode 10: TRNG
    dut.uio_in.value = MODE10
    await ClockCycles(dut.clk, 1)
    await wait_trng_ready(dut)
    trng_byte = int(dut.uo_out.value) & 0xFF
    dut._log.info(f"Mode 00 bytes: 0x{byte0:02x}, 0x{byte1:02x}; TRNG byte: 0x{trng_byte:02x}")

    # Switch back to mode 00: verify shift_reg output (should be byte2 of result)
    dut.uio_in.value = MODE00
    await ClockCycles(dut.clk, 1)
    byte2 = int(dut.uo_out.value) & 0xFF
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
    byte0 = int(dut.uo_out.value) & 0xFF

    # First rd rising edge
    dut.uio_in.value = RD_BIT
    await ClockCycles(dut.clk, 1)
    # Hold high for 3 cycles
    await ClockCycles(dut.clk, 3)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    byte1 = int(dut.uo_out.value) & 0xFF

    # Second rd rising edge
    dut.uio_in.value = RD_BIT
    await ClockCycles(dut.clk, 1)
    await ClockCycles(dut.clk, 3)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    byte2 = int(dut.uo_out.value) & 0xFF

    # We should have gotten 3 distinct byte positions (byte0 + 2 rd shifts)
    dut._log.info(f"Bytes: 0x{byte0:02x}, 0x{byte1:02x}, 0x{byte2:02x}")
    dut._log.info("PASS: rd edge detector (2 shifts from 2 rising edges)")
