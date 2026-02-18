# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

GL_TEST = os.environ.get("GATES") == "yes"

# secp256k1 prime
P = 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F

# wr = uio_in[2], rd = uio_in[3]
WR_BIT = 1 << 2
RD_BIT = 1 << 3


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


async def write_256bit(dut, value):
    """Write a 256-bit value as 32 bytes MSB-first."""
    for i in range(32):
        byte = (value >> (8 * (31 - i))) & 0xFF
        dut.ui_in.value = byte
        await pulse_wr(dut)


async def read_256bit(dut):
    """Read a 256-bit result as 32 bytes MSB-first."""
    result = 0
    # First byte is already on uo_out
    dut._log.info(f"read byte  0: {dut.uo_out.value}")
    result = int(dut.uo_out.value) & 0xFF
    for i in range(31):
        await pulse_rd(dut)
        dut._log.info(f"read byte {i+1:2d}: {dut.uo_out.value}")
        result = (result << 8) | (int(dut.uo_out.value) & 0xFF)
    return result


@cocotb.test()
async def test_inverse(dut):
    """Test modular inverse of 2: inv(2) mod p = (p+1)/2."""
    dut._log.info("Start")

    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Print raw logic values after reset to expose any x bits
    dut._log.info(f"After reset uo_out:  {dut.uo_out.value}")
    dut._log.info(f"After reset uio_out: {dut.uio_out.value}")

    # Verify ready is asserted (uio_out[0])
    assert (int(dut.uio_out.value) & 1) == 1, "ready should be high in LOAD state"

    # Write input: a = 2
    a = 2
    expected = pow(a, P - 2, P)  # Fermat's little theorem: a^(p-2) mod p
    dut._log.info(f"Input:    0x{a:064x}")
    dut._log.info(f"Expected: 0x{expected:064x}")

    await write_256bit(dut, a)

    # Wait for valid (uio_out[1]), log internal registers each cycle (RTL only)
    inv = dut.user_project.u_inv if not GL_TEST else None
    dut._log.info("Waiting for computation to complete...")
    for cycle in range(800):
        await RisingEdge(dut.clk)
        if inv is not None:
            dut._log.info(
                f"cycle {cycle:3d}: "
                f"delta={inv.delta_reg.value} "
                f"f={inv.f_reg.value} "
                f"g={inv.g_reg.value} "
                f"d={inv.d_reg.value} "
                f"e={inv.e_reg.value}"
            )
        if (int(dut.uio_out.value) >> 1) & 1:
            break
    else:
        assert False, "Timed out waiting for valid"

    dut._log.info("Valid asserted, reading result")
    dut._log.info(f"Raw uo_out:  {dut.uo_out.value}")
    dut._log.info(f"Raw uio_out: {dut.uio_out.value}")

    # Read result
    result = await read_256bit(dut)
    dut._log.info(f"Result:   0x{result:064x}")

    # Read 2 extra bytes for cycle count (10 bits, padded to 16)
    await pulse_rd(dut)
    cycle_hi = int(dut.uo_out.value) & 0xFF
    await pulse_rd(dut)
    cycle_lo = int(dut.uo_out.value) & 0xFF
    cycle_count = (cycle_hi << 8) | cycle_lo
    dut._log.info(f"Cycle count: {cycle_count}")

    assert result == expected, f"Mismatch: got 0x{result:064x}, expected 0x{expected:064x}"

    # Verify: a * result mod p == 1
    assert (a * result) % P == 1, "Verification failed: a * inv(a) != 1 mod p"
    dut._log.info("PASS: inv(2) verified")


async def read_result_and_cycles(dut):
    """Read 256-bit result + 2-byte cycle count from shift register."""
    result = 0
    result = int(dut.uo_out.value) & 0xFF
    for i in range(31):
        await pulse_rd(dut)
        result = (result << 8) | (int(dut.uo_out.value) & 0xFF)
    await pulse_rd(dut)
    cycle_hi = int(dut.uo_out.value) & 0xFF
    await pulse_rd(dut)
    cycle_lo = int(dut.uo_out.value) & 0xFF
    return result, (cycle_hi << 8) | cycle_lo


@cocotb.test()
async def test_pipelined_load(dut):
    """Test loading next input during S_BUSY (overlapped I/O)."""
    dut._log.info("Start pipelined load test")

    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    a1 = 7
    a2 = 0xDEADBEEF
    expected1 = pow(a1, P - 2, P)
    expected2 = pow(a2, P - 2, P)

    # Load first input
    dut._log.info(f"Loading a1 = 0x{a1:064x}")
    await write_256bit(dut, a1)

    # Now in S_BUSY — ready should still be high (accepting next input)
    assert (int(dut.uio_out.value) & 1) == 1, "ready should be high during S_BUSY"
    dut._log.info("S_BUSY: ready is high, loading a2 while core computes")

    # Load second input while core is computing first
    dut._log.info(f"Loading a2 = 0x{a2:064x}")
    await write_256bit(dut, a2)

    # ready should now be low (next_loaded)
    assert (int(dut.uio_out.value) & 1) == 0, "ready should be low after next input loaded"
    dut._log.info("next_loaded: ready is low")

    # Wait for first result
    for cycle in range(800):
        await RisingEdge(dut.clk)
        if (int(dut.uio_out.value) >> 1) & 1:
            break
    else:
        assert False, "Timed out waiting for first result"

    dut._log.info(f"First result ready after {cycle} cycles in wait loop")
    result1, cycles1 = await read_result_and_cycles(dut)
    dut._log.info(f"Result1:  0x{result1:064x} ({cycles1} iterations)")

    assert result1 == expected1, f"Result1 mismatch: got 0x{result1:064x}, expected 0x{expected1:064x}"
    assert (a1 * result1) % P == 1, "Verification failed: a1 * inv(a1) != 1 mod p"
    dut._log.info("PASS: inv(a1) verified")

    # Pulse wr to transition from S_READ -> S_BUSY (pick up pipelined result)
    dut._log.info("Pulsing wr to pick up pipelined result")
    dut.ui_in.value = 0
    await pulse_wr(dut)

    # valid should now be low (in S_BUSY waiting for result2)
    assert ((int(dut.uio_out.value) >> 1) & 1) == 0, "valid should be low in S_BUSY"

    # Wait for second result
    for cycle in range(800):
        await RisingEdge(dut.clk)
        if (int(dut.uio_out.value) >> 1) & 1:
            break
    else:
        assert False, "Timed out waiting for second result"

    dut._log.info(f"Second result ready after {cycle} cycles in wait loop")
    result2, cycles2 = await read_result_and_cycles(dut)
    dut._log.info(f"Result2:  0x{result2:064x} ({cycles2} iterations)")

    assert result2 == expected2, f"Result2 mismatch: got 0x{result2:064x}, expected 0x{expected2:064x}"
    assert (a2 * result2) % P == 1, "Verification failed: a2 * inv(a2) != 1 mod p"
    dut._log.info("PASS: inv(a2) verified — pipelined load works")
