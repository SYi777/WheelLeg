"""Run the Keil-built offline control test; no board or CAN connection.

Requires Unicorn, either installed normally or in tmp/control_test_deps.
The image contains the actual balance.c, VMC.c and forward_solution.c.
Only console semihosting is provided. This is a CPU/math test, not an RTOS
timing, CAN-bus or physical-robot simulation.
"""
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tmp" / "control_test_deps"))
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_INTR
from unicorn.arm_const import (
    UC_ARM_REG_SP, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC,
)


def run(path):
    data = Path(path).read_bytes()
    assert data[:7] == b"\x7fELF\x01\x01\x01", "Expected 32-bit little-endian ELF"
    entry, phoff = struct.unpack_from("<II", data, 24)
    phsize, phcount = struct.unpack_from("<HH", data, 42)
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    uc.mem_map(0, 0x100000)
    uc.mem_map(0x20000000, 0x20000)
    uc.mem_map(0xE000E000, 0x1000)
    for i in range(phcount):
        kind, offset, vaddr, _, size, _, _, _ = struct.unpack_from(
            "<8I", data, phoff + i * phsize)
        if kind == 1 and size:
            uc.mem_write(vaddr, data[offset:offset+size])
    uc.reg_write(UC_ARM_REG_C1_C0_2, 0x00F00000)
    uc.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    uc.reg_write(UC_ARM_REG_SP, 0x20010000)
    output, exit_code = [], []

    def word(address):
        return struct.unpack("<I", uc.mem_read(address, 4))[0]

    def string(address):
        result = bytearray()
        while True:
            ch = uc.mem_read(address, 1)[0]
            if ch == 0:
                return result.decode("utf-8", errors="replace")
            result.append(ch)
            address += 1

    def interrupt(_uc, number, _context):
        pc = uc.reg_read(UC_ARM_REG_PC)
        if bytes(uc.mem_read(pc, 2)) == b"\xab\xbe":
            uc.reg_write(UC_ARM_REG_PC, (pc + 2) | 1)
        elif bytes(uc.mem_read(pc - 2, 2)) != b"\xab\xbe":
            raise RuntimeError(f"Unexpected CPU interrupt {number} at {pc:#x}")
        operation = uc.reg_read(UC_ARM_REG_R0)
        argument = uc.reg_read(UC_ARM_REG_R1)
        result = 0
        if operation == 1:  # SYS_OPEN: only the standard console is available.
            filename = bytes(uc.mem_read(word(argument), word(argument+8)))
            result = 1 if filename == b":tt" else 0xFFFFFFFF
        elif operation == 2:  # SYS_CLOSE
            pass
        elif operation == 3:  # SYS_WRITEC
            output.append(bytes(uc.mem_read(argument, 1)).decode("utf-8"))
        elif operation == 4:  # SYS_WRITE0
            output.append(string(argument))
        elif operation == 5:  # SYS_WRITE
            text = bytes(uc.mem_read(word(argument+4), word(argument+8)))
            output.append(text.decode("utf-8", errors="replace"))
        elif operation == 9:  # SYS_ISTTY
            result = 1
        elif operation in (0x10, 0x11):  # SYS_CLOCK / SYS_TIME
            result = 0
        elif operation == 0x13:  # SYS_ERRNO
            result = 0
        elif operation in (0x18, 0x20):  # SYS_EXIT / SYS_EXIT_EXTENDED
            code = word(argument+4) if operation == 0x20 else (
                0 if argument == 0x20026 else 1)
            exit_code.append(code)
            uc.emu_stop()
        else:
            raise RuntimeError(f"Unsupported semihost operation {operation:#x}")
        uc.reg_write(UC_ARM_REG_R0, result)

    uc.hook_add(UC_HOOK_INTR, interrupt)
    try:
        uc.emu_start(entry | 1, 0x100000, timeout=30000000, count=10000000)
    except UcError as exc:
        raise RuntimeError(f"CPU error at PC={uc.reg_read(UC_ARM_REG_PC):#x}: {exc}") from exc
    text = "".join(output)
    print(text, end="")
    if not exit_code:
        raise RuntimeError("Test did not exit within the execution limit")
    if exit_code[0] or "FAIL" in text or "PASS:" not in text:
        raise RuntimeError(f"Control test failed: exit={exit_code[0]}")
    return text


if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv) > 1 else
        ROOT / "tmp" / "control_audit_build" / "control_pair_test.axf")
