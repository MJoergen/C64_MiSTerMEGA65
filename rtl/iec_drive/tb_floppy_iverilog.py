#!/usr/bin/env python3
#
# tb_floppy_iverilog.py
#
# Test helper for tb_fdc1772_image_golden.sv (C64MEGA65 issue #244, M2).
#
# floppy.v (upstream MiSTer, Till Harbaum) uses several module-level signals
# and localparams before their declaration (sec_state, SECTOR_STATE_*, rate,
# step_busy, index_pulse_start, byte_clk_en, ...). Vivado accepts that;
# iverilog -g2012 rejects it ("declared after use"). This script writes a copy
# of floppy.v in which every module-level declaration line (reg / wire /
# localparam) is hoisted, in its original order, directly below the
# `parameter CLK_EN` line. Nothing else moves and no line is changed: the
# script asserts that the output is a pure permutation of the input lines and
# that every declaration is a single line, so the logic simulated is exactly
# the logic in floppy.v -- the same kind of pure declaration reorder that was
# applied to fdc1772.v itself for iverilog.
#
# Usage:  python3 -I tb_floppy_iverilog.py floppy.v > floppy_iverilog.v
#
# C64MEGA65 project, GPLv3.

import re
import sys


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: tb_floppy_iverilog.py <floppy.v>")
    with open(sys.argv[1]) as f:
        lines = f.read().split("\n")

    try:
        anchor = next(i for i, l in enumerate(lines)
                      if re.match(r"^\s*parameter\s+CLK_EN\b", l))
    except StopIteration:
        sys.exit("tb_floppy_iverilog: no 'parameter CLK_EN' line found")

    decl = re.compile(r"^(reg|wire|localparam)\b")
    hoisted, rest = [], []
    depth = 0          # begin/end nesting: only module-level lines are hoisted
    for i, l in enumerate(lines):
        if i <= anchor:
            rest.append(l)
            continue
        code = l.split("//", 1)[0]
        if depth == 0 and decl.match(l):
            if not code.rstrip().endswith(";"):
                sys.exit("tb_floppy_iverilog: multi-line declaration at line %d" % (i + 1))
            hoisted.append(l)
        else:
            rest.append(l)
        depth += len(re.findall(r"\bbegin\b", code)) - len(re.findall(r"\bend\b", code))
        if depth < 0:
            sys.exit("tb_floppy_iverilog: unbalanced begin/end at line %d" % (i + 1))

    out = rest[:anchor + 1] + hoisted + rest[anchor + 1:]
    if sorted(out) != sorted(lines):
        sys.exit("tb_floppy_iverilog: output is not a permutation of the input")
    sys.stdout.write("\n".join(out))


if __name__ == "__main__":
    main()
