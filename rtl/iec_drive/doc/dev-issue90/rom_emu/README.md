# 1581-ROM-in-the-loop emulator (issue #90 bring-up tool)

Runs the genuine 1581 DOS ROM (318045-02, from
`CORE/C64_MiSTerMEGA65/rtl/iec_drive/c1581_rom.mif.hex`) on a small Python 6502
against device models that mirror the C64MEGA65 phys-mode RTL semantics (WD1772
front end incl. its busy/DRQ pipeline, CIA senses, controller readiness). This
is how the round-5 hardware failure (DOS error `$09` after six Read Addresses)
was reproduced and root-caused offline: the ROM software-CRC-checks the 6-byte
Read Address reply (`$DA63`, CCITT preset `$B230`), and busy dropping at
PRESENTATION (instead of consumption) of the last byte truncated the reply.

Usage (from this directory):

    python3 - <<PY
    import sys; sys.path.insert(0,'.')
    from machine import Machine
    # convert the ROM once: strip one hex byte per line of c1581_rom.mif.hex
    PY

See `run_boot.py` for the boot harness; `machine.py` holds the device models
(`wd_busy()` is where the presentation-vs-consumption semantics live);
`dis65.py` is the matching disassembler. Useful ROM anchors, found during the
bring-up: `$C343` power-up WD register self test (error `$0D`), `$C104` job
pipeline, `$C900` read-job entry, `$CAE4` sector spin loop, `$CD00` Read
Address (`$CDBC` instant PA1 ready check), `$CE78` seek stage, `$DA63` reply
CRC check (error `$09`), `$B095` spin-up allowance (`$50` dispatcher ticks).

Working notes, deletable; not part of the shipped feature.
