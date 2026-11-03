# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# RIE: FE level, cause 60H, offset 060H, resumable; from RIE, reserved and
# undefined encodings, with handler base RBASE or EBASE.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    SETPSW 0x2b
    RESUME d1
i1: .hword 0x0040
d1: stsr 5, r11, 0
    CHECK 11, 0x2b, 1
    CHECKR REC_CNT, 1, 2
    CHECKFE 0x60, 0x60, i1, 0x2b, 0xeb, 3
    # RIE imm5, imm4 (Format X)
    SETPSW 0x2b
    RESUME d2
i2: .hword 0xfff5, 0x0000
d2: CHECKR REC_CNT, 2, 10
    CHECKFE 0x60, 0x60, i2, 0x2b, 0xeb, 11
    # reserved: MOV r1, r0
    SETPSW 0x2b
    RESUME d3
i3: .hword 0x0001
d3: CHECKFE 0x60, 0x60, i3, 0x2b, 0xeb, 20
    # reserved: LOOP with reg1 = r0
    SETPSW 0x2b
    RESUME d4
i4: .hword 0x06e0, 0x0001
d4: CHECKFE 0x60, 0x60, i4, 0x2b, 0xeb, 30
    # opcode 111111 with sub-opcode 0002H: no instruction
    SETPSW 0x2b
    RESUME d5
i5: .hword 0x07e0, 0x0002
d5: CHECKFE 0x60, 0x60, i5, 0x2b, 0xeb, 40
    # EBV = 1: EBASE table; flags and CU0 kept
    LA 10, ebase_tab
    ldsr r10, 3, 1
    SETPSW 0x00018024
    RESUME d6
i6: .hword 0x0040
d6: stsr 5, r11, 0
    CHECK 11, 0x00018024, 50
    CHECKFE 0x1060, 0x60, i6, 0x00018024, 0x000180e4, 51
    CHECKR REC_CNT, 6, 56
    PASS
    .balign 512
ebase_tab:
    VECTORS 0x1000
