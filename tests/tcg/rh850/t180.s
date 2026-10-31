# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# EIRET/FERET restore PC (bit 0 fixed 0) and PSW; EIRET with PSW.EP = 0
# and INTCFG.ISPC = 0 clears the highest priority ISPR bit.
    .include "macros.inc"
    .text
    VECTORS 0
    # EIPSW with CU1 and CU0 set, flags 05H; EIPC odd
    LA 10, e1
    add 1, r10
    ldsr r10, 0, 0
    LI32 10, 0x00030025
    ldsr r10, 1, 0
    .hword 0x07e0, 0x0148
    FAIL 1
    .balign 4
e1: stsr 5, r11, 0
    CHECK 11, 0x00010025, 2
    # FERET: same rules
    LA 10, e2
    add 1, r10
    ldsr r10, 2, 0
    LI32 10, 0x0006802a
    ldsr r10, 3, 0
    .hword 0x07e0, 0x014a
    FAIL 3
    .balign 4
e2: stsr 5, r11, 0
    CHECK 11, 0x0000802a, 4
    # ISPR = 14H (written with ISPC = 1), then ISPC = 0
    mov 1, r10
    ldsr r10, 13, 2
    mov 0x14, r10
    ldsr r10, 10, 2
    ldsr r0, 13, 2
    stsr 10, r11, 2
    CHECK 11, 0x14, 5
    # EIRET with EP = 0: bit 2 cleared
    SETPSW 0x20
    LA 10, e3
    ldsr r10, 0, 0
    LI32 10, 0x20
    ldsr r10, 1, 0
    .hword 0x07e0, 0x0148
e3: stsr 10, r11, 2
    CHECK 11, 0x10, 6
    # EIRET with EP = 1: unchanged
    SETPSW 0x60
    LA 10, e4
    ldsr r10, 0, 0
    .hword 0x07e0, 0x0148
e4: stsr 10, r11, 2
    CHECK 11, 0x10, 7
    # FERET with EP = 0: unchanged
    SETPSW 0x20
    LA 10, e5
    ldsr r10, 2, 0
    LI32 10, 0x20
    ldsr r10, 3, 0
    .hword 0x07e0, 0x014a
e5: stsr 10, r11, 2
    CHECK 11, 0x10, 8
    # EIRET with EP = 0 and ISPC = 1: unchanged
    mov 1, r10
    ldsr r10, 13, 2
    SETPSW 0x20
    LA 10, e6
    ldsr r10, 0, 0
    .hword 0x07e0, 0x0148
e6: stsr 10, r11, 2
    CHECK 11, 0x10, 9
    ldsr r0, 13, 2
    # EP = 0, ISPC = 0: last bit cleared
    SETPSW 0x20
    LA 10, e7
    ldsr r10, 0, 0
    .hword 0x07e0, 0x0148
e7: stsr 10, r11, 2
    CHECK 11, 0, 10
    PASS
