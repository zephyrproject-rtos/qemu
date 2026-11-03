# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# PSW by LDSR/STSR: CU2/CU1 fixed 0, user mode writes only the flags, an
# SV-only LDSR in user mode raises PIE.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    # flags only
    LI32 10, 0x0000003f
    ldsr r10, 5, 0
    stsr 5, r11, 0
    setf 0x2, r20
    setf 0x4, r21
    setf 0x0, r22
    setf 0x1, r23
    setf 0xd, r24
    CHECK 11, 0x0000003f, 1
    CHECK 20, 1, 2
    CHECK 21, 1, 2
    CHECK 22, 1, 2
    CHECK 23, 1, 2
    CHECK 24, 1, 2
    # all bits but UM: CU2/CU1 stay 0, reserved bits stay 0
    LI32 10, 0xbfffffff
    ldsr r10, 5, 0
    stsr 5, r11, 0
    CHECK 11, 0x03f180ff, 3
    LI32 10, 0x00000020
    ldsr r10, 5, 0
    stsr 5, r11, 0
    CHECK 11, 0x00000020, 4
    # enter user mode by LDSR PSW in supervisor mode
    LI32 10, 0x40000020
    ldsr r10, 5, 0
    stsr 5, r11, 0
    CHECK 11, 0x40000020, 5
    # user mode: SV bits ignored, flags written, no PIE
    LI32 10, 0x0000000f
    ldsr r10, 5, 0
    stsr 5, r11, 0
    CHECK 11, 0x4000002f, 6
    # UM registers stay accessible (PEID, CTBP)
    stsr 0, r11, 2
    CHECK 11, 0, 7
    LI32 10, 0x00001234
    ldsr r10, 20, 0
    stsr 20, r11, 0
    CHECK 11, 0x00001234, 8
    # SV-only register in user mode: PIE
    SETPSW 0x0b
    RESUME d1
i1: ldsr r10, 0, 0
d1: CHECKFE 0xa0, 0xa0, i1, 0x4000002b, 0xeb, 9
    CHECKR REC_CNT, 1, 14
    PASS
