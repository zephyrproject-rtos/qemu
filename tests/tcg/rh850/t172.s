# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# TRAP (EI level, 40H + vector) and FETRAP (FE level, 30H + vector) are
# pending and return to the next instruction.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    SETPSW 0x2b
    trap 5
n1: stsr 5, r11, 0
    CHECK 11, 0x2b, 1
    CHECKEI 0x40, 0x45, n1, 0x2b, 0x6b, 2
    # NP = 1 is kept, ID = 0 is set
    SETPSW 0x8b
    trap 0x0f
n2: stsr 5, r11, 0
    CHECK 11, 0x8b, 10
    CHECKEI 0x40, 0x4f, n2, 0x8b, 0xeb, 11
    # TRAP1: vectors 16-31 at 050H
    SETPSW 0x0b
    trap 0x10
n3: CHECKEI 0x50, 0x50, n3, 0x0b, 0x6b, 20
    SETPSW 0x0b
    trap 0x1f
n4: CHECKEI 0x50, 0x5f, n4, 0x0b, 0x6b, 30
    # FETRAP: next instruction is PC + 2
    SETPSW 0x0b
    fetrap 3
n5: stsr 5, r11, 0
    CHECK 11, 0x0b, 40
    CHECKFE 0x30, 0x33, n5, 0x0b, 0xeb, 41
    SETPSW 0x0b
    fetrap 15
n6: CHECKFE 0x30, 0x3f, n6, 0x0b, 0xeb, 50
    CHECKR REC_CNT, 6, 60
    # EBV = 1
    LA 10, ebase_tab
    ldsr r10, 3, 1
    SETPSW 0x00008000
    trap 1
n7: CHECKEI 0x1040, 0x41, n7, 0x8000, 0x8060, 70
    SETPSW 0x00008000
    trap 0x11
n8: CHECKEI 0x1050, 0x51, n8, 0x8000, 0x8060, 80
    SETPSW 0x00008000
    fetrap 1
n9: CHECKFE 0x1030, 0x31, n9, 0x8000, 0x80e0, 90
    CHECKR REC_CNT, 9, 100
    PASS
    .balign 512
ebase_tab:
    VECTORS 0x1000
