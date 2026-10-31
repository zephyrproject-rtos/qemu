# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# EIINT by direct vector: EICn/EEICn reset values, acknowledgment after EI,
# ISPR set and cleared by EIRET, software withdrawal of a request.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    EICR 10, 8
    CHECK 10, 0x008f, 1
    EICR 10, 37
    CHECK 10, 0x808f, 2
    EEICR 10, 8
    CHECK 10, 0x0080000f, 3
    EICW 8, 0x0003
    EICW 8, 0x1003
    EICR 10, 8
    CHECK 10, 0x1003, 4
    EEICR 10, 8
    CHECK 10, 0x10000003, 5
    CHECKR REC_CNT, 0, 6
    SETPSW 0x20
    ei
n1: CHECKR REC_CNT, 1, 7
    CHECKEI 0x130, 0x1008, n1, 0, 0x20, 8
    CHECKR REC_LOG + 12, 0x8, 13
    stsr 10, r10, 2
    CHECK 10, 0, 14
    EICR 10, 8
    CHECK 10, 0x0003, 15
    di
    EEICW 63, 0x0000003f
    EEICR 10, 63
    CHECK 10, 0x0000000f, 16
    EEICW 63, 0x10000000
    EICR 10, 63
    CHECK 10, 0x1000, 17
    SETPSW 0x20
    ei
n2: CHECKR REC_CNT, 2, 18
    CHECKEI 0x100, 0x103f, n2, 0, 0x20, 19
    CHECKR REC_LOG + 16 + 12, 0x1, 24
    di
    EICW 9, 0x1005
    EICW 9, 0x0005
    EICR 10, 9
    CHECK 10, 0x0005, 25
    SETPSW 0x20
    ei
    nop
    CHECKR REC_CNT, 2, 26
    PASS
