# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# EI clears PSW.ID, DI sets it; in user mode with MCTL.UIC = 0 both raise
# PIE.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    SETPSW 0x0b
    ei
    stsr 5, r11, 0
    CHECK 11, 0x0b, 1
    SETPSW 0x0b
    di
    stsr 5, r11, 0
    CHECK 11, 0x2b, 2
    SETPSW 0x2b
    di
    stsr 5, r11, 0
    CHECK 11, 0x2b, 3
    # user mode, ID = 1, UIC = 0
    SETPSW 0x40000020
    SETPSW 0x0b
    RESUME d1
i1: ei
d1: stsr 5, r11, 0
    CHECK 11, 0x4000002b, 4
    CHECKFE 0xa0, 0xa0, i1, 0x4000002b, 0xeb, 5
    SETPSW 0x0b
    RESUME d2
i2: di
d2: CHECKFE 0xa0, 0xa0, i2, 0x4000002b, 0xeb, 10
    CHECKR REC_CNT, 2, 15
    PASS
