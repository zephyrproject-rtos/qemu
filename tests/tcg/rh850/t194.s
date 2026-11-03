# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# PSW.CU0 = 0 raises UCPOP for every FPU encoding, before RIE; with
# CU0 = 1 reserved and undefined encodings raise RIE.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    SETPSW 0x20
    RESUME d1
i1: .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
d1: CHECKFE 0x80, 0x80, i1, 0x20, 0xe0, 1
    SETPSW 0x20
    RESUME d2
i2: .hword 0x07e0, 0x0400 | 3 << 1
d2: CHECKFE 0x80, 0x80, i2, 0x20, 0xe0, 10
    SETPSW 0x20
    RESUME d3
i3: .hword 2 << 11 | 0x07e0 | 1, 3 << 11 | 0x0460
d3: CHECKFE 0x80, 0x80, i3, 0x20, 0xe0, 20
    SETPSW 0x20
    RESUME d4
i4: .hword 2 << 11 | 0x07e0 | 1, 0x0400
d4: CHECKFE 0x80, 0x80, i4, 0x20, 0xe0, 30
    SETPSW 0x20
    RESUME d5
i5: .hword 3 << 11 | 0x07e0, 6 << 11 | 0x0458
d5: CHECKFE 0x80, 0x80, i5, 0x20, 0xe0, 40
    # CU0 = 1
    SETPSW 0x00010020
    RESUME d6
i6: .hword 2 << 11 | 0x07e0 | 1, 0x0400
d6: CHECKFE 0x60, 0x60, i6, 0x00010020, 0x000100e0, 50
    SETPSW 0x00010020
    RESUME d7
i7: .hword 3 << 11 | 0x07e0, 6 << 11 | 0x0458
d7: CHECKFE 0x60, 0x60, i7, 0x00010020, 0x000100e0, 60
    CHECKR REC_CNT, 7, 70
    PASS
