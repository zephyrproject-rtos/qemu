# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# ABSF/NEGF change only the sign bit, also for NaN; no FP exception.
    .include "macros.inc"
    .text
    SETPSW 0x00010020
    # single 0x00000000
    LI32 2, 0x00000000
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x00000000, 1
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0x80000000, 2
    # single 0x80000000
    LI32 2, 0x80000000
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x00000000, 3
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0x00000000, 4
    # single 0x7f800000
    LI32 2, 0x7f800000
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x7f800000, 5
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0xff800000, 6
    # single 0xff800000
    LI32 2, 0xff800000
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x7f800000, 7
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0x7f800000, 8
    # single 0xffc12345
    LI32 2, 0xffc12345
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x7fc12345, 9
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0x7fc12345, 10
    # single 0x7fc00001
    LI32 2, 0x7fc00001
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x7fc00001, 11
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0xffc00001, 12
    # single 0x7f812345
    LI32 2, 0x7f812345
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x7f812345, 13
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0xff812345, 14
    # single 0xff812345
    LI32 2, 0xff812345
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x7f812345, 15
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0x7f812345, 16
    # single 0x80000001
    LI32 2, 0x80000001
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x00000001, 17
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0x00000001, 18
    # single 0x00000001
    LI32 2, 0x00000001
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x00000001, 19
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0x80000001, 20
    # single 0xc0490fdb
    LI32 2, 0xc0490fdb
    .hword 2 << 11 | 0x07e0, 3 << 11 | 0x0448
    CHECK 3, 0x40490fdb, 21
    .hword 2 << 11 | 0x07e1, 3 << 11 | 0x0448
    CHECK 3, 0x40490fdb, 22
    # double 0x0000000000000000
    LI32 4, 0x00000000
    LI32 5, 0x00000000
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x00000000, 23
    CHECK 7, 0x00000000, 24
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x00000000, 25
    CHECK 7, 0x80000000, 26
    # double 0x8000000000000000
    LI32 4, 0x00000000
    LI32 5, 0x80000000
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x00000000, 27
    CHECK 7, 0x00000000, 28
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x00000000, 29
    CHECK 7, 0x00000000, 30
    # double 0x7ff0000000000000
    LI32 4, 0x00000000
    LI32 5, 0x7ff00000
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x00000000, 31
    CHECK 7, 0x7ff00000, 32
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x00000000, 33
    CHECK 7, 0xfff00000, 34
    # double 0xfff0000000000000
    LI32 4, 0x00000000
    LI32 5, 0xfff00000
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x00000000, 35
    CHECK 7, 0x7ff00000, 36
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x00000000, 37
    CHECK 7, 0x7ff00000, 38
    # double 0xfff8000012345678
    LI32 4, 0x12345678
    LI32 5, 0xfff80000
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x12345678, 39
    CHECK 7, 0x7ff80000, 40
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x12345678, 41
    CHECK 7, 0x7ff80000, 42
    # double 0x7ff0000000000001
    LI32 4, 0x00000001
    LI32 5, 0x7ff00000
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x00000001, 43
    CHECK 7, 0x7ff00000, 44
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x00000001, 45
    CHECK 7, 0xfff00000, 46
    # double 0xfff0000000000001
    LI32 4, 0x00000001
    LI32 5, 0xfff00000
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x00000001, 47
    CHECK 7, 0x7ff00000, 48
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x00000001, 49
    CHECK 7, 0x7ff00000, 50
    # double 0x8000000000000001
    LI32 4, 0x00000001
    LI32 5, 0x80000000
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x00000001, 51
    CHECK 7, 0x00000000, 52
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x00000001, 53
    CHECK 7, 0x00000000, 54
    # double 0x000fffffffffffff
    LI32 4, 0xffffffff
    LI32 5, 0x000fffff
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0xffffffff, 55
    CHECK 7, 0x000fffff, 56
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0xffffffff, 57
    CHECK 7, 0x800fffff, 58
    # double 0xc00921fb54442d18
    LI32 4, 0x54442d18
    LI32 5, 0xc00921fb
    .hword 4 << 11 | 0x07e0, 6 << 11 | 0x0458
    CHECK 6, 0x54442d18, 59
    CHECK 7, 0x400921fb, 60
    .hword 4 << 11 | 0x07e1, 6 << 11 | 0x0458
    CHECK 6, 0x54442d18, 61
    CHECK 7, 0x400921fb, 62
    # no flag, no flush (FS = 1)
    stsr 6, r11, 0
    CHECK 11, 0x00020000, 63
    PASS
