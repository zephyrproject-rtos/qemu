# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# MAE: FE level, cause C0H, resumable; always for misaligned LD.DW/ST.DW,
# CAXI, LDL and STC, else with MCTL.MA = 0; MEA and MEI are recorded.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    LI32 1, 0xfe000102
    # LD.DW (48-bit, disp23): LEN 6, REG 14, DS 3, ITYPE 2, read
    SETPSW 0x20
    RESUME d1
i1: ld.dw 0[r1], r14
d1: CHECKFE 0xc0, 0xc0, i1, 0x20, 0xe0, 1
    CHECKR REC_MEA, 0xfe000102, 6
    CHECKR REC_MEI, 0x600e0604, 7
    # ST.DW: write
    SETPSW 0x20
    RESUME d2
i2: st.dw r14, 0[r1]
d2: CHECKFE 0xc0, 0xc0, i2, 0x20, 0xe0, 10
    CHECKR REC_MEI, 0x600e0605, 16
    # CAXI: REG = reg3, DS 2, ITYPE 8, read
    LI32 2, 0xfe000101
    SETPSW 0x20
    RESUME d3
i3: caxi [r2], r4, r3
d3: CHECKFE 0xc0, 0xc0, i3, 0x20, 0xe0, 20
    CHECKR REC_MEA, 0xfe000101, 25
    CHECKR REC_MEI, 0x40030410, 26
    # LDL.W: ITYPE 7
    SETPSW 0x20
    RESUME d4
i4: ldl.w [r1], r5
d4: CHECKFE 0xc0, 0xc0, i4, 0x20, 0xe0, 30
    CHECKR REC_MEI, 0x4005040e, 35
    # LDL.HU at an odd address: DS 1, unsigned
    LI32 2, 0xfe000101
    SETPSW 0x20
    RESUME d5
i5: LDL_HU 2, 6
d5: CHECKFE 0xc0, 0xc0, i5, 0x20, 0xe0, 40
    CHECKR REC_MEI, 0x4006030e, 45
    # STC.W: write
    SETPSW 0x20
    RESUME d6
i6: stc.w r7, [r1]
d6: CHECKFE 0xc0, 0xc0, i6, 0x20, 0xe0, 50
    CHECKR REC_MEI, 0x4007040f, 55
    # MCTL.MA = 0
    ldsr r0, 5, 1
    # LD.DW on a word boundary is allowed
    LI32 2, 0xfe000104
    ld.dw 0[r2], r14
    ld.w 0[r2], r3
    CHECKR REC_CNT, 6, 60
    # LD.W disp16: LEN 4, REG 3, DS 2, ITYPE 1
    SETPSW 0x20
    RESUME d7
i7: ld.w 0[r1], r3
d7: CHECKFE 0xc0, 0xc0, i7, 0x20, 0xe0, 61
    CHECKR REC_MEA, 0xfe000102, 66
    CHECKR REC_MEI, 0x40030402, 67
    # LD.HU disp23 (48-bit): unsigned, ITYPE 2
    LI32 2, 0xfe000101
    SETPSW 0x20
    RESUME d8
i8: ld.hu 0x10000[r2], r8
d8: CHECKFE 0xc0, 0xc0, i8, 0x20, 0xe0, 70
    CHECKR REC_MEA, 0xfe010101, 75
    CHECKR REC_MEI, 0x60080304, 76
    # SST.H (16-bit): LEN 2, ITYPE 0, write
    LI32 30, 0xfe000101
    SETPSW 0x20
    RESUME d9
i9: sst.h r9, 0[ep]
d9: CHECKFE 0xc0, 0xc0, i9, 0x20, 0xe0, 80
    CHECKR REC_MEI, 0x20090201, 85
    # ST.W [reg1]+: ITYPE 4, write, base register not updated
    LI32 2, 0xfe000102
    SETPSW 0x20
    RESUME d10
i10: .hword 0x17e2, (10 << 11) | 0x37a
d10: CHECKFE 0xc0, 0xc0, i10, 0x20, 0xe0, 90
    CHECKR REC_MEI, 0x400a0409, 95
    CHECK 2, 0xfe000102, 96
    # LD.H [reg1]-: signed, ITYPE 5
    LI32 2, 0xfe000101
    SETPSW 0x20
    RESUME d11
i11: .hword 0x27e2, (11 << 11) | 0x374
d11: CHECKFE 0xc0, 0xc0, i11, 0x20, 0xe0, 100
    CHECKR REC_MEI, 0x400b020a, 105
    CHECKR REC_CNT, 11, 106
    # byte access never raises MAE
    ld.b 1[r1], r3
    CHECKR REC_CNT, 11, 107
    PASS
