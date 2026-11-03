# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# An FPI with PSW.ID = 1 is reported and later FPU instructions are
# invalidated until EI lets it be taken.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    SETPSW 0x00010020
    LI32 15, 0x05020200
    ldsr r15, 6, 0
    LI32 4, 0x7f800000
    LI32 5, 0xff800000
    LI32 6, 0x55555555
i1: addf.s r5, r4, r6
    CHECKR REC_CNT, 0, 1
    stsr 11, r15, 0
    CHECK 15, 1, 2
    stsr 7, r15, 0
    LA 14, i1
    cmp r14, r15
    be 1f
    FAIL 3
1:  stsr 6, r15, 0
    CHECK 15, 0x05024200, 4
    CHECK 6, 0x55555555, 5
    LI32 7, 0x77777777
    LI32 8, 0x3f800000
    addf.s r8, r8, r7
    CHECK 7, 0x77777777, 6
    addf.s r5, r4, r7
    CHECK 7, 0x77777777, 7
    cmpf.s 2, r8, r8, 1
    cmovf.s 0, r8, r4, r7
    CHECK 7, 0x77777777, 8
    stsr 6, r15, 0
    CHECK 15, 0x05024200, 9
    stsr 11, r15, 0
    CHECK 15, 1, 10
    CHECKR REC_CNT, 0, 11
    trfsr 1
    bnz 1f
    FAIL 12
1:  trfsr 0
    bz 1f
    FAIL 13
1:  LI32 15, 0x07024200
    ldsr r15, 6, 0
    stsr 6, r14, 0
    CHECK 14, 0x07024200, 14
    cmp r0, r8
    trfsr 1
    bz 1f
    FAIL 15
1:  CHECKR REC_CNT, 0, 16
    SETPSW 0x00010020
    ei
e1: CHECKR REC_CNT, 1, 20
    CHECKEI 0x70, 0x72, e1, 0x00010000, 0x00010060, 21
    CHECKRL REC_FPEPC, i1, 26
    CHECKR REC_FPSR, 0x07024200, 27
    CHECKR REC_FPEC, 0, 28
    stsr 11, r15, 0
    CHECK 15, 0, 29
    addf.s r8, r8, r7
    CHECK 7, 0x40000000, 30
    PASS
