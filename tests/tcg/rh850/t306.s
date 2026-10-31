# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# Register bank, save mode 0: a table-reference EIINT with RBCR0.BE set
# saves the context, RESBANK restores it.
    .include "macros.inc"
    .equ TBL, 0xfe120000
    .equ RBIP, 0xfe121000
    .text
    VECTORS 0
    RECCLR
    LI32 10, TBL
    ldsr r10, 4, 1
    LA 11, th
    LI32 12, TBL + 8 * 4
    st.w r11, 0[r12]
    LI32 10, RBIP
    ldsr r10, 18, 2
    LI32 10, 0x8
    ldsr r10, 15, 2
    EICW 8, 0x1043
    SETPSW 0x00010020
    LI32 1, 0x101
    LI32 2, 0x102
    LI32 3, 0x103
    LI32 4, 0x104
    LI32 5, 0x105
    LI32 6, 0x106
    LI32 7, 0x107
    LI32 8, 0x108
    LI32 9, 0x109
    LI32 10, 0x10a
    LI32 11, 0x10b
    LI32 12, 0x10c
    LI32 13, 0x10d
    LI32 14, 0x10e
    LI32 15, 0x10f
    LI32 16, 0x110
    LI32 17, 0x111
    LI32 18, 0x112
    LI32 19, 0x113
    LI32 30, 0x11e
    ei
n1: CHECKML RBIP - 4, n1, 1
    CHECKM RBIP - 8, 0x00010000, 2
    CHECKM RBIP - 0xc, 0x1008, 3
    stsr 6, r10, 0
    LI32 25, RBIP - 0x10
    ld.w 0[r25], r24
    cmp r10, r24
    be 2f
    FAIL 4
2:
    CHECKM RBIP - 0x14, 0x101, 5
    CHECKM RBIP - 0x18, 0x102, 6
    CHECKM RBIP - 0x1c, 0x103, 7
    CHECKM RBIP - 0x20, 0x104, 8
    CHECKM RBIP - 0x24, 0x105, 9
    CHECKM RBIP - 0x28, 0x106, 10
    CHECKM RBIP - 0x2c, 0x107, 11
    CHECKM RBIP - 0x30, 0x108, 12
    CHECKM RBIP - 0x34, 0x109, 13
    CHECKM RBIP - 0x38, 0x10a, 14
    CHECKM RBIP - 0x3c, 0x10b, 15
    CHECKM RBIP - 0x40, 0x10c, 16
    CHECKM RBIP - 0x44, 0x10d, 17
    CHECKM RBIP - 0x48, 0x10e, 18
    CHECKM RBIP - 0x4c, 0x10f, 19
    CHECKM RBIP - 0x50, 0x110, 20
    CHECKM RBIP - 0x54, 0x111, 21
    CHECKM RBIP - 0x58, 0x112, 22
    CHECKM RBIP - 0x5c, 0x113, 23
    CHECKM RBIP - 0x60, 0x11e, 24
    CHECKM RBIP - 0x64, 0, 25
    CHECK 20, 1, 26
    CHECK 21, 0x00010020, 27
    stsr 17, r10, 2
    CHECK 10, 1, 28
    mov 0, r1
    mov 0, r2
    mov 0, r3
    mov 0, r4
    mov 0, r5
    mov 0, r6
    mov 0, r7
    mov 0, r8
    mov 0, r9
    mov 0, r10
    mov 0, r11
    mov 0, r12
    mov 0, r13
    mov 0, r14
    mov 0, r15
    mov 0, r16
    mov 0, r17
    mov 0, r18
    mov 0, r19
    mov 0, r30
    ldsr r0, 0, 0
    RESBANK
    CHECK 1, 0x101, 30
    CHECK 2, 0x102, 31
    CHECK 3, 0x103, 32
    CHECK 4, 0x104, 33
    CHECK 5, 0x105, 34
    CHECK 6, 0x106, 35
    CHECK 7, 0x107, 36
    CHECK 8, 0x108, 37
    CHECK 9, 0x109, 38
    CHECK 10, 0x10a, 39
    CHECK 11, 0x10b, 40
    CHECK 12, 0x10c, 41
    CHECK 13, 0x10d, 42
    CHECK 14, 0x10e, 43
    CHECK 15, 0x10f, 44
    CHECK 16, 0x110, 45
    CHECK 17, 0x111, 46
    CHECK 18, 0x112, 47
    CHECK 19, 0x113, 48
    CHECK 30, 0x11e, 49
    stsr 17, r10, 2
    CHECK 10, 0, 50
    stsr 13, r10, 0
    CHECK 10, 0x1008, 51
    stsr 0, r10, 0
    CHECKL 10, n1, 52
    PASS
th: stsr 17, r20, 2
    stsr 5, r21, 0
    eiret
