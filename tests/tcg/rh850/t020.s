# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# CMP flags Z/S/OV/CY and the branch conditions.
    .include "macros.inc"

# Fail with code n..n+3 unless Z, S, OV, CY equal z, s, ov, cy
    .macro FLAGS z, s, ov, cy, n
    .if \z
    bz 7f
    .else
    bnz 7f
    .endif
    FAIL \n
7:  .if \s
    bn 7f
    .else
    bp 7f
    .endif
    FAIL (\n + 1)
7:  .if \ov
    bv 7f
    .else
    bnv 7f
    .endif
    FAIL (\n + 2)
7:  .if \cy
    bc 7f
    .else
    bnc 7f
    .endif
    FAIL (\n + 3)
7:
    .endm

    .text
    # equal: 5 - 5 = 0
    mov 5, r1
    mov 5, r2
    cmp r1, r2
    FLAGS 1, 0, 0, 0, 10
    # less, signed: -1 - 1 = -2, no borrow
    mov 1, r1
    mov -1, r2
    cmp r1, r2
    FLAGS 0, 1, 0, 0, 20
    # less, unsigned: 1 - 2 = -1, borrow
    mov 2, r1
    mov 1, r2
    cmp r1, r2
    FLAGS 0, 1, 0, 1, 30
    # greater: 2 - 1 = 1
    mov 1, r1
    mov 2, r2
    cmp r1, r2
    FLAGS 0, 0, 0, 0, 40
    # overflow: 0x80000000 - 1 = 0x7fffffff
    LI32 2, 0x80000000
    mov 1, r1
    cmp r1, r2
    FLAGS 0, 0, 1, 0, 50
    # overflow: 0x7fffffff - (-1) = 0x80000000, borrow
    LI32 2, 0x7fffffff
    mov -1, r1
    cmp r1, r2
    FLAGS 0, 1, 1, 1, 60
    # CMP imm5: -1 - (-1) = 0
    mov -1, r2
    cmp -1, r2
    FLAGS 1, 0, 0, 0, 70
    # CMP imm5: 0 - 15 = -15, borrow
    mov 0, r2
    cmp 15, r2
    FLAGS 0, 1, 0, 1, 80
    # CMP imm5: 0x80000000 - (-16) = 0x80000010, borrow, no overflow
    LI32 2, 0x80000000
    cmp -16, r2
    FLAGS 0, 1, 0, 1, 90
    # registers unchanged
    CHECK 2, 0x80000000, 100
    # r0 as operand
    cmp r0, r0
    FLAGS 1, 0, 0, 0, 110
    PASS
