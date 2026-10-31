# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# MPU area registers by MPIDX (index 24 undefined), the SVLOCK.SVL write
# lock, ISPR written by LDSR only with INTCFG.ISPC = 1.
    .include "macros.inc"
    .text
    # MPIDX = 0
    ldsr r0, 16, 5
    LI32 10, 0x00001000
    ldsr r10, 20, 5
    LI32 10, 0x00001ffc
    ldsr r10, 21, 5
    LI32 10, 0xffffffff
    ldsr r10, 22, 5
    # MPIDX = 23
    mov 23, r12
    ldsr r12, 16, 5
    LI32 10, 0x00002000
    ldsr r10, 20, 5
    LI32 10, 0x00002ffc
    ldsr r10, 21, 5
    LI32 10, 0x00000007
    ldsr r10, 22, 5
    # MPIDX = 24: undefined, read 0, write ignored
    mov 24, r12
    ldsr r12, 16, 5
    LI32 10, 0x0badc0de
    ldsr r10, 20, 5
    stsr 20, r11, 5
    CHECK 11, 0, 1
    # read back entry 0 and 23
    ldsr r0, 16, 5
    stsr 20, r11, 5
    CHECK 11, 0x00001000, 2
    stsr 21, r11, 5
    CHECK 11, 0x00001ffc, 2
    stsr 22, r11, 5
    CHECK 11, 0xffffc0bf, 2
    mov 23, r12
    ldsr r12, 16, 5
    stsr 20, r11, 5
    CHECK 11, 0x00002000, 3
    stsr 21, r11, 5
    CHECK 11, 0x00002ffc, 3
    stsr 22, r11, 5
    CHECK 11, 0x00000007, 3
    # SVLOCK.SVL = 1: locked writes ignored, reads work
    mov 1, r12
    ldsr r12, 8, 1
    stsr 8, r11, 1
    CHECK 11, 1, 4
    mov 5, r10
    ldsr r10, 0, 1
    stsr 0, r11, 1
    CHECK 11, 0, 5
    mov 3, r10
    ldsr r10, 0, 5
    stsr 0, r11, 5
    CHECK 11, 0, 6
    LI32 10, 0x00003000
    ldsr r10, 20, 5
    stsr 20, r11, 5
    CHECK 11, 0x00002000, 7
    mov 7, r10
    ldsr r10, 27, 5
    stsr 27, r11, 5
    CHECK 11, 0, 8
    mov 1, r10
    ldsr r10, 17, 5
    stsr 17, r11, 5
    CHECK 11, 0, 9
    # MPIDX itself is not locked
    ldsr r0, 16, 5
    stsr 16, r11, 5
    CHECK 11, 0, 10
    # SVLOCK.SVL = 0: writable again
    ldsr r0, 8, 1
    mov 5, r10
    ldsr r10, 0, 1
    stsr 0, r11, 1
    CHECK 11, 5, 11
    mov 7, r10
    ldsr r10, 27, 5
    stsr 27, r11, 5
    CHECK 11, 7, 12
    # ISPR: INTCFG.ISPC = 0 ignores LDSR, = 1 accepts it (bits 15-0)
    LI32 10, 0x00001234
    ldsr r10, 10, 2
    stsr 10, r11, 2
    CHECK 11, 0, 13
    mov 1, r10
    ldsr r10, 13, 2
    LI32 10, 0xffffffff
    ldsr r10, 10, 2
    stsr 10, r11, 2
    CHECK 11, 0x0000ffff, 14
    ldsr r0, 13, 2
    ldsr r0, 10, 2
    stsr 10, r11, 2
    CHECK 11, 0x0000ffff, 15
    PASS
