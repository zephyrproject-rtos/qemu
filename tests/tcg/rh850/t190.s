# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# FPSR and its views FPST, FPCC and FPCFG; FPEC.FPIVD can only be cleared.
    .include "macros.inc"
    .text
    SETPSW 0x00010020
    # reset after reset
    stsr 6, r11, 0
    CHECK 11, 0x00020000, 1
    stsr 8, r11, 0
    CHECK 11, 0x00000000, 2
    stsr 9, r11, 0
    CHECK 11, 0x00000000, 3
    stsr 10, r11, 0
    CHECK 11, 0x00000000, 4
    # fpcc <- 0x000000a5
    LI32 10, 0x000000a5
    ldsr r10, 9, 0
    stsr 6, r11, 0
    CHECK 11, 0xa5020000, 5
    stsr 8, r11, 0
    CHECK 11, 0x00000000, 6
    stsr 9, r11, 0
    CHECK 11, 0x000000a5, 7
    stsr 10, r11, 0
    CHECK 11, 0x00000000, 8
    # fpst <- 0xffffffff
    LI32 10, 0xffffffff
    ldsr r10, 8, 0
    stsr 6, r11, 0
    CHECK 11, 0xa542fc1f, 9
    stsr 8, r11, 0
    CHECK 11, 0x00003f3f, 10
    stsr 9, r11, 0
    CHECK 11, 0x000000a5, 11
    stsr 10, r11, 0
    CHECK 11, 0x00000000, 12
    # fpcfg <- 0xffffffff
    LI32 10, 0xffffffff
    ldsr r10, 10, 0
    stsr 6, r11, 0
    CHECK 11, 0xa54effff, 13
    stsr 8, r11, 0
    CHECK 11, 0x00003f3f, 14
    stsr 9, r11, 0
    CHECK 11, 0x000000a5, 15
    stsr 10, r11, 0
    CHECK 11, 0x0000031f, 16
    # fpsr <- 0x00000000
    LI32 10, 0x00000000
    ldsr r10, 6, 0
    stsr 6, r11, 0
    CHECK 11, 0x00000000, 17
    stsr 8, r11, 0
    CHECK 11, 0x00000000, 18
    stsr 9, r11, 0
    CHECK 11, 0x00000000, 19
    stsr 10, r11, 0
    CHECK 11, 0x00000000, 20
    # fpsr <- 0xffffffff
    LI32 10, 0xffffffff
    ldsr r10, 6, 0
    stsr 6, r11, 0
    CHECK 11, 0xffeeffff, 21
    stsr 8, r11, 0
    CHECK 11, 0x00003f3f, 22
    stsr 9, r11, 0
    CHECK 11, 0x000000ff, 23
    stsr 10, r11, 0
    CHECK 11, 0x0000031f, 24
    # fpst <- 0x00000000
    LI32 10, 0x00000000
    ldsr r10, 8, 0
    stsr 6, r11, 0
    CHECK 11, 0xffae03e0, 25
    stsr 8, r11, 0
    CHECK 11, 0x00000000, 26
    stsr 9, r11, 0
    CHECK 11, 0x000000ff, 27
    stsr 10, r11, 0
    CHECK 11, 0x0000031f, 28
    # fpcc <- 0x0000005a
    LI32 10, 0x0000005a
    ldsr r10, 9, 0
    stsr 6, r11, 0
    CHECK 11, 0x5aae03e0, 29
    stsr 8, r11, 0
    CHECK 11, 0x00000000, 30
    stsr 9, r11, 0
    CHECK 11, 0x0000005a, 31
    stsr 10, r11, 0
    CHECK 11, 0x0000031f, 32
    # fpcfg <- 0x00000102
    LI32 10, 0x00000102
    ldsr r10, 10, 0
    stsr 6, r11, 0
    CHECK 11, 0x5aa60040, 33
    stsr 8, r11, 0
    CHECK 11, 0x00000000, 34
    stsr 9, r11, 0
    CHECK 11, 0x0000005a, 35
    stsr 10, r11, 0
    CHECK 11, 0x00000102, 36
    # fpsr <- 0x00020000
    LI32 10, 0x00020000
    ldsr r10, 6, 0
    stsr 6, r11, 0
    CHECK 11, 0x00020000, 37
    stsr 8, r11, 0
    CHECK 11, 0x00000000, 38
    stsr 9, r11, 0
    CHECK 11, 0x00000000, 39
    stsr 10, r11, 0
    CHECK 11, 0x00000000, 40
    # FPEC: write 1 ignored, write 0 kept 0
    mov 1, r10
    ldsr r10, 11, 0
    stsr 11, r11, 0
    CHECK 11, 0, 41
    ldsr r0, 11, 0
    stsr 11, r11, 0
    CHECK 11, 0, 42
    PASS
