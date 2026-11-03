# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# JR disp22 and JMP [reg1].
    .include "macros.inc"
    .text
    mov 0, r10
    # JR forward over a FAIL
    jr 1f
    FAIL 1
1:  # JR backward: go back once, count visits
    mov 1, r11
2:  movea 1, r10, r10
    cmp 2, r10
    be 3f
    jr 2b
3:  CHECK 10, 2, 2
    # JR forward beyond the disp17 range (> 64 KiB)
    jr far
    FAIL 3
back:
    # JMP [reg1]
    LA 5, 4f
    jmp [r5]
    FAIL 4
4:  # JMP [reg1]: bit 0 of the target is masked
    LA 6, 5f + 1
    jmp [r6]
    FAIL 5
5:  # reg1 itself is not modified
    LA 7, 5b + 1
    cmp r7, r6
    be 6f
    FAIL 6
6:
    PASS

    .space 0x30000, 0
far:
    # JR backward beyond the disp17 range
    jr back
    FAIL 7
