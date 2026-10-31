# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# An FE-level exception inside an EI-level handler keeps EIPC/EIPSW/EIIC.
    .include "macros.inc"
    .text
    VECTORS 0
    LA 10, ebase_tab
    ldsr r10, 3, 1
    SETPSW 0x0000800b
    trap 7
n1: stsr 5, r11, 0
    CHECK 11, 0x800b, 1
    # values captured in the handlers
    CHECK 2, 0x47, 2
    LA 28, n1
    cmp r28, r3
    be 1f
    FAIL 3
1:  CHECK 4, 0x0000800b, 4
    CHECK 5, 0x0000806b, 5
    CHECK 6, 0x60, 6
    LA 28, rie_i
    cmp r28, r7
    be 1f
    FAIL 7
1:  CHECK 8, 0x0000806b, 8
    CHECK 9, 0x000080eb, 9
    # after RIE and FETRAP: EI registers unchanged
    CHECK 12, 0x47, 10
    LA 28, n1
    cmp r28, r13
    be 1f
    FAIL 11
1:  CHECK 14, 0x0000800b, 12
    CHECK 15, 0x3f, 13
    LA 28, fet_n
    cmp r28, r20
    be 1f
    FAIL 14
1:  CHECK 21, 0x0000806b, 15
    PASS

    .balign 512
ebase_tab:
    jr ebase_tab
    .org ebase_tab + 0x30
    jr fetrap_h
    .org ebase_tab + 0x40
    jr trap_h
    .org ebase_tab + 0x60
    jr rie_h

trap_h:
    stsr 13, r2, 0
    stsr 0, r3, 0
    stsr 1, r4, 0
    stsr 5, r5, 0
rie_i:
    .hword 0x0040
    fetrap 15
fet_n:
    stsr 13, r12, 0
    stsr 0, r13, 0
    stsr 1, r14, 0
    eiret

rie_h:
    stsr 14, r6, 0
    stsr 2, r7, 0
    stsr 3, r8, 0
    stsr 5, r9, 0
    add 2, r7
    ldsr r7, 2, 0
    add -2, r7
    feret

fetrap_h:
    stsr 14, r15, 0
    stsr 2, r20, 0
    stsr 3, r21, 0
    feret
