# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# EIINT nesting through ISPR: a higher priority request interrupts the
# handler, a lower one waits for the outer EIRET.
    .include "macros.inc"
    .text
    VECTORS 0
    .balign 512
ebt:
    jr 1f
    .org ebt + 0x120
    movea 0x120, r0, r19
    jr exc_record
    .org ebt + 0x150
    jr h5
    .org ebt + 0x170
    movea 0x170, r0, r19
    jr exc_record
    .org ebt + 0x200
1:  RECCLR
    LA 10, ebt
    ldsr r10, 3, 1
    EICW 10, 0x0002
    EICW 11, 0x0007
    EICW 9, 0x1005
    SETPSW 0x8020
    ei
n1: CHECKR REC_CNT, 2, 1
    CHECKLOG 0, 0x120, 0x100a, 2
    CHECKR REC_LOG + 12, 0x24, 4
    CHECKLOG 1, 0x170, 0x100b, 6
    CHECKR REC_LOG + 16 + 12, 0x80, 8
    CHECKRL REC_EIPC, n1, 9
    CHECK 22, 0x20, 10
    CHECK 23, 0x20, 11
    CHECKL 20, n1, 12
    stsr 10, r10, 2
    CHECK 10, 0, 13
    PASS
h5: stsr 0, r20, 0
    stsr 1, r21, 0
    stsr 10, r22, 2
    EICW 10, 0x1002
    EICW 11, 0x1007
    ei
h5a:
    di
    stsr 10, r23, 2
    ldsr r20, 0, 0
    ldsr r21, 1, 0
    eiret
