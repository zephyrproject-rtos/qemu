# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# CAXI: flags of reg2 - token, store reg3 if equal, else the token; CAXI
# loses the link.
    .include "macros.inc"
    .text
    LI32 11, 0xfe000300
    # equal: memory = reg3, reg3 = token, Z = 1, CY = OV = S = 0
    LI32 1, 0x12345678
    st.w r1, 0[r11]
    LI32 12, 0x12345678
    LI32 13, 0xcafef00d
    caxi [r11], r12, r13
    setf 0x2, r20
    setf 0x4, r21
    setf 0x0, r22
    setf 0x1, r23
    CHECK 20, 1, 1
    CHECK 21, 0, 1
    CHECK 22, 0, 1
    CHECK 23, 0, 1
    CHECK 13, 0x12345678, 1
    CHECK 12, 0x12345678, 1
    CHECK 11, 0xfe000300, 1
    CHECKM 0xfe000300, 0xcafef00d, 1
    # not equal, reg2 < token: memory unchanged, reg3 = token, S = 1, CY = 1
    LI32 12, 1
    LI32 13, 0x11111111
    LI32 1, 2
    st.w r1, 0[r11]
    caxi [r11], r12, r13
    setf 0x2, r20
    setf 0x4, r21
    setf 0x0, r22
    setf 0x1, r23
    CHECK 20, 0, 2
    CHECK 21, 1, 2
    CHECK 22, 0, 2
    CHECK 23, 1, 2
    CHECK 13, 2, 2
    CHECKM 0xfe000300, 2, 2
    # not equal with overflow: 0x80000000 - 1
    LI32 12, 0x80000000
    LI32 13, 0x22222222
    LI32 1, 1
    st.w r1, 0[r11]
    caxi [r11], r12, r13
    setf 0x2, r20
    setf 0x4, r21
    setf 0x0, r22
    setf 0x1, r23
    CHECK 20, 0, 3
    CHECK 21, 0, 3
    CHECK 22, 1, 3
    CHECK 23, 0, 3
    CHECK 13, 1, 3
    CHECKM 0xfe000300, 1, 3
    # reg3 = r0: stores 0 on match, result discarded
    LI32 12, 1
    caxi [r11], r12, r0
    CHECKM 0xfe000300, 0, 4
    # CAXI on the linked address loses the link even on mismatch
    ldl.w [r11], r14
    LI32 12, 0x77
    caxi [r11], r12, r13
    LI32 13, 0x99
    stc.w r13, [r11]
    CHECK 13, 0, 5
    PASS
