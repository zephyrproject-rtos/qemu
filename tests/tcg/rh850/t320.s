# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# OSTM0/OSTM1 interval interrupts to INTC2 channels 63 and 484, each
# waking HALT.
    .include "macros.inc"
    .text
    VECTORS 0
    RECCLR
    EICW 63, 0x0003
    LI32 10, 0xffbf0000
    movea 99, r0, r11
    st.w r11, 0[r10]
    movea 0x80, r0, r11
    st.b r11, 0x20[r10]
    mov 1, r11
    st.b r11, 0x14[r10]
    SETPSW 0
    halt
    halt
    halt
    halt
    halt
    di
    mov 1, r11
    st.b r11, 0x18[r10]
    CHECKR REC_CNT, 5, 1
    CHECKR REC_EIIC, 0x103f, 2
    CHECKR REC_OFF, 0x130, 3
    EICW 63, 0x0083
    RECCLR
    EICW 484, 0x0004
    LI32 10, 0xff89a100
    movea 99, r0, r11
    st.w r11, 0[r10]
    movea 0x80, r0, r11
    st.b r11, 0x20[r10]
    mov 1, r11
    st.b r11, 0x14[r10]
    SETPSW 0
    halt
    halt
    halt
    halt
    halt
    di
    mov 1, r11
    st.b r11, 0x18[r10]
    CHECKR REC_CNT, 5, 4
    CHECKR REC_EIIC, 0x11e4, 5
    CHECKR REC_OFF, 0x140, 6
    PASS
