# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# RLIN311 polled transmission and UART buffer to serial port 0.
    .include "macros.inc"
    .data
msg: .ascii "Hello, RH850\n"
    .equ MSGLEN, 13
    .text
    LI32 10, RL311
    RLUART 1
    LA 12, msg
    movea MSGLEN, r0, r13
1:  ld.bu 0[r12], r11
    st.b r11, RL_LUTDR[r10]
2:  ld.bu RL_LST[r10], r11
    andi 0x10, r11, r11
    cmp r0, r11
    bne 2b
    add 1, r12
    add -1, r13
    bne 1b
    st.b r0, RL_LUOER[r10]
    movea 0x58, r0, r11
    st.b r11, RL_LUTDR[r10]
    mov 1, r11
    st.b r11, RL_LUOER[r10]
    movea 0x41, r0, r11
    st.b r11, RL_LDBR1[r10]
    movea 0x42, r0, r11
    st.b r11, RL_LDBR1 + 1[r10]
    movea 0x43, r0, r11
    st.b r11, RL_LDBR1 + 2[r10]
    mov 3, r11
    st.b r11, RL_LDFC[r10]
    mov 2, r11
    st.b r11, RL_LTRC[r10]
    ld.bu RL_LST[r10], r12
    CHECK 12, 1, 1
    ld.bu RL_LTRC[r10], r12
    CHECK 12, 0, 2
    mov 1, r11
    st.b r11, RL_LST[r10]
    ld.bu RL_LST[r10], r12
    CHECK 12, 1, 3
    st.b r0, RL_LST[r10]
    ld.bu RL_LST[r10], r12
    CHECK 12, 0, 4
    movea 0x30, r0, r11
    st.b r11, RL_LUDB0[r10]
    mov r10, r14
    mov 8, r13
    movea 0x31, r0, r11
3:  st.b r11, RL_LDBR1[r14]
    add 1, r11
    add 1, r14
    add -1, r13
    bne 3b
    st.b r0, RL_LDFC[r10]
    mov 2, r11
    st.b r11, RL_LTRC[r10]
    ld.bu RL_LST[r10], r12
    CHECK 12, 1, 5
    st.b r0, RL_LCUC[r10]
    ld.bu RL_LST[r10], r12
    CHECK 12, 0, 6
    RLUART 1
    movea 0x0a, r0, r11
    st.b r11, RL_LUTDR[r10]
    PASS
