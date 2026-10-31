# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# LD.W / ST.W with a 16-bit displacement.
    .include "macros.inc"
    .text
    # load from data section placed in cluster RAM by the ELF (checked
    # first: later stores overwrite 0xfe000000)
    LA 17, dval
    ld.w 0[r17], r18
    CHECK 18, 0x76543210, 9
    LI32 1, 0xfe000100
    LI32 2, 0xa5a55a5a
    st.w r2, 0[r1]
    ld.w 0[r1], r3
    CHECK 3, 0xa5a55a5a, 1
    # negative disp, seen from another base
    LI32 4, 0x11223344
    st.w r4, -4[r1]
    ld.w -4[r1], r5
    CHECK 5, 0x11223344, 2
    LI32 6, 0xfe0000fc
    ld.w 0[r6], r7
    CHECK 7, 0x11223344, 3
    ld.w 4[r6], r8
    CHECK 8, 0xa5a55a5a, 4
    # extremes of disp16
    LI32 9, 0xfe008000
    LI32 10, 0xcafef00d
    st.w r10, -0x8000[r9]
    LI32 11, 0xfe000000
    ld.w 0[r11], r12
    CHECK 12, 0xcafef00d, 5
    LI32 13, 0x0badc0de
    st.w r13, 0x7ffc[r11]
    ld.w -4[r9], r14
    CHECK 14, 0x0badc0de, 6
    # load from code flash
    LA 15, rodata
    ld.w 0[r15], r16
    CHECK 16, 0xdeadbeef, 7
    ld.w 4[r15], r16
    CHECK 16, 0x01234567, 8
    # load to r0 discarded, store of r0 writes 0
    ld.w 0[r15], r0
    CHECK 0, 0, 10
    st.w r0, 0[r1]
    ld.w 0[r1], r19
    CHECK 19, 0, 11
    PASS
    .balign 4
rodata:
    .long 0xdeadbeef, 0x01234567

    .data
    .balign 4
dval:
    .long 0x76543210
