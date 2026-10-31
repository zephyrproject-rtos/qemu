# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# SWSRESA software system reset (U2C HW UM 9.4.8, 9.4.10, 9.4.12): with the
# RESKCPROT0 key a write of 1 resets the machine. The second boot starts at
# RBASE with PSW ID set, finds the local RAM marker, RESF has SRES2F0 beside
# the power-on flags, RESKCPROT0, CLKKCPROT1, OSTM0, RLIN311 and the INTC are
# back at their reset values; RESFC clears RESF.
    .include "macros.inc"
    .set MARK, 0xfdc1ff00
    .set RESF, 0xff988500
    .set RESFC, 0xff980c00
    .set RESKCPROT0, 0xff980f00
    .set SWSRESA, 0xff98840c
    .set CLKKCPROT1, 0xff980700
    .set OSTM0_CMP, 0xffbf0000
    .set RLIN311, 0xff89bb00
    .set EIC0, 0xfffc4000
    .text
    LI32 10, MARK
    ld.w 0[r10], r11
    LI32 12, 0x52535431
    cmp r12, r11
    be 2f
    # first boot: power-on RESF, keep the EIC0 reset value next to the marker
    LI32 13, RESF
    ld.w 0[r13], r14
    CHECK 14, 0x5, 1
    LI32 13, EIC0
    ld.hu 0[r13], r14
    st.w r14, 4[r10]
    movea 0x0007, r0, r15
    st.h r15, 0[r13]
    LI32 13, OSTM0_CMP
    LI32 14, 0x1234
    st.w r14, 0[r13]
    LI32 13, RLIN311
    movea 10, r0, r14
    st.h r14, 2[r13]
    LI32 14, 0xa5a5a501
    LI32 13, CLKKCPROT1
    st.w r14, 0[r13]
    st.w r12, 0[r10]
    LI32 13, RESKCPROT0
    st.w r14, 0[r13]
    LI32 13, SWSRESA
    mov 1, r14
    st.w r14, 0[r13]
    # the reset request is taken at the end of this block
    LI32 15, 1000000
1:  add -1, r15
    bnz 1b
    FAIL 2
2:  # second boot
    stsr 5, r14, 0
    andi 0x80e0, r14, r14
    CHECK 14, 0x20, 3
    LI32 13, RESF
    ld.w 0[r13], r14
    CHECK 14, 0x15, 4
    LI32 13, RESKCPROT0
    ld.w 0[r13], r14
    CHECK 14, 0, 5
    LI32 13, CLKKCPROT1
    ld.w 0[r13], r14
    CHECK 14, 0, 6
    LI32 13, OSTM0_CMP
    ld.w 0[r13], r14
    CHECK 14, 0, 7
    LI32 13, RLIN311
    ld.hu 2[r13], r14
    CHECK 14, 0, 8
    LI32 13, EIC0
    ld.hu 0[r13], r14
    ld.w 4[r10], r15
    cmp r15, r14
    be 3f
    FAIL 9
3:  LI32 13, RESFC
    LI32 14, 0xf5dd
    st.w r14, 0[r13]
    LI32 13, RESF
    ld.w 0[r13], r14
    CHECK 14, 0, 10
    PASS
