# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz
#
# Test macros. Exit device at 0x40000000 (-M rh850-u2c,test-exit=on):
# 0x5555 passes, (n << 16) | 0x3333 fails with exit code n.

# 48-bit MOV imm32, reg1 written as raw halfwords
    .macro LI32 r, v
    .hword 0x0620 | \r, (\v) & 0xffff, ((\v) >> 16) & 0xffff
    .endm

    .macro PASS
    movhi 0x4000, r0, r30
    movea 0x5555, r0, r29
    st.w r29, 0[r30]
9:  br 9b
    .endm

    .macro FAIL n
    .if (\n) < 1 || (\n) > 0xff
    .error "FAIL code must be 1..255"
    .endif
    movhi 0x4000, r0, r30
    LI32 29, (((\n) << 16) | 0x3333)
    st.w r29, 0[r30]
9:  br 9b
    .endm

# Fail with code n unless reg == v (reg as number)
    .macro CHECK r, v, n
    LI32 28, \v
    cmp r28, r\r
    be 8f
    FAIL \n
8:
    .endm

# Address of a symbol: MOVHI hi() + MOVEA lo()
    .macro LA r, sym
    movhi hi(\sym), r0, r\r
    movea lo(\sym), r\r, r\r
    .endm

# Raw encodings missing from binutils
    .macro CLL
    .hword 0xffff, 0xf160
    .endm
    .macro RESBANK
    .hword 0x07e0, 0x8160
    .endm
    .macro LDL_BU r1, r3
    .hword 0x0fe0 | \r1, (\r3 << 11) | 0x0370
    .endm
    .macro LDL_HU r1, r3
    .hword 0x0fe0 | \r1, (\r3 << 11) | 0x0374
    .endm
    .macro STC_B r3, r1
    .hword 0x07e0 | \r1, (\r3 << 11) | 0x0372
    .endm
    .macro STC_H r3, r1
    .hword 0x07e0 | \r1, (\r3 << 11) | 0x0376
    .endm

# Fail with code n unless the word at address a equals v (uses r24, r25)
    .macro CHECKM a, v, n
    LI32 25, \a
    ld.w 0[r25], r24
    CHECK 24, \v, \n
    .endm

# Exception record in cluster RAM, written by the VECTORS handlers
    .equ REC, 0xfe130000
    .equ REC_CNT, 0
    .equ REC_OFF, 4
    .equ REC_EIIC, 8
    .equ REC_EIPC, 12
    .equ REC_EIPSW, 16
    .equ REC_FEIC, 20
    .equ REC_FEPC, 24
    .equ REC_FEPSW, 28
    .equ REC_PSW, 32
    .equ REC_RESUME, 36
    .equ REC_MEA, 40
    .equ REC_MEI, 44
    .equ REC_FPEPC, 48
    .equ REC_FPSR, 52
    .equ REC_FPEC, 56
    .equ REC_FPSRW, 60
    .equ REC_WADDR, 64
    .equ REC_WVAL, 68
    .equ REC_LOGN, 72
    .equ REC_LOG, 80
    .equ REC_SYNC, 240

# Vector table: VECTORS 0 goes first in .text (RBASE = 0); each exception
# is recorded in REC and returns to REC_RESUME if set, else to the saved PC.
    .macro VECTORS tag
    .balign 512
vt\@:
    jr 1f
    .set k\@, 0x10
    .rept 31
    .org vt\@ + k\@
    movea \tag + k\@, r0, r19
    jr exc_record
    .set k\@, k\@ + 0x10
    .endr
    .org vt\@ + 0x200
    .if \tag == 0
exc_record:
    stsr 5, r18, 0
    LI32 17, REC
    st.w r18, REC_PSW[r17]
    st.w r19, REC_OFF[r17]
    ld.w REC_CNT[r17], r18
    add 1, r18
    st.w r18, REC_CNT[r17]
    stsr 13, r18, 0
    st.w r18, REC_EIIC[r17]
    stsr 0, r18, 0
    st.w r18, REC_EIPC[r17]
    stsr 1, r18, 0
    st.w r18, REC_EIPSW[r17]
    stsr 14, r18, 0
    st.w r18, REC_FEIC[r17]
    stsr 2, r18, 0
    st.w r18, REC_FEPC[r17]
    stsr 3, r18, 0
    st.w r18, REC_FEPSW[r17]
    stsr 6, r18, 2
    st.w r18, REC_MEA[r17]
    stsr 8, r18, 2
    st.w r18, REC_MEI[r17]
    andi 0xff0, r19, r18
    movea 0x70, r0, r16
    cmp r16, r18
    bne 5f
    stsr 7, r18, 0
    st.w r18, REC_FPEPC[r17]
    stsr 6, r18, 0
    st.w r18, REC_FPSR[r17]
    stsr 11, r18, 0
    st.w r18, REC_FPEC[r17]
    ld.w REC_FPSRW[r17], r18
    cmp r0, r18
    be 5f
    ldsr r18, 6, 0
    st.w r0, REC_FPSRW[r17]
5:  ld.w REC_WADDR[r17], r18
    cmp r0, r18
    be 6f
    ld.w REC_WVAL[r17], r16
    st.w r16, 0[r18]
    st.w r0, REC_WADDR[r17]
6:  ld.w REC_LOGN[r17], r18
    andi 7, r18, r18
    shl 4, r18
    add r17, r18
    st.w r19, REC_LOG[r18]
    stsr 13, r16, 0
    st.w r16, REC_LOG + 4[r18]
    stsr 14, r16, 0
    st.w r16, REC_LOG + 8[r18]
    stsr 10, r16, 2
    st.w r16, REC_LOG + 12[r18]
    ld.w REC_LOGN[r17], r18
    add 1, r18
    st.w r18, REC_LOGN[r17]
    ld.w REC_RESUME[r17], r16
    st.w r0, REC_RESUME[r17]
    andi 0xff0, r19, r18
    movea 0x100, r0, r19
    cmp r19, r18
    bnl 3f
    movea 0x40, r0, r19
    cmp r19, r18
    be 3f
    movea 0x50, r0, r19
    cmp r19, r18
    be 3f
    movea 0x70, r0, r19
    cmp r19, r18
    be 3f
    cmp r0, r16
    be 2f
    ldsr r16, 2, 0
2:  feret
3:  cmp r0, r16
    be 4f
    ldsr r16, 0, 0
4:  eiret
    .endif
1:
    .endm

# The next exception returns to lbl (uses r16, r17)
    .macro RESUME lbl
    LI32 17, REC
    LA 16, \lbl
    st.w r16, REC_RESUME[r17]
    .endm

# Clear the exception record (uses r17)
    .macro RECCLR
    LI32 17, REC
    .set f\@, 0
    .rept 52
    st.w r0, f\@[r17]
    .set f\@, f\@ + 4
    .endr
    .endm

# Fail with code n unless REC field f equals v (uses r24, r25, r28)
    .macro CHECKR f, v, n
    CHECKM REC + \f, \v, \n
    .endm

# Fail with code n unless REC field f holds the address of lbl
    .macro CHECKRL f, lbl, n
    LI32 25, REC + \f
    ld.w 0[r25], r24
    LA 28, \lbl
    cmp r28, r24
    be 8f
    FAIL \n
8:
    .endm

# REC after an FE-level exception: offset, FEIC, FEPC = lbl, FEPSW, PSW in
# the handler (fail codes n to n + 4)
    .macro CHECKFE off, cause, lbl, fepsw, psw, n
    CHECKR REC_OFF, \off, \n
    CHECKR REC_FEIC, \cause, \n + 1
    CHECKRL REC_FEPC, \lbl, \n + 2
    CHECKR REC_FEPSW, \fepsw, \n + 3
    CHECKR REC_PSW, \psw, \n + 4
    .endm

# Same for an EI-level exception
    .macro CHECKEI off, cause, lbl, eipsw, psw, n
    CHECKR REC_OFF, \off, \n
    CHECKR REC_EIIC, \cause, \n + 1
    CHECKRL REC_EIPC, \lbl, \n + 2
    CHECKR REC_EIPSW, \eipsw, \n + 3
    CHECKR REC_PSW, \psw, \n + 4
    .endm

# The next FPE/FPI handler writes v to FPSR (uses r16, r17)
    .macro FPSRW v
    LI32 17, REC
    LI32 16, \v
    st.w r16, REC_FPSRW[r17]
    .endm

# PSW = v (uses r16)
    .macro SETPSW v
    LI32 16, \v
    ldsr r16, 5, 0
    .endm

# The next exception handler stores word v at address a (uses r16, r17)
    .macro HWR a, v
    LI32 17, REC
    LI32 16, \a
    st.w r16, REC_WADDR[r17]
    LI32 16, \v
    st.w r16, REC_WVAL[r17]
    .endm

# Log entry i: offset (with tag) and EIIC (fail codes n, n + 1)
    .macro CHECKLOG i, off, eiic, n
    CHECKR REC_LOG + 16 * (\i), \off, \n
    CHECKR REC_LOG + 16 * (\i) + 4, \eiic, \n + 1
    .endm

# Record that step v is reached (uses r16, r17)
    .macro SYNC v
    LI32 17, REC
    movea \v, r0, r16
    st.w r16, REC_SYNC[r17]
    .endm

# INTC: EICn and EEICn of INTC1 (PE0) for n < 32, else of INTC2
    .equ INTC1, 0xfffc4000
    .equ INTC2, 0xfff80000
    .equ FENC, 0xff9a3a00
    .equ FEINC, 0xff9a3b00

    .macro EICA r, ch
    .if (\ch) < 32
    LI32 \r, INTC1 + 2 * (\ch)
    .else
    LI32 \r, INTC2 + 2 * (\ch)
    .endif
    .endm

    .macro EEICA r, ch
    .if (\ch) < 32
    LI32 \r, INTC1 + 0x200 + 4 * (\ch)
    .else
    LI32 \r, INTC2 + 0x4000 + 4 * (\ch)
    .endif
    .endm

# EICn = v (16-bit store, uses r16, r17)
    .macro EICW ch, v
    EICA 17, \ch
    LI32 16, \v
    st.h r16, 0[r17]
    .endm

# reg = EICn (uses r17)
    .macro EICR r, ch
    EICA 17, \ch
    ld.hu 0[r17], r\r
    .endm

# EEICn = v (32-bit store, uses r16, r17)
    .macro EEICW ch, v
    EEICA 17, \ch
    LI32 16, \v
    st.w r16, 0[r17]
    .endm

# reg = EEICn (uses r17)
    .macro EEICR r, ch
    EEICA 17, \ch
    ld.w 0[r17], r\r
    .endm

# word at address a = v (uses r16, r17)
    .macro STW a, v
    LI32 17, \a
    LI32 16, \v
    st.w r16, 0[r17]
    .endm

# reg = word at address a (uses r17)
    .macro LDW r, a
    LI32 17, \a
    ld.w 0[r17], r\r
    .endm

# Fail with code n unless the word at address a holds the address of lbl
    .macro CHECKML a, lbl, n
    LI32 25, \a
    ld.w 0[r25], r24
    LA 28, \lbl
    cmp r28, r24
    be 8f
    FAIL \n
8:
    .endm

# Fail with code n unless reg holds the address of lbl
    .macro CHECKL r, lbl, n
    LA 28, \lbl
    cmp r28, r\r
    be 8f
    FAIL \n
8:
    .endm

# RLIN3 units, register offsets and the CLKC/STBC area
    .equ RL30, 0xffc7c000
    .equ RL311, 0xff89bb00
    .equ RL_LWBR, 0x01
    .equ RL_LBRP0, 0x02
    .equ RL_LMD, 0x08
    .equ RL_LBFC, 0x09
    .equ RL_LEDE, 0x0d
    .equ RL_LCUC, 0x0e
    .equ RL_LTRC, 0x10
    .equ RL_LMST, 0x11
    .equ RL_LST, 0x12
    .equ RL_LEST, 0x13
    .equ RL_LDFC, 0x14
    .equ RL_LUDB0, 0x17
    .equ RL_LDBR1, 0x18
    .equ RL_LUOER, 0x20
    .equ RL_LUOR1, 0x21
    .equ RL_LUTDR, 0x24
    .equ RL_LURDR, 0x26
    .equ RL_LUWTDR, 0x28
    .equ SYSCTL, 0xff980000

# RLIN3 at base in r10 to UART mode, LUOER = oe (uses r11)
    .macro RLUART oe
    mov 1, r11
    st.b r11, RL_LMD[r10]
    st.b r11, RL_LCUC[r10]
7:  ld.bu RL_LMST[r10], r11
    andi 1, r11, r11
    cmp r0, r11
    be 7b
    movea \oe, r0, r11
    st.b r11, RL_LUOER[r10]
    .endm
