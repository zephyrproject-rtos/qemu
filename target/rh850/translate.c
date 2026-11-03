/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 translation
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "exec/target_page.h"
#include "insn-len.h"
#include "sysreg.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

#define DISAS_JUMP    DISAS_TARGET_0
#define DISAS_EXIT    DISAS_TARGET_1

enum {
    RH850_COND_R = 0x5,
    RH850_COND_SA = 0xd,
};

typedef struct DisasContext {
    DisasContextBase base;
    CPURH850State *env;
    uint32_t pc;
    uint32_t insn;
    /* MO_ALIGN while MCTL.MA = 0, else 0 */
    MemOp align;
    /* PSW.UM */
    bool um;
    /* PSW.CU0 */
    bool cu0;
    /* FPEC.FPIVD */
    bool fpivd;
    TCGOp *insn_start;
} DisasContext;

/* MEI ITYPE */
enum {
    MEI_SLD = 0,
    MEI_D16 = 1,
    MEI_D23 = 2,
    MEI_INC = 4,
    MEI_DEC = 5,
    MEI_LDL = 7,
    MEI_CAXI = 8,
};

static TCGv_i32 cpu_gpr[RH850_NUM_GPRS];
static TCGv_i32 cpu_pc;
static TCGv_i32 cpu_psw_z, cpu_psw_s, cpu_psw_ov, cpu_psw_cy, cpu_psw_sat;
static TCGv_i32 cpu_ll_addr, cpu_ll_size, cpu_ll_val, cpu_ll_valid;

static int ex_shift_1(DisasContext *ctx, int x)
{
    return x << 1;
}

static int ex_shift_2(DisasContext *ctx, int x)
{
    return x << 2;
}

static int ex_plus_16(DisasContext *ctx, int x)
{
    return x + 16;
}

/* Next instruction halfword, for 48/64-bit forms */
static uint32_t next_hw(DisasContext *ctx)
{
    uint32_t hw = translator_lduw_end(ctx->env, &ctx->base,
                                      ctx->base.pc_next, MO_LE);

    ctx->base.pc_next += 2;
    return hw;
}

static int append_hi16(DisasContext *ctx, int x)
{
    return x | next_hw(ctx) << 16;
}

/* disp23: x holds bits 6-0, the next halfword bits 22-7 */
static int ex_disp23(DisasContext *ctx, int x)
{
    return sextract32(next_hw(ctx) << 7 | x, 0, 23);
}

static int ex_disp23_h(DisasContext *ctx, int x)
{
    return ex_disp23(ctx, x << 1);
}

/* disp32: x holds bits 15-1, the next halfword bits 31-16 */
static int ex_disp32(DisasContext *ctx, int x)
{
    return next_hw(ctx) << 16 | x << 1;
}

/* list12: insn bits 31-21 and bit 0 map to the same list bits */
static int ex_list12(DisasContext *ctx, int x)
{
    return (x >> 1) << 21 | (x & 1);
}

/* PREPARE ep source by ff; ff = 0 means sp */
static int ex_prepare_ep(DisasContext *ctx, int ff)
{
    uint32_t lo;

    switch (ff) {
    case 1:
        return (int16_t)next_hw(ctx);
    case 2:
        return next_hw(ctx) << 16;
    case 3:
        lo = next_hw(ctx);
        return lo | next_hw(ctx) << 16;
    default:
        return 0;
    }
}

#include "decode-insns16.c.inc"
#include "decode-insns32.c.inc"

/* r0 reads 0, writes are ignored */
static TCGv_i32 get_gpr(int r)
{
    return r ? cpu_gpr[r] : tcg_constant_i32(0);
}

static void set_gpr(int r, TCGv_i32 val)
{
    if (r) {
        tcg_gen_mov_i32(cpu_gpr[r], val);
    }
}

static void set_gpr_i(int r, uint32_t val)
{
    if (r) {
        tcg_gen_movi_i32(cpu_gpr[r], val);
    }
}

/*
 * A store by this CPU to the 32-byte block holding the link address
 * loses the link
 */
static void gen_link_kill(TCGv_i32 addr, int size)
{
    TCGv_i32 t = tcg_temp_new_i32();
    TCGv_i32 k = tcg_temp_new_i32();

    tcg_gen_xor_i32(t, addr, cpu_ll_addr);
    tcg_gen_andi_i32(t, t, ~31u);
    tcg_gen_setcondi_i32(TCG_COND_EQ, k, t, 0);
    tcg_gen_addi_i32(t, addr, size - 1);
    tcg_gen_xor_i32(t, t, cpu_ll_addr);
    tcg_gen_andi_i32(t, t, ~31u);
    tcg_gen_setcondi_i32(TCG_COND_EQ, t, t, 0);
    tcg_gen_or_i32(k, k, t);
    tcg_gen_andc_i32(cpu_ll_valid, cpu_ll_valid, k);
}

static void gen_goto_tb(DisasContext *ctx, unsigned slot, uint32_t dest)
{
    if (translator_use_goto_tb(&ctx->base, dest)) {
        tcg_gen_goto_tb(slot);
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_exit_tb(ctx->base.tb, slot);
    } else {
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_lookup_and_goto_ptr();
    }
    ctx->base.is_jmp = DISAS_NORETURN;
}

/*
 * MEI of the current memory instruction, kept in insn_start word 1
 * for the MAE raised by the alignment hook
 */
static uint32_t gen_mei(DisasContext *ctx, int itype, int reg, MemOp mop,
                        bool st)
{
    uint32_t mei = (ctx->base.pc_next - ctx->pc) << 28 | reg << 16 |
                   (mop & MO_SIZE) << 9 | itype << 1 | st;

    if (!st && !(mop & MO_SIGN) && (mop & MO_SIZE) < MO_32) {
        mei |= 1 << 8;
    }
    tcg_set_insn_start_param(ctx->insn_start, 1, mei);
    return mei;
}

/* Exception with saved PC pc (current or next instruction) */
static void gen_raise(DisasContext *ctx, int excp, uint32_t cause,
                      uint32_t pc)
{
    tcg_gen_movi_i32(cpu_pc, pc);
    gen_helper_raise(tcg_env, tcg_constant_i32(excp),
                     tcg_constant_i32(cause));
    ctx->base.is_jmp = DISAS_NORETURN;
}

/* RIE: resumable, FEPC = this instruction */
static void gen_rie(DisasContext *ctx)
{
    gen_raise(ctx, RH850_EXCP_RIE, RH850_CAUSE_RIE, ctx->pc);
}

/* Supervisor-only instruction: PIE in user mode. Returns true if raised */
static bool gen_check_sv(DisasContext *ctx)
{
    if (ctx->um) {
        gen_raise(ctx, RH850_EXCP_PIE, RH850_CAUSE_PIE, ctx->pc);
        return true;
    }
    return false;
}

/* Undefined operation: log a guest error and go on */
static void gen_log_undefined(DisasContext *ctx, const char *what)
{
    tcg_gen_movi_i32(cpu_pc, ctx->pc);
    gen_helper_log_undefined(tcg_env, tcg_constant_ptr(what));
}

/* Encodings not to be used: RIE */
static bool trans_reserved16(DisasContext *ctx, arg_reserved16 *a)
{
    gen_rie(ctx);
    return true;
}

static bool trans_reserved32(DisasContext *ctx, arg_reserved32 *a)
{
    gen_rie(ctx);
    return true;
}

static bool trans_RIE1(DisasContext *ctx, arg_RIE1 *a)
{
    gen_rie(ctx);
    return true;
}

static bool trans_RIE2(DisasContext *ctx, arg_RIE2 *a)
{
    gen_rie(ctx);
    return true;
}

/*
 * UCPOP before RIE. CU1 and CU2 are fixed to 0 on U2C, so FXU and
 * reserved coprocessor codes always raise UCPOP
 */
static bool trans_copro(DisasContext *ctx, arg_copro *a)
{
    gen_raise(ctx, RH850_EXCP_UCPOP, RH850_CAUSE_UCPOP + 1, ctx->pc);
    return true;
}

/* Reserved FPU encodings: UCPOP before RIE */
static bool trans_reservedfpu(DisasContext *ctx, arg_reservedfpu *a)
{
    if (!ctx->cu0) {
        gen_raise(ctx, RH850_EXCP_UCPOP, RH850_CAUSE_UCPOP, ctx->pc);
    } else {
        gen_rie(ctx);
    }
    return true;
}

static bool trans_COPRO1(DisasContext *ctx, arg_COPRO1 *a)
{
    unsigned cu;

    if (a->sel & 0x10) {
        cu = 2;
    } else if (a->sel & 0x08) {
        cu = (a->sel & 0x04) ? 1 : 2;
    } else if ((a->sel & 0x07) == 0x06) {
        cu = 1;
    } else {
        cu = 0;
    }
    if (cu || !ctx->cu0) {
        gen_raise(ctx, RH850_EXCP_UCPOP, RH850_CAUSE_UCPOP + cu, ctx->pc);
        return true;
    }
    gen_rie(ctx);
    return true;
}

/* TRAP: pending, EIPC = next, EIIC = 40H + vector */
static bool trans_TRAP(DisasContext *ctx, arg_TRAP *a)
{
    gen_raise(ctx, RH850_EXCP_TRAP, RH850_CAUSE_TRAP + a->vector,
              ctx->base.pc_next);
    return true;
}

/* FETRAP: pending, FEPC = next, FEIC = 30H + vector */
static bool trans_FETRAP(DisasContext *ctx, arg_FETRAP *a)
{
    gen_raise(ctx, RH850_EXCP_FETRAP, RH850_CAUSE_FETRAP + a->vector,
              ctx->base.pc_next);
    return true;
}

/* SYSCALL: handler from the SCBP table */
static bool trans_SYSCALL(DisasContext *ctx, arg_SYSCALL *a)
{
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    gen_helper_syscall(tcg_env, tcg_constant_i32(a->vector));
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

/* EIRET, FERET: supervisor only */
static bool trans_EIRET(DisasContext *ctx, arg_EIRET *a)
{
    if (!gen_check_sv(ctx)) {
        gen_helper_eiret(tcg_env);
        ctx->base.is_jmp = DISAS_EXIT;
    }
    return true;
}

static bool trans_FERET(DisasContext *ctx, arg_FERET *a)
{
    if (!gen_check_sv(ctx)) {
        gen_helper_feret(tcg_env);
        ctx->base.is_jmp = DISAS_EXIT;
    }
    return true;
}

/*
 * EI ends the TB so that a pending interrupt is taken before the next
 * instruction
 */
static void gen_set_id(DisasContext *ctx, int id)
{
    TCGv_i32 t = tcg_temp_new_i32();

    if (ctx->um) {
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        gen_helper_check_uic(tcg_env);
    }
    tcg_gen_ld_i32(t, tcg_env, offsetof(CPURH850State, psw_ctrl));
    tcg_gen_deposit_i32(t, t, tcg_constant_i32(id), R_PSW_ID_SHIFT, 1);
    tcg_gen_st_i32(t, tcg_env, offsetof(CPURH850State, psw_ctrl));
}

static bool trans_EI(DisasContext *ctx, arg_EI *a)
{
    gen_set_id(ctx, 0);
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    ctx->base.is_jmp = DISAS_EXIT;
    return true;
}

static bool trans_DI(DisasContext *ctx, arg_DI *a)
{
    gen_set_id(ctx, 1);
    return true;
}

/* HALT: supervisor only; resumes at the next instruction */
static bool trans_HALT(DisasContext *ctx, arg_HALT *a)
{
    if (!gen_check_sv(ctx)) {
        tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
        gen_helper_halt(tcg_env);
        ctx->base.is_jmp = DISAS_NORETURN;
    }
    return true;
}

/*
 * SNOOZE is not timed and ends at once; the TB ends so that a pending
 * interrupt is taken
 */
static bool trans_SNOOZE(DisasContext *ctx, arg_SNOOZE *a)
{
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    ctx->base.is_jmp = DISAS_EXIT;
    return true;
}

/* RESBANK: supervisor only */
static bool trans_RESBANK(DisasContext *ctx, arg_RESBANK *a)
{
    if (!gen_check_sv(ctx)) {
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        gen_helper_resbank(tcg_env);
    }
    return true;
}

static TCGv_i32 gen_ld_sr(int n)
{
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_ld_i32(t, tcg_env, offsetof(CPURH850State, sr[n >> 5][n & 31]));
    return t;
}

static void gen_st_sr(int n, TCGv_i32 val)
{
    tcg_gen_st_i32(val, tcg_env, offsetof(CPURH850State, sr[n >> 5][n & 31]));
}

/*
 * CALLT: CTPC = PC + 2, CTPSW = PSW(4:0), PC = CTBP + the halfword at
 * CTBP + imm6 * 2; the table read never raises MAE
 */
static bool trans_CALLT(DisasContext *ctx, arg_CALLT *a)
{
    TCGv_i32 ctbp = gen_ld_sr(RH850_SR_CTBP);
    TCGv_i32 ent = tcg_temp_new_i32();
    TCGv_i32 psw = tcg_temp_new_i32();

    tcg_gen_addi_i32(ent, ctbp, a->imm << 1);
    tcg_gen_qemu_ld_i32(ent, ent, 0, MO_LEUW);
    gen_st_sr(RH850_SR_CTPC, tcg_constant_i32(ctx->base.pc_next));
    tcg_gen_shli_i32(psw, cpu_psw_sat, 4);
    tcg_gen_deposit_i32(psw, psw, cpu_psw_cy, 3, 1);
    tcg_gen_deposit_i32(psw, psw, cpu_psw_ov, 2, 1);
    tcg_gen_deposit_i32(psw, psw, cpu_psw_s, 1, 1);
    tcg_gen_or_i32(psw, psw, cpu_psw_z);
    gen_st_sr(RH850_SR_CTPSW, psw);
    tcg_gen_add_i32(cpu_pc, ctbp, ent);
    tcg_gen_andi_i32(cpu_pc, cpu_pc, ~1u);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

/* CTRET: PC = CTPC, PSW(4:0) = CTPSW(4:0) */
static bool trans_CTRET(DisasContext *ctx, arg_CTRET *a)
{
    TCGv_i32 psw = gen_ld_sr(RH850_SR_CTPSW);

    tcg_gen_andi_i32(cpu_pc, gen_ld_sr(RH850_SR_CTPC), ~1u);
    tcg_gen_extract_i32(cpu_psw_z, psw, 0, 1);
    tcg_gen_extract_i32(cpu_psw_s, psw, 1, 1);
    tcg_gen_extract_i32(cpu_psw_ov, psw, 2, 1);
    tcg_gen_extract_i32(cpu_psw_cy, psw, 3, 1);
    tcg_gen_extract_i32(cpu_psw_sat, psw, 4, 1);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static bool trans_NOP(DisasContext *ctx, arg_NOP *a)
{
    return true;
}

static bool trans_MOV_rr(DisasContext *ctx, arg_rr16 *a)
{
    if (a->r2 == 0) {
        return false;
    }
    set_gpr(a->r2, get_gpr(a->r1));
    return true;
}

static bool trans_MOV_i5(DisasContext *ctx, arg_ri16 *a)
{
    if (a->r2 == 0) {
        return false;
    }
    set_gpr_i(a->r2, a->imm);
    return true;
}

static bool trans_MOV_i32(DisasContext *ctx, arg_MOV_i32 *a)
{
    set_gpr_i(a->r1, a->imm);
    return true;
}

static bool trans_MOVEA(DisasContext *ctx, arg_MOVEA *a)
{
    TCGv_i32 t;

    if (a->r2 == 0) {
        return false;
    }
    t = tcg_temp_new_i32();
    tcg_gen_addi_i32(t, get_gpr(a->r1), a->imm);
    set_gpr(a->r2, t);
    return true;
}

static bool trans_MOVHI(DisasContext *ctx, arg_MOVHI *a)
{
    TCGv_i32 t;

    if (a->r2 == 0) {
        return false;
    }
    t = tcg_temp_new_i32();
    tcg_gen_addi_i32(t, get_gpr(a->r1), (uint32_t)a->imm << 16);
    set_gpr(a->r2, t);
    return true;
}

static bool trans_LD_W(DisasContext *ctx, arg_LD_W *a)
{
    TCGv_i32 addr = tcg_temp_new_i32();
    TCGv_i32 val = tcg_temp_new_i32();

    gen_mei(ctx, MEI_D16, a->r2, MO_UL, false);
    tcg_gen_addi_i32(addr, get_gpr(a->r1), a->disp);
    tcg_gen_qemu_ld_i32(val, addr, 0, MO_LEUL | ctx->align);
    set_gpr(a->r2, val);
    return true;
}

static bool trans_ST_W(DisasContext *ctx, arg_ST_W *a)
{
    TCGv_i32 addr = tcg_temp_new_i32();

    gen_mei(ctx, MEI_D16, a->r2, MO_UL, true);
    tcg_gen_addi_i32(addr, get_gpr(a->r1), a->disp);
    tcg_gen_qemu_st_i32(get_gpr(a->r2), addr, 0, MO_LEUL | ctx->align);
    gen_link_kill(addr, 4);
    return true;
}

/* Flags of reg2 - op */
static void gen_cmp(TCGv_i32 r2, TCGv_i32 op)
{
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_sub_i32(res, r2, op);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_z, res, 0);
    tcg_gen_shri_i32(cpu_psw_s, res, 31);
    tcg_gen_setcond_i32(TCG_COND_LTU, cpu_psw_cy, r2, op);
    tcg_gen_xor_i32(t, r2, op);
    tcg_gen_xor_i32(cpu_psw_ov, r2, res);
    tcg_gen_and_i32(cpu_psw_ov, cpu_psw_ov, t);
    tcg_gen_shri_i32(cpu_psw_ov, cpu_psw_ov, 31);
}

static bool trans_CMP_rr(DisasContext *ctx, arg_rr16 *a)
{
    gen_cmp(get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_CMP_i5(DisasContext *ctx, arg_ri16 *a)
{
    gen_cmp(get_gpr(a->r2), tcg_constant_i32(a->imm));
    return true;
}

/* Set t to 1 if cond holds */
static void gen_cond(TCGv_i32 t, int cond)
{
    if (cond == RH850_COND_SA) {
        tcg_gen_mov_i32(t, cpu_psw_sat);
        return;
    }
    switch (cond & 7) {
    case 0:
        tcg_gen_mov_i32(t, cpu_psw_ov);
        break;
    case 1:
        tcg_gen_mov_i32(t, cpu_psw_cy);
        break;
    case 2:
        tcg_gen_mov_i32(t, cpu_psw_z);
        break;
    case 3:
        tcg_gen_or_i32(t, cpu_psw_cy, cpu_psw_z);
        break;
    case 4:
        tcg_gen_mov_i32(t, cpu_psw_s);
        break;
    case 6:
        tcg_gen_xor_i32(t, cpu_psw_s, cpu_psw_ov);
        break;
    case 7:
        tcg_gen_xor_i32(t, cpu_psw_s, cpu_psw_ov);
        tcg_gen_or_i32(t, t, cpu_psw_z);
        break;
    default:
        g_assert_not_reached();
    }
    if (cond & 8) {
        tcg_gen_xori_i32(t, t, 1);
    }
}

/* Conditional branch to pc + disp; both paths end the TB */
static void gen_bcond(DisasContext *ctx, int cond, int32_t disp)
{
    uint32_t dest = ctx->pc + disp;
    TCGv_i32 t;
    TCGLabel *taken;

    if (cond == RH850_COND_R) {
        gen_goto_tb(ctx, 0, dest);
        return;
    }
    t = tcg_temp_new_i32();
    taken = gen_new_label();
    gen_cond(t, cond);
    tcg_gen_brcondi_i32(TCG_COND_NE, t, 0, taken);
    gen_goto_tb(ctx, 0, ctx->base.pc_next);
    gen_set_label(taken);
    gen_goto_tb(ctx, 1, dest);
}

static bool trans_Bcond9(DisasContext *ctx, arg_Bcond9 *a)
{
    gen_bcond(ctx, a->cond, a->disp);
    return true;
}

static bool trans_JR22(DisasContext *ctx, arg_JR22 *a)
{
    gen_goto_tb(ctx, 0, ctx->pc + a->disp);
    return true;
}

static bool trans_JMP_r(DisasContext *ctx, arg_JMP_r *a)
{
    tcg_gen_andi_i32(cpu_pc, get_gpr(a->r1), ~1u);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

#include "trans_alu.c.inc"
#include "trans_muldiv.c.inc"
#include "trans_mem.c.inc"
#include "trans_branch.c.inc"
#include "trans_excl.c.inc"
#include "trans_fpu.c.inc"

/* LDSR ends the TB: its result applies from the next instruction */
static bool trans_LDSR(DisasContext *ctx, arg_LDSR *a)
{
    tcg_gen_movi_i32(cpu_pc, ctx->pc);
    gen_helper_ldsr(tcg_env, tcg_constant_i32(a->selid),
                    tcg_constant_i32(a->regid), get_gpr(a->rs));
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    ctx->base.is_jmp = DISAS_EXIT;
    return true;
}

static bool trans_STSR(DisasContext *ctx, arg_STSR *a)
{
    TCGv_i32 val = tcg_temp_new_i32();

    tcg_gen_movi_i32(cpu_pc, ctx->pc);
    gen_helper_stsr(val, tcg_env, tcg_constant_i32(a->selid),
                    tcg_constant_i32(a->regid));
    set_gpr(a->rd, val);
    return true;
}

static void rh850_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    ctx->env = cpu_env(cs);
    ctx->align = (ctx->base.tb->flags & RH850_TBFLAG_ALIGN) ? MO_ALIGN : 0;
    ctx->um = FIELD_EX32(ctx->base.tb->flags, PSW, UM);
    ctx->cu0 = ctx->base.tb->flags & RH850_TBFLAG_CU0;
    ctx->fpivd = ctx->base.tb->flags & RH850_TBFLAG_FPIVD;
}

static void rh850_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
}

static void rh850_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(ctx->base.pc_next, 0, 0);
    ctx->insn_start = tcg_last_op();
}

static void rh850_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    uint32_t insn;
    int len;
    bool ok;

    ctx->pc = ctx->base.pc_next;
    insn = next_hw(ctx);
    ctx->insn = insn;

    if (rh850_insn_is_16bit(insn)) {
        ok = decode_insn16(ctx, insn);
    } else {
        insn = deposit32(insn, 16, 16, next_hw(ctx));
        ctx->insn = insn;
        len = rh850_insn_len(insn, insn >> 16);
        ok = decode_insn32(ctx, insn);
        tcg_debug_assert(!ok || ctx->base.is_jmp == DISAS_NORETURN ||
                         ctx->base.pc_next == ctx->pc + len);
    }
    /* No defined instruction: RIE */
    if (!ok) {
        gen_rie(ctx);
    }

    if (ctx->base.is_jmp == DISAS_NEXT &&
        !translator_is_same_page(&ctx->base, ctx->base.pc_next)) {
        ctx->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void rh850_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    switch (ctx->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_goto_tb(ctx, 0, ctx->base.pc_next);
        break;
    case DISAS_JUMP:
        tcg_gen_lookup_and_goto_ptr();
        break;
    case DISAS_EXIT:
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps rh850_tr_ops = {
    .init_disas_context = rh850_tr_init_disas_context,
    .tb_start           = rh850_tr_tb_start,
    .insn_start         = rh850_tr_insn_start,
    .translate_insn     = rh850_tr_translate_insn,
    .tb_stop            = rh850_tr_tb_stop,
};

void rh850_translate_code(CPUState *cs, TranslationBlock *tb,
                          int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &rh850_tr_ops, &dc.base,
                    TCG_TYPE_VA);
}

#define ALLOC_REGISTER(sym, name) \
    cpu_##sym = tcg_global_mem_new_i32(tcg_env, \
                                       offsetof(CPURH850State, sym), name)

void rh850_translate_init(void)
{
    static const char * const regnames[RH850_NUM_GPRS] = {
        "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
        "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
        "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
        "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31",
    };
    int i;

    for (i = 0; i < RH850_NUM_GPRS; i++) {
        cpu_gpr[i] = tcg_global_mem_new_i32(tcg_env,
                                            offsetof(CPURH850State, gpr[i]),
                                            regnames[i]);
    }
    ALLOC_REGISTER(pc, "pc");
    ALLOC_REGISTER(psw_z, "psw_z");
    ALLOC_REGISTER(psw_s, "psw_s");
    ALLOC_REGISTER(psw_ov, "psw_ov");
    ALLOC_REGISTER(psw_cy, "psw_cy");
    ALLOC_REGISTER(psw_sat, "psw_sat");
    ALLOC_REGISTER(ll_addr, "ll_addr");
    ALLOC_REGISTER(ll_size, "ll_size");
    ALLOC_REGISTER(ll_val, "ll_val");
    ALLOC_REGISTER(ll_valid, "ll_valid");
}
