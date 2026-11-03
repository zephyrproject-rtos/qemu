/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 exclusive access, synchronization and cache instructions. STC
 * stores with an atomic compare-and-exchange against the loaded value.
 */

static void gen_ldl(DisasContext *ctx, arg_memi *a, MemOp mop)
{
    TCGv_i32 addr = tcg_temp_new_i32();
    TCGv_i32 val = tcg_temp_new_i32();
    int size = memop_size(mop);

    tcg_gen_mov_i32(addr, get_gpr(a->r1));
    if (size > 1) {
        /* A misaligned LDL always raises MAE */
        gen_mae_if_unaligned(ctx, addr, size - 1,
                             gen_mei(ctx, MEI_LDL, a->r3, mop, false));
    }
    tcg_gen_movi_i32(cpu_ll_valid, 1);
    tcg_gen_st_i32(tcg_constant_i32(1), tcg_env,
                   offsetof(CPURH850State, ldl_active));
    tcg_gen_qemu_ld_i32(val, addr, 0, mop);
    tcg_gen_st_i32(tcg_constant_i32(0), tcg_env,
                   offsetof(CPURH850State, ldl_active));
    tcg_gen_mov_i32(cpu_ll_addr, addr);
    tcg_gen_movi_i32(cpu_ll_size, size);
    tcg_gen_mov_i32(cpu_ll_val, val);
    set_gpr(a->r3, val);
}

static bool trans_LDL_BU(DisasContext *ctx, arg_memi *a)
{
    gen_ldl(ctx, a, MO_UB);
    return true;
}

static bool trans_LDL_HU(DisasContext *ctx, arg_memi *a)
{
    gen_ldl(ctx, a, MO_LEUW);
    return true;
}

static bool trans_LDL_W(DisasContext *ctx, arg_memi *a)
{
    gen_ldl(ctx, a, MO_LEUL);
    return true;
}

/* STC: reg3 = 1 on success, 0 on failure; the link is lost either way */
static void gen_stc(DisasContext *ctx, arg_memi *a, MemOp mop)
{
    TCGv_i32 addr = tcg_temp_new_i32();
    TCGv_i32 old = tcg_temp_new_i32();
    TCGv_i32 ok = tcg_temp_new_i32();
    TCGLabel *fail = gen_new_label();
    TCGLabel *done = gen_new_label();
    int size = memop_size(mop);

    tcg_gen_mov_i32(addr, get_gpr(a->r1));
    if (size > 1) {
        gen_mae_if_unaligned(ctx, addr, size - 1,
                             gen_mei(ctx, MEI_LDL, a->r3, mop, true));
    }
    tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_ll_valid, 0, fail);
    tcg_gen_brcond_i32(TCG_COND_NE, cpu_ll_addr, addr, fail);
    tcg_gen_brcondi_i32(TCG_COND_NE, cpu_ll_size, size, fail);
    tcg_gen_atomic_cmpxchg_i32(old, addr, cpu_ll_val, get_gpr(a->r3), 0,
                               mop);
    tcg_gen_setcond_i32(TCG_COND_EQ, ok, old, cpu_ll_val);
    tcg_gen_br(done);
    gen_set_label(fail);
    tcg_gen_movi_i32(ok, 0);
    gen_set_label(done);
    tcg_gen_movi_i32(cpu_ll_valid, 0);
    set_gpr(a->r3, ok);
}

static bool trans_STC_B(DisasContext *ctx, arg_memi *a)
{
    gen_stc(ctx, a, MO_UB);
    return true;
}

static bool trans_STC_H(DisasContext *ctx, arg_memi *a)
{
    gen_stc(ctx, a, MO_LEUW);
    return true;
}

static bool trans_STC_W(DisasContext *ctx, arg_memi *a)
{
    gen_stc(ctx, a, MO_LEUL);
    return true;
}

/* CLL (also CACHE with cacheop 1111110) */
static bool trans_CLL(DisasContext *ctx, arg_CLL *a)
{
    tcg_gen_movi_i32(cpu_ll_valid, 0);
    return true;
}

/*
 * CAXI uses an atomic compare-and-exchange: rewriting the unchanged
 * token is not visible in RAM
 */
static bool trans_CAXI(DisasContext *ctx, arg_rrr *a)
{
    TCGv_i32 addr = tcg_temp_new_i32();
    TCGv_i32 cmpv = tcg_temp_new_i32();
    TCGv_i32 token = tcg_temp_new_i32();

    tcg_gen_mov_i32(addr, get_gpr(a->r1));
    tcg_gen_mov_i32(cmpv, get_gpr(a->r2));
    /* MEI: REG = dst, read */
    gen_mae_if_unaligned(ctx, addr, 3,
                         gen_mei(ctx, MEI_CAXI, a->r3, MO_UL, false));
    tcg_gen_atomic_cmpxchg_i32(token, addr, cmpv, get_gpr(a->r3), 0,
                               MO_LEUL);
    gen_cmp(cmpv, token);
    gen_link_kill(addr, 4);
    set_gpr(a->r3, token);
    return true;
}

/*
 * SYNCM is a full barrier and SYNCI ends the TB; SYNCE ends the TB so
 * that a pending FPI is taken before the next instruction
 */
static bool trans_SYNCP(DisasContext *ctx, arg_SYNCP *a)
{
    return true;
}

static bool trans_SYNCE(DisasContext *ctx, arg_SYNCE *a)
{
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    ctx->base.is_jmp = DISAS_EXIT;
    return true;
}

static bool trans_SYNCM(DisasContext *ctx, arg_SYNCM *a)
{
    tcg_gen_mb(TCG_MO_ALL | TCG_BAR_SC);
    return true;
}

static bool trans_SYNCI(DisasContext *ctx, arg_SYNCI *a)
{
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    ctx->base.is_jmp = DISAS_EXIT;
    return true;
}

/*
 * The instruction cache is not modelled: privileged cacheops raise PIE
 * in user mode, otherwise they do nothing
 */
static bool trans_CACHE(DisasContext *ctx, arg_CACHE *a)
{
    switch (a->op) {
    case 0x20:  /* CIBII */
    case 0x40:  /* CFALI */
    case 0x60:  /* CISTI */
    case 0x61:  /* CILDI */
        if (ctx->um) {
            gen_raise(ctx, RH850_EXCP_PIE, RH850_CAUSE_PIE, ctx->pc);
        }
        break;
    }
    return true;
}

/* PREF is a hint without effect on program state */
static bool trans_PREF(DisasContext *ctx, arg_PREF *a)
{
    return true;
}
