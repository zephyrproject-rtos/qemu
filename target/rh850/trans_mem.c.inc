/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 load, store and memory bit instructions (little endian)
 */

#define RH850_EP    30

static TCGv_i32 gen_addr(TCGv_i32 base, int32_t disp)
{
    TCGv_i32 addr = tcg_temp_new_i32();

    tcg_gen_addi_i32(addr, base, disp);
    return addr;
}

static void gen_load(DisasContext *ctx, int itype, int rd, TCGv_i32 addr,
                     MemOp mop)
{
    TCGv_i32 val = tcg_temp_new_i32();

    gen_mei(ctx, itype, rd, mop, false);
    tcg_gen_qemu_ld_i32(val, addr, 0, mop | ctx->align);
    set_gpr(rd, val);
}

static void gen_store(DisasContext *ctx, int itype, int rs, TCGv_i32 addr,
                      MemOp mop)
{
    gen_mei(ctx, itype, rs, mop, true);
    tcg_gen_qemu_st_i32(get_gpr(rs), addr, 0, mop | ctx->align);
    gen_link_kill(addr, memop_size(mop));
}

/* Loads write reg3 first so that reg1 == reg3 keeps the reg1 update */
static void gen_load_post(DisasContext *ctx, arg_memi *a, MemOp mop,
                          int step)
{
    TCGv_i32 addr = tcg_temp_new_i32();
    TCGv_i32 val = tcg_temp_new_i32();

    gen_mei(ctx, step > 0 ? MEI_INC : MEI_DEC, a->r3, mop, false);
    tcg_gen_mov_i32(addr, get_gpr(a->r1));
    tcg_gen_qemu_ld_i32(val, addr, 0, mop | ctx->align);
    set_gpr(a->r3, val);
    tcg_gen_addi_i32(addr, addr, step);
    set_gpr(a->r1, addr);
}

static void gen_store_post(DisasContext *ctx, arg_memi *a, MemOp mop,
                           int step)
{
    TCGv_i32 addr = tcg_temp_new_i32();

    gen_mei(ctx, step > 0 ? MEI_INC : MEI_DEC, a->r3, mop, true);
    tcg_gen_mov_i32(addr, get_gpr(a->r1));
    tcg_gen_qemu_st_i32(get_gpr(a->r3), addr, 0, mop | ctx->align);
    gen_link_kill(addr, memop_size(mop));
    tcg_gen_addi_i32(addr, addr, step);
    set_gpr(a->r1, addr);
}

#define TRANS_LD(NAME, ARG, RD, MOP, IT) \
    static bool trans_##NAME(DisasContext *ctx, arg_##ARG * a) \
    {                                                           \
        gen_load(ctx, IT, a->RD, gen_addr(get_gpr(a->r1), a->disp), MOP); \
        return true;                                            \
    }

#define TRANS_ST(NAME, ARG, RS, MOP, IT) \
    static bool trans_##NAME(DisasContext *ctx, arg_##ARG * a) \
    {                                                           \
        gen_store(ctx, IT, a->RS, gen_addr(get_gpr(a->r1), a->disp), MOP); \
        return true;                                            \
    }

#define TRANS_POST(NAME, FN, MOP, STEP) \
    static bool trans_##NAME(DisasContext *ctx, arg_memi * a) \
    {                                                         \
        FN(ctx, a, MOP, STEP);                                     \
        return true;                                          \
    }

TRANS_LD(LD_B16, mem, r2, MO_SB, MEI_D16)
TRANS_LD(LD_BU16, LD_BU16, r2, MO_UB, MEI_D16)
TRANS_LD(LD_H16, mem, r2, MO_LESW, MEI_D16)
TRANS_LD(LD_HU16, LD_HU16, r2, MO_LEUW, MEI_D16)
TRANS_LD(LD_B23, mem23, r3, MO_SB, MEI_D23)
TRANS_LD(LD_BU23, mem23, r3, MO_UB, MEI_D23)
TRANS_LD(LD_H23, mem23, r3, MO_LESW, MEI_D23)
TRANS_LD(LD_HU23, mem23, r3, MO_LEUW, MEI_D23)
TRANS_LD(LD_W23, mem23, r3, MO_LEUL, MEI_D23)
TRANS_POST(LD_B_INC, gen_load_post, MO_SB, 1)
TRANS_POST(LD_B_DEC, gen_load_post, MO_SB, -1)
TRANS_POST(LD_BU_INC, gen_load_post, MO_UB, 1)
TRANS_POST(LD_BU_DEC, gen_load_post, MO_UB, -1)
TRANS_POST(LD_H_INC, gen_load_post, MO_LESW, 2)
TRANS_POST(LD_H_DEC, gen_load_post, MO_LESW, -2)
TRANS_POST(LD_HU_INC, gen_load_post, MO_LEUW, 2)
TRANS_POST(LD_HU_DEC, gen_load_post, MO_LEUW, -2)
TRANS_POST(LD_W_INC, gen_load_post, MO_LEUL, 4)
TRANS_POST(LD_W_DEC, gen_load_post, MO_LEUL, -4)

TRANS_ST(ST_B16, mem, r2, MO_UB, MEI_D16)
TRANS_ST(ST_H16, mem, r2, MO_LEUW, MEI_D16)
TRANS_ST(ST_B23, mem23, r3, MO_UB, MEI_D23)
TRANS_ST(ST_H23, mem23, r3, MO_LEUW, MEI_D23)
TRANS_ST(ST_W23, mem23, r3, MO_LEUL, MEI_D23)
TRANS_POST(ST_B_INC, gen_store_post, MO_UB, 1)
TRANS_POST(ST_B_DEC, gen_store_post, MO_UB, -1)
TRANS_POST(ST_H_INC, gen_store_post, MO_LEUW, 2)
TRANS_POST(ST_H_DEC, gen_store_post, MO_LEUW, -2)
TRANS_POST(ST_W_INC, gen_store_post, MO_LEUL, 4)
TRANS_POST(ST_W_DEC, gen_store_post, MO_LEUL, -4)

/*
 * LD.DW/ST.DW: reg3 bit 0 is ignored; a non word-aligned address
 * raises MAE whatever MCTL.MA says
 */
static void gen_mae_if_unaligned(DisasContext *ctx, TCGv_i32 addr, int mask,
                                 uint32_t mei)
{
    TCGLabel *ok = gen_new_label();
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_andi_i32(t, addr, mask);
    tcg_gen_brcondi_i32(TCG_COND_EQ, t, 0, ok);
    tcg_gen_movi_i32(cpu_pc, ctx->pc);
    gen_helper_mae(tcg_env, addr, tcg_constant_i32(mei));
    gen_set_label(ok);
}

static bool trans_LD_DW(DisasContext *ctx, arg_mem23 *a)
{
    TCGv_i32 addr = gen_addr(get_gpr(a->r1), a->disp);
    TCGv_i64 val = tcg_temp_new_i64();
    TCGv_i32 lo = tcg_temp_new_i32();
    TCGv_i32 hi = tcg_temp_new_i32();
    int r3 = a->r3 & ~1;

    gen_mae_if_unaligned(ctx, addr, 3,
                         gen_mei(ctx, MEI_D23, a->r3, MO_UQ, false));
    tcg_gen_qemu_ld_i64(val, addr, 0, MO_LEUQ);
    tcg_gen_extr_i64_i32(lo, hi, val);
    set_gpr(r3, lo);
    set_gpr(r3 + 1, hi);
    return true;
}

static bool trans_ST_DW(DisasContext *ctx, arg_mem23 *a)
{
    TCGv_i32 addr = gen_addr(get_gpr(a->r1), a->disp);
    TCGv_i64 val = tcg_temp_new_i64();
    int r3 = a->r3 & ~1;

    gen_mae_if_unaligned(ctx, addr, 3,
                         gen_mei(ctx, MEI_D23, a->r3, MO_UQ, true));
    tcg_gen_concat_i32_i64(val, get_gpr(r3), get_gpr(r3 + 1));
    tcg_gen_qemu_st_i64(val, addr, 0, MO_LEUQ);
    gen_link_kill(addr, 8);
    return true;
}

/* SLD/SST: ep (r30) + zero-extended displacement, scaled by the decoder */
#define TRANS_SLD(NAME, MOP) \
    static bool trans_##NAME(DisasContext *ctx, arg_sld * a) \
    {                                                        \
        gen_load(ctx, MEI_SLD, a->r2, gen_addr(get_gpr(RH850_EP), a->disp), \
                 MOP);                                       \
        return true;                                         \
    }

#define TRANS_SST(NAME, MOP) \
    static bool trans_##NAME(DisasContext *ctx, arg_sld * a) \
    {                                                        \
        gen_store(ctx, MEI_SLD, a->r2, gen_addr(get_gpr(RH850_EP), a->disp), \
                  MOP);                                      \
        return true;                                         \
    }

TRANS_SLD(SLD_B, MO_SB)
TRANS_SLD(SLD_BU, MO_UB)
TRANS_SLD(SLD_H, MO_LESW)
TRANS_SLD(SLD_HU, MO_LEUW)
TRANS_SLD(SLD_W, MO_LEUL)
TRANS_SST(SST_B, MO_UB)
TRANS_SST(SST_H, MO_LEUW)
TRANS_SST(SST_W, MO_LEUL)

/* SET1/NOT1/CLR1/TST1: byte read-modify-write, Z = NOT (old bit) */
enum { BIT_SET, BIT_NOT, BIT_CLR, BIT_TST };

static void gen_bitop(int op, TCGv_i32 addr, TCGv_i32 bit)
{
    TCGv_i32 val = tcg_temp_new_i32();
    TCGv_i32 mask = tcg_temp_new_i32();
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_andi_i32(t, bit, 7);
    tcg_gen_shl_i32(mask, tcg_constant_i32(1), t);
    tcg_gen_qemu_ld_i32(val, addr, 0, MO_UB);
    tcg_gen_and_i32(t, val, mask);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_z, t, 0);
    switch (op) {
    case BIT_SET:
        tcg_gen_or_i32(val, val, mask);
        break;
    case BIT_NOT:
        tcg_gen_xor_i32(val, val, mask);
        break;
    case BIT_CLR:
        tcg_gen_andc_i32(val, val, mask);
        break;
    case BIT_TST:
        return;
    }
    tcg_gen_qemu_st_i32(val, addr, 0, MO_UB);
    gen_link_kill(addr, 1);
}

#define TRANS_BIT(NAME, OP) \
    static bool trans_##NAME(DisasContext *ctx, arg_bit * a) \
    {                                                        \
        gen_bitop(OP, gen_addr(get_gpr(a->r1), a->disp),     \
                  tcg_constant_i32(a->bit));                 \
        return true;                                         \
    }                                                        \
    static bool trans_##NAME##_r(DisasContext *ctx, arg_rr * a) \
    {                                                        \
        gen_bitop(OP, get_gpr(a->r1), get_gpr(a->r2));       \
        return true;                                         \
    }

TRANS_BIT(SET1, BIT_SET)
TRANS_BIT(NOT1, BIT_NOT)
TRANS_BIT(CLR1, BIT_CLR)
TRANS_BIT(TST1, BIT_TST)
