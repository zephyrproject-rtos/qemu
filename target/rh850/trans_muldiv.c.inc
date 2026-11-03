/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 multiply, multiply-accumulate and divide instructions
 */

/* Writing reg3 last leaves only the high word when reg2 == reg3 */
static void gen_mul64(int r2, int r3, TCGv_i32 a, TCGv_i32 b, bool sign)
{
    TCGv_i32 lo = tcg_temp_new_i32();
    TCGv_i32 hi = tcg_temp_new_i32();

    if (sign) {
        tcg_gen_muls2_i32(lo, hi, a, b);
    } else {
        tcg_gen_mulu2_i32(lo, hi, a, b);
    }
    set_gpr(r2, lo);
    set_gpr(r3, hi);
}

static bool trans_MUL_r(DisasContext *ctx, arg_rrr *a)
{
    gen_mul64(a->r2, a->r3, get_gpr(a->r2), get_gpr(a->r1), true);
    return true;
}

static bool trans_MUL_i(DisasContext *ctx, arg_MUL_i *a)
{
    gen_mul64(a->r2, a->r3, get_gpr(a->r2), tcg_constant_i32(a->imm), true);
    return true;
}

static bool trans_MULU_r(DisasContext *ctx, arg_rrr *a)
{
    gen_mul64(a->r2, a->r3, get_gpr(a->r2), get_gpr(a->r1), false);
    return true;
}

static bool trans_MULU_i(DisasContext *ctx, arg_MULU_i *a)
{
    gen_mul64(a->r2, a->r3, get_gpr(a->r2), tcg_constant_i32(a->imm), false);
    return true;
}

/* Signed halfword product of the lower halfwords */
static void gen_mulh(int rd, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 x = tcg_temp_new_i32();
    TCGv_i32 y = tcg_temp_new_i32();

    tcg_gen_ext16s_i32(x, a);
    tcg_gen_ext16s_i32(y, b);
    tcg_gen_mul_i32(x, x, y);
    set_gpr(rd, x);
}

static bool trans_MULH_rr(DisasContext *ctx, arg_rr16 *a)
{
    gen_mulh(a->r2, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_MULH_i5(DisasContext *ctx, arg_ri16 *a)
{
    gen_mulh(a->r2, get_gpr(a->r2), tcg_constant_i32(a->imm));
    return true;
}

/* MULHI: signed halfword by imm16 */
static bool trans_MULHI(DisasContext *ctx, arg_rri *a)
{
    gen_mulh(a->r2, get_gpr(a->r1), tcg_constant_i32(a->imm));
    return true;
}

/* reg3/reg4 are even by encoding (the decoder fixes their bit 0) */
static void gen_mac(arg_mac *a, bool sign)
{
    TCGv_i32 lo = tcg_temp_new_i32();
    TCGv_i32 hi = tcg_temp_new_i32();
    TCGv_i64 p = tcg_temp_new_i64();
    TCGv_i64 acc = tcg_temp_new_i64();

    if (sign) {
        tcg_gen_muls2_i32(lo, hi, get_gpr(a->r2), get_gpr(a->r1));
    } else {
        tcg_gen_mulu2_i32(lo, hi, get_gpr(a->r2), get_gpr(a->r1));
    }
    tcg_gen_concat_i32_i64(p, lo, hi);
    tcg_gen_concat_i32_i64(acc, get_gpr(a->r3), get_gpr(a->r3 + 1));
    tcg_gen_add_i64(acc, acc, p);
    tcg_gen_extr_i64_i32(lo, hi, acc);
    set_gpr(a->r4, lo);
    set_gpr(a->r4 + 1, hi);
}

static bool trans_MAC(DisasContext *ctx, arg_mac *a)
{
    gen_mac(a, true);
    return true;
}

static bool trans_MACU(DisasContext *ctx, arg_mac *a)
{
    gen_mac(a, false);
    return true;
}

/*
 * Quotient to reg2, then remainder to reg3, so reg2 == reg3 keeps the
 * remainder
 */
static void gen_div(int r2, int r3, TCGv_i32 dividend, TCGv_i32 divisor,
                    bool sign, bool rem)
{
    TCGv_i64 qr = tcg_temp_new_i64();
    TCGv_i32 q = tcg_temp_new_i32();
    TCGv_i32 r = tcg_temp_new_i32();

    if (sign) {
        gen_helper_divs(qr, tcg_env, dividend, divisor);
    } else {
        gen_helper_divu(qr, tcg_env, dividend, divisor);
    }
    tcg_gen_extr_i64_i32(q, r, qr);
    set_gpr(r2, q);
    if (rem) {
        set_gpr(r3, r);
    }
}

/* DIVQ: same result and flags as DIV */
static bool trans_DIV(DisasContext *ctx, arg_rrr *a)
{
    gen_div(a->r2, a->r3, get_gpr(a->r2), get_gpr(a->r1), true, true);
    return true;
}

static bool trans_DIVQ(DisasContext *ctx, arg_rrr *a)
{
    return trans_DIV(ctx, a);
}

static bool trans_DIVU(DisasContext *ctx, arg_rrr *a)
{
    gen_div(a->r2, a->r3, get_gpr(a->r2), get_gpr(a->r1), false, true);
    return true;
}

static bool trans_DIVQU(DisasContext *ctx, arg_rrr *a)
{
    return trans_DIVU(ctx, a);
}

/* DIVH: divisor is the sign-extended lower halfword of reg1 */
static void gen_divh(int r2, int r3, int r1, bool sign, bool rem)
{
    TCGv_i32 d = tcg_temp_new_i32();

    if (sign) {
        tcg_gen_ext16s_i32(d, get_gpr(r1));
    } else {
        tcg_gen_ext16u_i32(d, get_gpr(r1));
    }
    gen_div(r2, r3, get_gpr(r2), d, sign, rem);
}

static bool trans_DIVH_rr(DisasContext *ctx, arg_rr16 *a)
{
    gen_divh(a->r2, 0, a->r1, true, false);
    return true;
}

static bool trans_DIVH_r3(DisasContext *ctx, arg_rrr *a)
{
    gen_divh(a->r2, a->r3, a->r1, true, true);
    return true;
}

/* DIVHU: divisor is the zero-extended lower halfword of reg1 */
static bool trans_DIVHU(DisasContext *ctx, arg_rrr *a)
{
    gen_divh(a->r2, a->r3, a->r1, false, true);
    return true;
}
