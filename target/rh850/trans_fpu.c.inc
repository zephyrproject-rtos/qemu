/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 FPU instructions. Doubles use the pair {r(n + 1), r(n)}, n even;
 * the pair {r1, r0} reads 0 in its low word and drops writes to it.
 */

/* PSW.CU0 = 0: UCPOP for every FPU instruction */
static bool fpu_usable(DisasContext *ctx)
{
    if (!ctx->cu0) {
        gen_raise(ctx, RH850_EXCP_UCPOP, RH850_CAUSE_UCPOP, ctx->pc);
        return false;
    }
    return true;
}

/* FPEC.FPIVD = 1: FPU instructions except TRFSR have no effect */
static bool fpu_ok(DisasContext *ctx)
{
    return fpu_usable(ctx) && !ctx->fpivd;
}

static TCGv_i64 gen_ld_pair(int r)
{
    TCGv_i64 t = tcg_temp_new_i64();

    tcg_gen_concat_i32_i64(t, get_gpr(r), get_gpr(r + 1));
    return t;
}

static void gen_st_pair(int r, TCGv_i64 v)
{
    TCGv_i32 lo = tcg_temp_new_i32();
    TCGv_i32 hi = tcg_temp_new_i32();

    tcg_gen_extr_i64_i32(lo, hi, v);
    set_gpr(r, lo);
    set_gpr(r + 1, hi);
}

static TCGv_i32 gen_fpcc(int fcbit)
{
    TCGv_i32 t = gen_ld_sr(RH850_SR_FPSR);

    tcg_gen_extract_i32(t, t, 24 + fcbit, 1);
    return t;
}

/* TRFSR: PSW.Z = CC[fcbit] */
static bool trans_TRFSR(DisasContext *ctx, arg_TRFSR *a)
{
    if (fpu_usable(ctx)) {
        tcg_gen_mov_i32(cpu_psw_z, gen_fpcc(a->fcbit));
    }
    return true;
}

/* CMOVF: no FP exception, no flush; reg3 = r0 is reserved */
static bool trans_CMOVF_S(DisasContext *ctx, arg_CMOVF_S *a)
{
    TCGv_i32 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i32();
        tcg_gen_movcond_i32(TCG_COND_NE, t, gen_fpcc(a->fcbit),
                            tcg_constant_i32(0), get_gpr(a->r1),
                            get_gpr(a->r2));
        set_gpr(a->r3, t);
    }
    return true;
}

static bool trans_CMOVF_D(DisasContext *ctx, arg_CMOVF_D *a)
{
    TCGv_i64 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i64();
        tcg_gen_extu_i32_i64(t, gen_fpcc(a->fcbit));
        tcg_gen_movcond_i64(TCG_COND_NE, t, t, tcg_constant_i64(0),
                            gen_ld_pair(a->r1), gen_ld_pair(a->r2));
        gen_st_pair(a->r3, t);
    }
    return true;
}

static bool trans_CMPF_S(DisasContext *ctx, arg_CMPF_S *a)
{
    if (fpu_ok(ctx)) {
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        gen_helper_cmpf_float32(tcg_env, get_gpr(a->r2), get_gpr(a->r1),
                                tcg_constant_i32(a->fcbit << 4 | a->fcond));
    }
    return true;
}

static bool trans_CMPF_D(DisasContext *ctx, arg_CMPF_D *a)
{
    if (fpu_ok(ctx)) {
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        gen_helper_cmpf_float64(tcg_env, gen_ld_pair(a->r2),
                                gen_ld_pair(a->r1),
                                tcg_constant_i32(a->fcbit << 4 | a->fcond));
    }
    return true;
}

/* ABSF/NEGF change the sign bit only, also for NaN; no FP exception */
static bool gen_sign_s(DisasContext *ctx, arg_fp2 *a, bool neg)
{
    TCGv_i32 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i32();
        if (neg) {
            tcg_gen_xori_i32(t, get_gpr(a->r2), 0x80000000u);
        } else {
            tcg_gen_andi_i32(t, get_gpr(a->r2), 0x7fffffffu);
        }
        set_gpr(a->r3, t);
    }
    return true;
}

static bool gen_sign_d(DisasContext *ctx, arg_fp2 *a, bool neg)
{
    TCGv_i64 t;

    if (fpu_ok(ctx)) {
        t = gen_ld_pair(a->r2);
        if (neg) {
            tcg_gen_xori_i64(t, t, INT64_MIN);
        } else {
            tcg_gen_andi_i64(t, t, INT64_MAX);
        }
        gen_st_pair(a->r3, t);
    }
    return true;
}

static bool trans_ABSF_S(DisasContext *ctx, arg_ABSF_S *a)
{
    return gen_sign_s(ctx, a, false);
}

static bool trans_NEGF_S(DisasContext *ctx, arg_NEGF_S *a)
{
    return gen_sign_s(ctx, a, true);
}

static bool trans_ABSF_D(DisasContext *ctx, arg_ABSF_D *a)
{
    return gen_sign_d(ctx, a, false);
}

static bool trans_NEGF_D(DisasContext *ctx, arg_NEGF_D *a)
{
    return gen_sign_d(ctx, a, true);
}

/* The PC is set for FPEPC */
typedef void gen_fop_s_fn(TCGv_i32, TCGv_env, TCGv_i32, TCGv_i32);
typedef void gen_fop_d_fn(TCGv_i64, TCGv_env, TCGv_i64, TCGv_i64);

static bool gen_fop_s(DisasContext *ctx, int r1, int r2, int r3,
                      gen_fop_s_fn *fn)
{
    TCGv_i32 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i32();
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        fn(t, tcg_env, get_gpr(r2), get_gpr(r1));
        set_gpr(r3, t);
    }
    return true;
}

static bool gen_fop_d(DisasContext *ctx, int r1, int r2, int r3,
                      gen_fop_d_fn *fn)
{
    TCGv_i64 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i64();
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        fn(t, tcg_env, gen_ld_pair(r2), gen_ld_pair(r1));
        gen_st_pair(r3, t);
    }
    return true;
}

/* One operand (reg2): 0 is passed for the absent reg1 field */
#define TRANS_FOP1(NAME, HELPER)                                        \
    static bool trans_##NAME##_S(DisasContext *ctx, arg_##NAME##_S * a) \
    {                                                                   \
        return gen_fop_s(ctx, 0, a->r2, a->r3, gen_helper_##HELPER##_s); \
    }                                                                   \
    static bool trans_##NAME##_D(DisasContext *ctx, arg_##NAME##_D * a) \
    {                                                                   \
        return gen_fop_d(ctx, 0, a->r2, a->r3, gen_helper_##HELPER##_d); \
    }

TRANS_FOP1(SQRTF, sqrtf)
TRANS_FOP1(RECIPF, recipf)
TRANS_FOP1(RSQRTF, rsqrtf)

/* FMAF/FMSF/FNMAF/FNMSF: single only */
typedef void gen_fma_fn(TCGv_i32, TCGv_env, TCGv_i32, TCGv_i32, TCGv_i32);

static bool gen_fma(DisasContext *ctx, arg_fp3 *a, gen_fma_fn *fn)
{
    TCGv_i32 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i32();
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        fn(t, tcg_env, get_gpr(a->r2), get_gpr(a->r1), get_gpr(a->r3));
        set_gpr(a->r3, t);
    }
    return true;
}

static bool trans_FMAF_S(DisasContext *ctx, arg_fp3 *a)
{
    return gen_fma(ctx, a, gen_helper_fmaf_s);
}

static bool trans_FMSF_S(DisasContext *ctx, arg_fp3 *a)
{
    return gen_fma(ctx, a, gen_helper_fmsf_s);
}

static bool trans_FNMAF_S(DisasContext *ctx, arg_fp3 *a)
{
    return gen_fma(ctx, a, gen_helper_fnmaf_s);
}

static bool trans_FNMSF_S(DisasContext *ctx, arg_fp3 *a)
{
    return gen_fma(ctx, a, gen_helper_fnmsf_s);
}

#define TRANS_FOP(NAME, HELPER)                                         \
    static bool trans_##NAME##_S(DisasContext *ctx, arg_##NAME##_S * a) \
    {                                                                   \
        return gen_fop_s(ctx, a->r1, a->r2, a->r3,                      \
                         gen_helper_##HELPER##_s);                      \
    }                                                                   \
    static bool trans_##NAME##_D(DisasContext *ctx, arg_##NAME##_D * a) \
    {                                                                   \
        return gen_fop_d(ctx, a->r1, a->r2, a->r3,                      \
                         gen_helper_##HELPER##_d);                      \
    }

TRANS_FOP(ADDF, addf)
TRANS_FOP(SUBF, subf)
TRANS_FOP(MULF, mulf)
TRANS_FOP(DIVF, divf)

TRANS_FOP(MAXF, maxf)
TRANS_FOP(MINF, minf)

/*
 * S/D name the register width of source and destination (single
 * register or pair)
 */
typedef void gen_cvt_ss_fn(TCGv_i32, TCGv_env, TCGv_i32, TCGv_i32);
typedef void gen_cvt_sd_fn(TCGv_i64, TCGv_env, TCGv_i32, TCGv_i32);
typedef void gen_cvt_ds_fn(TCGv_i32, TCGv_env, TCGv_i64, TCGv_i32);
typedef void gen_cvt_dd_fn(TCGv_i64, TCGv_env, TCGv_i64, TCGv_i32);

static bool gen_cvt_ss(DisasContext *ctx, arg_fp2 *a, gen_cvt_ss_fn *fn,
                       int ctl)
{
    TCGv_i32 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i32();
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        fn(t, tcg_env, get_gpr(a->r2), tcg_constant_i32(ctl));
        set_gpr(a->r3, t);
    }
    return true;
}

static bool gen_cvt_sd(DisasContext *ctx, arg_fp2 *a, gen_cvt_sd_fn *fn,
                       int ctl)
{
    TCGv_i64 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i64();
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        fn(t, tcg_env, get_gpr(a->r2), tcg_constant_i32(ctl));
        gen_st_pair(a->r3, t);
    }
    return true;
}

static bool gen_cvt_ds(DisasContext *ctx, arg_fp2 *a, gen_cvt_ds_fn *fn,
                       int ctl)
{
    TCGv_i32 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i32();
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        fn(t, tcg_env, gen_ld_pair(a->r2), tcg_constant_i32(ctl));
        set_gpr(a->r3, t);
    }
    return true;
}

static bool gen_cvt_dd(DisasContext *ctx, arg_fp2 *a, gen_cvt_dd_fn *fn,
                       int ctl)
{
    TCGv_i64 t;

    if (fpu_ok(ctx)) {
        t = tcg_temp_new_i64();
        tcg_gen_movi_i32(cpu_pc, ctx->pc);
        fn(t, tcg_env, gen_ld_pair(a->r2), tcg_constant_i32(ctl));
        gen_st_pair(a->r3, t);
    }
    return true;
}

#define TRANS_CVT(W, NAME, HELPER, CTL)                                 \
    static bool trans_##NAME(DisasContext *ctx, arg_##NAME * a)         \
    {                                                                   \
        return gen_cvt_##W(ctx, a, HELPER, CTL);                        \
    }
#define TRANS_CVT_SS(NAME, HELPER, CTL) TRANS_CVT(ss, NAME, HELPER, CTL)
#define TRANS_CVT_SD(NAME, HELPER, CTL) TRANS_CVT(sd, NAME, HELPER, CTL)
#define TRANS_CVT_DS(NAME, HELPER, CTL) TRANS_CVT(ds, NAME, HELPER, CTL)
#define TRANS_CVT_DD(NAME, HELPER, CTL) TRANS_CVT(dd, NAME, HELPER, CTL)

TRANS_CVT_DD(CEILF_DL, gen_helper_f2i_dl, CVT_RP)
TRANS_CVT_DD(CEILF_DUL, gen_helper_f2i_dl, CVT_RP | CVT_U)
TRANS_CVT_DS(CEILF_DUW, gen_helper_f2i_dw, CVT_RP | CVT_U)
TRANS_CVT_DS(CEILF_DW, gen_helper_f2i_dw, CVT_RP)
TRANS_CVT_SD(CEILF_SL, gen_helper_f2i_sl, CVT_RP)
TRANS_CVT_SD(CEILF_SUL, gen_helper_f2i_sl, CVT_RP | CVT_U)
TRANS_CVT_SS(CEILF_SUW, gen_helper_f2i_sw, CVT_RP | CVT_U)
TRANS_CVT_SS(CEILF_SW, gen_helper_f2i_sw, CVT_RP)
TRANS_CVT_DD(CVTF_DL, gen_helper_f2i_dl, CVT_RM)
TRANS_CVT_DD(CVTF_DUL, gen_helper_f2i_dl, CVT_RM | CVT_U)
TRANS_CVT_DS(CVTF_DUW, gen_helper_f2i_dw, CVT_RM | CVT_U)
TRANS_CVT_DS(CVTF_DW, gen_helper_f2i_dw, CVT_RM)
TRANS_CVT_DD(CVTF_LD, gen_helper_i2f_ld, 0)
TRANS_CVT_DS(CVTF_LS, gen_helper_i2f_ls, 0)
TRANS_CVT_SD(CVTF_SL, gen_helper_f2i_sl, CVT_RM)
TRANS_CVT_SD(CVTF_SUL, gen_helper_f2i_sl, CVT_RM | CVT_U)
TRANS_CVT_SS(CVTF_SUW, gen_helper_f2i_sw, CVT_RM | CVT_U)
TRANS_CVT_SS(CVTF_SW, gen_helper_f2i_sw, CVT_RM)
TRANS_CVT_DD(CVTF_ULD, gen_helper_i2f_ld, CVT_U)
TRANS_CVT_DS(CVTF_ULS, gen_helper_i2f_ls, CVT_U)
TRANS_CVT_SD(CVTF_UWD, gen_helper_i2f_wd, CVT_U)
TRANS_CVT_SS(CVTF_UWS, gen_helper_i2f_ws, CVT_U)
TRANS_CVT_SD(CVTF_WD, gen_helper_i2f_wd, 0)
TRANS_CVT_SS(CVTF_WS, gen_helper_i2f_ws, 0)
TRANS_CVT_DD(FLOORF_DL, gen_helper_f2i_dl, CVT_RMI)
TRANS_CVT_DD(FLOORF_DUL, gen_helper_f2i_dl, CVT_RMI | CVT_U)
TRANS_CVT_DS(FLOORF_DUW, gen_helper_f2i_dw, CVT_RMI | CVT_U)
TRANS_CVT_DS(FLOORF_DW, gen_helper_f2i_dw, CVT_RMI)
TRANS_CVT_SD(FLOORF_SL, gen_helper_f2i_sl, CVT_RMI)
TRANS_CVT_SD(FLOORF_SUL, gen_helper_f2i_sl, CVT_RMI | CVT_U)
TRANS_CVT_SS(FLOORF_SUW, gen_helper_f2i_sw, CVT_RMI | CVT_U)
TRANS_CVT_SS(FLOORF_SW, gen_helper_f2i_sw, CVT_RMI)
TRANS_CVT_DD(ROUNDF_DL, gen_helper_f2i_dl, CVT_RN)
TRANS_CVT_DD(ROUNDF_DUL, gen_helper_f2i_dl, CVT_RN | CVT_U)
TRANS_CVT_DS(ROUNDF_DUW, gen_helper_f2i_dw, CVT_RN | CVT_U)
TRANS_CVT_DS(ROUNDF_DW, gen_helper_f2i_dw, CVT_RN)
TRANS_CVT_SD(ROUNDF_SL, gen_helper_f2i_sl, CVT_RN)
TRANS_CVT_SD(ROUNDF_SUL, gen_helper_f2i_sl, CVT_RN | CVT_U)
TRANS_CVT_SS(ROUNDF_SUW, gen_helper_f2i_sw, CVT_RN | CVT_U)
TRANS_CVT_SS(ROUNDF_SW, gen_helper_f2i_sw, CVT_RN)
TRANS_CVT_DD(TRNCF_DL, gen_helper_f2i_dl, CVT_RZ)
TRANS_CVT_DD(TRNCF_DUL, gen_helper_f2i_dl, CVT_RZ | CVT_U)
TRANS_CVT_DS(TRNCF_DUW, gen_helper_f2i_dw, CVT_RZ | CVT_U)
TRANS_CVT_DS(TRNCF_DW, gen_helper_f2i_dw, CVT_RZ)
TRANS_CVT_SD(TRNCF_SL, gen_helper_f2i_sl, CVT_RZ)
TRANS_CVT_SD(TRNCF_SUL, gen_helper_f2i_sl, CVT_RZ | CVT_U)
TRANS_CVT_SS(TRNCF_SUW, gen_helper_f2i_sw, CVT_RZ | CVT_U)
TRANS_CVT_SS(TRNCF_SW, gen_helper_f2i_sw, CVT_RZ)

/* CVTF.SD, DS, SH and HS take no rounding control */
static void gen_cvtf_sd(TCGv_i64 r, TCGv_env env, TCGv_i32 a,
                                 TCGv_i32 ctl)
{
    gen_helper_cvtf_sd(r, env, a);
}

static void gen_cvtf_ds(TCGv_i32 r, TCGv_env env, TCGv_i64 a,
                                 TCGv_i32 ctl)
{
    gen_helper_cvtf_ds(r, env, a);
}

static void gen_cvtf_sh(TCGv_i32 r, TCGv_env env, TCGv_i32 a,
                                 TCGv_i32 ctl)
{
    gen_helper_cvtf_sh(r, env, a);
}

static void gen_cvtf_hs(TCGv_i32 r, TCGv_env env, TCGv_i32 a,
                                 TCGv_i32 ctl)
{
    gen_helper_cvtf_hs(r, env, a);
}

TRANS_CVT_SD(CVTF_SD, gen_cvtf_sd, 0)
TRANS_CVT_DS(CVTF_DS, gen_cvtf_ds, 0)
TRANS_CVT_SS(CVTF_SH, gen_cvtf_sh, 0)
TRANS_CVT_SS(CVTF_HS, gen_cvtf_hs, 0)
