/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 integer ALU instructions. Flags are kept split, one 0/1 value
 * per TCG global.
 */

static void gen_flags_zs(TCGv_i32 res)
{
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_z, res, 0);
    tcg_gen_shri_i32(cpu_psw_s, res, 31);
}

/* Logic result: OV = 0, S and Z from the result, CY unchanged */
static void gen_flags_logic(TCGv_i32 res)
{
    tcg_gen_movi_i32(cpu_psw_ov, 0);
    gen_flags_zs(res);
}

/* res = a + b + c (c is 0 or 1), all four arithmetic flags */
static void gen_adc_cc(TCGv_i32 res, TCGv_i32 a, TCGv_i32 b, TCGv_i32 c)
{
    TCGv_i32 r = tcg_temp_new_i32();
    TCGv_i32 cy = tcg_temp_new_i32();
    TCGv_i32 t = tcg_temp_new_i32();
    TCGv_i32 zero = tcg_constant_i32(0);

    tcg_gen_add2_i32(r, cy, a, zero, c, zero);
    tcg_gen_add2_i32(r, cy, r, cy, b, zero);
    tcg_gen_xor_i32(cpu_psw_ov, r, a);
    tcg_gen_xor_i32(t, a, b);
    tcg_gen_andc_i32(cpu_psw_ov, cpu_psw_ov, t);
    tcg_gen_shri_i32(cpu_psw_ov, cpu_psw_ov, 31);
    tcg_gen_mov_i32(cpu_psw_cy, cy);
    gen_flags_zs(r);
    tcg_gen_mov_i32(res, r);
}

/* res = a - b - c (c is 0 or 1), CY is the borrow */
static void gen_sbc_cc(TCGv_i32 res, TCGv_i32 a, TCGv_i32 b, TCGv_i32 c)
{
    TCGv_i32 r = tcg_temp_new_i32();
    TCGv_i32 hi = tcg_temp_new_i32();
    TCGv_i32 t = tcg_temp_new_i32();
    TCGv_i32 zero = tcg_constant_i32(0);

    tcg_gen_sub2_i32(r, hi, a, zero, b, zero);
    tcg_gen_sub2_i32(r, hi, r, hi, c, zero);
    tcg_gen_xor_i32(cpu_psw_ov, a, r);
    tcg_gen_xor_i32(t, a, b);
    tcg_gen_and_i32(cpu_psw_ov, cpu_psw_ov, t);
    tcg_gen_shri_i32(cpu_psw_ov, cpu_psw_ov, 31);
    tcg_gen_andi_i32(cpu_psw_cy, hi, 1);
    gen_flags_zs(r);
    tcg_gen_mov_i32(res, r);
}

static void gen_add_cc(TCGv_i32 res, TCGv_i32 a, TCGv_i32 b)
{
    gen_adc_cc(res, a, b, tcg_constant_i32(0));
}

static void gen_sub_cc(TCGv_i32 res, TCGv_i32 a, TCGv_i32 b)
{
    gen_sbc_cc(res, a, b, tcg_constant_i32(0));
}

/*
 * Saturate: on overflow the true result has the sign of a; SAT is set
 * on OV and otherwise kept
 */
static void gen_saturate(TCGv_i32 res, TCGv_i32 a)
{
    TCGv_i32 lim = tcg_temp_new_i32();

    tcg_gen_sari_i32(lim, a, 31);
    tcg_gen_xori_i32(lim, lim, INT32_MAX);
    tcg_gen_movcond_i32(TCG_COND_NE, res, cpu_psw_ov, tcg_constant_i32(0),
                        lim, res);
    tcg_gen_or_i32(cpu_psw_sat, cpu_psw_sat, cpu_psw_ov);
    gen_flags_zs(res);
}

static void gen_satadd(int rd, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_add_cc(res, a, b);
    gen_saturate(res, a);
    set_gpr(rd, res);
}

static void gen_satsub(int rd, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_sub_cc(res, a, b);
    gen_saturate(res, a);
    set_gpr(rd, res);
}

static TCGv_i32 gen_cond_val(int cond)
{
    TCGv_i32 t = tcg_temp_new_i32();

    if (cond == RH850_COND_R) {
        tcg_gen_movi_i32(t, 1);
    } else {
        gen_cond(t, cond);
    }
    return t;
}

static bool trans_ADD_rr(DisasContext *ctx, arg_rr16 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_add_cc(res, get_gpr(a->r2), get_gpr(a->r1));
    set_gpr(a->r2, res);
    return true;
}

static bool trans_ADD_i5(DisasContext *ctx, arg_ri16 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_add_cc(res, get_gpr(a->r2), tcg_constant_i32(a->imm));
    set_gpr(a->r2, res);
    return true;
}

static bool trans_ADDI(DisasContext *ctx, arg_rri *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_add_cc(res, get_gpr(a->r1), tcg_constant_i32(a->imm));
    set_gpr(a->r2, res);
    return true;
}

static bool trans_ADF(DisasContext *ctx, arg_rrrc *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_adc_cc(res, get_gpr(a->r1), get_gpr(a->r2), gen_cond_val(a->cond));
    set_gpr(a->r3, res);
    return true;
}

static bool trans_SUB(DisasContext *ctx, arg_rr16 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_sub_cc(res, get_gpr(a->r2), get_gpr(a->r1));
    set_gpr(a->r2, res);
    return true;
}

static bool trans_SUBR(DisasContext *ctx, arg_rr16 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_sub_cc(res, get_gpr(a->r1), get_gpr(a->r2));
    set_gpr(a->r2, res);
    return true;
}

static bool trans_SBF(DisasContext *ctx, arg_rrrc *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    gen_sbc_cc(res, get_gpr(a->r2), get_gpr(a->r1), gen_cond_val(a->cond));
    set_gpr(a->r3, res);
    return true;
}

static bool gen_logic_rr(arg_rr16 *a, void (*op)(TCGv_i32, TCGv_i32, TCGv_i32),
                         bool store)
{
    TCGv_i32 res = tcg_temp_new_i32();

    op(res, get_gpr(a->r2), get_gpr(a->r1));
    gen_flags_logic(res);
    if (store) {
        set_gpr(a->r2, res);
    }
    return true;
}

static bool gen_logic_ri(arg_rri *a, void (*op)(TCGv_i32, TCGv_i32, int32_t))
{
    TCGv_i32 res = tcg_temp_new_i32();

    op(res, get_gpr(a->r1), a->imm);
    gen_flags_logic(res);
    set_gpr(a->r2, res);
    return true;
}

static bool trans_AND(DisasContext *ctx, arg_rr16 *a)
{
    return gen_logic_rr(a, tcg_gen_and_i32, true);
}

static bool trans_OR(DisasContext *ctx, arg_rr16 *a)
{
    return gen_logic_rr(a, tcg_gen_or_i32, true);
}

static bool trans_XOR(DisasContext *ctx, arg_rr16 *a)
{
    return gen_logic_rr(a, tcg_gen_xor_i32, true);
}

static bool trans_TST(DisasContext *ctx, arg_rr16 *a)
{
    return gen_logic_rr(a, tcg_gen_and_i32, false);
}

/* ANDI: S = 0 since imm16 is zero-extended */
static bool trans_ANDI(DisasContext *ctx, arg_rri *a)
{
    return gen_logic_ri(a, tcg_gen_andi_i32);
}

static bool trans_ORI(DisasContext *ctx, arg_rri *a)
{
    return gen_logic_ri(a, tcg_gen_ori_i32);
}

static bool trans_XORI(DisasContext *ctx, arg_rri *a)
{
    return gen_logic_ri(a, tcg_gen_xori_i32);
}

static bool trans_NOT(DisasContext *ctx, arg_rr16 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    tcg_gen_not_i32(res, get_gpr(a->r1));
    gen_flags_logic(res);
    set_gpr(a->r2, res);
    return true;
}

/* SETF, SASF: flags unchanged */
static bool trans_SETF(DisasContext *ctx, arg_SETF *a)
{
    set_gpr(a->r2, gen_cond_val(a->cond));
    return true;
}

static bool trans_SASF(DisasContext *ctx, arg_SASF *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    tcg_gen_shli_i32(res, get_gpr(a->r2), 1);
    tcg_gen_or_i32(res, res, gen_cond_val(a->cond));
    set_gpr(a->r2, res);
    return true;
}

/* CMOV: flags unchanged */
static void gen_cmov(int cond, int r3, TCGv_i32 t, TCGv_i32 f)
{
    TCGv_i32 res = tcg_temp_new_i32();

    tcg_gen_movcond_i32(TCG_COND_NE, res, gen_cond_val(cond),
                        tcg_constant_i32(0), t, f);
    set_gpr(r3, res);
}

static bool trans_CMOV_r(DisasContext *ctx, arg_rrrc *a)
{
    gen_cmov(a->cond, a->r3, get_gpr(a->r1), get_gpr(a->r2));
    return true;
}

static bool trans_CMOV_i(DisasContext *ctx, arg_CMOV_i *a)
{
    gen_cmov(a->cond, a->r3, tcg_constant_i32(a->imm), get_gpr(a->r2));
    return true;
}

/* Shifts: count 0-31, OV = 0, CY = last bit shifted out or 0 */
enum { SH_SHL, SH_SHR, SH_SAR };

static void gen_shift(int kind, int rd, TCGv_i32 src, TCGv_i32 cnt)
{
    TCGv_i64 v = tcg_temp_new_i64();
    TCGv_i64 n = tcg_temp_new_i64();
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 hi = tcg_temp_new_i32();

    tcg_gen_extu_i32_i64(n, cnt);
    tcg_gen_andi_i64(n, n, 31);
    switch (kind) {
    case SH_SHL:
        /* bit 32 of the 64-bit shift is the last bit shifted out */
        tcg_gen_extu_i32_i64(v, src);
        tcg_gen_shl_i64(v, v, n);
        tcg_gen_extrh_i64_i32(cpu_psw_cy, v);
        tcg_gen_andi_i32(cpu_psw_cy, cpu_psw_cy, 1);
        tcg_gen_extrl_i64_i32(res, v);
        break;
    case SH_SHR:
    case SH_SAR:
        /* shift (src << 1) right: bit 0 is the last bit shifted out */
        if (kind == SH_SHR) {
            tcg_gen_extu_i32_i64(v, src);
        } else {
            tcg_gen_ext_i32_i64(v, src);
        }
        tcg_gen_shli_i64(v, v, 1);
        if (kind == SH_SHR) {
            tcg_gen_shr_i64(v, v, n);
        } else {
            tcg_gen_sar_i64(v, v, n);
        }
        tcg_gen_extrl_i64_i32(res, v);
        tcg_gen_andi_i32(cpu_psw_cy, res, 1);
        tcg_gen_extrh_i64_i32(hi, v);
        tcg_gen_extract2_i32(res, res, hi, 1);
        break;
    default:
        g_assert_not_reached();
    }
    tcg_gen_movi_i32(cpu_psw_ov, 0);
    gen_flags_zs(res);
    set_gpr(rd, res);
}

static bool trans_SHL_rr(DisasContext *ctx, arg_rr *a)
{
    gen_shift(SH_SHL, a->r2, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SHL_i5(DisasContext *ctx, arg_ri16 *a)
{
    gen_shift(SH_SHL, a->r2, get_gpr(a->r2), tcg_constant_i32(a->imm));
    return true;
}

static bool trans_SHL_r3(DisasContext *ctx, arg_rrr *a)
{
    gen_shift(SH_SHL, a->r3, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SHR_rr(DisasContext *ctx, arg_rr *a)
{
    gen_shift(SH_SHR, a->r2, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SHR_i5(DisasContext *ctx, arg_ri16 *a)
{
    gen_shift(SH_SHR, a->r2, get_gpr(a->r2), tcg_constant_i32(a->imm));
    return true;
}

static bool trans_SHR_r3(DisasContext *ctx, arg_rrr *a)
{
    gen_shift(SH_SHR, a->r3, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SAR_rr(DisasContext *ctx, arg_rr *a)
{
    gen_shift(SH_SAR, a->r2, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SAR_i5(DisasContext *ctx, arg_ri16 *a)
{
    gen_shift(SH_SAR, a->r2, get_gpr(a->r2), tcg_constant_i32(a->imm));
    return true;
}

static bool trans_SAR_r3(DisasContext *ctx, arg_rrr *a)
{
    gen_shift(SH_SAR, a->r3, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

/* ROTL: CY = result bit 0, also for a rotate amount of 0 */
static void gen_rotl(int rd, TCGv_i32 src, TCGv_i32 cnt)
{
    TCGv_i32 n = tcg_temp_new_i32();
    TCGv_i32 res = tcg_temp_new_i32();

    tcg_gen_andi_i32(n, cnt, 31);
    tcg_gen_rotl_i32(res, src, n);
    tcg_gen_andi_i32(cpu_psw_cy, res, 1);
    tcg_gen_movi_i32(cpu_psw_ov, 0);
    gen_flags_zs(res);
    set_gpr(rd, res);
}

static bool trans_ROTL_i(DisasContext *ctx, arg_ROTL_i *a)
{
    gen_rotl(a->r3, get_gpr(a->r2), tcg_constant_i32(a->imm));
    return true;
}

static bool trans_ROTL_r(DisasContext *ctx, arg_rrr *a)
{
    gen_rotl(a->r3, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

/* CY = 1 if any (mask-wide) lane of v is zero, lanes given by mask */
static void gen_any_zero(TCGv_i32 cy, TCGv_i32 v, uint32_t m0, uint32_t m1)
{
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_andi_i32(t, v, m0);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cy, t, 0);
    tcg_gen_andi_i32(t, v, m1);
    tcg_gen_setcondi_i32(TCG_COND_EQ, t, t, 0);
    tcg_gen_or_i32(cy, cy, t);
}

/* BSH: CY = zero byte in the low halfword, Z = low halfword zero */
static bool trans_BSH(DisasContext *ctx, arg_r23 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_bswap32_i32(res, get_gpr(a->r2));
    tcg_gen_rotli_i32(res, res, 16);
    gen_any_zero(cpu_psw_cy, res, 0xff, 0xff00);
    tcg_gen_movi_i32(cpu_psw_ov, 0);
    tcg_gen_shri_i32(cpu_psw_s, res, 31);
    tcg_gen_andi_i32(t, res, 0xffff);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_z, t, 0);
    set_gpr(a->r3, res);
    return true;
}

/* BSW: CY = zero byte anywhere in the result */
static bool trans_BSW(DisasContext *ctx, arg_r23 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_bswap32_i32(res, get_gpr(a->r2));
    gen_any_zero(cpu_psw_cy, res, 0xff, 0xff00);
    gen_any_zero(t, res, 0xff0000, 0xff000000);
    tcg_gen_or_i32(cpu_psw_cy, cpu_psw_cy, t);
    gen_flags_logic(res);
    set_gpr(a->r3, res);
    return true;
}

/* HSH: result = reg2; CY and Z = low halfword zero */
static bool trans_HSH(DisasContext *ctx, arg_r23 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_mov_i32(res, get_gpr(a->r2));
    tcg_gen_andi_i32(t, res, 0xffff);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_z, t, 0);
    tcg_gen_mov_i32(cpu_psw_cy, cpu_psw_z);
    tcg_gen_movi_i32(cpu_psw_ov, 0);
    tcg_gen_shri_i32(cpu_psw_s, res, 31);
    set_gpr(a->r3, res);
    return true;
}

/* HSW: CY = zero halfword in the result */
static bool trans_HSW(DisasContext *ctx, arg_r23 *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    tcg_gen_rotli_i32(res, get_gpr(a->r2), 16);
    gen_any_zero(cpu_psw_cy, res, 0xffff, 0xffff0000);
    gen_flags_logic(res);
    set_gpr(a->r3, res);
    return true;
}

/* SXB, SXH, ZXB, ZXH: flags unchanged */
static bool gen_extend(int r, void (*op)(TCGv_i32, TCGv_i32))
{
    TCGv_i32 res = tcg_temp_new_i32();

    op(res, get_gpr(r));
    set_gpr(r, res);
    return true;
}

static bool trans_SXB(DisasContext *ctx, arg_r1_16 *a)
{
    return gen_extend(a->r1, tcg_gen_ext8s_i32);
}

static bool trans_SXH(DisasContext *ctx, arg_r1_16 *a)
{
    return gen_extend(a->r1, tcg_gen_ext16s_i32);
}

static bool trans_ZXB(DisasContext *ctx, arg_r1_16 *a)
{
    return gen_extend(a->r1, tcg_gen_ext8u_i32);
}

static bool trans_ZXH(DisasContext *ctx, arg_r1_16 *a)
{
    return gen_extend(a->r1, tcg_gen_ext16u_i32);
}

/*
 * SCH0L/R, SCH1L/R: position of the first matching bit from 1, or 0
 * with Z = 1; CY = 1 if it is the last bit searched
 */
static void gen_search(int r3, TCGv_i32 src, bool ones, bool left)
{
    TCGv_i32 v = tcg_temp_new_i32();
    TCGv_i32 n = tcg_temp_new_i32();
    TCGv_i32 res = tcg_temp_new_i32();

    if (ones) {
        tcg_gen_mov_i32(v, src);
    } else {
        tcg_gen_not_i32(v, src);
    }
    if (left) {
        tcg_gen_clzi_i32(n, v, 32);
    } else {
        tcg_gen_ctzi_i32(n, v, 32);
    }
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_z, v, 0);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_psw_cy, n, 31);
    tcg_gen_addi_i32(res, n, 1);
    tcg_gen_movcond_i32(TCG_COND_NE, res, cpu_psw_z, tcg_constant_i32(0),
                        tcg_constant_i32(0), res);
    tcg_gen_movi_i32(cpu_psw_ov, 0);
    tcg_gen_movi_i32(cpu_psw_s, 0);
    set_gpr(r3, res);
}

static bool trans_SCH0L(DisasContext *ctx, arg_r23 *a)
{
    gen_search(a->r3, get_gpr(a->r2), false, true);
    return true;
}

static bool trans_SCH0R(DisasContext *ctx, arg_r23 *a)
{
    gen_search(a->r3, get_gpr(a->r2), false, false);
    return true;
}

static bool trans_SCH1L(DisasContext *ctx, arg_r23 *a)
{
    gen_search(a->r3, get_gpr(a->r2), true, true);
    return true;
}

static bool trans_SCH1R(DisasContext *ctx, arg_r23 *a)
{
    gen_search(a->r3, get_gpr(a->r2), true, false);
    return true;
}

/*
 * BINS with msb < lsb is undefined: reg2 is kept, the flags are set
 * from it and a guest error is logged
 */
static bool trans_BINS(DisasContext *ctx, arg_bins *a)
{
    TCGv_i32 res = tcg_temp_new_i32();

    if (a->msb < a->lsb) {
        gen_log_undefined(ctx, "BINS with msb < lsb");
        tcg_gen_mov_i32(res, get_gpr(a->r2));
    } else {
        tcg_gen_deposit_i32(res, get_gpr(a->r2), get_gpr(a->r1), a->lsb,
                            a->msb - a->lsb + 1);
    }
    gen_flags_logic(res);
    set_gpr(a->r2, res);
    return true;
}

/* CLIP: CY = 0, OV = saturation, SAT set on OV and otherwise kept */
static void gen_clip(arg_rr *a, bool sign, int32_t lo, int32_t hi)
{
    TCGv_i32 src = get_gpr(a->r1);
    TCGv_i32 res = tcg_temp_new_i32();

    if (sign) {
        tcg_gen_smax_i32(res, src, tcg_constant_i32(lo));
        tcg_gen_smin_i32(res, res, tcg_constant_i32(hi));
    } else {
        tcg_gen_umin_i32(res, src, tcg_constant_i32(hi));
    }
    tcg_gen_setcond_i32(TCG_COND_NE, cpu_psw_ov, res, src);
    tcg_gen_or_i32(cpu_psw_sat, cpu_psw_sat, cpu_psw_ov);
    tcg_gen_movi_i32(cpu_psw_cy, 0);
    gen_flags_zs(res);
    set_gpr(a->r2, res);
}

static bool trans_CLIP_B(DisasContext *ctx, arg_rr *a)
{
    gen_clip(a, true, INT8_MIN, INT8_MAX);
    return true;
}

static bool trans_CLIP_BU(DisasContext *ctx, arg_rr *a)
{
    gen_clip(a, false, 0, UINT8_MAX);
    return true;
}

static bool trans_CLIP_H(DisasContext *ctx, arg_rr *a)
{
    gen_clip(a, true, INT16_MIN, INT16_MAX);
    return true;
}

static bool trans_CLIP_HU(DisasContext *ctx, arg_rr *a)
{
    gen_clip(a, false, 0, UINT16_MAX);
    return true;
}

static bool trans_SATADD_rr(DisasContext *ctx, arg_rr16 *a)
{
    gen_satadd(a->r2, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SATADD_i5(DisasContext *ctx, arg_ri16 *a)
{
    gen_satadd(a->r2, get_gpr(a->r2), tcg_constant_i32(a->imm));
    return true;
}

static bool trans_SATADD_r3(DisasContext *ctx, arg_rrr *a)
{
    gen_satadd(a->r3, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SATSUB_rr(DisasContext *ctx, arg_rr16 *a)
{
    gen_satsub(a->r2, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SATSUB_r3(DisasContext *ctx, arg_rrr *a)
{
    gen_satsub(a->r3, get_gpr(a->r2), get_gpr(a->r1));
    return true;
}

static bool trans_SATSUBI(DisasContext *ctx, arg_rri *a)
{
    gen_satsub(a->r2, get_gpr(a->r1), tcg_constant_i32(a->imm));
    return true;
}

static bool trans_SATSUBR(DisasContext *ctx, arg_rr16 *a)
{
    gen_satsub(a->r2, get_gpr(a->r1), get_gpr(a->r2));
    return true;
}
