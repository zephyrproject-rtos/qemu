/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 branch, jump, loop, table-jump and stack-frame instructions.
 * PC-relative targets use the address of the branch; PC bit 0 is fixed 0.
 */

#define RH850_SP    3

/* Bcond disp17 (cond BR is reserved at decode) */
static bool trans_Bcond17(DisasContext *ctx, arg_Bcond17 *a)
{
    gen_bcond(ctx, a->cond, a->disp);
    return true;
}

static bool trans_JR32(DisasContext *ctx, arg_JR32 *a)
{
    gen_goto_tb(ctx, 0, ctx->pc + a->disp);
    return true;
}

/* JARL disp22 / disp32: link = PC + 4 / PC + 6 */
static bool trans_JARL22(DisasContext *ctx, arg_JARL22 *a)
{
    set_gpr_i(a->r2, ctx->pc + 4);
    gen_goto_tb(ctx, 0, ctx->pc + a->disp);
    return true;
}

static bool trans_JARL32(DisasContext *ctx, arg_JARL32 *a)
{
    set_gpr_i(a->r1, ctx->pc + 6);
    gen_goto_tb(ctx, 0, ctx->pc + a->disp);
    return true;
}

/* JARL [reg1]: target read before the link is stored */
static bool trans_JARL_r(DisasContext *ctx, arg_JARL_r *a)
{
    tcg_gen_andi_i32(cpu_pc, get_gpr(a->r1), ~1u);
    set_gpr_i(a->r3, ctx->pc + 4);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static bool trans_JMP32(DisasContext *ctx, arg_JMP32 *a)
{
    tcg_gen_addi_i32(cpu_pc, get_gpr(a->r1), a->disp);
    tcg_gen_andi_i32(cpu_pc, cpu_pc, ~1u);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

/* LOOP: reg1 += -1 with the flags of ADD, branch back while reg1 != 0 */
static bool trans_LOOP(DisasContext *ctx, arg_LOOP *a)
{
    TCGv_i32 res = tcg_temp_new_i32();
    TCGLabel *done = gen_new_label();

    gen_add_cc(res, get_gpr(a->r1), tcg_constant_i32(-1));
    set_gpr(a->r1, res);
    tcg_gen_brcondi_i32(TCG_COND_EQ, res, 0, done);
    gen_goto_tb(ctx, 1, ctx->pc - (uint32_t)a->disp);
    gen_set_label(done);
    gen_goto_tb(ctx, 0, ctx->base.pc_next);
    return true;
}

/*
 * SWITCH: target = PC + 2 + (sign-extended halfword at
 * PC + 2 + (reg1 << 1)) << 1
 */
static bool trans_SWITCH(DisasContext *ctx, arg_SWITCH *a)
{
    TCGv_i32 adr = tcg_temp_new_i32();
    TCGv_i32 off = tcg_temp_new_i32();

    tcg_gen_shli_i32(adr, get_gpr(a->r1), 1);
    tcg_gen_addi_i32(adr, adr, ctx->pc + 2);
    tcg_gen_qemu_ld_i32(off, adr, 0, MO_LESW);
    tcg_gen_shli_i32(off, off, 1);
    tcg_gen_addi_i32(cpu_pc, off, ctx->pc + 2);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

/*
 * list12 bits 31-21: r24 r25 r26 r27 r20 r21 r22 r23 r28 r29 r31;
 * bit 0: r30
 */
static const uint8_t list12_bit[12] = {
    27, 26, 25, 24, 31, 30, 29, 28, 23, 22, 0, 21,
};

static bool list12_has(uint32_t list, int reg)
{
    return list & (1u << list12_bit[reg - 20]);
}

/* Word access at tmp with the lower 2 bits masked */
static TCGv_i32 gen_stack_addr(TCGv_i32 tmp)
{
    TCGv_i32 adr = tcg_temp_new_i32();

    tcg_gen_andi_i32(adr, tmp, ~3u);
    return adr;
}

/*
 * PREPARE: store r20..r31 ascending with pre-decrement, then
 * sp = tmp - (imm5 << 2)
 */
static void gen_prepare(uint32_t list, int imm)
{
    TCGv_i32 tmp = tcg_temp_new_i32();
    TCGv_i32 adr;
    int r;

    tcg_gen_mov_i32(tmp, get_gpr(RH850_SP));
    for (r = 20; r <= 31; r++) {
        if (list12_has(list, r)) {
            tcg_gen_subi_i32(tmp, tmp, 4);
            adr = gen_stack_addr(tmp);
            tcg_gen_qemu_st_i32(get_gpr(r), adr, 0, MO_LEUL);
            gen_link_kill(adr, 4);
        }
    }
    tcg_gen_subi_i32(tmp, tmp, imm << 2);
    set_gpr(RH850_SP, tmp);
}

static bool trans_PREPARE(DisasContext *ctx, arg_PREPARE *a)
{
    gen_prepare(a->list, a->imm);
    return true;
}

static bool trans_PREPARE2(DisasContext *ctx, arg_PREPARE2 *a)
{
    gen_prepare(a->list, a->imm);
    if (a->ff == 0) {
        set_gpr(RH850_EP, get_gpr(RH850_SP));
    } else {
        set_gpr_i(RH850_EP, a->ep);
    }
    return true;
}

/* DISPOSE: load r31..r20 descending with post-increment, then sp = tmp */
static void gen_dispose(DisasContext *ctx, uint32_t list, int imm, int r1)
{
    TCGv_i32 tmp = tcg_temp_new_i32();
    TCGv_i32 val = tcg_temp_new_i32();
    int r;

    tcg_gen_addi_i32(tmp, get_gpr(RH850_SP), imm << 2);
    for (r = 31; r >= 20; r--) {
        if (list12_has(list, r)) {
            tcg_gen_qemu_ld_i32(val, gen_stack_addr(tmp), 0, MO_LEUL);
            set_gpr(r, val);
            tcg_gen_addi_i32(tmp, tmp, 4);
        }
    }
    if (r1) {
        tcg_gen_andi_i32(cpu_pc, get_gpr(r1), ~1u);
        ctx->base.is_jmp = DISAS_JUMP;
    }
    set_gpr(RH850_SP, tmp);
}

static bool trans_DISPOSE(DisasContext *ctx, arg_DISPOSE *a)
{
    gen_dispose(ctx, a->list, a->imm, 0);
    return true;
}

static bool trans_DISPOSE_r(DisasContext *ctx, arg_DISPOSE_r *a)
{
    gen_dispose(ctx, a->list, a->imm, a->r1);
    return true;
}

/* PUSHSP/POPSP: nothing is done if rh > rt; POPSP does not load sp */
static bool trans_PUSHSP(DisasContext *ctx, arg_PUSHSP *a)
{
    TCGv_i32 tmp = tcg_temp_new_i32();
    TCGv_i32 adr;
    int r;

    if (a->rh > a->rt) {
        return true;
    }
    tcg_gen_mov_i32(tmp, get_gpr(RH850_SP));
    for (r = a->rh; r <= a->rt; r++) {
        tcg_gen_subi_i32(tmp, tmp, 4);
        adr = gen_stack_addr(tmp);
        tcg_gen_qemu_st_i32(get_gpr(r), adr, 0, MO_LEUL);
        gen_link_kill(adr, 4);
    }
    set_gpr(RH850_SP, tmp);
    return true;
}

static bool trans_POPSP(DisasContext *ctx, arg_POPSP *a)
{
    TCGv_i32 tmp = tcg_temp_new_i32();
    TCGv_i32 val = tcg_temp_new_i32();
    int r;

    if (a->rh > a->rt) {
        return true;
    }
    tcg_gen_mov_i32(tmp, get_gpr(RH850_SP));
    for (r = a->rt; r >= a->rh; r--) {
        tcg_gen_qemu_ld_i32(val, gen_stack_addr(tmp), 0, MO_LEUL);
        if (r != RH850_SP) {
            set_gpr(r, val);
        }
        tcg_gen_addi_i32(tmp, tmp, 4);
    }
    set_gpr(RH850_SP, tmp);
    return true;
}
