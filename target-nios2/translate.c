/*
 * Altera Nios II emulation for qemu: main translation routines.
 *
 * Copyright (C) 2012 Chris Wulff <crwulff@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>

#include "cpu.h"
#include "exec/exec-all.h"
#include "disas/disas.h"
#include "exec/helper-proto.h"
#include "qemu-common.h"

#include "instruction.h"

#include "exec/cpu_ldst.h"
#include "exec/helper-gen.h"
#include "exec/log.h"

static const char *regnames[] = {
    "zero",     "at",       "r2",       "r3",
    "r4",       "r5",       "r6",       "r7",
    "r8",       "r9",       "r10",      "r11",
    "r12",      "r13",      "r14",      "r15",
    "r16",      "r17",      "r18",      "r19",
    "r20",      "r21",      "r22",      "r23",
    "et",       "bt",       "gp",       "sp",
    "fp",       "ea",       "ba",       "ra",
    "status",   "estatus",  "bstatus",  "ienable",
    "ipending", "cpuid",    "reserved", "exception",
    "pteaddr",  "tlbacc",   "tlbmisc",  "reserved",
    "badaddr",  "config",   "mpubase",  "mpuacc",
    "reserved", "reserved", "reserved", "reserved",
    "reserved", "reserved", "reserved", "reserved",
    "reserved", "reserved", "reserved", "reserved",
    "reserved", "reserved", "reserved", "reserved",
    "rpc"
};

static TCGv_ptr cpu_env;
static TCGv cpu_R[NUM_CORE_REGS];

#include "exec/gen-icount.h"

static void gen_exception(DisasContext *dc, uint32_t excp)
{
    TCGv_i32 tmp = tcg_const_i32(excp);

    tcg_gen_movi_tl(cpu_R[R_PC], dc->pc);
    gen_helper_raise_exception(cpu_env, tmp);
    tcg_temp_free_i32(tmp);
    dc->is_jmp = DISAS_UPDATE;
}

/* generate intermediate code for basic block 'tb'.  */
void gen_intermediate_code(CPUNios2State *env, TranslationBlock *tb)
{
    Nios2CPU *cpu = nios2_env_get_cpu(env);
    CPUState *cs = CPU(cpu);
    DisasContext dc1, *dc = &dc1;
    int num_insns;
    int max_insns;
    uint32_t next_page_start;

    /* Initialize DC */
    dc->cpu_env = cpu_env;
    dc->cpu_R   = cpu_R;
    dc->is_jmp  = DISAS_NEXT;
    dc->pc      = tb->pc;
    dc->tb      = tb;
    dc->mem_idx = cpu_mmu_index(env, false);

#ifndef CONFIG_USER_ONLY
    if ((env->regs[CR_STATUS] & CR_STATUS_U) == CR_STATUS_U) {
        dc->user = true;
    } else {
        dc->user = false;
    }
#endif
    /* Dump the CPU state to the log */
    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("--------------\n");
        log_cpu_state(cs, 0);
    }

    /* Set up instruction counts */
    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0) {
        max_insns = CF_COUNT_MASK;
    }
    if (max_insns > TCG_MAX_INSNS) {
        max_insns = TCG_MAX_INSNS;
    }
    next_page_start = (tb->pc & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;

    gen_tb_start(tb);
    do {
        LOG_DIS("%8.8x:\t", dc->pc);

        tcg_gen_insn_start(dc->pc);
        num_insns++;

#if SIM_COMPAT
        if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
            tcg_gen_movi_tl(cpu_SR[SR_PC], dc->pc);
            gen_helper_debug();
        }
#endif

        if (unlikely(cpu_breakpoint_test(cs, dc->pc, BP_ANY))) {
            gen_exception(dc, EXCP_DEBUG);
            /* The address covered by the breakpoint must be included in
               [tb->pc, tb->pc + tb->size) in order to for it to be
               properly cleared -- thus we increment the PC here so that
               the logic setting tb->size below does the right thing.  */
            dc->pc += 4;
            break;
        }

        if (num_insns + 1 == max_insns && (tb->cflags & CF_LAST_IO)) {
            gen_io_start();
        }

        /* Decode an instruction */
        handle_instruction(dc, env);

        dc->pc += 4;

        /* Translation stops when a conditional branch is encountered.
         * Otherwise the subsequent code could get translated several times.
         * Also stop translation when a page boundary is reached.  This
         * ensures prefetch aborts occur at the right place.  */
    } while (!dc->is_jmp &&
             !tcg_op_buf_full() &&
             !cs->singlestep_enabled &&
             !singlestep &&
             dc->pc < next_page_start &&
             num_insns < max_insns);

    if (tb->cflags & CF_LAST_IO) {
        gen_io_end();
    }

    /* Indicate where the next block should start */
    switch (dc->is_jmp) {
    case DISAS_NEXT:
        /* Save the current PC back into the CPU register */
        tcg_gen_movi_tl(cpu_R[R_PC], dc->pc);
        tcg_gen_exit_tb(0);
        break;

    default:
    case DISAS_JUMP:
    case DISAS_UPDATE:
        /* The jump will already have updated the PC register */
        tcg_gen_exit_tb(0);
        break;

    case DISAS_TB_JUMP:
        /* nothing more to generate */
        break;
    }

    /* End off the block */
    gen_tb_end(tb, num_insns);

    /* Mark instruction starts for the final generated instruction */
    tb->size = dc->pc - tb->pc;
    tb->icount = num_insns;

#ifdef DEBUG_DISAS
    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("----------------\n");
        qemu_log("IN: %s\n", lookup_symbol(tb->pc));
        log_target_disas(cs, tb->pc, dc->pc - tb->pc, 0);
        qemu_log("\nisize=%d osize=%d\n",
                 dc->pc - tb->pc, tcg_op_buf_count());
    }
#endif
}

void nios2_cpu_dump_state(CPUState *cs, FILE *f, fprintf_function cpu_fprintf,
                          int flags)
{
    Nios2CPU *cpu = NIOS2_CPU(cs);
    CPUNios2State *env = &cpu->env;
    int i;

    if (!env || !f) {
        return;
    }

    cpu_fprintf(f, "IN: PC=%x %s\n",
                env->regs[R_PC], lookup_symbol(env->regs[R_PC]));

    for (i = 0; i < NUM_CORE_REGS; i++) {
        cpu_fprintf(f, "%9s=%8.8x ", regnames[i], env->regs[i]);
        if ((i + 1) % 4 == 0) {
            cpu_fprintf(f, "\n");
        }
    }
#if !defined(CONFIG_USER_ONLY)
    cpu_fprintf(f, " mmu write: VPN=%05X PID %02X TLBACC %08X\n",
                env->mmu.pteaddr_wr & CR_PTEADDR_VPN_MASK,
                (env->mmu.tlbmisc_wr & CR_TLBMISC_PID_MASK) >> 4,
                env->mmu.tlbacc_wr);
#endif
    cpu_fprintf(f, "\n\n");
}

void nios2_tcg_init(void)
{
    int i;

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");

    for (i = 0; i < NUM_CORE_REGS; i++) {
        cpu_R[i] = tcg_global_mem_new(cpu_env,
                                      offsetof(CPUNios2State, regs[i]),
                                      regnames[i]);
    }
}

void restore_state_to_opc(CPUNios2State *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->regs[R_PC] = data[0];
}
