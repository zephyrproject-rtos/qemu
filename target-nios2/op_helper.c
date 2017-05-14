/*
 * Altera Nios II helper routines.
 *
 * Copyright (C) 2012 Chris Wulff <crwulff@gmail.com>
 * Copyright (C) 2016 Intel Corporation.
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

#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/cpu_ldst.h"
#include "hw/nios2/nios2_iic.h"

#if !defined(CONFIG_USER_ONLY)
uint32_t helper_mmu_read(CPUNios2State *env, uint32_t rn)
{
    return mmu_read(env, rn);
}

void helper_mmu_write(CPUNios2State *env, uint32_t rn, uint32_t v)
{
    mmu_write(env, rn, v);
}

void helper_cr_ienable_write(CPUNios2State *env, uint32_t value)
{
    env->regs[CR_IENABLE] = value;
    nios2_iic_update_cr_ienable(env->pic_state);
}

void helper_cr_status_write(CPUNios2State *env, uint32_t value)
{
    env->regs[CR_STATUS] = value;
    nios2_iic_update_cr_status(env->pic_state);
}
#endif /* !CONFIG_USER_ONLY */

void helper_raise_exception(CPUNios2State *env, uint32_t index)
{
    CPUState *cs = ENV_GET_CPU(env);
    cs->exception_index = index;
    cpu_loop_exit(cs);
}

void helper_memalign(CPUNios2State *env, uint32_t addr, uint32_t dr, uint32_t wr, uint32_t mask)
{
    if (addr & mask) {
        qemu_log("unaligned access addr=%x mask=%x, wr=%d dr=r%d\n",
                 addr, mask, wr, dr);
        env->regs[CR_BADADDR] = addr;
        env->regs[CR_EXCEPTION] = EXCP_UNALIGN << 2;
        helper_raise_exception(env, EXCP_UNALIGN);
    }
}

uint32_t helper_divs(uint32_t a, uint32_t b)
{
    return (int32_t)a / (int32_t)b;
}

uint32_t helper_divu(uint32_t a, uint32_t b)
{
    return a / b;
}

#ifdef CALL_TRACING
void helper_call_status(uint32_t pc, uint32_t target)
{
    qemu_log("%08X: CALL %08X %s\n", pc, target, lookup_symbol(target));
}

void helper_eret_status(uint32_t pc)
{
    qemu_log("%08X: ERET STATUS %08X, ESTATUS %08X, EA %08X\n",
             pc, env->regs[CR_STATUS], env->regs[CR_ESTATUS], env->regs[R_EA]);
}

void helper_ret_status(uint32_t pc)
{
    qemu_log("%08X: RET RA %08X\n", pc, env->regs[R_RA]);
}
#endif
