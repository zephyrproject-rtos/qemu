/*
 * QEMU ARC CPU
 *
 * Copyright (c) 2020 Synppsys Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * http://www.gnu.org/licenses/lgpl-2.1.html
 */

#include "qemu/osdep.h"
#include "exec/gdbstub.h"
#include "arc-common.h"
#include "target/arc/regs.h"
#include "irq.h"
#include "gdbstub.h"
#include "exec/helper-proto.h"

/* gets the register address for a particular processor */
#define REG_ADDR(reg, processor_type) \
    arc_aux_reg_address_for((reg), (processor_type))

#ifdef TARGET_ARCV2
#define GDB_GET_REG gdb_get_reg32
#elif defined(TARGET_ARCV3)
#define GDB_GET_REG gdb_get_reg64
#else
    #error No target is selected.
#endif

int arc_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    ARCCPU *cpu = ARC_CPU(cs);
    CPUARCState *env = &cpu->env;
    target_ulong regval = 0;

    switch (n) {
    case 0 ... 31:
       regval = env->r[n];
       break;
    case GDB_REG_58:
       regval = env->r[58];
       break;
    case GDB_REG_59:
       regval = env->r[59];
       break;
    case GDB_REG_60:
       regval = env->r[60];
       break;
    case GDB_REG_63:
       regval = env->r[63];
       break;
    default:
       assert(!"Unsupported register is being read.");
    }

    return GDB_GET_REG(mem_buf, regval);
}

int arc_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    ARCCPU *cpu = ARC_CPU(cs);
    CPUARCState *env = &cpu->env;
    target_ulong regval = ldl_p(mem_buf);

    switch (n) {
    case 0 ... 31:
        env->r[n] = regval;
        break;
    case GDB_REG_58:
        env->r[58] = regval;
        break;
    case GDB_REG_59:
        env->r[59] = regval;
        break;
    case GDB_REG_60:
        env->r[60] = regval;
        break;
    case GDB_REG_63:
        env->r[63] = regval;
        break;
    default:
        assert(!"Unsupported register is being written.");
    }

    return 4;
}


static int
arc_aux_minimal_gdb_get_reg(CPUARCState *env, GByteArray *mem_buf, int regnum)
{
    ARCCPU *cpu = env_archcpu(env);
    target_ulong regval = 0;

    switch (regnum) {
    case GDB_AUX_MIN_REG_PC:
        regval = env->pc & ((target_ulong) 0xfffffffffffffffe);
        break;
    case GDB_AUX_MIN_REG_LPS:
        regval = helper_lr(env, REG_ADDR(AUX_ID_lp_start, cpu->family));
        break;
    case GDB_AUX_MIN_REG_LPE:
        regval = helper_lr(env, REG_ADDR(AUX_ID_lp_end, cpu->family));
        break;
    case GDB_AUX_MIN_REG_STATUS:
        regval = pack_status32(&env->stat);
        break;
    default:
        assert(!"Unsupported minimal auxiliary register is being read.");
    }
    return GDB_GET_REG(mem_buf, regval);
}


static int
arc_aux_minimal_gdb_set_reg(CPUARCState *env, uint8_t *mem_buf, int regnum)
{
    ARCCPU *cpu = env_archcpu(env);
    target_ulong regval = ldl_p(mem_buf);
    switch (regnum) {
    case GDB_AUX_MIN_REG_PC:
        env->pc = regval & ((target_ulong) 0xfffffffffffffffe);
        break;
    case GDB_AUX_MIN_REG_LPS:
        helper_sr(env, regval, REG_ADDR(AUX_ID_lp_start, cpu->family));
        break;
    case GDB_AUX_MIN_REG_LPE:
        helper_sr(env, regval, REG_ADDR(AUX_ID_lp_end, cpu->family));
        break;
    case GDB_AUX_MIN_REG_STATUS:
        unpack_status32(&env->stat, regval);
        break;
    default:
        assert(!"Unsupported minimal auxiliary register is being written.");
    }
    return 4;
}


static int
arc_aux_other_gdb_get_reg(CPUARCState *env, GByteArray *mem_buf, int regnum)
{
    ARCCPU *cpu = env_archcpu(env);
    target_ulong regval = 0;
    switch (regnum) {
    case GDB_AUX_OTHER_REG_TIMER_BUILD:
        regval = helper_lr(env, REG_ADDR(AUX_ID_timer_build, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_BUILD:
        regval = helper_lr(env, REG_ADDR(AUX_ID_irq_build, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_MPY_BUILD:
        regval = helper_lr(env, REG_ADDR(AUX_ID_mpy_build, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_VECBASE_BUILD:
        regval = cpu->vecbase_build;
        break;
    case GDB_AUX_OTHER_REG_ISA_CONFIG:
        regval = cpu->isa_config;
        break;
    case GDB_AUX_OTHER_REG_TIMER_CNT0:
        regval = helper_lr(env, REG_ADDR(AUX_ID_count0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_CTRL0:
        regval = helper_lr(env, REG_ADDR(AUX_ID_control0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_LIM0:
        regval = helper_lr(env, REG_ADDR(AUX_ID_limit0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_CNT1:
        regval = helper_lr(env, REG_ADDR(AUX_ID_count1, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_CTRL1:
        regval = helper_lr(env, REG_ADDR(AUX_ID_control1, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_LIM1:
        regval = helper_lr(env, REG_ADDR(AUX_ID_limit1, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_PID:
        regval = helper_lr(env, REG_ADDR(AUX_ID_pid, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TLBPD0:
        regval = helper_lr(env, REG_ADDR(AUX_ID_tlbpd0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TLBPD1:
        regval = helper_lr(env, REG_ADDR(AUX_ID_tlbpd1, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TLB_INDEX:
        regval = helper_lr(env, REG_ADDR(AUX_ID_tlbindex, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TLB_CMD:
        regval = helper_lr(env, REG_ADDR(AUX_ID_tlbcommand, cpu->family));
        break;
    /* MPU */
    case GDB_AUX_OTHER_REG_MPU_BUILD:
        regval = helper_lr(env, REG_ADDR(AUX_ID_mpu_build, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_MPU_EN:
        regval = helper_lr(env, REG_ADDR(AUX_ID_mpuen, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_MPU_ECR:
        regval = helper_lr(env, REG_ADDR(AUX_ID_mpuic, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_MPU_BASE0 ... GDB_AUX_OTHER_REG_MPU_BASE15: {
        const uint8_t index = regnum - GDB_AUX_OTHER_REG_MPU_BASE0;
        regval = helper_lr(env, REG_ADDR(AUX_ID_mpurdb0 + index, cpu->family));
        break;
    }
    case GDB_AUX_OTHER_REG_MPU_PERM0 ... GDB_AUX_OTHER_REG_MPU_PERM15: {
        const uint8_t index = regnum - GDB_AUX_OTHER_REG_MPU_PERM0;
        regval = helper_lr(env, REG_ADDR(AUX_ID_mpurdp0 + index, cpu->family));
        break;
    }
    /* exceptions */
    case GDB_AUX_OTHER_REG_ERSTATUS:
        regval = helper_lr(env, REG_ADDR(AUX_ID_erstatus, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_ERBTA:
        regval = helper_lr(env, REG_ADDR(AUX_ID_erbta, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_ECR:
        regval = helper_lr(env, REG_ADDR(AUX_ID_ecr, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_ERET:
        regval = helper_lr(env, REG_ADDR(AUX_ID_eret, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_EFA:
        regval = helper_lr(env, REG_ADDR(AUX_ID_efa, cpu->family));
        break;
    /* interrupt */
    case GDB_AUX_OTHER_REG_ICAUSE:
        regval = helper_lr(env, REG_ADDR(AUX_ID_icause, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_CTRL:
        regval = helper_lr(env, REG_ADDR(AUX_ID_aux_irq_ctrl, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_ACT:
        regval = helper_lr(env, REG_ADDR(AUX_ID_aux_irq_act, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_PRIO_PEND:
        regval = env->irq_priority_pending;
        break;
    case GDB_AUX_OTHER_REG_IRQ_HINT:
        regval = helper_lr(env, REG_ADDR(AUX_ID_aux_irq_hint, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_SELECT:
        regval = helper_lr(env, REG_ADDR(AUX_ID_irq_select, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_ENABLE:
        regval = env->irq_bank[env->irq_select & 0xff].enable;
        break;
    case GDB_AUX_OTHER_REG_IRQ_TRIGGER:
        regval = helper_lr(env, REG_ADDR(AUX_ID_irq_trigger, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_STATUS:
        regval = helper_lr(env, REG_ADDR(AUX_ID_irq_status, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_PULSE:
        regval = 0; /* write only for clearing the pulse triggered interrupt */
        break;
    case GDB_AUX_OTHER_REG_IRQ_PENDING:
        regval = helper_lr(env, REG_ADDR(AUX_ID_irq_pending, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_PRIO:
        regval = helper_lr(env, REG_ADDR(AUX_ID_irq_priority, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_BTA:
        regval = helper_lr(env, REG_ADDR(AUX_ID_bta, cpu->family));
        break;
#ifdef TARGET_ARCV3
    /* MMUv6 */
    case GDB_AUX_OTHER_REG_MMU_CTRL:
        regval = helper_lr(env, REG_ADDR(AUX_ID_mmu_ctrl, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_RTP0:
        regval = helper_lr(env, REG_ADDR(AUX_ID_mmu_rtp0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_RTP1:
        regval = helper_lr(env, REG_ADDR(AUX_ID_mmu_rtp1, cpu->family));
        break;
#endif
    default:
        assert(!"Unsupported other auxiliary register is being read.");
    }
    return GDB_GET_REG(mem_buf, regval);
}


static int
arc_aux_other_gdb_set_reg(CPUARCState *env, uint8_t *mem_buf, int regnum)
{
    ARCCPU *cpu = env_archcpu(env);
    target_ulong regval = ldl_p(mem_buf);
    switch (regnum) {
    case GDB_AUX_OTHER_REG_TIMER_BUILD:
    case GDB_AUX_OTHER_REG_IRQ_BUILD:
    case GDB_AUX_OTHER_REG_MPY_BUILD:
    case GDB_AUX_OTHER_REG_VECBASE_BUILD:
    case GDB_AUX_OTHER_REG_ISA_CONFIG:
    case GDB_AUX_OTHER_REG_MPU_BUILD:
    case GDB_AUX_OTHER_REG_MPU_ECR:
    case GDB_AUX_OTHER_REG_ICAUSE:
    case GDB_AUX_OTHER_REG_IRQ_PRIO_PEND:
    case GDB_AUX_OTHER_REG_IRQ_STATUS:
    case GDB_AUX_OTHER_REG_IRQ_PENDING:
        /* builds/configs/exceptions/irqs cannot be changed */
        break;
    case GDB_AUX_OTHER_REG_TIMER_CNT0:
        helper_sr(env, regval, REG_ADDR(AUX_ID_count0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_CTRL0:
        helper_sr(env, regval, REG_ADDR(AUX_ID_control0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_LIM0:
        helper_sr(env, regval, REG_ADDR(AUX_ID_limit0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_CNT1:
        helper_sr(env, regval, REG_ADDR(AUX_ID_count1, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_CTRL1:
        helper_sr(env, regval, REG_ADDR(AUX_ID_control1, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TIMER_LIM1:
        helper_sr(env, regval, REG_ADDR(AUX_ID_limit1, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_PID:
        helper_sr(env, regval, REG_ADDR(AUX_ID_pid, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TLBPD0:
        helper_sr(env, regval, REG_ADDR(AUX_ID_tlbpd0, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TLBPD1:
        helper_sr(env, regval, REG_ADDR(AUX_ID_tlbpd1, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TLB_INDEX:
        helper_sr(env, regval, REG_ADDR(AUX_ID_tlbindex, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_TLB_CMD:
        helper_sr(env, regval, REG_ADDR(AUX_ID_tlbcommand, cpu->family));
        break;
    /* MPU */
    case GDB_AUX_OTHER_REG_MPU_EN:
        helper_sr(env, regval, REG_ADDR(AUX_ID_mpuen, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_MPU_BASE0 ... GDB_AUX_OTHER_REG_MPU_BASE15: {
        const uint8_t index = regnum - GDB_AUX_OTHER_REG_MPU_BASE0;
        helper_sr(env, regval, REG_ADDR(AUX_ID_mpurdb0 + index, cpu->family));
        break;
    }
    case GDB_AUX_OTHER_REG_MPU_PERM0 ... GDB_AUX_OTHER_REG_MPU_PERM15: {
        const uint8_t index = regnum - GDB_AUX_OTHER_REG_MPU_PERM0;
        helper_sr(env, regval, REG_ADDR(AUX_ID_mpurdp0 + index, cpu->family));
        break;
    }
    /* exceptions */
    case GDB_AUX_OTHER_REG_ERSTATUS:
        helper_sr(env, regval, REG_ADDR(AUX_ID_erstatus, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_ERBTA:
        helper_sr(env, regval, REG_ADDR(AUX_ID_erbta, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_ECR:
        helper_sr(env, regval, REG_ADDR(AUX_ID_ecr, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_ERET:
        helper_sr(env, regval, REG_ADDR(AUX_ID_eret, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_EFA:
        helper_sr(env, regval, REG_ADDR(AUX_ID_efa, cpu->family));
        break;
    /* interrupt */
    case GDB_AUX_OTHER_REG_IRQ_CTRL:
        helper_sr(env, regval, REG_ADDR(AUX_ID_aux_irq_ctrl, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_ACT:
        helper_sr(env, regval, REG_ADDR(AUX_ID_aux_irq_act, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_HINT:
        helper_sr(env, regval, REG_ADDR(AUX_ID_aux_irq_hint, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_SELECT:
        helper_sr(env, regval, REG_ADDR(AUX_ID_irq_select, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_ENABLE:
        helper_sr(env, regval, REG_ADDR(AUX_ID_irq_enable, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_TRIGGER:
        helper_sr(env, regval, REG_ADDR(AUX_ID_irq_trigger, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_PULSE:
        helper_sr(env, regval, REG_ADDR(AUX_ID_irq_pulse_cancel, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_IRQ_PRIO:
        helper_sr(env, regval, REG_ADDR(AUX_ID_irq_priority, cpu->family));
        break;
    case GDB_AUX_OTHER_REG_BTA:
        helper_sr(env, regval, REG_ADDR(AUX_ID_bta, cpu->family));
        break;
    default:
        assert(!"Unsupported other auxiliary register is being written.");
    }
    return 4;
}

#ifdef TARGET_ARCV2
#define GDB_TARGET_MINIMAL_XML "arc-v2-aux.xml"
#define GDB_TARGET_AUX_XML     "arc-v2-other.xml"
#else
#define GDB_TARGET_MINIMAL_XML "arc64-aux-minimal.xml"
#define GDB_TARGET_AUX_XML     "arc64-aux-other.xml"
#endif

void arc_cpu_register_gdb_regs_for_features(ARCCPU *cpu)
{
    CPUState *cs = CPU(cpu);

    gdb_register_coprocessor(cs,
                             arc_aux_minimal_gdb_get_reg, /* getter */
                             arc_aux_minimal_gdb_set_reg, /* setter */
                             GDB_AUX_MIN_REG_LAST,   /* number of registers */
                             GDB_TARGET_MINIMAL_XML, /* feature file */
                             0);                     /* position in g packet */

    gdb_register_coprocessor(cs,
                             arc_aux_other_gdb_get_reg,
                             arc_aux_other_gdb_set_reg,
                             GDB_AUX_OTHER_REG_LAST,
                             GDB_TARGET_AUX_XML,
                             0);
}

/*-*-indent-tabs-mode:nil;tab-width:4;indent-line-function:'insert-tab'-*-*/
/* vim: set ts=4 sw=4 et: */