/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 exception entry and return
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "hw/core/irq.h"
#include "hw/intc/rh850_intc.h"
#include "cpu.h"
#include "sysreg.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/cpu-loop.h"

/* Level, handler offset and interrupt flag of each exception */
static const struct {
    bool fe;
    bool intr;
    uint16_t offset;
    const char *name;
} rh850_excp_info[RH850_EXCP_NUM] = {
    [RH850_EXCP_SYSERR] = { true, false, 0x010, "SYSERR" },
    [RH850_EXCP_UCPOP] = { true, false, 0x080, "UCPOP" },
    [RH850_EXCP_RIE] = { true, false, 0x060, "RIE" },
    [RH850_EXCP_PIE] = { true, false, 0x0a0, "PIE" },
    [RH850_EXCP_MAE] = { true, false, 0x0c0, "MAE" },
    [RH850_EXCP_SYSCALL] = { false, false, 0, "SYSCALL" },
    [RH850_EXCP_FETRAP] = { true, false, 0x030, "FETRAP" },
    [RH850_EXCP_TRAP] = { false, false, 0x040, "TRAP" },
    [RH850_EXCP_FPE] = { false, false, 0x070, "FPE" },
    [RH850_EXCP_FPI] = { false, false, 0x070, "FPI" },
    [RH850_EXCP_FENMI] = { true, true, 0x0e0, "FENMI" },
    [RH850_EXCP_FEINT] = { true, true, 0x0f0, "FEINT" },
    [RH850_EXCP_EIINT] = { false, true, 0, "EIINT" },
};

/*
 * Record the exception and leave the TB; with retaddr the PC of the
 * faulting instruction is restored
 */
G_NORETURN void rh850_raise_exception(CPURH850State *env, int excp,
                                      uint32_t cause, uintptr_t retaddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    env->excp_cause = cause;
    cpu_loop_exit_restore(cs, retaddr);
}

/*
 * Save PC/PSW and the cause, then UM = 0, ID = 1, EP = 0 for interrupts
 * else 1, NP = 1 for FE level; the LDL link is lost
 */
void rh850_cpu_do_interrupt(CPUState *cs)
{
    CPURH850State *env = cpu_env(cs);
    int excp = cs->exception_index;
    uint32_t psw = rh850_cpu_pack_psw(env);
    uint32_t base, offset;

    g_assert(excp >= 0 && excp < RH850_EXCP_NUM);
    if (rh850_excp_info[excp].fe) {
        RH850_SR(env, RH850_SR_FEPC) = env->pc;
        RH850_SR(env, RH850_SR_FEPSW) = psw;
        RH850_SR(env, RH850_SR_FEIC) = env->excp_cause;
        psw = FIELD_DP32(psw, PSW, NP, 1);
    } else {
        RH850_SR(env, RH850_SR_EIPC) = env->pc;
        RH850_SR(env, RH850_SR_EIPSW) = psw;
        RH850_SR(env, RH850_SR_EIIC) = env->excp_cause;
    }
    psw = FIELD_DP32(psw, PSW, UM, 0);
    psw = FIELD_DP32(psw, PSW, ID, 1);
    psw = FIELD_DP32(psw, PSW, EP, !rh850_excp_info[excp].intr);
    rh850_cpu_unpack_psw(env, psw);
    env->ll_valid = 0;

    if (excp == RH850_EXCP_SYSCALL || excp == RH850_EXCP_EIINT) {
        env->pc = env->excp_target;
    } else {
        base = FIELD_EX32(psw, PSW, EBV) ? RH850_SR(env, RH850_SR_EBASE) :
               RH850_SR(env, RH850_SR_RBASE);
        offset = rh850_excp_info[excp].offset;
        /* TRAP1 (vector 16-31) at 050H */
        if (excp == RH850_EXCP_TRAP && (env->excp_cause & 0x10)) {
            offset = 0x050;
        }
        env->pc = (base & RH850_RBASE_MASK) + offset;
    }
    qemu_log_mask(CPU_LOG_INT, "rh850: %s cause 0x%08x saved pc 0x%08x "
                  "handler 0x%08x\n", rh850_excp_info[excp].name,
                  env->excp_cause, rh850_excp_info[excp].fe ?
                  RH850_SR(env, RH850_SR_FEPC) : RH850_SR(env, RH850_SR_EIPC),
                  env->pc);
    cs->exception_index = -1;
}

/* EIINT priority masks (INTCFG.EPL is fixed 0 on G4KH) */
static bool rh850_ispr_masked(CPURH850State *env, uint32_t prio)
{
    /* An ISPR bit for the same or a higher priority masks it */
    return RH850_SR(env, RH850_SR_ISPR) & ((2u << prio) - 1);
}

static bool rh850_plmr_masked(CPURH850State *env, uint32_t prio)
{
    /* PLM = n masks priority n and lower */
    return prio >= RH850_SR(env, RH850_SR_PLMR);
}

/* ICSR.PMEI: the request is masked by PLMR, after the ISPR mask */
uint32_t rh850_cpu_icsr(CPURH850State *env)
{
    return cpu_test_interrupt(env_cpu(env), RH850_INTERRUPT_EIINT) &&
           !rh850_ispr_masked(env, env->eiint_prio) &&
           rh850_plmr_masked(env, env->eiint_prio);
}

/*
 * Table reference with RBCR0.BE set saves to the register bank; with
 * BN > 15 SYSERR 1CH is raised instead and the request stays
 */
bool rh850_cpu_eiint(CPUState *cs)
{
    CPURH850State *env = cpu_env(cs);
    uint32_t ch = env->eiint_ch, prio = env->eiint_prio;
    uint32_t psw = rh850_cpu_pack_psw(env);
    uint32_t base, cause = RH850_CAUSE_EIINT + ch;
    uint32_t adr, rbcr0 = RH850_SR(env, RH850_SR_RBCR0);
    bool table, bank;
    int r;

    if (rh850_ispr_masked(env, prio) || rh850_plmr_masked(env, prio)) {
        return false;
    }
    base = FIELD_EX32(psw, PSW, EBV) ? RH850_SR(env, RH850_SR_EBASE) :
           RH850_SR(env, RH850_SR_RBASE);
    table = env->eiint_tb && !(base & RH850_BASE_DV);
    bank = table && (rbcr0 & (1u << prio));
    if (bank && RH850_SR(env, RH850_SR_RBNR) > 15) {
        cs->exception_index = RH850_EXCP_SYSERR;
        env->excp_cause = RH850_CAUSE_SYSERR_BANK;
        rh850_cpu_do_interrupt(cs);
        return true;
    }
    if (table) {
        env->excp_target = cpu_ldl_le_data(env, RH850_SR(env, RH850_SR_INTBP)
                                           + ch * 4);
    } else {
        env->excp_target = (base & RH850_RBASE_MASK) + 0x100 +
                           (base & RH850_BASE_RINT ? 0 : prio * 0x10);
    }
    if (bank) {
        bool md = rbcr0 & (1u << 16);

        adr = RH850_SR(env, RH850_SR_RBIP) -
              RH850_SR(env, RH850_SR_RBNR) * (md ? 0x90 : 0x60);
        cpu_stl_le_data(env, adr - 0x04, env->pc);
        cpu_stl_le_data(env, adr - 0x08, psw);
        cpu_stl_le_data(env, adr - 0x0c, cause);
        cpu_stl_le_data(env, adr - 0x10, RH850_SR(env, RH850_SR_FPSR));
        for (r = 1; r <= (md ? 31 : 19); r++) {
            cpu_stl_le_data(env, adr - 0x10 - r * 4, env->gpr[r]);
        }
        if (!md) {
            cpu_stl_le_data(env, adr - 0x60, env->gpr[30]);
        }
        RH850_SR(env, RH850_SR_RBNR)++;
    }
    cs->exception_index = RH850_EXCP_EIINT;
    env->excp_cause = cause;
    rh850_cpu_do_interrupt(cs);
    if (bank && !(RH850_SR(env, RH850_SR_RBCR1) & (1u << prio))) {
        env->psw_ctrl = FIELD_DP32(env->psw_ctrl, PSW, ID, 0);
    }
    if (!(RH850_SR(env, RH850_SR_INTCFG) & RH850_INTCFG_ISPC)) {
        RH850_SR(env, RH850_SR_ISPR) |= 1u << prio;
    }
    qemu_set_irq(env_archcpu(env)->intc_ack[RH850_INTC_ACK_EIINT], ch);
    return true;
}

/* EIRET/FERET with PSW.EP = 0 notify the INTC of the end of interrupt */
void rh850_cpu_eoi(CPURH850State *env)
{
    BQL_LOCK_GUARD();
    qemu_set_irq(env_archcpu(env)->intc_ack[RH850_INTC_ACK_EOI], 1);
}

/* Exception with env->pc already set by the translator */
G_NORETURN void HELPER(raise)(CPURH850State *env, uint32_t excp,
                              uint32_t cause)
{
    rh850_raise_exception(env, excp, cause, 0);
}

/* MAE found by an explicit alignment check: set MEA and MEI */
G_NORETURN void HELPER(mae)(CPURH850State *env, uint32_t addr, uint32_t mei)
{
    RH850_SR(env, RH850_SR_MEA) = addr;
    RH850_SR(env, RH850_SR_MEI) = mei;
    rh850_raise_exception(env, RH850_EXCP_MAE, RH850_CAUSE_MAE, 0);
}

/* SYSCALL: entry = vector if vector <= SCCFG.SIZE, else 0 */
G_NORETURN void HELPER(syscall)(CPURH850State *env, uint32_t vector)
{
    uint32_t scbp = RH850_SR(env, RH850_SR_SCBP);
    uint32_t entry = vector <= (RH850_SR(env, RH850_SR_SCCFG) & 0xff) ?
                     vector : 0;

    env->excp_target = scbp + cpu_ldl_le_data(env, scbp + entry * 4);
    rh850_raise_exception(env, RH850_EXCP_SYSCALL,
                          RH850_CAUSE_SYSCALL + vector, 0);
}

/* PC bit 0 and the fixed PSW bits stay 0; the LDL link is lost */
static void rh850_return(CPURH850State *env, uint32_t pc, uint32_t psw)
{
    env->pc = pc & ~1u;
    rh850_cpu_unpack_psw(env, psw & rh850_sysreg_wmask(RH850_SR_PSWREG));
    env->ll_valid = 0;
}

/*
 * EIRET with PSW.EP = 0 clears the highest priority ISPR bit
 * (INTCFG.ISPC = 0)
 */
void HELPER(eiret)(CPURH850State *env)
{
    uint32_t *ispr = &RH850_SR(env, RH850_SR_ISPR);

    if (!FIELD_EX32(env->psw_ctrl, PSW, EP)) {
        rh850_cpu_eoi(env);
        if (!(RH850_SR(env, RH850_SR_INTCFG) & RH850_INTCFG_ISPC)) {
            *ispr &= *ispr - 1;
        }
    }
    rh850_return(env, RH850_SR(env, RH850_SR_EIPC),
                 RH850_SR(env, RH850_SR_EIPSW));
}

/* FERET: INTC notification for EP = 0, ISPR unchanged */
void HELPER(feret)(CPURH850State *env)
{
    if (!FIELD_EX32(env->psw_ctrl, PSW, EP)) {
        rh850_cpu_eoi(env);
    }
    rh850_return(env, RH850_SR(env, RH850_SR_FEPC),
                 RH850_SR(env, RH850_SR_FEPSW));
}

/* EI/DI: supervisor only unless MCTL.UIC = 1 */
void HELPER(check_uic)(CPURH850State *env)
{
    if (FIELD_EX32(env->psw_ctrl, PSW, UM) &&
        !(RH850_SR(env, RH850_SR_MCTL) & RH850_MCTL_UIC)) {
        rh850_raise_exception(env, RH850_EXCP_PIE, RH850_CAUSE_PIE, 0);
    }
}

/*
 * HALT: env->pc holds the next instruction, the return PC of an
 * exception taken in HALT state
 */
G_NORETURN void HELPER(halt)(CPURH850State *env)
{
    CPUState *cs = env_cpu(env);

    qemu_log_mask(CPU_LOG_INT, "rh850: HALT, resume pc 0x%08x\n", env->pc);
    cs->halted = 1;
    rh850_raise_exception(env, EXCP_HLT, 0, 0);
}

static void rh850_sr_write(CPURH850State *env, unsigned n, uint32_t val)
{
    uint32_t mask = rh850_sysreg_wmask(n);

    RH850_SR(env, n) = (RH850_SR(env, n) & ~mask) | (val & mask);
}

/*
 * RESBANK: BN = 0 raises SYSERR 1DH; else bank BN - 1 is restored,
 * BN decremented and PSW.ID set
 */
void HELPER(resbank)(CPURH850State *env)
{
    uintptr_t ra = GETPC();
    uint32_t bn = RH850_SR(env, RH850_SR_RBNR);
    bool md = RH850_SR(env, RH850_SR_RBCR0) & (1u << 16);
    uint32_t adr;
    int r;

    if (bn == 0) {
        rh850_raise_exception(env, RH850_EXCP_SYSERR,
                              RH850_CAUSE_SYSERR_RESBANK, ra);
    }
    adr = RH850_SR(env, RH850_SR_RBIP) - bn * (md ? 0x90 : 0x60);
    if (!md) {
        env->gpr[30] = cpu_ldl_le_data_ra(env, adr, ra);
    }
    for (r = md ? 31 : 19; r > 0; r--) {
        adr += 4;
        env->gpr[r] = cpu_ldl_le_data_ra(env, adr, ra);
    }
    rh850_sr_write(env, RH850_SR_FPSR, cpu_ldl_le_data_ra(env, adr + 4, ra));
    rh850_fpu_sync(env);
    rh850_sr_write(env, RH850_SR_EIIC, cpu_ldl_le_data_ra(env, adr + 8, ra));
    rh850_sr_write(env, RH850_SR_EIPSW, cpu_ldl_le_data_ra(env, adr + 12, ra));
    rh850_sr_write(env, RH850_SR_EIPC, cpu_ldl_le_data_ra(env, adr + 16, ra));
    RH850_SR(env, RH850_SR_RBNR) = bn - 1;
    env->psw_ctrl = FIELD_DP32(env->psw_ctrl, PSW, ID, 1);
}
