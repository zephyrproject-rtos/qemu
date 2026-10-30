/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 CPU
 */

#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "sysreg.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/translation-block.h"
#include "exec/target_page.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/intc/rh850_intc.h"
#include "fpu/softfloat.h"
#include "tcg/debug-assert.h"
#include "accel/tcg/cpu-ops.h"
#include "accel/tcg/cpu-loop.h"

static void rh850_cpu_set_pc(CPUState *cs, vaddr value)
{
    RH850CPU *cpu = RH850_CPU(cs);

    cpu->env.pc = value;
}

static vaddr rh850_cpu_get_pc(CPUState *cs)
{
    RH850CPU *cpu = RH850_CPU(cs);

    return cpu->env.pc;
}

static TCGTBCPUState rh850_get_tb_cpu_state(CPUState *cs)
{
    CPURH850State *env = cpu_env(cs);
    uint32_t flags = 0;

    flags = FIELD_DP32(flags, PSW, UM, FIELD_EX32(env->psw_ctrl, PSW, UM));
    if (!(RH850_SR(env, RH850_SR_MCTL) & RH850_MCTL_MA)) {
        flags |= RH850_TBFLAG_ALIGN;
    }
    if (FIELD_EX32(env->psw_ctrl, PSW, CU) & 1) {
        flags |= RH850_TBFLAG_CU0;
    }
    if (RH850_SR(env, RH850_SR_FPEC) & 1) {
        flags |= RH850_TBFLAG_FPIVD;
    }

    return (TCGTBCPUState){ .pc = env->pc, .flags = flags };
}

static void rh850_cpu_synchronize_from_tb(CPUState *cs,
                                          const TranslationBlock *tb)
{
    RH850CPU *cpu = RH850_CPU(cs);

    tcg_debug_assert(!tcg_cflags_has(cs, CF_PCREL));
    cpu->env.pc = tb->pc;
}

static void rh850_restore_state_to_opc(CPUState *cs,
                                       const TranslationBlock *tb,
                                       const uint64_t *data)
{
    RH850CPU *cpu = RH850_CPU(cs);

    cpu->env.pc = data[0];
    cpu->env.insn_mei = data[1];
}

/*
 * Any terminating exception request releases HALT/SNOOZE, also when
 * PSW.ID/NP, ISPR or PLMR keep it from being acknowledged
 */
static bool rh850_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, RH850_INTERRUPT_EIINT |
                                  RH850_INTERRUPT_FEINT |
                                  RH850_INTERRUPT_FENMI |
                                  RH850_INTERRUPT_FPI);
}

/* A single-stepped HALT stops when the CPU leaves the halt state */
static void rh850_cpu_leaving_halt(CPUState *cs)
{
    if (cpu_single_stepping(cs)) {
        cs->exception_index = EXCP_DEBUG;
    }
}

static bool rh850_cpu_exec_halt(CPUState *cs)
{
    if (!rh850_cpu_has_work(cs)) {
        return false;
    }
    rh850_cpu_leaving_halt(cs);
    return true;
}

static int rh850_cpu_mmu_index(CPUState *cs, bool ifunc)
{
    return 0;
}

/*
 * Order by level and priority: FENMI, FEINT (NP = 0), then FPI and
 * EIINT (ID = 0 and NP = 0)
 */
static bool rh850_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPURH850State *env = cpu_env(cs);
    bool id = FIELD_EX32(env->psw_ctrl, PSW, ID);
    bool np = FIELD_EX32(env->psw_ctrl, PSW, NP);

    if (interrupt_request & RH850_INTERRUPT_FENMI) {
        cs->exception_index = RH850_EXCP_FENMI;
        env->excp_cause = RH850_CAUSE_FENMI;
        rh850_cpu_do_interrupt(cs);
        qemu_set_irq(RH850_CPU(cs)->intc_ack[RH850_INTC_ACK_FENMI], 1);
        return true;
    }
    if ((interrupt_request & RH850_INTERRUPT_FEINT) && !np) {
        cs->exception_index = RH850_EXCP_FEINT;
        env->excp_cause = RH850_CAUSE_FEINT;
        rh850_cpu_do_interrupt(cs);
        return true;
    }
    if (id || np) {
        return false;
    }
    if (interrupt_request & RH850_INTERRUPT_FPI) {
        RH850_SR(env, RH850_SR_FPEC) = 0;
        cpu_reset_interrupt(cs, RH850_INTERRUPT_FPI);
        cs->exception_index = RH850_EXCP_FPI;
        env->excp_cause = RH850_CAUSE_FPI;
        rh850_cpu_do_interrupt(cs);
        return true;
    }
    if (interrupt_request & RH850_INTERRUPT_EIINT) {
        return rh850_cpu_eiint(cs);
    }
    return false;
}

/* EIINT carries channel, priority and EITB; FEINT and FENMI are levels */
static void rh850_cpu_set_irq(void *opaque, int n, int level)
{
    RH850CPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    static const int mask[] = {
        [RH850_INTC_OUT_EIINT] = RH850_INTERRUPT_EIINT,
        [RH850_INTC_OUT_FEINT] = RH850_INTERRUPT_FEINT,
        [RH850_INTC_OUT_FENMI] = RH850_INTERRUPT_FENMI,
    };

    if (n == RH850_INTC_OUT_EIINT && level) {
        cpu->env.eiint_ch = level & RH850_INTC_REQ_CH_MASK;
        cpu->env.eiint_prio = extract32(level, RH850_INTC_REQ_PRIO, 4);
        cpu->env.eiint_tb = extract32(level, RH850_INTC_REQ_TB, 1);
    }
    if (level) {
        cpu_interrupt(cs, mask[n]);
    } else {
        cpu_reset_interrupt(cs, mask[n]);
    }
}

QEMU_BUILD_BUG_ON(RH850_CPU_NACK != RH850_INTC_ACK_NUM);

static void rh850_cpu_init(Object *obj)
{
    RH850CPU *cpu = RH850_CPU(obj);

    qdev_init_gpio_in(DEVICE(obj), rh850_cpu_set_irq, RH850_INTC_OUT_NUM);
    qdev_init_gpio_out_named(DEVICE(obj), cpu->intc_ack, "intc-ack",
                             RH850_INTC_ACK_NUM);
}

static void rh850_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    RH850CPU *cpu = RH850_CPU(obj);
    RH850CPUClass *rcc = RH850_CPU_GET_CLASS(obj);
    CPURH850State *env = cpu_env(cs);

    if (rcc->parent_phases.hold) {
        rcc->parent_phases.hold(obj, type);
    }

    memset(env, 0, offsetof(CPURH850State, end_reset_fields));
    rh850_sysreg_reset(env);

    /* PC = RBASE, PSW = 00000020H (ID set) */
    env->sr[RH850_SR_RBASE_SELID][RH850_SR_RBASE_REGID] =
        cpu->rbase & RH850_RBASE_MASK;
    env->pc = env->sr[RH850_SR_RBASE_SELID][RH850_SR_RBASE_REGID];
    rh850_cpu_unpack_psw(env, RH850_PSW_RESET);
    rh850_fpu_sync(env);
}

static ObjectClass *rh850_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename;

    oc = object_class_by_name(cpu_model);
    if (oc != NULL && object_class_dynamic_cast(oc, TYPE_RH850_CPU) != NULL) {
        return oc;
    }
    typename = g_strdup_printf(RH850_CPU_TYPE_NAME("%s"), cpu_model);
    oc = object_class_by_name(typename);
    g_free(typename);

    return oc;
}

static void rh850_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    RH850CPU *cpu = RH850_CPU(dev);
    RH850CPUClass *rcc = RH850_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    if (cpu->rbase & ~RH850_RBASE_MASK) {
        error_setg(errp, "rbase 0x%08" PRIx32 " is not 512-byte aligned",
                   cpu->rbase);
        return;
    }

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

    rcc->parent_realize(dev, errp);
}

static bool rh850_cpu_tlb_fill(CPUState *cs, vaddr addr, int size,
                               MMUAccessType access_type, int mmu_idx,
                               bool probe, uintptr_t retaddr)
{
    uint32_t address, physical, prot;

    /* Flat mapping until the MPU is modelled */
    address = physical = addr & TARGET_PAGE_MASK;
    prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    tlb_set_page(cs, address, physical, prot, mmu_idx, TARGET_PAGE_SIZE);
    return true;
}

/*
 * Unmapped area: a fetch raises SYSERR 11H; a data access is logged,
 * reads 0 and drops the write
 */
static void rh850_cpu_do_transaction_failed(CPUState *cs, hwaddr physaddr,
                                            vaddr addr, unsigned size,
                                            MMUAccessType access_type,
                                            int mmu_idx, MemTxAttrs attrs,
                                            MemTxResult response,
                                            uintptr_t retaddr)
{
    if (access_type == MMU_INST_FETCH) {
        qemu_log_mask(LOG_GUEST_ERROR, "rh850: fetch error at 0x%08"
                      HWADDR_PRIx ", SYSERR\n", physaddr);
        rh850_raise_exception(cpu_env(cs), RH850_EXCP_SYSERR,
                              RH850_CAUSE_SYSERR_FETCH, retaddr);
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rh850: data access error: %s of %u "
                  "bytes at 0x%08" HWADDR_PRIx " (ECM error 255)\n",
                  access_type == MMU_DATA_STORE ? "write" : "read", size,
                  physaddr);
    /* A bus error of LDL loses the link */
    if (cpu_env(cs)->ldl_active) {
        cpu_env(cs)->ll_valid = 0;
    }
}

/* With MCTL.MA = 0 a misaligned access raises MAE */
static G_NORETURN void rh850_cpu_do_unaligned_access(CPUState *cs,
                                                     vaddr addr,
                                                     MMUAccessType access_type,
                                                     int mmu_idx,
                                                     uintptr_t retaddr)
{
    CPURH850State *env = cpu_env(cs);

    cpu_restore_state(cs, retaddr);
    RH850_SR(env, RH850_SR_MEA) = addr;
    RH850_SR(env, RH850_SR_MEI) = env->insn_mei;
    rh850_raise_exception(env, RH850_EXCP_MAE, RH850_CAUSE_MAE, 0);
}

static bool rh850_cpu_translate_for_debug(CPUState *cs, vaddr addr,
                                          TranslateForDebugResult *result)
{
    *result = (TranslateForDebugResult) {
        .physaddr = addr,
        .lg_page_size = TARGET_PAGE_BITS,
        .attrs.debug = 1,
    };
    return true;
}

void rh850_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPURH850State *env = cpu_env(cs);
    int i;

    qemu_fprintf(f, "pc=0x%08x psw=0x%08x rbase=0x%08x\n", env->pc,
                 rh850_cpu_pack_psw(env),
                 env->sr[RH850_SR_RBASE_SELID][RH850_SR_RBASE_REGID]);
    for (i = 0; i < RH850_NUM_GPRS; i++) {
        qemu_fprintf(f, "r%-2d=0x%08x%c", i, env->gpr[i],
                     (i % 4) == 3 ? '\n' : ' ');
    }
}

static const Property rh850_cpu_properties[] = {
    DEFINE_PROP_UINT32("rbase", RH850CPU, rbase, 0),
};

#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps rh850_sysemu_ops = {
    .has_work = rh850_cpu_has_work,
    .translate_for_debug = rh850_cpu_translate_for_debug,
};

static const TCGCPUOps rh850_tcg_ops = {
    /* MTTCG not yet supported: require strict ordering */
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = rh850_translate_init,
    .translate_code = rh850_translate_code,
    .get_tb_cpu_state = rh850_get_tb_cpu_state,
    .synchronize_from_tb = rh850_cpu_synchronize_from_tb,
    .restore_state_to_opc = rh850_restore_state_to_opc,
    .mmu_index = rh850_cpu_mmu_index,
    .tlb_fill = rh850_cpu_tlb_fill,
    .do_transaction_failed = rh850_cpu_do_transaction_failed,
    .do_unaligned_access = rh850_cpu_do_unaligned_access,
    .pointer_wrap = cpu_pointer_wrap_uint32,

    .cpu_exec_interrupt = rh850_cpu_exec_interrupt,
    .cpu_exec_halt = rh850_cpu_exec_halt,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = rh850_cpu_do_interrupt,
};

static void rh850_cpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    CPUClass *cc = CPU_CLASS(klass);
    RH850CPUClass *rcc = RH850_CPU_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_parent_realize(dc, rh850_cpu_realize,
                                    &rcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, rh850_cpu_reset_hold, NULL,
                                       &rcc->parent_phases);
    device_class_set_props(dc, rh850_cpu_properties);

    cc->class_by_name = rh850_cpu_class_by_name;
    cc->dump_state = rh850_cpu_dump_state;
    cc->set_pc = rh850_cpu_set_pc;
    cc->get_pc = rh850_cpu_get_pc;
    cc->gdb_read_register = rh850_cpu_gdb_read_register;
    cc->gdb_write_register = rh850_cpu_gdb_write_register;
    cc->gdb_num_core_regs = RH850_GDB_NUM_REGS;

    cc->sysemu_ops = &rh850_sysemu_ops;
    cc->tcg_ops = &rh850_tcg_ops;
}

static const TypeInfo rh850_cpu_info = {
    .name = TYPE_RH850_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(RH850CPU),
    .instance_align = __alignof(RH850CPU),
    .instance_init = rh850_cpu_init,
    .abstract = true,
    .class_size = sizeof(RH850CPUClass),
    .class_init = rh850_cpu_class_init,
};

static const TypeInfo rh850_g4kh_cpu_info = {
    .name = TYPE_RH850_G4KH_CPU,
    .parent = TYPE_RH850_CPU,
};

static void rh850_cpu_register_types(void)
{
    type_register_static(&rh850_cpu_info);
    type_register_static(&rh850_g4kh_cpu_info);
}

type_init(rh850_cpu_register_types)
