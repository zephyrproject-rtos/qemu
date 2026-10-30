/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 CPU definitions
 */

#ifndef RH850_CPU_H
#define RH850_CPU_H

#include "hw/core/registerfields.h"
#include "cpu-qom.h"

#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "qemu/cpu-float.h"

#ifdef CONFIG_USER_ONLY
#error "RH850 does not support user mode emulation"
#endif

REG32(PSW, 0)
FIELD(PSW, Z, 0, 1)
FIELD(PSW, S, 1, 1)
FIELD(PSW, OV, 2, 1)
FIELD(PSW, CY, 3, 1)
FIELD(PSW, SAT, 4, 1)
FIELD(PSW, ID, 5, 1)
FIELD(PSW, EP, 6, 1)
FIELD(PSW, NP, 7, 1)
FIELD(PSW, EBV, 15, 1)
FIELD(PSW, CU, 16, 3)
FIELD(PSW, EIMASK, 20, 6)
FIELD(PSW, UM, 30, 1)

#define RH850_PSW_FLAGS_MASK    0x0000001fu
#define RH850_PSW_RESET         0x00000020u

/* RBASE (selID 1, regID 2) */
#define RH850_SR_RBASE_SELID    1
#define RH850_SR_RBASE_REGID    2
#define RH850_RBASE_MASK        0xfffffe00u

/* TB flag: MCTL.MA = 0, misaligned accesses raise MAE */
#define RH850_TBFLAG_ALIGN      (1u << 0)
/* TB flag: PSW.CU0 = 1, FPU instructions usable */
#define RH850_TBFLAG_CU0        (1u << 1)
/* TB flag: FPEC.FPIVD = 1, FPU instructions invalidated */
#define RH850_TBFLAG_FPIVD      (1u << 2)

/* Interrupt request lines from the INTC */
#define RH850_INTERRUPT_EIINT   CPU_INTERRUPT_HARD
#define RH850_INTERRUPT_FEINT   CPU_INTERRUPT_TGT_EXT_0
#define RH850_INTERRUPT_FENMI   CPU_INTERRUPT_TGT_EXT_1
/* Lines to the INTC */
#define RH850_CPU_NACK          3
/* FPI reported and pending (FPEC.FPIVD = 1) */
#define RH850_INTERRUPT_FPI     CPU_INTERRUPT_TGT_INT_0

/* Synchronous exceptions (cs->exception_index) */
enum {
    RH850_EXCP_SYSERR,
    RH850_EXCP_UCPOP,
    RH850_EXCP_RIE,
    RH850_EXCP_PIE,
    RH850_EXCP_MAE,
    RH850_EXCP_SYSCALL,
    RH850_EXCP_FETRAP,
    RH850_EXCP_TRAP,
    RH850_EXCP_FPE,
    RH850_EXCP_FPI,
    RH850_EXCP_FENMI,
    RH850_EXCP_FEINT,
    RH850_EXCP_EIINT,
    RH850_EXCP_NUM,
};

/* Low 16 bits of the cause codes; the detail code (high 16 bits) is 0 */
enum {
    RH850_CAUSE_SYSERR_FETCH = 0x11,
    RH850_CAUSE_SYSERR_BANK = 0x1c,
    RH850_CAUSE_SYSERR_RESBANK = 0x1d,
    RH850_CAUSE_FETRAP = 0x30,
    RH850_CAUSE_TRAP = 0x40,
    RH850_CAUSE_RIE = 0x60,
    RH850_CAUSE_FPE = 0x71,
    RH850_CAUSE_FPI = 0x72,
    RH850_CAUSE_UCPOP = 0x80,
    RH850_CAUSE_PIE = 0xa0,
    RH850_CAUSE_MAE = 0xc0,
    RH850_CAUSE_FENMI = 0xe0,
    RH850_CAUSE_FEINT = 0xf0,
    RH850_CAUSE_EIINT = 0x1000,
    RH850_CAUSE_SYSCALL = 0x8000,
};

enum {
    RH850_NUM_GPRS = 32,
    RH850_NUM_SELID = 13,
    RH850_NUM_REGID = 32,
    RH850_MPU_ENTRIES = 24,
    RH850_MPU_MPID = 8,
    /* GDB v850e3v5 register count */
    RH850_GDB_NUM_REGS = 406,
};

typedef struct CPUArchState {
    uint32_t gpr[RH850_NUM_GPRS];
    uint32_t pc;

    /* PSW Z/S/OV/CY/SAT, kept split; each holds 0 or 1 */
    uint32_t psw_z;
    uint32_t psw_s;
    uint32_t psw_ov;
    uint32_t psw_cy;
    uint32_t psw_sat;
    /* Remaining PSW bits (UM, EIMASK, CU, EBV, NP, EP, ID), packed */
    uint32_t psw_ctrl;

    /* System registers indexed [selID][regID]; PSW (SR5,0) is not here */
    uint32_t sr[RH850_NUM_SELID][RH850_NUM_REGID];

    /* MPU area registers, selected through MPIDX */
    uint32_t mpla[RH850_MPU_ENTRIES];
    uint32_t mpua[RH850_MPU_ENTRIES];
    uint32_t mpat[RH850_MPU_ENTRIES];
    uint32_t mpid[RH850_MPU_MPID];

    /* LDL/STC link: address, size in bytes, loaded value, valid flag */
    uint32_t ll_addr;
    uint32_t ll_size;
    uint32_t ll_val;
    uint32_t ll_valid;
    /* Set while an LDL access is in progress (bus error clears the link) */
    uint32_t ldl_active;

    /* Pending exception: EIIC/FEIC value and, for SYSCALL, the handler */
    uint32_t excp_cause;
    uint32_t excp_target;
    /* MEI value of the current instruction (insn_start word 1) */
    uint32_t insn_mei;

    /* Fields up to this point are cleared by a CPU reset */
    struct {} end_reset_fields;

    /* EIINT request from the INTC: channel, priority, table reference */
    uint32_t eiint_ch;
    uint32_t eiint_prio;
    uint32_t eiint_tb;

    float_status fp_status;
} CPURH850State;

struct ArchCPU {
    CPUState parent_obj;

    CPURH850State env;

    /* Reset vector base (RBASE) */
    uint32_t rbase;

    /* To the INTC: EIINT and FENMI acceptance, end of interrupt */
    qemu_irq intc_ack[RH850_CPU_NACK];
};

struct RH850CPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define CPU_RESOLVING_TYPE TYPE_RH850_CPU

void rh850_translate_init(void);
void rh850_translate_code(CPUState *cs, TranslationBlock *tb,
                          int *max_insns, vaddr pc, void *host_pc);
void rh850_cpu_dump_state(CPUState *cs, FILE *f, int flags);
void rh850_cpu_do_interrupt(CPUState *cs);
int rh850_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n);
int rh850_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n);
bool rh850_cpu_eiint(CPUState *cs);
uint32_t rh850_cpu_icsr(CPURH850State *env);
void rh850_cpu_eoi(CPURH850State *env);
G_NORETURN void rh850_raise_exception(CPURH850State *env, int excp,
                                      uint32_t cause, uintptr_t retaddr);
/*
 * FP conversion control: bits 2-0 rounding (CVT_RM = current mode),
 * CVT_U unsigned integer
 */
enum {
    CVT_RM, CVT_RN, CVT_RP, CVT_RMI, CVT_RZ,
    CVT_U = 8,
};

void rh850_fpu_sync(CPURH850State *env);
bool rh850_fpu_flags(CPURH850State *env, uint32_t causes, bool unimpl,
                     uintptr_t retaddr);

static inline uint32_t rh850_cpu_pack_psw(CPURH850State *env)
{
    uint32_t psw = env->psw_ctrl & ~RH850_PSW_FLAGS_MASK;

    psw = FIELD_DP32(psw, PSW, Z, env->psw_z);
    psw = FIELD_DP32(psw, PSW, S, env->psw_s);
    psw = FIELD_DP32(psw, PSW, OV, env->psw_ov);
    psw = FIELD_DP32(psw, PSW, CY, env->psw_cy);
    psw = FIELD_DP32(psw, PSW, SAT, env->psw_sat);
    return psw;
}

static inline void rh850_cpu_unpack_psw(CPURH850State *env, uint32_t psw)
{
    env->psw_z = FIELD_EX32(psw, PSW, Z);
    env->psw_s = FIELD_EX32(psw, PSW, S);
    env->psw_ov = FIELD_EX32(psw, PSW, OV);
    env->psw_cy = FIELD_EX32(psw, PSW, CY);
    env->psw_sat = FIELD_EX32(psw, PSW, SAT);
    env->psw_ctrl = psw & ~RH850_PSW_FLAGS_MASK;
}

#endif /* RH850_CPU_H */
