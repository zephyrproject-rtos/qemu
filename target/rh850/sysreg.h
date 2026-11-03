/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 system registers
 */

#ifndef RH850_SYSREG_H
#define RH850_SYSREG_H

/* Who may access a register */
enum {
    RH850_SR_SV,        /* supervisor */
    RH850_SR_UM,        /* any mode */
    RH850_SR_PSW,       /* PSW: per-bit write rules */
    RH850_SR_CU0_SV,    /* PSW.CU0 and supervisor */
    RH850_SR_CU0,       /* PSW.CU0 */
    RH850_SR_CU1_SV,    /* PSW.CU1 and supervisor */
    RH850_SR_CU1,       /* PSW.CU1 */
    RH850_SR_PMU,       /* supervisor, user if PMUMCTRL.PMUEn */
    RH850_SR_UMR_SVW,   /* read in any mode, write in supervisor */
};

/* Side effects */
enum {
    RH850_SRF_RO = 1 << 0,      /* writes ignored */
    RH850_SRF_PSW = 1 << 1,     /* split PSW flags */
    RH850_SRF_ISPR = 1 << 2,    /* writable only if INTCFG.ISPC = 1 */
    RH850_SRF_SVLOCK = 1 << 3,  /* writes ignored if SVLOCK.SVL = 1 */
    RH850_SRF_MPIDX = 1 << 4,   /* MPU area register selected by MPIDX */
};

typedef struct RH850SysRegInfo {
    const char *name;
    uint8_t sel;
    uint8_t id;
    uint32_t reset;
    uint32_t wmask;
    uint8_t access;
    uint8_t flags;
} RH850SysRegInfo;

/* Registers used by number: selID << 5 | regID */
enum {
    RH850_SR_EIPC = 0 << 5 | 0,
    RH850_SR_EIPSW = 0 << 5 | 1,
    RH850_SR_FEPC = 0 << 5 | 2,
    RH850_SR_FEPSW = 0 << 5 | 3,
    RH850_SR_PSWREG = 0 << 5 | 5,
    RH850_SR_FPSR = 0 << 5 | 6,
    RH850_SR_FPEPC = 0 << 5 | 7,
    RH850_SR_FPST = 0 << 5 | 8,
    RH850_SR_FPCC = 0 << 5 | 9,
    RH850_SR_FPCFG = 0 << 5 | 10,
    RH850_SR_FPEC = 0 << 5 | 11,
    RH850_SR_EIIC = 0 << 5 | 13,
    RH850_SR_FEIC = 0 << 5 | 14,
    RH850_SR_CTPC = 0 << 5 | 16,
    RH850_SR_CTPSW = 0 << 5 | 17,
    RH850_SR_CTBP = 0 << 5 | 20,
    RH850_SR_RBASE = 1 << 5 | 2,
    RH850_SR_EBASE = 1 << 5 | 3,
    RH850_SR_MCTL = 1 << 5 | 5,
    RH850_SR_SVLOCK = 1 << 5 | 8,
    RH850_SR_SCCFG = 1 << 5 | 11,
    RH850_SR_SCBP = 1 << 5 | 12,
    RH850_SR_INTBP = 1 << 5 | 4,
    RH850_SR_MEA = 2 << 5 | 6,
    RH850_SR_MEI = 2 << 5 | 8,
    RH850_SR_ISPR = 2 << 5 | 10,
    RH850_SR_ICSR = 2 << 5 | 12,
    RH850_SR_PLMR = 2 << 5 | 14,
    RH850_SR_RBCR1 = 2 << 5 | 16,
    RH850_SR_RBCR0 = 2 << 5 | 15,
    RH850_SR_RBNR = 2 << 5 | 17,
    RH850_SR_RBIP = 2 << 5 | 18,
    RH850_SR_INTCFG = 2 << 5 | 13,
    RH850_SR_MPCFG = 5 << 5 | 2,
    RH850_SR_MPIDX = 5 << 5 | 16,
    RH850_SR_PMUMCTRL = 11 << 5 | 8,
};

#define RH850_SR(env, n)    ((env)->sr[(n) >> 5][(n) & 31])

#define RH850_MCTL_UIC      (1u << 0)
#define RH850_MCTL_MA       (1u << 1)
/* RBASE/EBASE.RINT and DV, INTCFG.ISPC */
#define RH850_BASE_RINT     (1u << 0)
#define RH850_BASE_DV       (1u << 1)
#define RH850_INTCFG_ISPC   (1u << 0)

void rh850_sysreg_reset(CPURH850State *env);
uint32_t rh850_sysreg_wmask(unsigned n);
bool rh850_sysreg_get(CPURH850State *env, unsigned sel, unsigned id,
                      uint32_t *val);
bool rh850_sysreg_set(CPURH850State *env, unsigned sel, unsigned id,
                      uint32_t val);

#endif
