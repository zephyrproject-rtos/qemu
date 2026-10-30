/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850/U2C clock controller and standby controller register stub
 */

#ifndef HW_RH850_U2C_SYSCTL_H
#define HW_RH850_U2C_SYSCTL_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RH850_U2C_SYSCTL "rh850-u2c-sysctl"
OBJECT_DECLARE_SIMPLE_TYPE(RH850U2CSysctlState, RH850_U2C_SYSCTL)

#define RH850_U2C_SYSCTL_BASE   0xff980000
#define RH850_U2C_SYSCTL_SIZE   0x10000
#define RH850_U2C_SYSCTL_NREGS  132

/* Register kinds */
enum {
    SC_RW,      /* read/write storage */
    SC_RO,      /* status, read-only */
    SC_TRIG,    /* enable/disable trigger, write-only, reads 0 */
    SC_KEY,     /* key code protection register */
};

/* Key code protection register that guards a register */
enum {
    SC_KEY_NONE,
    SC_KEY_CLKKCPROT1,
    SC_KEY_CLKKCPROT4,
    SC_KEY_MSRKCPROT,
    SC_KEY_STBCKCPROT,
    SC_NKEYS,
};

typedef struct RH850U2CSysctlReg {
    uint16_t offset;
    uint32_t reset;
    uint32_t wmask;
    uint8_t kind;
    uint8_t key;
    /* index of the status register this one is mirrored to, or -1 */
    int16_t mirror;
} RH850U2CSysctlReg;

struct RH850U2CSysctlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t val[RH850_U2C_SYSCTL_NREGS];
    /* Reset controller: RESKCPROT0.KCE and RESF */
    uint32_t reskce;
    uint32_t resf;
};

#endif
