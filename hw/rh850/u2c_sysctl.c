/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850/U2C clock and standby controller register stub: registers keep
 * their values, clocks are fixed machine properties, nothing is gated.
 * Of the reset controller only the software system reset is modelled.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/runstate.h"
#include "hw/rh850/u2c_sysctl.h"
#include "migration/vmstate.h"

#include "u2c_sysctl_regs.c.inc"

QEMU_BUILD_BUG_ON(ARRAY_SIZE(rh850_u2c_sysctl_regs) !=
                  RH850_U2C_SYSCTL_NREGS);

#define KCPROT_ENABLE   0xa5a5a501u
#define KCPROT_DISABLE  0xa5a5a500u

/* Reset controller registers (HW UM 9.4) */
#define RES_RESFC       0x0c00
#define RES_RESKCPROT0  0x0f00
#define RES_SWSRESA     0x840c
#define RES_RESF        0x8500
/* RESF after Power On Reset: PRESF0 and SRES1F1 */
#define RESF_POR        0x00000005u
#define RESF_SRES2F0    0x00000010u
#define RESFC_MASK      0x0000f5ddu

/* Offsets of the key registers, by SC_KEY_* */
static const uint16_t sysctl_key_offset[SC_NKEYS] = {
    [SC_KEY_CLKKCPROT1] = 0x0700,
    [SC_KEY_CLKKCPROT4] = 0x0730,
    [SC_KEY_MSRKCPROT] = 0x1710,
    [SC_KEY_STBCKCPROT] = 0x1700,
};

static int sysctl_find(hwaddr addr)
{
    int i;

    for (i = 0; i < RH850_U2C_SYSCTL_NREGS; i++) {
        if (rh850_u2c_sysctl_regs[i].offset == addr) {
            return i;
        }
    }
    return -1;
}

/* Reset controller access: true if addr is one of its registers */
static bool sysctl_reset_read(RH850U2CSysctlState *s, hwaddr addr,
                              uint64_t *val)
{
    switch (addr) {
    case RES_RESKCPROT0:
        *val = s->reskce;
        return true;
    case RES_RESF:
        *val = s->resf;
        return true;
    case RES_RESFC:
    case RES_SWSRESA:
        *val = 0;
        return true;
    }
    return false;
}

static bool sysctl_reset_write(RH850U2CSysctlState *s, hwaddr addr,
                               uint64_t val)
{
    switch (addr) {
    case RES_RESKCPROT0:
        if (val == KCPROT_ENABLE || val == KCPROT_DISABLE) {
            s->reskce = val & 1;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "rh850_u2c_sysctl: key register "
                          "0x%" HWADDR_PRIx " = 0x%" PRIx64 " ignored\n",
                          addr, val);
        }
        return true;
    case RES_RESFC:
        s->resf &= ~(val & RESFC_MASK);
        return true;
    case RES_SWSRESA:
        if (!s->reskce) {
            qemu_log_mask(LOG_GUEST_ERROR, "rh850_u2c_sysctl: protected "
                          "write to 0x%" HWADDR_PRIx " ignored\n", addr);
        } else if (val & 1) {
            /* System Reset 2; RESF is kept, only Power On Reset clears it */
            s->resf |= RESF_SRES2F0;
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        return true;
    case RES_RESF:
        qemu_log_mask(LOG_GUEST_ERROR, "rh850_u2c_sysctl: write to read-only "
                      "0x%" HWADDR_PRIx "\n", addr);
        return true;
    }
    return false;
}

static bool sysctl_unlocked(RH850U2CSysctlState *s, int key)
{
    return key == SC_KEY_NONE || s->val[sysctl_find(sysctl_key_offset[key])];
}

static uint64_t sysctl_read(void *opaque, hwaddr addr, unsigned size)
{
    RH850U2CSysctlState *s = opaque;
    int i = sysctl_find(addr);
    uint64_t val;

    if (size == 4 && sysctl_reset_read(s, addr, &val)) {
        return val;
    }
    if (i < 0) {
        qemu_log_mask(LOG_UNIMP, "rh850_u2c_sysctl: read of 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "rh850_u2c_sysctl: %u-byte read at "
                      "0x%" HWADDR_PRIx "\n", size, addr);
        return 0;
    }
    return rh850_u2c_sysctl_regs[i].kind == SC_TRIG ? 0 : s->val[i];
}

static void sysctl_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    RH850U2CSysctlState *s = opaque;
    int i = sysctl_find(addr);
    const RH850U2CSysctlReg *r;

    if (size == 4 && sysctl_reset_write(s, addr, val)) {
        return;
    }
    if (i < 0) {
        qemu_log_mask(LOG_UNIMP, "rh850_u2c_sysctl: write of 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", addr, val);
        return;
    }
    r = &rh850_u2c_sysctl_regs[i];
    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "rh850_u2c_sysctl: %u-byte write at "
                      "0x%" HWADDR_PRIx "\n", size, addr);
        return;
    }
    switch (r->kind) {
    case SC_KEY:
        if (val == KCPROT_ENABLE || val == KCPROT_DISABLE) {
            s->val[i] = val & 1;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "rh850_u2c_sysctl: key register "
                          "0x%" HWADDR_PRIx " = 0x%" PRIx64 " ignored\n",
                          addr, val);
        }
        return;
    case SC_RO:
        qemu_log_mask(LOG_GUEST_ERROR, "rh850_u2c_sysctl: write to read-only "
                      "0x%" HWADDR_PRIx "\n", addr);
        return;
    }
    if (!sysctl_unlocked(s, r->key)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rh850_u2c_sysctl: protected write to "
                      "0x%" HWADDR_PRIx " ignored\n", addr);
        return;
    }
    if (r->kind == SC_TRIG) {
        /* bit 0 enables (status enabled + stable), bit 1 stops */
        if ((val & 3) == 1) {
            s->val[r->mirror] = 3;
        } else if ((val & 3) == 2) {
            s->val[r->mirror] = 0;
        }
        return;
    }
    s->val[i] = (s->val[i] & ~r->wmask) | (val & r->wmask);
    if (r->mirror >= 0) {
        s->val[r->mirror] = s->val[i];
    }
}

static const MemoryRegionOps sysctl_ops = {
    .read = sysctl_read,
    .write = sysctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void sysctl_reset_hold(Object *obj, ResetType type)
{
    RH850U2CSysctlState *s = RH850_U2C_SYSCTL(obj);
    int i;

    for (i = 0; i < RH850_U2C_SYSCTL_NREGS; i++) {
        s->val[i] = rh850_u2c_sysctl_regs[i].reset;
    }
    s->reskce = 0;
}

static void sysctl_init(Object *obj)
{
    RH850U2CSysctlState *s = RH850_U2C_SYSCTL(obj);

    memory_region_init_io(&s->iomem, obj, &sysctl_ops, s,
                          "rh850-u2c-sysctl", RH850_U2C_SYSCTL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    /* Object creation is the Power On Reset; system resets keep RESF */
    s->resf = RESF_POR;
}

static const VMStateDescription vmstate_sysctl = {
    .name = TYPE_RH850_U2C_SYSCTL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(val, RH850U2CSysctlState,
                             RH850_U2C_SYSCTL_NREGS),
        VMSTATE_UINT32(reskce, RH850U2CSysctlState),
        VMSTATE_UINT32(resf, RH850U2CSysctlState),
        VMSTATE_END_OF_LIST()
    }
};

static void sysctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_sysctl;
    rc->phases.hold = sysctl_reset_hold;
}

static const TypeInfo sysctl_types[] = {
    {
        .name           = TYPE_RH850_U2C_SYSCTL,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(RH850U2CSysctlState),
        .instance_init  = sysctl_init,
        .class_init     = sysctl_class_init,
    },
};

DEFINE_TYPES(sysctl_types)
