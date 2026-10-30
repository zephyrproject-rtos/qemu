/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 interrupt controller: INTC1 of PE0, INTC2, FENMI and FEINT of
 * PE0. Only PE0 is modelled; broadcast, IPI and the guards are not.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/intc/rh850_intc.h"
#include "migration/vmstate.h"

#include "rh850_u2c8_level.c.inc"

#define INTC1_IMR0      0x0f0
#define INTC1_EIBD      0x100
#define INTC1_FIBD      0x1c0
#define INTC1_EEIC      0x200
#define INTC2_IMR       0x1000
#define INTC2_EIBD      0x2000
#define INTC2_EEIC      0x4000
#define INTC2_NIMR      20

#define FENC_FENMIF     0x0
#define FENC_FENMIC     0x8
#define FEINC_FEINTF    0x0
#define FEINC_FEINTMSK  0x4
#define FEINC_FEINTC    0x8
#define FEINT_BITS      0x7

/* EIBDn of INTC2: CST (edge channels only), BCP, PEID */
#define EIBD_CST        (1u << 31)
#define EIBD_WMASK      0x03000007u
#define EIBD_PEID       0x7u

static bool rh850_intc_to_pe0(RH850INTCState *s, int ch)
{
    return ch < RH850_INTC1_NCH || !(s->eibd[ch] & EIBD_PEID);
}

/*
 * Highest priority request: lowest EIP, then lowest channel.
 * EIMK = 1 keeps EIRF but gives no request
 */
static void rh850_intc_update(RH850INTCState *s)
{
    int ch, best = -1;
    uint32_t req = 0;

    for (ch = 0; ch < RH850_INTC_NCH; ch++) {
        if (s->eirf[ch] && !s->eimk[ch] && rh850_intc_to_pe0(s, ch) &&
            (best < 0 || s->eip[ch] < s->eip[best])) {
            best = ch;
        }
    }
    if (best >= 0) {
        req = RH850_INTC_REQ_VALID | s->eitb[best] << RH850_INTC_REQ_TB |
              s->eip[best] << RH850_INTC_REQ_PRIO | best;
    }
    qemu_set_irq(s->out[RH850_INTC_OUT_EIINT], req);
    qemu_set_irq(s->out[RH850_INTC_OUT_FEINT],
                 !!(s->feintf & ~s->feintmsk & FEINT_BITS));
    qemu_set_irq(s->out[RH850_INTC_OUT_FENMI], s->fenmi_req);
}

/*
 * Edge channel: an edge while EIRF = 1 sets EIOV. Level channel:
 * EIRF follows the input
 */
static void rh850_intc_set_irq(void *opaque, int ch, int level)
{
    RH850INTCState *s = opaque;

    level = !!level;
    if (s->level[ch]) {
        s->eirf[ch] = level;
    } else if (level && !s->in[ch]) {
        if (s->eirf[ch]) {
            s->eiov[ch] = 1;
        }
        s->eirf[ch] = 1;
    }
    s->in[ch] = level;
    rh850_intc_update(s);
}

/* FENMIF is set on the NMI edge and cleared only by FENMIC */
static void rh850_intc_set_fenmi(void *opaque, int n, int level)
{
    RH850INTCState *s = opaque;

    if (level && !s->fenmi_in) {
        s->fenmif = 1;
        s->fenmi_req = 1;
    }
    s->fenmi_in = !!level;
    rh850_intc_update(s);
}

/*
 * FEINT sources (level): 0 SWDT, 1 TPTM, 2 ECM. FEINTF is cleared by
 * FEINTC and set again while the source stays active
 */
static void rh850_intc_set_feint(void *opaque, int n, int level)
{
    RH850INTCState *s = opaque;

    s->feint_in = deposit32(s->feint_in, n, 1, !!level);
    s->feintf |= s->feint_in;
    rh850_intc_update(s);
}

/*
 * Accepting an EIINT clears EIRF of an edge channel, accepting FENMI
 * drops the held request; end of interrupt is not modelled
 */
static void rh850_intc_ack(void *opaque, int n, int level)
{
    RH850INTCState *s = opaque;

    switch (n) {
    case RH850_INTC_ACK_EIINT:
        if (level >= 0 && level < RH850_INTC_NCH && !s->level[level]) {
            s->eirf[level] = 0;
        }
        break;
    case RH850_INTC_ACK_FENMI:
        s->fenmi_req = 0;
        break;
    default:
        return;
    }
    rh850_intc_update(s);
}

static uint32_t rh850_eic_read(RH850INTCState *s, int ch)
{
    return s->level[ch] << 15 | s->eirf[ch] << 12 | s->eimk[ch] << 7 |
           s->eitb[ch] << 6 | s->eiov[ch] << 5 | s->eip[ch];
}

static uint32_t rh850_eeic_read(RH850INTCState *s, int ch)
{
    return (uint32_t)s->level[ch] << 31 | s->eirf[ch] << 28 |
           s->eimk[ch] << 23 | s->eitb[ch] << 22 | s->eiov[ch] << 15 |
           s->eip[ch];
}

/* One byte of EICn: byte 0 EIMK/EITB/EIOV/EIP, byte 1 EIRF (edge only) */
static void rh850_eic_write(RH850INTCState *s, int ch, int lane, uint8_t v)
{
    if (lane == 0) {
        s->eimk[ch] = extract32(v, 7, 1);
        s->eitb[ch] = extract32(v, 6, 1);
        s->eiov[ch] = extract32(v, 5, 1);
        s->eip[ch] = extract32(v, 0, 4);
    } else if (!s->level[ch]) {
        s->eirf[ch] = extract32(v, 4, 1);
    }
}

/*
 * One byte of EEICn: byte 0 EIP, byte 1 EIOV, byte 2 EIMK/EITB,
 * byte 3 EIRF (edge only)
 */
static void rh850_eeic_write(RH850INTCState *s, int ch, int lane, uint8_t v)
{
    switch (lane) {
    case 0:
        s->eip[ch] = extract32(v, 0, 4);
        break;
    case 1:
        s->eiov[ch] = extract32(v, 7, 1);
        break;
    case 2:
        s->eimk[ch] = extract32(v, 7, 1);
        s->eitb[ch] = extract32(v, 6, 1);
        break;
    default:
        if (!s->level[ch]) {
            s->eirf[ch] = extract32(v, 4, 1);
        }
        break;
    }
}

/* IMRm bit k = EIMK of channel 32m + k; absent channels read 1 */
static uint32_t rh850_imr_read(RH850INTCState *s, int m)
{
    uint32_t v = 0;
    int k, ch;

    for (k = 0; k < 32; k++) {
        ch = m * 32 + k;
        v |= (uint32_t)(ch < RH850_INTC_NCH ? s->eimk[ch] : 1) << k;
    }
    return v;
}

/* A write from PE0 changes EIMK only of channels bound to PE0 */
static void rh850_imr_write(RH850INTCState *s, int m, int lane, uint8_t v)
{
    int k, ch;

    for (k = 0; k < 8; k++) {
        ch = m * 32 + lane * 8 + k;
        if (ch < RH850_INTC_NCH && rh850_intc_to_pe0(s, ch)) {
            s->eimk[ch] = extract32(v, k, 1);
        }
    }
}

static uint32_t rh850_eibd_wmask(RH850INTCState *s, int ch)
{
    return EIBD_WMASK | (s->level[ch] ? 0 : EIBD_CST);
}

/*
 * Registers are handled one byte at a time; decode returns the
 * register kind and the channel or IMR number
 */
enum { R_NONE, R_EIC, R_EEIC, R_IMR, R_EIBD, R_RO0 };

static int rh850_intc_decode(hwaddr addr, bool intc2, int *idx)
{
    int lo = intc2 ? RH850_INTC1_NCH : 0;
    int hi = intc2 ? RH850_INTC_NCH : RH850_INTC1_NCH;
    hwaddr eibd = intc2 ? INTC2_EIBD : INTC1_EIBD;
    hwaddr eeic = intc2 ? INTC2_EEIC : INTC1_EEIC;

    if (addr >= 2 * lo && addr < 2 * hi) {
        *idx = addr / 2;
        return R_EIC;
    }
    if (addr >= eeic + 4 * lo && addr < eeic + 4 * hi) {
        *idx = (addr - eeic) / 4;
        return R_EEIC;
    }
    if (addr >= eibd + 4 * lo && addr < eibd + 4 * hi) {
        *idx = (addr - eibd) / 4;
        return intc2 ? R_EIBD : R_RO0;
    }
    if (!intc2 && (addr & ~3) == INTC1_IMR0) {
        *idx = 0;
        return R_IMR;
    }
    if (!intc2 && (addr & ~3) == INTC1_FIBD) {
        return R_RO0;
    }
    if (intc2 && addr >= INTC2_IMR + 4 && addr < INTC2_IMR + 4 * INTC2_NIMR) {
        *idx = (addr - INTC2_IMR) / 4;
        return R_IMR;
    }
    return R_NONE;
}

static uint64_t rh850_intc_read(RH850INTCState *s, hwaddr addr, bool intc2)
{
    int idx = 0;
    uint32_t v;

    switch (rh850_intc_decode(addr, intc2, &idx)) {
    case R_EIC:
        return extract32(rh850_eic_read(s, idx), (addr & 1) * 8, 8);
    case R_EEIC:
        v = rh850_eeic_read(s, idx);
        break;
    case R_IMR:
        v = rh850_imr_read(s, idx);
        break;
    case R_EIBD:
        v = s->eibd[idx];
        break;
    case R_RO0:
        /* EIBD0-31 and FIBD: PEID fixed to PE0 */
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rh850_intc: read of reserved "
                      "INTC%d offset 0x%" HWADDR_PRIx "\n", intc2 + 1, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, 8);
}

static void rh850_intc_write(RH850INTCState *s, hwaddr addr, uint8_t v,
                             bool intc2)
{
    int idx = 0;
    int lane = addr & 3;
    uint32_t m;

    switch (rh850_intc_decode(addr, intc2, &idx)) {
    case R_EIC:
        rh850_eic_write(s, idx, addr & 1, v);
        break;
    case R_EEIC:
        rh850_eeic_write(s, idx, lane, v);
        break;
    case R_IMR:
        rh850_imr_write(s, idx, lane, v);
        break;
    case R_EIBD:
        m = rh850_eibd_wmask(s, idx) & (0xffu << (lane * 8));
        s->eibd[idx] = (s->eibd[idx] & ~m) | (((uint32_t)v << lane * 8) & m);
        break;
    case R_RO0:
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rh850_intc: write of reserved "
                      "INTC%d offset 0x%" HWADDR_PRIx "\n", intc2 + 1, addr);
        return;
    }
    rh850_intc_update(s);
}

static uint64_t rh850_intc1_read(void *opaque, hwaddr addr, unsigned size)
{
    return rh850_intc_read(opaque, addr, false);
}

static void rh850_intc1_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    rh850_intc_write(opaque, addr, val, false);
}

static uint64_t rh850_intc2_read(void *opaque, hwaddr addr, unsigned size)
{
    return rh850_intc_read(opaque, addr, true);
}

static void rh850_intc2_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    rh850_intc_write(opaque, addr, val, true);
}

static const MemoryRegionOps rh850_intc1_ops = {
    .read = rh850_intc1_read,
    .write = rh850_intc1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static const MemoryRegionOps rh850_intc2_ops = {
    .read = rh850_intc2_read,
    .write = rh850_intc2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

/* FENC: FENMIF (R) and FENMIC (W), 32-bit */
static uint64_t rh850_fenc_read(void *opaque, hwaddr addr, unsigned size)
{
    RH850INTCState *s = opaque;

    return addr == FENC_FENMIF ? s->fenmif : 0;
}

static void rh850_fenc_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    RH850INTCState *s = opaque;

    if (addr == FENC_FENMIC && (val & 1)) {
        s->fenmif = 0;
    }
}

/* FEINC of PE0: FEINTF (R), FEINTMSK (R/W), FEINTC (W), 32-bit */
static uint64_t rh850_feinc_read(void *opaque, hwaddr addr, unsigned size)
{
    RH850INTCState *s = opaque;

    switch (addr) {
    case FEINC_FEINTF:
        return s->feintf;
    case FEINC_FEINTMSK:
        return s->feintmsk;
    default:
        return 0;
    }
}

static void rh850_feinc_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    RH850INTCState *s = opaque;

    switch (addr) {
    case FEINC_FEINTMSK:
        s->feintmsk = val & FEINT_BITS;
        break;
    case FEINC_FEINTC:
        s->feintf = (s->feintf & ~val) | s->feint_in;
        break;
    default:
        return;
    }
    rh850_intc_update(s);
}

static const MemoryRegionOps rh850_fenc_ops = {
    .read = rh850_fenc_read,
    .write = rh850_fenc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps rh850_feinc_ops = {
    .read = rh850_feinc_read,
    .write = rh850_feinc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void rh850_intc_reset_hold(Object *obj, ResetType type)
{
    RH850INTCState *s = RH850_INTC(obj);
    int ch;

    for (ch = 0; ch < RH850_INTC_NCH; ch++) {
        s->eirf[ch] = s->level[ch] ? s->in[ch] : 0;
        s->eimk[ch] = 1;
        s->eitb[ch] = 0;
        s->eiov[ch] = 0;
        s->eip[ch] = 0xf;
        s->eibd[ch] = 0;
    }
    s->fenmif = 0;
    s->fenmi_req = 0;
    s->feintf = s->feint_in;
    s->feintmsk = FEINT_BITS;
}

static void rh850_intc_reset_exit(Object *obj, ResetType type)
{
    rh850_intc_update(RH850_INTC(obj));
}

static void rh850_intc_init(Object *obj)
{
    RH850INTCState *s = RH850_INTC(obj);
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    size_t i;
    int ch;

    for (i = 0; i < ARRAY_SIZE(rh850_u2c8_level); i++) {
        for (ch = rh850_u2c8_level[i][0]; ch <= rh850_u2c8_level[i][1];
             ch++) {
            s->level[ch] = 1;
        }
    }
    memory_region_init_io(&s->intc1, obj, &rh850_intc1_ops, s,
                          "rh850-intc1", RH850_INTC1_SIZE);
    memory_region_init_io(&s->intc2, obj, &rh850_intc2_ops, s,
                          "rh850-intc2", RH850_INTC2_SIZE);
    memory_region_init_io(&s->fenc, obj, &rh850_fenc_ops, s,
                          "rh850-fenc", RH850_INTC_FE_SIZE);
    memory_region_init_io(&s->feinc, obj, &rh850_feinc_ops, s,
                          "rh850-feinc", RH850_INTC_FE_SIZE);
    sysbus_init_mmio(d, &s->intc1);
    sysbus_init_mmio(d, &s->intc2);
    sysbus_init_mmio(d, &s->fenc);
    sysbus_init_mmio(d, &s->feinc);
    for (i = 0; i < RH850_INTC_OUT_NUM; i++) {
        sysbus_init_irq(d, &s->out[i]);
    }
    qdev_init_gpio_in(DEVICE(obj), rh850_intc_set_irq, RH850_INTC_NCH);
    qdev_init_gpio_in_named(DEVICE(obj), rh850_intc_set_fenmi, "fenmi", 1);
    qdev_init_gpio_in_named(DEVICE(obj), rh850_intc_set_feint, "feint",
                            RH850_INTC_NFEINT);
    qdev_init_gpio_in_named(DEVICE(obj), rh850_intc_ack, "ack",
                            RH850_INTC_ACK_NUM);
}

static const VMStateDescription vmstate_rh850_intc = {
    .name = TYPE_RH850_INTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(eirf, RH850INTCState, RH850_INTC_NCH),
        VMSTATE_UINT8_ARRAY(eimk, RH850INTCState, RH850_INTC_NCH),
        VMSTATE_UINT8_ARRAY(eitb, RH850INTCState, RH850_INTC_NCH),
        VMSTATE_UINT8_ARRAY(eiov, RH850INTCState, RH850_INTC_NCH),
        VMSTATE_UINT8_ARRAY(eip, RH850INTCState, RH850_INTC_NCH),
        VMSTATE_UINT32_ARRAY(eibd, RH850INTCState, RH850_INTC_NCH),
        VMSTATE_UINT8_ARRAY(in, RH850INTCState, RH850_INTC_NCH),
        VMSTATE_UINT32(fenmif, RH850INTCState),
        VMSTATE_UINT32(fenmi_req, RH850INTCState),
        VMSTATE_UINT32(fenmi_in, RH850INTCState),
        VMSTATE_UINT32(feintf, RH850INTCState),
        VMSTATE_UINT32(feintmsk, RH850INTCState),
        VMSTATE_UINT32(feint_in, RH850INTCState),
        VMSTATE_END_OF_LIST()
    }
};

static void rh850_intc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_rh850_intc;
    rc->phases.hold = rh850_intc_reset_hold;
    rc->phases.exit = rh850_intc_reset_exit;
}

static const TypeInfo rh850_intc_types[] = {
    {
        .name           = TYPE_RH850_INTC,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(RH850INTCState),
        .instance_init  = rh850_intc_init,
        .class_init     = rh850_intc_class_init,
    },
};

DEFINE_TYPES(rh850_intc_types)
