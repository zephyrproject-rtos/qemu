/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * Renesas RLIN3 LIN/UART interface, UART mode only. Bytes move at once;
 * baud rate, line errors and LIN mode are not modelled.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "chardev/char-fe.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/char/renesas_rlin3.h"
#include "migration/vmstate.h"

#define LWBR        0x01
#define LBRP0       0x02
#define LBRP1       0x03
#define LSTC        0x04
#define LMD         0x08
#define LBFC        0x09
#define LSC         0x0a
#define LWUP        0x0b
#define LIE         0x0c
#define LEDE        0x0d
#define LCUC        0x0e
#define LTRC        0x10
#define LMST        0x11
#define LST         0x12
#define LEST        0x13
#define LDFC        0x14
#define LIDB        0x15
#define LCBR        0x16
#define LUDB0       0x17
#define LDBR1       0x18
#define LDBR8       0x1f
#define LUOER       0x20
#define LUOR1       0x21
#define LUTDR       0x24
#define LURDR       0x26
#define LUWTDR      0x28
#define LBSS        0x30
#define LRSS        0x34

#define LMD_UART    0x01
#define LBFC_UBLS   0x01
#define LCUC_OM0    0x01
#define LTRC_RTS    0x02
#define LMST_OMM0   0x01
#define LST_URS     0x20
#define LST_UTS     0x10
#define LST_ERR     0x08
#define LST_FTC     0x01
#define LEST_FLAGS  0x7d
#define LUOER_UROE  0x02
#define LUOER_UTOE  0x01
#define LUOR1_UEBE  0x01

/* Writable bits per register; 0 marks a reserved offset */
static const uint8_t rlin3_wmask[RENESAS_RLIN3_SIZE] = {
    [LWBR] = 0xfe, [LBRP0] = 0xff, [LBRP1] = 0xff, [LSTC] = 0xff,
    [LMD] = 0xff, [LBFC] = 0xff, [LSC] = 0xff, [LWUP] = 0xff, [LIE] = 0xff,
    [LEDE] = 0xff, [LCUC] = 0xff, [LDFC] = 0xff, [LIDB] = 0xff,
    [LCBR] = 0xff, [LUDB0] = 0xff,
    [LDBR1] = 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    [LUOR1] = 0xff, [LUTDR] = 0xff, [LUTDR + 1] = 0x01,
    [LUWTDR] = 0xff, [LUWTDR + 1] = 0x01, [LBSS] = 0xff, [LRSS] = 0xff,
};

static bool rlin3_running(RenesasRLIN3State *s)
{
    return s->regs[LMST] & LMST_OMM0;
}

static void rlin3_pulse(RenesasRLIN3State *s, int n)
{
    qemu_irq_pulse(s->irq[n]);
}

/* ERR is set while any LEST flag is set */
static void rlin3_update_err(RenesasRLIN3State *s)
{
    if (s->regs[LEST] & LEST_FLAGS) {
        s->regs[LST] |= LST_ERR;
    } else {
        s->regs[LST] &= ~LST_ERR;
    }
}

static void rlin3_send(RenesasRLIN3State *s, uint8_t c)
{
    /* 7-bit frames without the expansion bit carry bits 6-0 */
    if ((s->regs[LBFC] & LBFC_UBLS) && !(s->regs[LUOR1] & LUOR1_UEBE)) {
        c &= 0x7f;
    }
    qemu_chr_fe_write_all(&s->chr, &c, 1);
}

/*
 * A LUTDR write with UTOE = 1 starts a transmission; UR0 comes at
 * once whatever UTIGTS selects
 */
static void rlin3_tx_single(RenesasRLIN3State *s)
{
    if (!rlin3_running(s) || !(s->regs[LUOER] & LUOER_UTOE)) {
        return;
    }
    rlin3_send(s, s->regs[LUTDR]);
    rlin3_pulse(s, RENESAS_RLIN3_IRQ_UR0);
}

/*
 * UART buffer: LUDB0 goes first, then LDBR1 onward, MDL bytes in all
 * (0 means 9)
 */
static void rlin3_tx_buffer(RenesasRLIN3State *s)
{
    int mdl = s->regs[LDFC] & 0xf;
    int i;

    if (!mdl) {
        mdl = 9;
    }
    if (mdl > 9) {
        qemu_log_mask(LOG_GUEST_ERROR, "renesas_rlin3: MDL %d prohibited\n",
                      mdl);
        return;
    }
    if (mdl == 9) {
        rlin3_send(s, s->regs[LUDB0]);
        mdl = 8;
    }
    for (i = 0; i < mdl; i++) {
        rlin3_send(s, s->regs[LDBR1 + i]);
    }
    s->regs[LST] |= LST_FTC;
    rlin3_pulse(s, RENESAS_RLIN3_IRQ_UR0);
}

/* Entering LIN reset mode clears LTRC, LST, LEST and LUOER */
static void rlin3_set_mode(RenesasRLIN3State *s)
{
    if (s->regs[LCUC] & LCUC_OM0) {
        if ((s->regs[LMD] & 3) != LMD_UART) {
            qemu_log_mask(LOG_UNIMP, "renesas_rlin3: LMD %d (LIN mode) "
                          "not modelled\n", s->regs[LMD] & 3);
        }
        s->regs[LMST] |= LMST_OMM0;
    } else {
        s->regs[LMST] &= ~LMST_OMM0;
        s->regs[LTRC] = 0;
        s->regs[LST] = 0;
        s->regs[LEST] = 0;
        s->regs[LUOER] = 0;
    }
    qemu_chr_fe_accept_input(&s->chr);
}

/*
 * LBRP01, LUTDR and LUWTDR take 16-bit or 8-bit accesses, LURDR
 * 16-bit reads, the others 8-bit
 */
static bool rlin3_size_ok(hwaddr addr, unsigned size)
{
    switch (addr) {
    case LBRP0:
    case LUTDR:
    case LURDR:
    case LUWTDR:
        return size == 2 || (size == 1 && addr != LURDR);
    default:
        return size == 1;
    }
}

static uint64_t rlin3_read(void *opaque, hwaddr addr, unsigned size)
{
    RenesasRLIN3State *s = opaque;

    if (!rlin3_size_ok(addr, size)) {
        qemu_log_mask(LOG_GUEST_ERROR, "renesas_rlin3: %u-byte read at "
                      "0x%" HWADDR_PRIx "\n", size, addr);
        return 0;
    }
    switch (addr) {
    case LURDR: {
        uint16_t v = lduw_le_p(&s->regs[LURDR]);

        /*
         * Take the value before accept_input: a mux front end delivers the
         * next byte into LURDR at once
         */
        if (s->rx_full) {
            s->rx_full = 0;
            qemu_chr_fe_accept_input(&s->chr);
        }
        return v;
    }
    case LUWTDR:
    case LUWTDR + 1:
        /* LUWTDR reads return LUTDR */
        addr -= LUWTDR - LUTDR;
        break;
    case LST:
    case LEST:
    case LMST:
    case LTRC:
    case LUOER:
        break;
    default:
        if (!rlin3_wmask[addr]) {
            qemu_log_mask(LOG_GUEST_ERROR, "renesas_rlin3: read of reserved "
                          "offset 0x%" HWADDR_PRIx "\n", addr);
            return 0;
        }
        break;
    }
    return size == 2 ? lduw_le_p(&s->regs[addr]) : s->regs[addr];
}

static void rlin3_write_byte(RenesasRLIN3State *s, hwaddr addr, uint8_t v)
{
    switch (addr) {
    case LCUC:
        s->regs[LCUC] = v;
        rlin3_set_mode(s);
        break;
    case LST:
        /* Not writable in LIN reset mode; FTC: 0 clears, 1 keeps */
        if (rlin3_running(s)) {
            s->regs[LST] &= v | ~LST_FTC;
        }
        break;
    case LEST:
        /* Not writable in LIN reset mode; 0 clears, 1 keeps */
        if (rlin3_running(s)) {
            s->regs[LEST] &= v | ~LEST_FLAGS;
            rlin3_update_err(s);
        }
        break;
    case LUOER:
        /* Not writable in LIN reset mode */
        if (rlin3_running(s)) {
            s->regs[LUOER] = v & (LUOER_UROE | LUOER_UTOE);
            qemu_chr_fe_accept_input(&s->chr);
        }
        break;
    case LTRC:
        /* RTS: only 1 is written, not in LIN reset mode */
        if (rlin3_running(s) && (v & LTRC_RTS) &&
            (s->regs[LUOER] & LUOER_UTOE)) {
            rlin3_tx_buffer(s);
        }
        break;
    case LMST:
    case LURDR:
    case LURDR + 1:
        qemu_log_mask(LOG_GUEST_ERROR, "renesas_rlin3: write to read-only "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    default:
        if (!rlin3_wmask[addr]) {
            qemu_log_mask(LOG_GUEST_ERROR, "renesas_rlin3: write of reserved "
                          "offset 0x%" HWADDR_PRIx "\n", addr);
            break;
        }
        s->regs[addr] = v & rlin3_wmask[addr];
        break;
    }
}

static void rlin3_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
    RenesasRLIN3State *s = opaque;

    if (!rlin3_size_ok(addr, size)) {
        qemu_log_mask(LOG_GUEST_ERROR, "renesas_rlin3: %u-byte write at "
                      "0x%" HWADDR_PRIx "\n", size, addr);
        return;
    }
    rlin3_write_byte(s, addr, val);
    if (size == 2) {
        rlin3_write_byte(s, addr + 1, val >> 8);
    }
    if (addr == LUTDR) {
        rlin3_tx_single(s);
    }
}

static const MemoryRegionOps rlin3_ops = {
    .read = rlin3_read,
    .write = rlin3_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
};

/* One byte at a time, while UROE = 1 and the last byte has been read */
static int rlin3_can_receive(void *opaque)
{
    RenesasRLIN3State *s = opaque;

    return rlin3_running(s) && (s->regs[LUOER] & LUOER_UROE) &&
           !s->rx_full;
}

/* The byte reaches LURDR at the stop bit and raises UR1 */
static void rlin3_receive(void *opaque, const uint8_t *buf, int size)
{
    RenesasRLIN3State *s = opaque;

    stw_le_p(&s->regs[LURDR], buf[0]);
    s->rx_full = 1;
    rlin3_pulse(s, RENESAS_RLIN3_IRQ_UR1);
}

static void rlin3_reset_hold(Object *obj, ResetType type)
{
    RenesasRLIN3State *s = RENESAS_RLIN3(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->rx_full = 0;
}

static void rlin3_realize(DeviceState *dev, Error **errp)
{
    RenesasRLIN3State *s = RENESAS_RLIN3(dev);

    qemu_chr_fe_set_handlers(&s->chr, rlin3_can_receive, rlin3_receive,
                             NULL, NULL, s, NULL, true);
}

static void rlin3_init(Object *obj)
{
    RenesasRLIN3State *s = RENESAS_RLIN3(obj);
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &rlin3_ops, s, "renesas-rlin3",
                          RENESAS_RLIN3_SIZE);
    sysbus_init_mmio(d, &s->iomem);
    for (i = 0; i < RENESAS_RLIN3_NIRQ; i++) {
        sysbus_init_irq(d, &s->irq[i]);
    }
}

static const Property rlin3_properties[] = {
    DEFINE_PROP_CHR("chardev", RenesasRLIN3State, chr),
};

static const VMStateDescription vmstate_rlin3 = {
    .name = TYPE_RENESAS_RLIN3,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, RenesasRLIN3State, RENESAS_RLIN3_SIZE),
        VMSTATE_UINT8(rx_full, RenesasRLIN3State),
        VMSTATE_END_OF_LIST()
    }
};

static void rlin3_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rlin3_realize;
    dc->vmsd = &vmstate_rlin3;
    rc->phases.hold = rlin3_reset_hold;
    device_class_set_props(dc, rlin3_properties);
}

static const TypeInfo rlin3_types[] = {
    {
        .name           = TYPE_RENESAS_RLIN3,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(RenesasRLIN3State),
        .instance_init  = rlin3_init,
        .class_init     = rlin3_class_init,
    },
};

DEFINE_TYPES(rlin3_types)
