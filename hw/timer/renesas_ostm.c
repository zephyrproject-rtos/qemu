/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * Renesas OS timer (OSTM): interval and free-run compare modes, TINT as
 * an edge pulse; the counter follows the virtual clock.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/clock.h"
#include "hw/core/qdev-clock.h"
#include "hw/timer/renesas_ostm.h"
#include "migration/vmstate.h"

#define OSTM_CMP        0x00
#define OSTM_CNT        0x04
#define OSTM_TO         0x08
#define OSTM_TOE        0x0c
#define OSTM_TE         0x10
#define OSTM_TS         0x14
#define OSTM_TT         0x18
#define OSTM_CTL        0x20

#define CTL_IE          0x80
#define CTL_MD2         0x04
#define CTL_MD1         0x02
#define CTL_MD0         0x01
#define CTL_MASK        (CTL_IE | CTL_MD2 | CTL_MD1 | CTL_MD0)

static bool ostm_freerun(RenesasOSTMState *s)
{
    return s->ctl & CTL_MD1;
}

static bool ostm_counting(RenesasOSTMState *s)
{
    return s->te && !s->tint_held && clock_is_enabled(s->clk);
}

/* Clocks from the base to the next TINT */
static uint64_t ostm_next(RenesasOSTMState *s)
{
    if (ostm_freerun(s)) {
        return (uint64_t)(uint32_t)(s->cmp - s->cnt) + 1;
    }
    return (uint64_t)s->cnt + 1;
}

static void ostm_pulse(RenesasOSTMState *s)
{
    if (s->ctl & CTL_IE) {
        qemu_irq_pulse(s->tint);
    }
}

/* Interval mode with CMP = 0: TINT stays asserted while TE = 1 */
static void ostm_hold(RenesasOSTMState *s, bool on)
{
    s->tint_held = on;
    qemu_set_irq(s->tint, on && (s->ctl & CTL_IE));
}

/* TINT at base + ostm_next(): interval reloads CMP, free-run goes on */
static void ostm_event(RenesasOSTMState *s)
{
    s->base += clock_ticks_to_ns(s->clk, ostm_next(s));
    if (ostm_freerun(s)) {
        s->cnt = s->cmp + 1;
        ostm_pulse(s);
    } else if (s->cmp) {
        s->cnt = s->cmp;
        ostm_pulse(s);
    } else {
        s->cnt = 0;
        ostm_hold(s, true);
    }
}

static void ostm_catch_up(RenesasOSTMState *s, int64_t now)
{
    while (ostm_counting(s) &&
           s->base + (int64_t)clock_ticks_to_ns(s->clk, ostm_next(s)) <= now) {
        ostm_event(s);
    }
}

/* Move the base to the last counter clock before now */
static void ostm_fold(RenesasOSTMState *s, int64_t now)
{
    uint64_t n;

    ostm_catch_up(s, now);
    if (!ostm_counting(s)) {
        return;
    }
    n = clock_ns_to_ticks(s->clk, now - s->base);
    s->cnt = ostm_freerun(s) ? s->cnt + n : s->cnt - n;
    s->base += clock_ticks_to_ns(s->clk, n);
}

static void ostm_schedule(RenesasOSTMState *s)
{
    if (!ostm_counting(s)) {
        timer_del(&s->timer);
        return;
    }
    timer_mod(&s->timer, s->base + clock_ticks_to_ns(s->clk, ostm_next(s)));
}

static void ostm_timer_cb(void *opaque)
{
    RenesasOSTMState *s = opaque;

    ostm_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    ostm_schedule(s);
}

static uint32_t ostm_read_cnt(RenesasOSTMState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    ostm_fold(s, now);
    ostm_schedule(s);
    return s->cnt;
}

/*
 * Start: MD2 = 0 or a forced restart loads CMP (interval) or 0
 * (free-run), MD2 = 1 keeps CNT; MD0 = 1 raises TINT at the start
 */
static void ostm_start(RenesasOSTMState *s, bool restart)
{
    if (restart || !(s->ctl & CTL_MD2)) {
        s->cnt = ostm_freerun(s) ? 0 : s->cmp;
    }
    s->te = 1;
    s->base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (!ostm_freerun(s) && !s->cnt && !s->cmp) {
        ostm_hold(s, true);
    } else {
        ostm_hold(s, false);
        if (s->ctl & CTL_MD0) {
            ostm_pulse(s);
        }
    }
    ostm_schedule(s);
}

/* TT: TE = 0, CNT holds its value */
static void ostm_stop(RenesasOSTMState *s)
{
    if (!s->te) {
        return;
    }
    ostm_fold(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    s->te = 0;
    ostm_hold(s, false);
    timer_del(&s->timer);
}

/* CMP and CNT take 32-bit accesses, the others 8-bit */
static bool ostm_size_ok(hwaddr addr, unsigned size)
{
    unsigned want = addr < OSTM_TO ? 4 : 1;

    if (size != want) {
        qemu_log_mask(LOG_GUEST_ERROR, "renesas_ostm: %u-byte access at "
                      "0x%" HWADDR_PRIx " (expected %u)\n", size, addr, want);
        return false;
    }
    return true;
}

static uint64_t ostm_read(void *opaque, hwaddr addr, unsigned size)
{
    RenesasOSTMState *s = opaque;

    if (!ostm_size_ok(addr, size)) {
        return 0;
    }
    switch (addr) {
    case OSTM_CMP:
        return s->cmp;
    case OSTM_CNT:
        return ostm_read_cnt(s);
    case OSTM_TO:
        return s->to;
    case OSTM_TOE:
        return s->toe;
    case OSTM_TE:
        return s->te;
    case OSTM_TS:
    case OSTM_TT:
        return 0;
    case OSTM_CTL:
        return s->ctl;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "renesas_ostm: read of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void ostm_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    RenesasOSTMState *s = opaque;
    int64_t now;

    if (!ostm_size_ok(addr, size)) {
        return;
    }
    switch (addr) {
    case OSTM_CMP:
        /* Interval: used at the next reload; free-run: compared at once */
        now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        ostm_fold(s, now);
        s->cmp = val;
        if (s->tint_held && s->cmp) {
            ostm_hold(s, false);
            s->cnt = s->cmp;
            s->base = now;
        }
        ostm_schedule(s);
        break;
    case OSTM_CNT:
        /* Writable only with TE = 0 */
        if (!s->te) {
            s->cnt = val;
        }
        break;
    case OSTM_TO:
        /* Writable only with TOE = 0 */
        if (!s->toe) {
            s->to = val & 1;
        }
        break;
    case OSTM_TOE:
        s->toe = val & 1;
        break;
    case OSTM_TS:
        /* TE = 1: interval restarts, free-run ignores it */
        if (val & 1) {
            if (!s->te) {
                ostm_start(s, false);
            } else if (!ostm_freerun(s)) {
                ostm_start(s, true);
            }
        }
        break;
    case OSTM_TT:
        if (val & 1) {
            ostm_stop(s);
        }
        break;
    case OSTM_CTL:
        /*
         * Read-only while TE = 1; a mode change loads FFFFFFFFH (interval)
         * or 0 (free-run)
         */
        if (s->te) {
            break;
        }
        if ((val ^ s->ctl) & CTL_MD1) {
            s->cnt = (val & CTL_MD1) ? 0 : 0xffffffff;
        }
        s->ctl = val & CTL_MASK;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "renesas_ostm: write of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps ostm_ops = {
    .read = ostm_read,
    .write = ostm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* A clock change keeps CNT and goes on with the new period */
static void ostm_clk_update(void *opaque, ClockEvent event)
{
    RenesasOSTMState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        ostm_fold(s, now);
    } else {
        s->base = now;
        ostm_schedule(s);
    }
}

static void ostm_reset_hold(Object *obj, ResetType type)
{
    RenesasOSTMState *s = RENESAS_OSTM(obj);

    timer_del(&s->timer);
    s->cmp = 0;
    s->cnt = 0xffffffff;
    s->to = 0;
    s->toe = 0;
    s->te = 0;
    s->ctl = 0;
    s->base = 0;
    ostm_hold(s, false);
}

static void ostm_init(Object *obj)
{
    RenesasOSTMState *s = RENESAS_OSTM(obj);
    SysBusDevice *d = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ostm_ops, s, "renesas-ostm",
                          RENESAS_OSTM_SIZE);
    sysbus_init_mmio(d, &s->iomem);
    sysbus_init_irq(d, &s->tint);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", ostm_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, ostm_timer_cb, s);
}

static const VMStateDescription vmstate_ostm = {
    .name = TYPE_RENESAS_OSTM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, RenesasOSTMState),
        VMSTATE_TIMER(timer, RenesasOSTMState),
        VMSTATE_UINT32(cmp, RenesasOSTMState),
        VMSTATE_UINT32(cnt, RenesasOSTMState),
        VMSTATE_UINT8(to, RenesasOSTMState),
        VMSTATE_UINT8(toe, RenesasOSTMState),
        VMSTATE_UINT8(te, RenesasOSTMState),
        VMSTATE_UINT8(ctl, RenesasOSTMState),
        VMSTATE_INT64(base, RenesasOSTMState),
        VMSTATE_UINT8(tint_held, RenesasOSTMState),
        VMSTATE_END_OF_LIST()
    }
};

static void ostm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_ostm;
    rc->phases.hold = ostm_reset_hold;
}

static const TypeInfo ostm_types[] = {
    {
        .name           = TYPE_RENESAS_OSTM,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(RenesasOSTMState),
        .instance_init  = ostm_init,
        .class_init     = ostm_class_init,
    },
};

DEFINE_TYPES(ostm_types)
