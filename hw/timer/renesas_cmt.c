/*
 * Renesas 16bit Compare-match timer
 *
 * Datasheet: RX62N Group, RX621 Group User's Manual: Hardware
 *            (Rev.1.40 R01UH0033EJ0140)
 *
 * Copyright (c) 2019 Yoshinori Sato
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "hw/timer/renesas_cmt.h"
#include "migration/vmstate.h"

/*
 *  +0 CMSTR - common control
 *  +2 CMCR  - ch0
 *  +4 CMCNT - ch0
 *  +6 CMCOR - ch0
 *  +8 CMCR  - ch1
 * +10 CMCNT - ch1
 * +12 CMCOR - ch1
 * If we think that the address of CH 0 has an offset of +2,
 * we can treat it with the same address as CH 1, so define it like that.
 */
REG16(CMSTR, 0)
  FIELD(CMSTR, STR0, 0, 1)
  FIELD(CMSTR, STR1, 1, 1)
  FIELD(CMSTR, STR,  0, 2)
/* This addeess is channel offset */
REG16(CMCR, 0)
  FIELD(CMCR, CKS,  0, 2)
  FIELD(CMCR, CMIE, 6, 1)
REG16(CMCNT, 2)
REG16(CMCOR, 4)

static uint16_t read_cmcnt(RCMTState *cmt, int ch) {
    if (!(cmt->cmstr & (1 << ch))) {
        return cmt->cmcnt[ch];
    }
    int64_t elapsed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - cmt->tick[ch];
    uint64_t ticks = (elapsed_ns * cmt->cmt_freq[ch]) / (NANOSECONDS_PER_SECOND);
    uint32_t limit = cmt->cmcor[ch] + 1;
    return (cmt->cmcnt[ch] + ticks) % limit;
}

static void update_events(RCMTState *cmt, int ch) {
    if (!(cmt->cmstr & (1 << ch))) {
        return;
    }
    uint16_t current = read_cmcnt(cmt, ch);
    uint32_t remain = (cmt->cmcor[ch] + 1) - current;
    if (remain == 0) {
        remain = cmt->cmcor[ch] + 1;
    }
    int64_t next_time = (int64_t)remain * NANOSECONDS_PER_SECOND / cmt->cmt_freq[ch];

    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t fire_time = now + next_time;

    if (fire_time <= now) {
        fire_time = now + 1;
    }

    timer_mod(&cmt->timer[ch], fire_time);
}

static uint64_t cmt_read(void *opaque, hwaddr offset, unsigned size)
{
    RCMTState *cmt = opaque;
    int ch = offset / 0x08;

    if (offset == A_CMSTR) {
        return FIELD_EX16(cmt->cmstr, CMSTR, STR);
    } else {
        offset &= 0x07;
        if (ch == 0) {
            offset -= 0x02;
        }
        switch (offset) {
        case A_CMCR:
            return cmt->cmcr[ch];
        case A_CMCNT:
            return read_cmcnt(cmt, ch);
        case A_CMCOR:
            return cmt->cmcor[ch];
        }
    }
    qemu_log_mask(LOG_UNIMP, "renesas_cmt: Register 0x%" HWADDR_PRIX " "
                             "not implemented\n",
                  offset);
    return UINT16_MAX;
}

static void start_stop(RCMTState *cmt, int ch, int enable) {
    if (enable) {
        cmt->cmcnt[ch] = 0;
        cmt->tick[ch] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        update_events(cmt, ch);
    } else {
        timer_del(&cmt->timer[ch]);
    }
}

static void cmt_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    RCMTState *cmt = opaque;
    int ch = offset / 0x08;

    if (offset == A_CMSTR) {
        cmt->cmstr = FIELD_EX16(val, CMSTR, STR);
        start_stop(cmt, 0, FIELD_EX16(cmt->cmstr, CMSTR, STR0));
        start_stop(cmt, 1, FIELD_EX16(cmt->cmstr, CMSTR, STR1));
    } else {
        offset &= 0x07;
        if (ch == 0) {
            offset -= 0x02;
        }
        switch (offset) {
        case A_CMCR:
            cmt->cmcr[ch] = FIELD_DP16(cmt->cmcr[ch], CMCR, CKS,
                                       FIELD_EX16(val, CMCR, CKS));
            cmt->cmcr[ch] = FIELD_DP16(cmt->cmcr[ch], CMCR, CMIE,
                                       FIELD_EX16(val, CMCR, CMIE));
            cmt->cmt_freq[ch] =
                cmt->input_freq / (8u << (2 * FIELD_EX16(cmt->cmcr[ch], CMCR, CKS)));
            update_events(cmt, ch);
            break;
        case 2:
            cmt->cmcnt[ch] = val;
            cmt->tick[ch] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            break;
        case 4:
            cmt->cmcor[ch] = val;
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "renesas_cmt: Register 0x%" HWADDR_PRIX " "
                                     "not implemented\n",
                          offset);
            return;
        }
        if (FIELD_EX16(cmt->cmstr, CMSTR, STR) & (1 << ch)) {
            update_events(cmt, ch);
        }
    }
}

static const MemoryRegionOps cmt_ops = {
    .write = cmt_write,
    .read  = cmt_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 2,
    },
    .valid = {
        .min_access_size = 2,
        .max_access_size = 2,
    },
};

static void timer_events(RCMTState *cmt, int ch)
{
    cmt->cmcnt[ch] = 0;
    cmt->tick[ch] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    update_events(cmt, ch);
    if (FIELD_EX16(cmt->cmcr[ch], CMCR, CMIE)) {
        qemu_irq_pulse(cmt->cmi[ch]);
    }
}

static void timer_event0(void *opaque)
{
    RCMTState *cmt = opaque;

    timer_events(cmt, 0);
}

static void timer_event1(void *opaque)
{
    RCMTState *cmt = opaque;

    timer_events(cmt, 1);
}

static void rcmt_reset(DeviceState *dev)
{
    RCMTState *cmt = RCMT(dev);
    cmt->cmstr = 0;
    for (int i = 0; i < CMT_CH; i++) {
        cmt->cmcr[i] = 0;
        cmt->cmcnt[i] = 0;
        cmt->cmcor[i] = 0xFFFF;
        cmt->tick[i] = 0;
    }
}

static void rcmt_init(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    RCMTState *cmt = RCMT(obj);

    memory_region_init_io(&cmt->memory, OBJECT(cmt), &cmt_ops,
                          cmt, "renesas-cmt", 0x10);
    sysbus_init_mmio(d, &cmt->memory);

    for (int i = 0; i < CMT_CH; i++) {
        sysbus_init_irq(d, &cmt->cmi[i]);
        cmt->cmt_freq[i] = cmt->input_freq >> 3;
        timer_init_ns(&cmt->timer[i], QEMU_CLOCK_VIRTUAL,
                      (i == 0 ? timer_event0 : timer_event1), cmt);
    }
}

static const VMStateDescription vmstate_rcmt = {
    .name = "rx-cmt",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(cmstr, RCMTState),
        VMSTATE_UINT16_ARRAY(cmcr, RCMTState, CMT_CH),
        VMSTATE_UINT16_ARRAY(cmcnt, RCMTState, CMT_CH),
        VMSTATE_UINT16_ARRAY(cmcor, RCMTState, CMT_CH),
        VMSTATE_INT64_ARRAY(tick, RCMTState, CMT_CH),
        VMSTATE_TIMER_ARRAY(timer, RCMTState, CMT_CH),
        VMSTATE_END_OF_LIST()
    }
};

static const Property rcmt_properties[] = {
    DEFINE_PROP_UINT64("input-freq", RCMTState, input_freq, 0),
};

static void rcmt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_rcmt;
    device_class_set_legacy_reset(dc, rcmt_reset);
    device_class_set_props(dc, rcmt_properties);
}

static const TypeInfo rcmt_info = {
    .name = TYPE_RENESAS_CMT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RCMTState),
    .instance_init = rcmt_init,
    .class_init = rcmt_class_init,
};

static void rcmt_register_types(void)
{
    type_register_static(&rcmt_info);
}

type_init(rcmt_register_types)
