/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * Renesas RLIN3 LIN/UART interface (UART mode)
 */

#ifndef HW_CHAR_RENESAS_RLIN3_H
#define HW_CHAR_RENESAS_RLIN3_H

#include "chardev/char-fe.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RENESAS_RLIN3 "renesas-rlin3"
OBJECT_DECLARE_SIMPLE_TYPE(RenesasRLIN3State, RENESAS_RLIN3)

#define RENESAS_RLIN3_SIZE 0x40

/* Interrupt outputs: INTRLIN3n, UR0 (transmit), UR1 (receive), UR2 (status) */
enum {
    RENESAS_RLIN3_IRQ_LIN,
    RENESAS_RLIN3_IRQ_UR0,
    RENESAS_RLIN3_IRQ_UR1,
    RENESAS_RLIN3_IRQ_UR2,
    RENESAS_RLIN3_NIRQ,
};

struct RenesasRLIN3State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq[RENESAS_RLIN3_NIRQ];

    /* 8-bit registers by offset; LBRP01, LUTDR, LURDR, LUWTDR included */
    uint8_t regs[RENESAS_RLIN3_SIZE];
    /* LURDR holds a byte not read yet */
    uint8_t rx_full;
};

#endif
