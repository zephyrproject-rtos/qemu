/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 interrupt controller (INTC1 of PE0, INTC2, FENMI, FEINT of PE0)
 */

#ifndef HW_INTC_RH850_INTC_H
#define HW_INTC_RH850_INTC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RH850_INTC "rh850-intc"
OBJECT_DECLARE_SIMPLE_TYPE(RH850INTCState, RH850_INTC)

/* U2C8: EIINT0-31 in INTC1, EIINT32-629 in INTC2 */
#define RH850_INTC_NCH          630
#define RH850_INTC1_NCH         32
#define RH850_INTC_NFEINT       3

#define RH850_INTC1_SIZE        0x280
#define RH850_INTC2_SIZE        0x8000
#define RH850_INTC_FE_SIZE      0x10

/*
 * Request to the CPU on sysbus IRQ 0 (EIINT): 0 = none, else
 * VALID | EITB << 20 | EIP << 16 | channel.
 */
#define RH850_INTC_REQ_VALID    (1 << 24)
#define RH850_INTC_REQ_TB       20
#define RH850_INTC_REQ_PRIO     16
#define RH850_INTC_REQ_CH_MASK  0x3ff

/* Sysbus IRQs to the CPU */
enum {
    RH850_INTC_OUT_EIINT,
    RH850_INTC_OUT_FEINT,
    RH850_INTC_OUT_FENMI,
    RH850_INTC_OUT_NUM,
};

/* Inputs "ack" from the CPU */
enum {
    RH850_INTC_ACK_EIINT,       /* level = accepted channel */
    RH850_INTC_ACK_FENMI,
    RH850_INTC_ACK_EOI,         /* EIRET/FERET with PSW.EP = 0 */
    RH850_INTC_ACK_NUM,
};

struct RH850INTCState {
    SysBusDevice parent_obj;

    MemoryRegion intc1;
    MemoryRegion intc2;
    MemoryRegion fenc;
    MemoryRegion feinc;
    qemu_irq out[RH850_INTC_OUT_NUM];

    /* EICn/EEICn fields (one state, two views) */
    uint8_t eirf[RH850_INTC_NCH];
    uint8_t eimk[RH850_INTC_NCH];
    uint8_t eitb[RH850_INTC_NCH];
    uint8_t eiov[RH850_INTC_NCH];
    uint8_t eip[RH850_INTC_NCH];
    uint32_t eibd[RH850_INTC_NCH];
    uint8_t in[RH850_INTC_NCH];
    uint8_t level[RH850_INTC_NCH];

    uint32_t fenmif;
    uint32_t fenmi_req;
    uint32_t fenmi_in;
    uint32_t feintf;
    uint32_t feintmsk;
    uint32_t feint_in;
};

#endif
