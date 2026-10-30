/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850/U2C machine
 */

#ifndef HW_RH850_U2C_H
#define HW_RH850_U2C_H

#include "hw/core/boards.h"
#include "hw/intc/rh850_intc.h"
#include "hw/timer/renesas_ostm.h"
#include "hw/char/renesas_rlin3.h"
#include "hw/rh850/u2c_sysctl.h"
#include "qemu/units.h"
#include "target/rh850/cpu.h"

#define TYPE_RH850_U2C_MACHINE MACHINE_TYPE_NAME("rh850-u2c")
OBJECT_DECLARE_SIMPLE_TYPE(RH850U2CMachineState, RH850_U2C_MACHINE)

/* U2C8, single map mode */
#define RH850_U2C_CFLASH_BASE       0x00000000
#define RH850_U2C_CFLASH_SIZE       (8 * MiB)
#define RH850_U2C_RRAM_BASE         0x12000000
#define RH850_U2C_RRAM_SIZE         (32 * KiB)
#define RH850_U2C_TEST_BASE         0x40000000
#define RH850_U2C_LRAM0_BASE        0xfdc00000
#define RH850_U2C_LRAM_SELF_BASE    0xfde00000
#define RH850_U2C_LRAM_SIZE         (128 * KiB)
#define RH850_U2C_CRAM_BASE         0xfe000000
#define RH850_U2C_CRAM_SIZE         0x138000
#define RH850_U2C_PBUS_BASE         0xff000000
#define RH850_U2C_PBUS_SIZE         0xfb8000
#define RH850_U2C_IBUS_BASE         0xfffb8000
#define RH850_U2C_IBUS_SIZE         0x8000
#define RH850_U2C_CPUPERI_BASE      0xfffc0000
#define RH850_U2C_CPUPERI_SIZE      0x8000

#define RH850_U2C_INTC1_SELF_BASE   0xfffc0000
#define RH850_U2C_INTC1_PE0_BASE    0xfffc4000
#define RH850_U2C_INTC2_BASE        0xfff80000
#define RH850_U2C_FENC_BASE         0xff9a3a00
#define RH850_U2C_FEINC_PE0_BASE    0xff9a3b00

#define RH850_U2C_NOSTM             2
#define RH850_U2C_OSTM0_BASE        0xffbf0000
#define RH850_U2C_OSTM1_BASE        0xff89a100
#define RH850_U2C_OSTM0_IRQ         63
#define RH850_U2C_OSTM1_IRQ         484

/*
 * RLIN3: 20 units (0-18, 23); even n at 0xffc7c000 + n / 2 * 0x200,
 * odd n at 0xff89b100 + (n - 1) / 2 * 0x200, RLIN323 at 0xff978000
 */
#define RH850_U2C_NRLIN3            20
#define RH850_U2C_RLIN3_EVEN_BASE   0xffc7c000
#define RH850_U2C_RLIN3_ODD_BASE    0xff89b100
#define RH850_U2C_RLIN323_BASE      0xff978000
#define RH850_U2C_CONSOLE_RLIN3     11

/* CLKC_HSB with the reset clock settings */
#define RH850_U2C_CLK_HSB_HZ        20000000

struct RH850U2CMachineState {
    MachineState parent_obj;

    RH850CPU cpu;
    RH850INTCState intc;
    MemoryRegion intc1_self;
    RenesasOSTMState ostm[RH850_U2C_NOSTM];
    RenesasRLIN3State rlin3[RH850_U2C_NRLIN3];
    RH850U2CSysctlState sysctl;
    Clock *clk_hsb;
    uint32_t clk_hsb_hz;
    bool test_exit;
    MemoryRegion cflash;
    MemoryRegion lram0;
    MemoryRegion lram_self;
    MemoryRegion cram;
    MemoryRegion rram;
};

#endif
