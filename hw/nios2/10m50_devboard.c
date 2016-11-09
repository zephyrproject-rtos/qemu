/*
 * Altera 10M50 Nios2 GHRD
 *
 * Copyright (c) 2016 Marek Vasut <marek.vasut@gmail.com>
 *
 * Based on LabX device code
 *
 * Copyright (c) 2012 Chris Wulff <crwulff@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/char/serial.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/config-file.h"
#include "hw/nios2/nios2_iic.h"
#include "hw/nios2/altera.h"

#include "boot.h"

#define BINARY_DEVICE_TREE_FILE		"10m50-devboard.dtb"

static void nios2_10m50_ghrd_init(MachineState *machine)
{
    Nios2CPU *cpu;

    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *phys_tcm = g_new(MemoryRegion, 1);
    MemoryRegion *phys_tcm_alias = g_new(MemoryRegion, 1);
    MemoryRegion *phys_ram = g_new(MemoryRegion, 1);
    MemoryRegion *phys_ram_alias = g_new(MemoryRegion, 1);
    ram_addr_t tcm_base = 0x0;
    ram_addr_t tcm_size = 0x1000;    /* 1 kiB, but QEMU limit is 4 kiB */
    ram_addr_t ram_base = 0x08000000;
    ram_addr_t ram_size = 0x08000000;
    qemu_irq irq[32];
    int i;

    /* Physical TCM (tb_ram_1k) with alias at 0xc0000000 */
    memory_region_init_ram(phys_tcm, NULL, "nios2.tcm", tcm_size, &error_abort);
    memory_region_init_alias(phys_tcm_alias, NULL, "nios2.tcm.alias",
                             phys_tcm, 0, tcm_size);
    vmstate_register_ram_global(phys_tcm);
    memory_region_add_subregion(address_space_mem, tcm_base, phys_tcm);
    memory_region_add_subregion(address_space_mem, 0xc0000000 + tcm_base,
                                phys_tcm_alias);

    /* Physical DRAM with alias at 0xc0000000 */
    memory_region_init_ram(phys_ram, NULL, "nios2.ram", ram_size, &error_abort);
    memory_region_init_alias(phys_ram_alias, NULL, "nios2.ram.alias",
                             phys_ram, 0, ram_size);
    vmstate_register_ram_global(phys_ram);
    memory_region_add_subregion(address_space_mem, ram_base, phys_ram);
    memory_region_add_subregion(address_space_mem, 0xc0000000 + ram_base,
                                phys_ram_alias);

    /* Create CPU -- FIXME */
    cpu = cpu_nios2_init("nios2");

    nios2_iic_create(cpu);
    for (i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in(cpu->env.pic_state, i);
    }

    /* Register: Altera 16550 UART */
    serial_mm_init(address_space_mem, 0xf8001600, 2, irq[1], 115200,
                   serial_hds[0], DEVICE_NATIVE_ENDIAN);

    /* Register: Timer sys_clk_timer  */
    altera_timer_create(0xf8001440, irq[0], 75 * 1000000);

    /* Register: Timer sys_clk_timer_1  */
    altera_timer_create(0xe0000880, irq[5], 75 * 1000000);

    /* Configure new exception vectors and reset CPU for it to take effect. */
    cpu->env.reset_addr = 0xd4000000;
    cpu->env.exception_addr = 0xc8000120;
    cpu->env.fast_tlb_miss_addr = 0xc0000100;

    nios2_load_kernel(cpu, ram_base, ram_size, machine->initrd_filename,
                      BINARY_DEVICE_TREE_FILE, NULL);
}

static void nios2_10m50_ghrd_machine_init(struct MachineClass *mc)
{
    mc->desc = "Altera 10M50 GHRD Nios II design";
    mc->init = nios2_10m50_ghrd_init;
    mc->is_default = 1;
}

DEFINE_MACHINE("10m50-ghrd", nios2_10m50_ghrd_machine_init);
