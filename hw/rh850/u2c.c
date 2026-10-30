/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850/U2C machine (U2C8, CPU0 only)
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "elf.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/sifive_test.h"
#include "hw/misc/unimp.h"
#include "hw/rh850/u2c.h"
#include "hw/core/qdev-properties-system.h"
#include "system/reset.h"
#include "system/system.h"
#include "system/address-spaces.h"

/*
 * CPU stores to the code flash user area are dropped and logged; the
 * flash sequencer is not modelled
 */
static uint64_t rh850_u2c_cflash_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    return 0;
}

static void rh850_u2c_cflash_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rh850: write of %u bytes to code flash "
                  "at 0x%08" HWADDR_PRIx " ignored\n", size, addr);
}

static const MemoryRegionOps rh850_u2c_cflash_ops = {
    .read = rh850_u2c_cflash_read,
    .write = rh850_u2c_cflash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void rh850_u2c_load_kernel(const char *filename)
{
    ssize_t size;

    size = load_elf(filename, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                    ELFDATA2LSB, EM_V800, 0, 0);
    if (size == ELF_LOAD_NOT_ELF) {
        size = load_image_targphys(filename, RH850_U2C_CFLASH_BASE,
                                   RH850_U2C_CFLASH_SIZE, NULL);
        if (size < 0) {
            error_report("could not load kernel '%s'", filename);
            exit(1);
        }
    } else if (size < 0) {
        error_report("could not load kernel '%s': %s (expected ELF "
                     "machine %d, little endian)", filename,
                     load_elf_strerror(size), EM_V800);
        exit(1);
    }
}

/*
 * INTC1 of PE0, INTC2, FENC and FEINC of PE0, at priority 0 above the
 * unimplemented-device regions
 */
static void rh850_u2c_intc_init(RH850U2CMachineState *s)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(&s->intc);
    DeviceState *cpu = DEVICE(&s->cpu);
    int i;

    sysbus_realize(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, RH850_U2C_INTC1_PE0_BASE);
    sysbus_mmio_map(sbd, 1, RH850_U2C_INTC2_BASE);
    sysbus_mmio_map(sbd, 2, RH850_U2C_FENC_BASE);
    sysbus_mmio_map(sbd, 3, RH850_U2C_FEINC_PE0_BASE);
    memory_region_init_alias(&s->intc1_self, OBJECT(s), "rh850.intc1-self",
                             sysbus_mmio_get_region(sbd, 0), 0,
                             RH850_INTC1_SIZE);
    memory_region_add_subregion(get_system_memory(),
                                RH850_U2C_INTC1_SELF_BASE, &s->intc1_self);
    for (i = 0; i < RH850_INTC_OUT_NUM; i++) {
        sysbus_connect_irq(sbd, i, qdev_get_gpio_in(cpu, i));
    }
    for (i = 0; i < RH850_INTC_ACK_NUM; i++) {
        qdev_connect_gpio_out_named(cpu, "intc-ack", i,
                                    qdev_get_gpio_in_named(DEVICE(sbd),
                                                           "ack", i));
    }
}

/* OSTM0/1 are clocked by CLKC_HSB; module standby is not gated */
static void rh850_u2c_ostm_init(RH850U2CMachineState *s)
{
    static const hwaddr base[RH850_U2C_NOSTM] = {
        RH850_U2C_OSTM0_BASE, RH850_U2C_OSTM1_BASE,
    };
    static const int irq[RH850_U2C_NOSTM] = {
        RH850_U2C_OSTM0_IRQ, RH850_U2C_OSTM1_IRQ,
    };
    int i;

    s->clk_hsb = clock_new(OBJECT(s), "clkc-hsb");
    clock_set_hz(s->clk_hsb, s->clk_hsb_hz);
    for (i = 0; i < RH850_U2C_NOSTM; i++) {
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->ostm[i]);

        qdev_connect_clock_in(DEVICE(sbd), "clk", s->clk_hsb);
        sysbus_realize(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, base[i]);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(DEVICE(&s->intc), irq[i]));
    }
}

#include "u2c_rlin3_irq.c.inc"

static hwaddr rh850_u2c_rlin3_base(int n)
{
    if (n == 23) {
        return RH850_U2C_RLIN323_BASE;
    }
    return n & 1 ? RH850_U2C_RLIN3_ODD_BASE + (n - 1) / 2 * 0x200 :
                   RH850_U2C_RLIN3_EVEN_BASE + n / 2 * 0x200;
}

/*
 * serial_hd(0) is RLIN311, serial_hd(1) onward go to the other units
 * in unit order
 */
static void rh850_u2c_rlin3_init(RH850U2CMachineState *s)
{
    int i, k, hd = 1;

    QEMU_BUILD_BUG_ON(ARRAY_SIZE(rh850_u2c_rlin3) != RH850_U2C_NRLIN3);
    for (i = 0; i < RH850_U2C_NRLIN3; i++) {
        int n = rh850_u2c_rlin3[i].unit;
        DeviceState *dev = DEVICE(&s->rlin3[i]);
        Chardev *chr;

        chr = serial_hd(n == RH850_U2C_CONSOLE_RLIN3 ? 0 : hd++);
        qdev_prop_set_chr(dev, "chardev", chr);
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, rh850_u2c_rlin3_base(n));
        for (k = 0; k < RENESAS_RLIN3_NIRQ; k++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), k,
                               qdev_get_gpio_in(DEVICE(&s->intc),
                                                rh850_u2c_rlin3[i].irq[k]));
        }
    }
}

static void rh850_u2c_init(MachineState *machine)
{
    RH850U2CMachineState *s = RH850_U2C_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    int i;

    object_initialize_child(OBJECT(machine), "cpu", &s->cpu,
                            machine->cpu_type);
    qdev_realize(DEVICE(&s->cpu), NULL, &error_fatal);
    /* The CPU is on no bus: reset it with the machine (PC = RBASE) */
    qemu_register_resettable(OBJECT(&s->cpu));
    object_initialize_child(OBJECT(machine), "intc", &s->intc,
                            TYPE_RH850_INTC);
    rh850_u2c_intc_init(s);
    object_initialize_child(OBJECT(machine), "ostm0", &s->ostm[0],
                            TYPE_RENESAS_OSTM);
    object_initialize_child(OBJECT(machine), "ostm1", &s->ostm[1],
                            TYPE_RENESAS_OSTM);
    rh850_u2c_ostm_init(s);
    for (i = 0; i < RH850_U2C_NRLIN3; i++) {
        g_autofree char *name = g_strdup_printf("rlin3%d",
                                                rh850_u2c_rlin3[i].unit);

        object_initialize_child(OBJECT(machine), name, &s->rlin3[i],
                                TYPE_RENESAS_RLIN3);
    }
    rh850_u2c_rlin3_init(s);
    /* CLKC/STBC register stub above the P-Bus stub */
    object_initialize_child(OBJECT(machine), "sysctl", &s->sysctl,
                            TYPE_RH850_U2C_SYSCTL);
    sysbus_realize(SYS_BUS_DEVICE(&s->sysctl), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sysctl), 0, RH850_U2C_SYSCTL_BASE);

    memory_region_init_rom_device(&s->cflash, NULL, &rh850_u2c_cflash_ops,
                                  s, "rh850.cflash", RH850_U2C_CFLASH_SIZE,
                                  &error_fatal);
    memory_region_add_subregion(sysmem, RH850_U2C_CFLASH_BASE, &s->cflash);

    memory_region_init_ram(&s->lram0, NULL, "rh850.lram0",
                           RH850_U2C_LRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, RH850_U2C_LRAM0_BASE, &s->lram0);
    memory_region_init_alias(&s->lram_self, NULL, "rh850.lram-self",
                             &s->lram0, 0, RH850_U2C_LRAM_SIZE);
    memory_region_add_subregion(sysmem, RH850_U2C_LRAM_SELF_BASE,
                                &s->lram_self);

    memory_region_init_ram(&s->cram, NULL, "rh850.cram",
                           RH850_U2C_CRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, RH850_U2C_CRAM_BASE, &s->cram);

    memory_region_init_ram(&s->rram, NULL, "rh850.rram",
                           RH850_U2C_RRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, RH850_U2C_RRAM_BASE, &s->rram);

    create_unimplemented_device("rh850.pbus", RH850_U2C_PBUS_BASE,
                                RH850_U2C_PBUS_SIZE);
    create_unimplemented_device("rh850.ibus", RH850_U2C_IBUS_BASE,
                                RH850_U2C_IBUS_SIZE);
    create_unimplemented_device("rh850.cpu-peri", RH850_U2C_CPUPERI_BASE,
                                RH850_U2C_CPUPERI_SIZE);

    /* Reserved address on silicon, test exit only when asked for */
    if (s->test_exit) {
        sifive_test_create(RH850_U2C_TEST_BASE);
    }

    if (machine->kernel_filename) {
        rh850_u2c_load_kernel(machine->kernel_filename);
    }
}

static bool rh850_u2c_get_test_exit(Object *obj, Error **errp)
{
    return RH850_U2C_MACHINE(obj)->test_exit;
}

static void rh850_u2c_set_test_exit(Object *obj, bool value, Error **errp)
{
    RH850_U2C_MACHINE(obj)->test_exit = value;
}

static void rh850_u2c_instance_init(Object *obj)
{
    RH850U2CMachineState *s = RH850_U2C_MACHINE(obj);

    s->clk_hsb_hz = RH850_U2C_CLK_HSB_HZ;
    object_property_add_uint32_ptr(obj, "clk-hsb", &s->clk_hsb_hz,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "clk-hsb",
                                    "CLKC_HSB frequency in Hz (OSTM clock)");
}

static void rh850_u2c_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        TYPE_RH850_G4KH_CPU,
        NULL
    };

    mc->desc = "Renesas RH850/U2C8 (G4KH, CPU0 only)";
    mc->init = rh850_u2c_init;
    mc->is_default = true;
    mc->default_cpu_type = TYPE_RH850_G4KH_CPU;
    mc->valid_cpu_types = valid_cpu_types;

    object_class_property_add_bool(oc, "test-exit", rh850_u2c_get_test_exit,
                                   rh850_u2c_set_test_exit);
    object_class_property_set_description(oc, "test-exit",
                                          "Exit device at 0x40000000 for "
                                          "test programs (default off)");
}

static const TypeInfo rh850_u2c_types[] = {
    {
        .name           = TYPE_RH850_U2C_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(RH850U2CMachineState),
        .instance_init  = rh850_u2c_instance_init,
        .class_init     = rh850_u2c_class_init,
    },
};

DEFINE_TYPES(rh850_u2c_types)
