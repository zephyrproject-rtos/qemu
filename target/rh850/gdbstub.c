/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 gdbstub: register numbers follow the GDB v850e3v5 layout;
 * numbers without QEMU state read 0 and ignore writes.
 */

#include "qemu/osdep.h"
#include "qemu/bitmap.h"
#include "qemu/log.h"
#include "cpu.h"
#include "sysreg.h"
#include "gdbstub/helpers.h"

enum {
    GDB_SELID0 = 32,
    GDB_PC = 64,
    GDB_FPU = 122,
    GDB_FPU_FIRST = GDB_FPU + 6,
    GDB_FPU_LAST = GDB_FPU + 11,
    GDB_SELID1 = 150,
    GDB_VR0 = 374,
};

QEMU_BUILD_BUG_ON(GDB_VR0 + 32 != RH850_GDB_NUM_REGS);
QEMU_BUILD_BUG_ON(GDB_SELID1 + 7 * 32 != GDB_VR0);

/* selID/regID of a GDB register number, false if it has none */
static bool rh850_gdb_sysreg(int n, unsigned *sel, unsigned *id)
{
    if (n >= GDB_SELID0 && n < GDB_PC) {
        *sel = 0;
        *id = n - GDB_SELID0;
        return true;
    }
    if (n >= GDB_FPU_FIRST && n <= GDB_FPU_LAST) {
        *sel = 0;
        *id = n - GDB_FPU;
        return true;
    }
    if (n >= GDB_SELID1 && n < GDB_VR0) {
        *sel = 1 + (n - GDB_SELID1) / 32;
        *id = (n - GDB_SELID1) % 32;
        return true;
    }
    return false;
}

static void rh850_gdb_unmapped(int n)
{
    static DECLARE_BITMAP(logged, RH850_GDB_NUM_REGS);

    if (!test_and_set_bit(n, logged)) {
        qemu_log_mask(LOG_UNIMP, "rh850: gdb register %d has no state, "
                      "write ignored\n", n);
    }
}

int rh850_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    CPURH850State *env = cpu_env(cs);
    unsigned sel, id;
    uint32_t val;

    if (n < RH850_NUM_GPRS) {
        return gdb_get_reg32(mem_buf, env->gpr[n]);
    }
    if (n == GDB_PC) {
        return gdb_get_reg32(mem_buf, env->pc);
    }
    if (n >= GDB_VR0) {
        return gdb_get_reg64(mem_buf, 0);
    }
    if (rh850_gdb_sysreg(n, &sel, &id) &&
        rh850_sysreg_get(env, sel, id, &val)) {
        return gdb_get_reg32(mem_buf, val);
    }
    return gdb_get_reg32(mem_buf, 0);
}

int rh850_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPURH850State *env = cpu_env(cs);
    uint32_t val = ldl_le_p(mem_buf);
    unsigned sel, id;

    if (n < RH850_NUM_GPRS) {
        /* r0 is fixed to 0 */
        if (n) {
            env->gpr[n] = val;
        }
        return 4;
    }
    if (n == GDB_PC) {
        /* PC bit 0 is fixed to 0 */
        env->pc = val & ~1u;
        return 4;
    }
    if (n >= GDB_VR0) {
        rh850_gdb_unmapped(n);
        return 8;
    }
    if (!rh850_gdb_sysreg(n, &sel, &id) ||
        !rh850_sysreg_set(env, sel, id, val)) {
        rh850_gdb_unmapped(n);
    }
    return 4;
}
