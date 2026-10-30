/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 TCG helpers
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/helper-proto.h"

/*
 * Divide by zero keeps reg2, remainder 0, OV = 1; 80000000H / -1 gives
 * 80000000H with OV = 1; the remainder takes the dividend's sign
 */
static uint64_t div_result(CPURH850State *env, uint32_t q, uint32_t r,
                           bool ov)
{
    env->psw_ov = ov;
    env->psw_s = q >> 31;
    env->psw_z = q == 0;
    return q | (uint64_t)r << 32;
}

uint64_t HELPER(divs)(CPURH850State *env, uint32_t a, uint32_t b)
{
    int32_t n = a, d = b;

    if (d == 0) {
        env->psw_ov = 1;
        env->psw_s = 0;
        env->psw_z = 0;
        return a;
    }
    if (n == INT32_MIN && d == -1) {
        return div_result(env, a, 0, true);
    }
    return div_result(env, n / d, n % d, false);
}

uint64_t HELPER(divu)(CPURH850State *env, uint32_t a, uint32_t b)
{
    if (b == 0) {
        env->psw_ov = 1;
        env->psw_s = 0;
        env->psw_z = 0;
        return a;
    }
    return div_result(env, a / b, a % b, false);
}

/* An operation with an undefined result: log it, execution continues */
void HELPER(log_undefined)(CPURH850State *env, void *what)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rh850: undefined operation: %s at pc "
                  "0x%08x\n", (const char *)what, env->pc);
}
