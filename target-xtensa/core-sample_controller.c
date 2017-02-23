#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-sample_controller/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig sample_controller __attribute__((unused)) = {
    .name = "sample_controller",
    .gdb_regmap = {
        .reg = {
#include "core-sample_controller/gdb-config.c"
        }
    },
    .clock_freq_khz = 10000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(sample_controller)
