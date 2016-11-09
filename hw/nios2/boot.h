#ifndef __NIOS2_BOOT__
#define __NIOS2_BOOT__

#include "hw/hw.h"
#include "cpu.h"

void nios2_load_kernel(Nios2CPU *cpu, hwaddr ddr_base, uint32_t ramsize,
                       const char *initrd_filename, const char *dtb_filename,
                       void (*machine_cpu_reset)(Nios2CPU *));

#endif /* __NIOS2_BOOT__ */
