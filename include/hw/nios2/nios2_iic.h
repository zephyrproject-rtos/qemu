#ifndef QEMU_HW_NIOS2_IIC_H
#define QEMU_HW_NIOS2_IIC_H

#include "qemu-common.h"

void nios2_iic_update_cr_status(DeviceState *d);
void nios2_iic_update_cr_ienable(DeviceState *d);
void nios2_iic_create(Nios2CPU *cpu);

#endif /* QEMU_HW_NIOS2_IIC_H */

