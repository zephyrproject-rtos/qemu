/*
 * Altera emulation
 *
 * Copyright (c) 2016 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ALTERA_H
#define ALTERA_H

void altera_timer_create(const hwaddr addr, qemu_irq irq, uint32_t frequency);
void altera_juart_create(int uart, const hwaddr addr, qemu_irq irq);

#endif /* ALTERA_H */
