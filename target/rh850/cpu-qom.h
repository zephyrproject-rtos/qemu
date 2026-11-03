/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 CPU QOM header (target agnostic)
 */

#ifndef RH850_CPU_QOM_H
#define RH850_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_RH850_CPU "rh850-cpu"

#define TYPE_RH850_G4KH_CPU RH850_CPU_TYPE_NAME("g4kh")

OBJECT_DECLARE_CPU_TYPE(RH850CPU, RH850CPUClass, RH850_CPU)

#define RH850_CPU_TYPE_SUFFIX "-" TYPE_RH850_CPU
#define RH850_CPU_TYPE_NAME(model) model RH850_CPU_TYPE_SUFFIX

#endif
