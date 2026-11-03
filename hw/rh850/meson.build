# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz

rh850_ss = ss.source_set()
rh850_ss.add(when: 'CONFIG_RH850_U2C', if_true: files('u2c.c', 'u2c_sysctl.c'))

hw_arch += {'rh850': rh850_ss}
