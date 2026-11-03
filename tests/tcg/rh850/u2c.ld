/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 */

MEMORY
{
    CFLASH (rx) : ORIGIN = 0x00000000, LENGTH = 8M
    CRAM (rw)   : ORIGIN = 0xfe000000, LENGTH = 0x138000
}

SECTIONS
{
    .text : { *(.text) } > CFLASH
    .data : { *(.data) } > CRAM
}
