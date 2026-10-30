/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * RH850 instruction length from the first one or two halfwords
 */

#ifndef RH850_INSN_LEN_H
#define RH850_INSN_LEN_H

#define RH850_HW_REG2(hw0)  (((hw0) >> 11) & 0x1f)
#define RH850_HW_OP6(hw0)   (((hw0) >> 5) & 0x3f)

/* True if hw0 alone gives the length (16-bit formats I-IV) */
static inline bool rh850_insn_is_16bit(uint16_t hw0)
{
    if (((hw0 >> 9) & 3) == 3) {
        return false;
    }
    /* JR/JARL disp32 */
    return !(RH850_HW_OP6(hw0) == 0x17 && RH850_HW_REG2(hw0) == 0);
}

/* Length in bytes of an instruction that is not 16-bit */
static inline int rh850_insn_len(uint16_t hw0, uint16_t hw1)
{
    unsigned op6 = RH850_HW_OP6(hw0);

    if (RH850_HW_REG2(hw0) != 0) {
        return 4;
    }
    switch (op6) {
    case 0x17:              /* JR/JARL disp32 */
    case 0x31:              /* MOV imm32 */
        return 6;
    case 0x37:              /* JMP disp32 if bit 16 = 0 (else LOOP) */
        return (hw1 & 1) ? 4 : 6;
    case 0x3c:
    case 0x3d:
        if (!(hw1 & 1)) {
            return 4;       /* JR disp22 */
        }
        if ((hw1 & 0x1f) == 0x01) {
            return 4;       /* PREPARE list12, imm5 */
        }
        if ((hw1 & 7) == 3) {
            /* PREPARE ff: sp, imm16, imm16 << 16, imm32 */
            static const int prepare_len[4] = { 4, 6, 6, 8 };
            return prepare_len[(hw1 >> 3) & 3];
        }
        /* disp23 loads/stores (format XIV), coprocessor format 2 */
        return 6;
    default:
        return 4;
    }
}

#endif
