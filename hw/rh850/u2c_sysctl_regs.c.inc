/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 Linumiz
 *
 * U2C clock controller and standby controller registers.
 */

static const RH850U2CSysctlReg rh850_u2c_sysctl_regs[] = {
    /* PLLE */
    { 0x0000, 0x00000000, 0xffffffff, SC_TRIG, SC_KEY_CLKKCPROT1, 1 },
    /* PLLS */
    { 0x0004, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* PLLSTPM */
    { 0x000c, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* SSCGC */
    { 0x002c, 0x000000f0, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* CKD_HSIOSCC */
    { 0x00c0, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* CKD_HSIOSCS */
    { 0x00c4, 0x00000002, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_CLEANC */
    { 0x0100, 0x00000001, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 7 },
    /* CKS_CLEANS */
    { 0x0108, 0x00000001, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_SSCGC */
    { 0x0110, 0x00000001, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 9 },
    /* CKS_SSCGS */
    { 0x0114, 0x00000001, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKD_PLLC */
    { 0x0120, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* CKD_PLLS */
    { 0x0128, 0x00000002, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKD_SSCGC */
    { 0x0130, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* CKD_SSCGS */
    { 0x0134, 0x00000002, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_RLINC */
    { 0x0140, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 15 },
    /* CKS_RLINS */
    { 0x0148, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_RCANC */
    { 0x0150, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 17 },
    /* CKS_RCANS */
    { 0x0158, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_MSPIC */
    { 0x0170, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 19 },
    /* CKS_MSPIS */
    { 0x0178, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_SWDTC */
    { 0x0190, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 21 },
    /* CKS_SWDTS */
    { 0x0198, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_CANXL0C */
    { 0x01a0, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 23 },
    /* CKS_CANXL0S */
    { 0x01a8, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_CANXL1C */
    { 0x01b0, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 25 },
    /* CKS_CANXL1S */
    { 0x01b8, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CLKKCPROT1 */
    { 0x0700, 0x00000000, 0xffffffff, SC_KEY, SC_KEY_NONE, -1 },
    /* CLKKCPROT4 */
    { 0x0730, 0x00000000, 0xffffffff, SC_KEY, SC_KEY_NONE, -1 },
    /* MSR_RSCFD */
    { 0x1000, 0x00000003, 0x80000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_FLXA */
    { 0x1010, 0x00000001, 0x00000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_GTM */
    { 0x1020, 0x00000001, 0x00000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_ETND */
    { 0x1030, 0x00000003, 0x00000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_ETNF */
    { 0x1034, 0x00000003, 0x00000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_RSENT */
    { 0x1040, 0x000000ff, 0x000000ff, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_MSPI */
    { 0x1050, 0x000003ff, 0x000003ff, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_RLIN3 */
    { 0x1060, 0x00ffffff, 0x80ffffff, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_ADCK_ISO */
    { 0x1078, 0x00000003, 0x00000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_CXPI */
    { 0x1080, 0x0000000f, 0x8000000f, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_MMCA */
    { 0x1090, 0x00000001, 0x00000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_ENCA */
    { 0x10a0, 0x00000001, 0x00000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_PSI5 */
    { 0x10b0, 0x00000023, 0x00000023, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_PSI5S */
    { 0x10c0, 0x00000001, 0x00000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_PWMD */
    { 0x10d0, 0x00000001, 0x00000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_SFMA */
    { 0x1110, 0x00000003, 0x00000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_TAPA */
    { 0x1120, 0x0000000f, 0x0000000f, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_TAUD */
    { 0x1130, 0x00000007, 0x00000007, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_TAUJ_ISO */
    { 0x1140, 0x00000003, 0x00000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_TSG3 */
    { 0x1160, 0x00000003, 0x00000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_OSTM */
    { 0x1180, 0x0000000f, 0x0000000f, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_RI3C */
    { 0x1250, 0x0000000f, 0x8000000f, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_SSIF */
    { 0x1260, 0x00000003, 0x00000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* STBCKCPROT */
    { 0x1700, 0x00000000, 0xffffffff, SC_KEY, SC_KEY_NONE, -1 },
    /* MSRKCPROT */
    { 0x1710, 0x00000000, 0xffffffff, SC_KEY, SC_KEY_NONE, -1 },
    /* MOSCE */
    { 0x8000, 0x00000000, 0xffffffff, SC_TRIG, SC_KEY_CLKKCPROT1, 54 },
    /* MOSCS */
    { 0x8004, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* MOSCSTPM */
    { 0x800c, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* SOSCE */
    { 0x8020, 0x00000000, 0xffffffff, SC_TRIG, SC_KEY_CLKKCPROT1, 57 },
    /* SOSCS */
    { 0x8024, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* SOSCST */
    { 0x8028, 0x0001199a, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* HSOSCS */
    { 0x8100, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* HSOSCSTPM */
    { 0x8104, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* HSOSCTRMPRODC */
    { 0x810c, 0x00000002, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT4, -1 },
    /* HSOSCTRMEN */
    { 0x8128, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT4, -1 },
    /* HSOSCFRETRMS */
    { 0x8130, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* HSOSCTRMCNT */
    { 0x8134, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* HSOSCTRMCODE */
    { 0x8138, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* HSOSCUSERTRMSET */
    { 0x8148, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT4, -1 },
    /* CKS_AWDTC */
    { 0x8200, 0x00000001, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 68 },
    /* CKS_AWDTS */
    { 0x8208, 0x00000001, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_ATAUJC */
    { 0x8210, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 70 },
    /* CKS_ATAUJS */
    { 0x8218, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_ARTCAC */
    { 0x8220, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 72 },
    /* CKS_ARTCAS */
    { 0x8228, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_AADCC */
    { 0x8230, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 74 },
    /* CKS_AADCS */
    { 0x8238, 0x00000000, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_EXTCLK0C */
    { 0x8250, 0x00000004, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 76 },
    /* CKS_EXTCLK0S */
    { 0x8258, 0x00000004, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKD_EXTCLK0C */
    { 0x8260, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* CKD_EXTCLK0S */
    { 0x8268, 0x00000001, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_EXTCLK1C */
    { 0x8270, 0x00000004, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 80 },
    /* CKS_EXTCLK1S */
    { 0x8278, 0x00000004, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKD_EXTCLK1C */
    { 0x8280, 0x00000000, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, -1 },
    /* CKD_EXTCLK1S */
    { 0x8288, 0x00000001, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* CKS_WDTC */
    { 0x8300, 0x00000001, 0xffffffff, SC_RW, SC_KEY_CLKKCPROT1, 84 },
    /* CKS_WDTS */
    { 0x8308, 0x00000001, 0xffffffff, SC_RO, SC_KEY_NONE, -1 },
    /* STBC0PSC */
    { 0x8c00, 0x00000000, 0xffffffff, SC_RW, SC_KEY_STBCKCPROT, -1 },
    /* STBC0STPT */
    { 0x8c04, 0x00000000, 0xffffffff, SC_RW, SC_KEY_STBCKCPROT, -1 },
    /* PWRGD_CNT */
    { 0x8c08, 0x00000000, 0xffffffff, SC_RW, SC_KEY_STBCKCPROT, -1 },
    /* RRAM_SDM */
    { 0x8c10, 0x00000000, 0xffffffff, SC_RW, SC_KEY_STBCKCPROT, -1 },
    /* AWOPSM */
    { 0x8c30, 0x00000000, 0xffffffff, SC_RW, SC_KEY_STBCKCPROT, -1 },
    /* IOHOLD0 */
    { 0x8d00, 0x00000000, 0xffffffff, SC_RW, SC_KEY_STBCKCPROT, -1 },
    /* IOHOLD2 */
    { 0x8d08, 0x00000000, 0xffffffff, SC_RW, SC_KEY_STBCKCPROT, -1 },
    /* MSR_ADCK_AWO */
    { 0x8e00, 0x00000001, 0x80000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_RTCA */
    { 0x8e10, 0x00000001, 0x80000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_TAUJ_AWO */
    { 0x8e20, 0x00000003, 0x80000003, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* MSR_WDTB_AWO */
    { 0x8e30, 0x00000000, 0x80000001, SC_RW, SC_KEY_MSRKCPROT, -1 },
    /* RREG00 */
    { 0x8f00, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* RREG01 */
    { 0x8f04, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* RREG02 */
    { 0x8f08, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* RREG03 */
    { 0x8f0c, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* RREG04 */
    { 0x8f10, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* RREG05 */
    { 0x8f14, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* RREG06 */
    { 0x8f18, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* RREG07 */
    { 0x8f1c, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF0_A0 */
    { 0xe000, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK0_A0 */
    { 0xe004, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC0_A0 */
    { 0xe008, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF0_A1 */
    { 0xe010, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK0_A1 */
    { 0xe014, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC0_A1 */
    { 0xe018, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF0_A2 */
    { 0xe020, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK0_A2 */
    { 0xe024, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC0_A2 */
    { 0xe028, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF1_A0 */
    { 0xe120, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK1_A0 */
    { 0xe124, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC1_A0 */
    { 0xe128, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF1_A1 */
    { 0xe130, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK1_A1 */
    { 0xe134, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC1_A1 */
    { 0xe138, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF1_A2 */
    { 0xe140, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK1_A2 */
    { 0xe144, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC1_A2 */
    { 0xe148, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMON */
    { 0xe1a0, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF0_I0 */
    { 0xe200, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK0_I0 */
    { 0xe204, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC0_I0 */
    { 0xe208, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF0_I1 */
    { 0xe210, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK0_I1 */
    { 0xe214, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC0_I1 */
    { 0xe218, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUF0_I2 */
    { 0xe220, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFMSK0_I2 */
    { 0xe224, 0xffffffff, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
    /* WUFC0_I2 */
    { 0xe228, 0x00000000, 0xffffffff, SC_RW, SC_KEY_NONE, -1 },
};
