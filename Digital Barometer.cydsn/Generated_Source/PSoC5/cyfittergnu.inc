.ifndef INCLUDED_CYFITTERGNU_INC
.set INCLUDED_CYFITTERGNU_INC, 1
.include "cydevicegnu.inc"
.include "cydevicegnu_trm.inc"

/* I2C_bI2C_UDB */
.set I2C_bI2C_UDB_Master_ClkGen_u0__A0_A1_REG, CYREG_B0_UDB15_A0_A1
.set I2C_bI2C_UDB_Master_ClkGen_u0__A0_REG, CYREG_B0_UDB15_A0
.set I2C_bI2C_UDB_Master_ClkGen_u0__A1_REG, CYREG_B0_UDB15_A1
.set I2C_bI2C_UDB_Master_ClkGen_u0__D0_D1_REG, CYREG_B0_UDB15_D0_D1
.set I2C_bI2C_UDB_Master_ClkGen_u0__D0_REG, CYREG_B0_UDB15_D0
.set I2C_bI2C_UDB_Master_ClkGen_u0__D1_REG, CYREG_B0_UDB15_D1
.set I2C_bI2C_UDB_Master_ClkGen_u0__DP_AUX_CTL_REG, CYREG_B0_UDB15_ACTL
.set I2C_bI2C_UDB_Master_ClkGen_u0__F0_F1_REG, CYREG_B0_UDB15_F0_F1
.set I2C_bI2C_UDB_Master_ClkGen_u0__F0_REG, CYREG_B0_UDB15_F0
.set I2C_bI2C_UDB_Master_ClkGen_u0__F1_REG, CYREG_B0_UDB15_F1
.set I2C_bI2C_UDB_Shifter_u0__16BIT_A0_REG, CYREG_B0_UDB12_13_A0
.set I2C_bI2C_UDB_Shifter_u0__16BIT_A1_REG, CYREG_B0_UDB12_13_A1
.set I2C_bI2C_UDB_Shifter_u0__16BIT_D0_REG, CYREG_B0_UDB12_13_D0
.set I2C_bI2C_UDB_Shifter_u0__16BIT_D1_REG, CYREG_B0_UDB12_13_D1
.set I2C_bI2C_UDB_Shifter_u0__16BIT_DP_AUX_CTL_REG, CYREG_B0_UDB12_13_ACTL
.set I2C_bI2C_UDB_Shifter_u0__16BIT_F0_REG, CYREG_B0_UDB12_13_F0
.set I2C_bI2C_UDB_Shifter_u0__16BIT_F1_REG, CYREG_B0_UDB12_13_F1
.set I2C_bI2C_UDB_Shifter_u0__A0_A1_REG, CYREG_B0_UDB12_A0_A1
.set I2C_bI2C_UDB_Shifter_u0__A0_REG, CYREG_B0_UDB12_A0
.set I2C_bI2C_UDB_Shifter_u0__A1_REG, CYREG_B0_UDB12_A1
.set I2C_bI2C_UDB_Shifter_u0__D0_D1_REG, CYREG_B0_UDB12_D0_D1
.set I2C_bI2C_UDB_Shifter_u0__D0_REG, CYREG_B0_UDB12_D0
.set I2C_bI2C_UDB_Shifter_u0__D1_REG, CYREG_B0_UDB12_D1
.set I2C_bI2C_UDB_Shifter_u0__DP_AUX_CTL_REG, CYREG_B0_UDB12_ACTL
.set I2C_bI2C_UDB_Shifter_u0__F0_F1_REG, CYREG_B0_UDB12_F0_F1
.set I2C_bI2C_UDB_Shifter_u0__F0_REG, CYREG_B0_UDB12_F0
.set I2C_bI2C_UDB_Shifter_u0__F1_REG, CYREG_B0_UDB12_F1
.set I2C_bI2C_UDB_StsReg__0__MASK, 0x01
.set I2C_bI2C_UDB_StsReg__0__POS, 0
.set I2C_bI2C_UDB_StsReg__16BIT_STATUS_AUX_CTL_REG, CYREG_B0_UDB11_12_ACTL
.set I2C_bI2C_UDB_StsReg__16BIT_STATUS_REG, CYREG_B0_UDB11_12_ST
.set I2C_bI2C_UDB_StsReg__1__MASK, 0x02
.set I2C_bI2C_UDB_StsReg__1__POS, 1
.set I2C_bI2C_UDB_StsReg__2__MASK, 0x04
.set I2C_bI2C_UDB_StsReg__2__POS, 2
.set I2C_bI2C_UDB_StsReg__3__MASK, 0x08
.set I2C_bI2C_UDB_StsReg__3__POS, 3
.set I2C_bI2C_UDB_StsReg__4__MASK, 0x10
.set I2C_bI2C_UDB_StsReg__4__POS, 4
.set I2C_bI2C_UDB_StsReg__5__MASK, 0x20
.set I2C_bI2C_UDB_StsReg__5__POS, 5
.set I2C_bI2C_UDB_StsReg__MASK, 0x3F
.set I2C_bI2C_UDB_StsReg__MASK_REG, CYREG_B0_UDB11_MSK
.set I2C_bI2C_UDB_StsReg__MASK_ST_AUX_CTL_REG, CYREG_B0_UDB11_MSK_ACTL
.set I2C_bI2C_UDB_StsReg__PER_ST_AUX_CTL_REG, CYREG_B0_UDB11_MSK_ACTL
.set I2C_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG, CYREG_B0_UDB11_ACTL
.set I2C_bI2C_UDB_StsReg__STATUS_CNT_REG, CYREG_B0_UDB11_ST_CTL
.set I2C_bI2C_UDB_StsReg__STATUS_CONTROL_REG, CYREG_B0_UDB11_ST_CTL
.set I2C_bI2C_UDB_StsReg__STATUS_REG, CYREG_B0_UDB11_ST
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_CONTROL_AUX_CTL_REG, CYREG_B0_UDB11_12_ACTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_CONTROL_CONTROL_REG, CYREG_B0_UDB11_12_CTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_CONTROL_COUNT_REG, CYREG_B0_UDB11_12_CTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_COUNT_CONTROL_REG, CYREG_B0_UDB11_12_CTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_COUNT_COUNT_REG, CYREG_B0_UDB11_12_CTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_MASK_MASK_REG, CYREG_B0_UDB11_12_MSK
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_MASK_PERIOD_REG, CYREG_B0_UDB11_12_MSK
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_PERIOD_MASK_REG, CYREG_B0_UDB11_12_MSK
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_PERIOD_PERIOD_REG, CYREG_B0_UDB11_12_MSK
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__1__MASK, 0x02
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__1__POS, 1
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__2__MASK, 0x04
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__2__POS, 2
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__4__MASK, 0x10
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__4__POS, 4
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__5__MASK, 0x20
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__5__POS, 5
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__6__MASK, 0x40
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__6__POS, 6
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__7__MASK, 0x80
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__7__POS, 7
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_AUX_CTL_REG, CYREG_B0_UDB11_ACTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG, CYREG_B0_UDB11_CTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_ST_REG, CYREG_B0_UDB11_ST_CTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__COUNT_REG, CYREG_B0_UDB11_CTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__COUNT_ST_REG, CYREG_B0_UDB11_ST_CTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__MASK, 0xF6
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__MASK_CTL_AUX_CTL_REG, CYREG_B0_UDB11_MSK_ACTL
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__PERIOD_REG, CYREG_B0_UDB11_MSK
.set I2C_bI2C_UDB_SyncCtl_CtrlReg__PER_CTL_AUX_CTL_REG, CYREG_B0_UDB11_MSK_ACTL

/* I2C_IntClock */
.set I2C_IntClock__CFG0, CYREG_CLKDIST_DCFG0_CFG0
.set I2C_IntClock__CFG1, CYREG_CLKDIST_DCFG0_CFG1
.set I2C_IntClock__CFG2, CYREG_CLKDIST_DCFG0_CFG2
.set I2C_IntClock__CFG2_SRC_SEL_MASK, 0x07
.set I2C_IntClock__INDEX, 0x00
.set I2C_IntClock__PM_ACT_CFG, CYREG_PM_ACT_CFG2
.set I2C_IntClock__PM_ACT_MSK, 0x01
.set I2C_IntClock__PM_STBY_CFG, CYREG_PM_STBY_CFG2
.set I2C_IntClock__PM_STBY_MSK, 0x01

/* I2C_I2C_IRQ */
.set I2C_I2C_IRQ__INTC_CLR_EN_REG, CYREG_NVIC_CLRENA0
.set I2C_I2C_IRQ__INTC_CLR_PD_REG, CYREG_NVIC_CLRPEND0
.set I2C_I2C_IRQ__INTC_MASK, 0x01
.set I2C_I2C_IRQ__INTC_NUMBER, 0
.set I2C_I2C_IRQ__INTC_PRIOR_NUM, 7
.set I2C_I2C_IRQ__INTC_PRIOR_REG, CYREG_NVIC_PRI_0
.set I2C_I2C_IRQ__INTC_SET_EN_REG, CYREG_NVIC_SETENA0
.set I2C_I2C_IRQ__INTC_SET_PD_REG, CYREG_NVIC_SETPEND0

/* LED_Out */
.set LED_Out_Sync_ctrl_reg__0__MASK, 0x01
.set LED_Out_Sync_ctrl_reg__0__POS, 0
.set LED_Out_Sync_ctrl_reg__16BIT_CONTROL_AUX_CTL_REG, CYREG_B0_UDB10_11_ACTL
.set LED_Out_Sync_ctrl_reg__16BIT_CONTROL_CONTROL_REG, CYREG_B0_UDB10_11_CTL
.set LED_Out_Sync_ctrl_reg__16BIT_CONTROL_COUNT_REG, CYREG_B0_UDB10_11_CTL
.set LED_Out_Sync_ctrl_reg__16BIT_COUNT_CONTROL_REG, CYREG_B0_UDB10_11_CTL
.set LED_Out_Sync_ctrl_reg__16BIT_COUNT_COUNT_REG, CYREG_B0_UDB10_11_CTL
.set LED_Out_Sync_ctrl_reg__16BIT_MASK_MASK_REG, CYREG_B0_UDB10_11_MSK
.set LED_Out_Sync_ctrl_reg__16BIT_MASK_PERIOD_REG, CYREG_B0_UDB10_11_MSK
.set LED_Out_Sync_ctrl_reg__16BIT_PERIOD_MASK_REG, CYREG_B0_UDB10_11_MSK
.set LED_Out_Sync_ctrl_reg__16BIT_PERIOD_PERIOD_REG, CYREG_B0_UDB10_11_MSK
.set LED_Out_Sync_ctrl_reg__1__MASK, 0x02
.set LED_Out_Sync_ctrl_reg__1__POS, 1
.set LED_Out_Sync_ctrl_reg__CONTROL_AUX_CTL_REG, CYREG_B0_UDB10_ACTL
.set LED_Out_Sync_ctrl_reg__CONTROL_REG, CYREG_B0_UDB10_CTL
.set LED_Out_Sync_ctrl_reg__CONTROL_ST_REG, CYREG_B0_UDB10_ST_CTL
.set LED_Out_Sync_ctrl_reg__COUNT_REG, CYREG_B0_UDB10_CTL
.set LED_Out_Sync_ctrl_reg__COUNT_ST_REG, CYREG_B0_UDB10_ST_CTL
.set LED_Out_Sync_ctrl_reg__MASK, 0x03
.set LED_Out_Sync_ctrl_reg__MASK_CTL_AUX_CTL_REG, CYREG_B0_UDB10_MSK_ACTL
.set LED_Out_Sync_ctrl_reg__PERIOD_REG, CYREG_B0_UDB10_MSK
.set LED_Out_Sync_ctrl_reg__PER_CTL_AUX_CTL_REG, CYREG_B0_UDB10_MSK_ACTL

/* LED_1 */
.set LED_1__0__MASK, 0x20
.set LED_1__0__PC, CYREG_IO_PC_PRT15_PC5
.set LED_1__0__PORT, 15
.set LED_1__0__SHIFT, 5
.set LED_1__AG, CYREG_PRT15_AG
.set LED_1__AMUX, CYREG_PRT15_AMUX
.set LED_1__BIE, CYREG_PRT15_BIE
.set LED_1__BIT_MASK, CYREG_PRT15_BIT_MASK
.set LED_1__BYP, CYREG_PRT15_BYP
.set LED_1__CTL, CYREG_PRT15_CTL
.set LED_1__DM0, CYREG_PRT15_DM0
.set LED_1__DM1, CYREG_PRT15_DM1
.set LED_1__DM2, CYREG_PRT15_DM2
.set LED_1__DR, CYREG_PRT15_DR
.set LED_1__INP_DIS, CYREG_PRT15_INP_DIS
.set LED_1__LCD_COM_SEG, CYREG_PRT15_LCD_COM_SEG
.set LED_1__LCD_EN, CYREG_PRT15_LCD_EN
.set LED_1__MASK, 0x20
.set LED_1__PORT, 15
.set LED_1__PRT, CYREG_PRT15_PRT
.set LED_1__PRTDSI__CAPS_SEL, CYREG_PRT15_CAPS_SEL
.set LED_1__PRTDSI__DBL_SYNC_IN, CYREG_PRT15_DBL_SYNC_IN
.set LED_1__PRTDSI__OE_SEL0, CYREG_PRT15_OE_SEL0
.set LED_1__PRTDSI__OE_SEL1, CYREG_PRT15_OE_SEL1
.set LED_1__PRTDSI__OUT_SEL0, CYREG_PRT15_OUT_SEL0
.set LED_1__PRTDSI__OUT_SEL1, CYREG_PRT15_OUT_SEL1
.set LED_1__PRTDSI__SYNC_OUT, CYREG_PRT15_SYNC_OUT
.set LED_1__PS, CYREG_PRT15_PS
.set LED_1__SHIFT, 5
.set LED_1__SLW, CYREG_PRT15_SLW

/* LED_2 */
.set LED_2__0__MASK, 0x10
.set LED_2__0__PC, CYREG_IO_PC_PRT15_PC4
.set LED_2__0__PORT, 15
.set LED_2__0__SHIFT, 4
.set LED_2__AG, CYREG_PRT15_AG
.set LED_2__AMUX, CYREG_PRT15_AMUX
.set LED_2__BIE, CYREG_PRT15_BIE
.set LED_2__BIT_MASK, CYREG_PRT15_BIT_MASK
.set LED_2__BYP, CYREG_PRT15_BYP
.set LED_2__CTL, CYREG_PRT15_CTL
.set LED_2__DM0, CYREG_PRT15_DM0
.set LED_2__DM1, CYREG_PRT15_DM1
.set LED_2__DM2, CYREG_PRT15_DM2
.set LED_2__DR, CYREG_PRT15_DR
.set LED_2__INP_DIS, CYREG_PRT15_INP_DIS
.set LED_2__LCD_COM_SEG, CYREG_PRT15_LCD_COM_SEG
.set LED_2__LCD_EN, CYREG_PRT15_LCD_EN
.set LED_2__MASK, 0x10
.set LED_2__PORT, 15
.set LED_2__PRT, CYREG_PRT15_PRT
.set LED_2__PRTDSI__CAPS_SEL, CYREG_PRT15_CAPS_SEL
.set LED_2__PRTDSI__DBL_SYNC_IN, CYREG_PRT15_DBL_SYNC_IN
.set LED_2__PRTDSI__OE_SEL0, CYREG_PRT15_OE_SEL0
.set LED_2__PRTDSI__OE_SEL1, CYREG_PRT15_OE_SEL1
.set LED_2__PRTDSI__OUT_SEL0, CYREG_PRT15_OUT_SEL0
.set LED_2__PRTDSI__OUT_SEL1, CYREG_PRT15_OUT_SEL1
.set LED_2__PRTDSI__SYNC_OUT, CYREG_PRT15_SYNC_OUT
.set LED_2__PS, CYREG_PRT15_PS
.set LED_2__SHIFT, 4
.set LED_2__SLW, CYREG_PRT15_SLW

/* SCL */
.set SCL__0__MASK, 0x01
.set SCL__0__PC, CYREG_PRT6_PC0
.set SCL__0__PORT, 6
.set SCL__0__SHIFT, 0
.set SCL__AG, CYREG_PRT6_AG
.set SCL__AMUX, CYREG_PRT6_AMUX
.set SCL__BIE, CYREG_PRT6_BIE
.set SCL__BIT_MASK, CYREG_PRT6_BIT_MASK
.set SCL__BYP, CYREG_PRT6_BYP
.set SCL__CTL, CYREG_PRT6_CTL
.set SCL__DM0, CYREG_PRT6_DM0
.set SCL__DM1, CYREG_PRT6_DM1
.set SCL__DM2, CYREG_PRT6_DM2
.set SCL__DR, CYREG_PRT6_DR
.set SCL__INP_DIS, CYREG_PRT6_INP_DIS
.set SCL__LCD_COM_SEG, CYREG_PRT6_LCD_COM_SEG
.set SCL__LCD_EN, CYREG_PRT6_LCD_EN
.set SCL__MASK, 0x01
.set SCL__PORT, 6
.set SCL__PRT, CYREG_PRT6_PRT
.set SCL__PRTDSI__CAPS_SEL, CYREG_PRT6_CAPS_SEL
.set SCL__PRTDSI__DBL_SYNC_IN, CYREG_PRT6_DBL_SYNC_IN
.set SCL__PRTDSI__OE_SEL0, CYREG_PRT6_OE_SEL0
.set SCL__PRTDSI__OE_SEL1, CYREG_PRT6_OE_SEL1
.set SCL__PRTDSI__OUT_SEL0, CYREG_PRT6_OUT_SEL0
.set SCL__PRTDSI__OUT_SEL1, CYREG_PRT6_OUT_SEL1
.set SCL__PRTDSI__SYNC_OUT, CYREG_PRT6_SYNC_OUT
.set SCL__PS, CYREG_PRT6_PS
.set SCL__SHIFT, 0
.set SCL__SLW, CYREG_PRT6_SLW

/* SDA */
.set SDA__0__MASK, 0x02
.set SDA__0__PC, CYREG_PRT6_PC1
.set SDA__0__PORT, 6
.set SDA__0__SHIFT, 1
.set SDA__AG, CYREG_PRT6_AG
.set SDA__AMUX, CYREG_PRT6_AMUX
.set SDA__BIE, CYREG_PRT6_BIE
.set SDA__BIT_MASK, CYREG_PRT6_BIT_MASK
.set SDA__BYP, CYREG_PRT6_BYP
.set SDA__CTL, CYREG_PRT6_CTL
.set SDA__DM0, CYREG_PRT6_DM0
.set SDA__DM1, CYREG_PRT6_DM1
.set SDA__DM2, CYREG_PRT6_DM2
.set SDA__DR, CYREG_PRT6_DR
.set SDA__INP_DIS, CYREG_PRT6_INP_DIS
.set SDA__LCD_COM_SEG, CYREG_PRT6_LCD_COM_SEG
.set SDA__LCD_EN, CYREG_PRT6_LCD_EN
.set SDA__MASK, 0x02
.set SDA__PORT, 6
.set SDA__PRT, CYREG_PRT6_PRT
.set SDA__PRTDSI__CAPS_SEL, CYREG_PRT6_CAPS_SEL
.set SDA__PRTDSI__DBL_SYNC_IN, CYREG_PRT6_DBL_SYNC_IN
.set SDA__PRTDSI__OE_SEL0, CYREG_PRT6_OE_SEL0
.set SDA__PRTDSI__OE_SEL1, CYREG_PRT6_OE_SEL1
.set SDA__PRTDSI__OUT_SEL0, CYREG_PRT6_OUT_SEL0
.set SDA__PRTDSI__OUT_SEL1, CYREG_PRT6_OUT_SEL1
.set SDA__PRTDSI__SYNC_OUT, CYREG_PRT6_SYNC_OUT
.set SDA__PS, CYREG_PRT6_PS
.set SDA__SHIFT, 1
.set SDA__SLW, CYREG_PRT6_SLW

/* Miscellaneous */
/* -- WARNING: define names containing LEOPARD or PANTHER are deprecated and will be removed in a future release */
.set CYDEV_DEBUGGING_DPS_SWD_SWV, 6
.set CYDEV_CONFIG_UNUSED_IO_AllowButWarn, 0
.set CYDEV_CONFIGURATION_MODE_COMPRESSED, 0
.set CYDEV_CONFIG_FASTBOOT_ENABLED, 1
.set CYDEV_CHIP_REV_PSOC5LP_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_5B_PRODUCTION, 0
.set CYDEV_CHIP_MEMBER_5B, 4
.set CYDEV_CHIP_FAMILY_PSOC5, 3
.set CYDEV_CHIP_DIE_PSOC5LP, 4
.set CYDEV_CHIP_DIE_EXPECT, CYDEV_CHIP_DIE_PSOC5LP
.set BCLK__BUS_CLK__HZ, 24000000
.set BCLK__BUS_CLK__KHZ, 24000
.set BCLK__BUS_CLK__MHZ, 24
.set CYDEV_CHIP_DIE_ACTUAL, CYDEV_CHIP_DIE_EXPECT
.set CYDEV_CHIP_DIE_LEOPARD, 1
.set CYDEV_CHIP_DIE_PANTHER, 3
.set CYDEV_CHIP_DIE_PSOC4A, 2
.set CYDEV_CHIP_DIE_UNKNOWN, 0
.set CYDEV_CHIP_FAMILY_PSOC3, 1
.set CYDEV_CHIP_FAMILY_PSOC4, 2
.set CYDEV_CHIP_FAMILY_UNKNOWN, 0
.set CYDEV_CHIP_FAMILY_USED, CYDEV_CHIP_FAMILY_PSOC5
.set CYDEV_CHIP_JTAG_ID, 0x2E123069
.set CYDEV_CHIP_MEMBER_3A, 1
.set CYDEV_CHIP_MEMBER_4A, 2
.set CYDEV_CHIP_MEMBER_5A, 3
.set CYDEV_CHIP_MEMBER_UNKNOWN, 0
.set CYDEV_CHIP_MEMBER_USED, CYDEV_CHIP_MEMBER_5B
.set CYDEV_CHIP_REVISION_3A_ES1, 0
.set CYDEV_CHIP_REVISION_3A_ES2, 1
.set CYDEV_CHIP_REVISION_3A_ES3, 3
.set CYDEV_CHIP_REVISION_3A_PRODUCTION, 3
.set CYDEV_CHIP_REVISION_4A_ES0, 17
.set CYDEV_CHIP_REVISION_4A_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_5A_ES0, 0
.set CYDEV_CHIP_REVISION_5A_ES1, 1
.set CYDEV_CHIP_REVISION_5A_PRODUCTION, 1
.set CYDEV_CHIP_REVISION_5B_ES0, 0
.set CYDEV_CHIP_REVISION_USED, CYDEV_CHIP_REVISION_5B_PRODUCTION
.set CYDEV_CHIP_REV_EXPECT, CYDEV_CHIP_REV_PSOC5LP_PRODUCTION
.set CYDEV_CHIP_REV_LEOPARD_ES1, 0
.set CYDEV_CHIP_REV_LEOPARD_ES2, 1
.set CYDEV_CHIP_REV_LEOPARD_ES3, 3
.set CYDEV_CHIP_REV_LEOPARD_PRODUCTION, 3
.set CYDEV_CHIP_REV_PANTHER_ES0, 0
.set CYDEV_CHIP_REV_PANTHER_ES1, 1
.set CYDEV_CHIP_REV_PANTHER_PRODUCTION, 1
.set CYDEV_CHIP_REV_PSOC4A_ES0, 17
.set CYDEV_CHIP_REV_PSOC4A_PRODUCTION, 17
.set CYDEV_CHIP_REV_PSOC5LP_ES0, 0
.set CYDEV_CONFIGURATION_COMPRESSED, 1
.set CYDEV_CONFIGURATION_DMA, 0
.set CYDEV_CONFIGURATION_ECC, 1
.set CYDEV_CONFIGURATION_IMOENABLED, CYDEV_CONFIG_FASTBOOT_ENABLED
.set CYDEV_CONFIGURATION_MODE, CYDEV_CONFIGURATION_MODE_COMPRESSED
.set CYDEV_CONFIGURATION_MODE_DMA, 2
.set CYDEV_CONFIGURATION_MODE_UNCOMPRESSED, 1
.set CYDEV_CONFIG_UNUSED_IO, CYDEV_CONFIG_UNUSED_IO_AllowButWarn
.set CYDEV_CONFIG_UNUSED_IO_AllowWithInfo, 1
.set CYDEV_CONFIG_UNUSED_IO_Disallowed, 2
.set CYDEV_DEBUGGING_DPS, CYDEV_DEBUGGING_DPS_SWD_SWV
.set CYDEV_DEBUGGING_DPS_Disable, 3
.set CYDEV_DEBUGGING_DPS_JTAG_4, 1
.set CYDEV_DEBUGGING_DPS_JTAG_5, 0
.set CYDEV_DEBUGGING_DPS_SWD, 2
.set CYDEV_DEBUGGING_ENABLE, 1
.set CYDEV_DEBUGGING_REQXRES, 1
.set CYDEV_DEBUGGING_XRES, 0
.set CYDEV_DEBUG_ENABLE_MASK, 0x20
.set CYDEV_DEBUG_ENABLE_REGISTER, CYREG_MLOGIC_DEBUG
.set CYDEV_DMA_CHANNELS_AVAILABLE, 24
.set CYDEV_ECC_ENABLE, 0
.set CYDEV_HEAP_SIZE, 0x1000
.set CYDEV_INSTRUCT_CACHE_ENABLED, 1
.set CYDEV_INTR_RISING, 0x00000001
.set CYDEV_PROJ_TYPE, 0
.set CYDEV_PROJ_TYPE_BOOTLOADER, 1
.set CYDEV_PROJ_TYPE_LOADABLE, 2
.set CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER, 3
.set CYDEV_PROJ_TYPE_STANDARD, 0
.set CYDEV_PROTECTION_ENABLE, 0
.set CYDEV_STACK_SIZE, 0x4000
.set CYDEV_USE_BUNDLED_CMSIS, 1
.set CYDEV_VARIABLE_VDDA, 0
.set CYDEV_VDDA_MV, 5000
.set CYDEV_VDDD_MV, 5000
.set CYDEV_VDDIO0_MV, 5000
.set CYDEV_VDDIO1_MV, 5000
.set CYDEV_VDDIO2_MV, 5000
.set CYDEV_VDDIO3_MV, 5000
.set CYDEV_VIO0, 5
.set CYDEV_VIO0_MV, 5000
.set CYDEV_VIO1, 5
.set CYDEV_VIO1_MV, 5000
.set CYDEV_VIO2, 5
.set CYDEV_VIO2_MV, 5000
.set CYDEV_VIO3, 5
.set CYDEV_VIO3_MV, 5000
.set DMA_CHANNELS_USED__MASK0, 0x00000000
.set CYDEV_BOOTLOADER_ENABLE, 0
.endif