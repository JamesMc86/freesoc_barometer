#ifndef INCLUDED_CYFITTER_H
#define INCLUDED_CYFITTER_H
#include <cydevice.h>
#include <cydevice_trm.h>

/* I2C_bI2C_UDB */
#define I2C_bI2C_UDB_Master_ClkGen_u0__A0_A1_REG CYREG_B0_UDB15_A0_A1
#define I2C_bI2C_UDB_Master_ClkGen_u0__A0_REG CYREG_B0_UDB15_A0
#define I2C_bI2C_UDB_Master_ClkGen_u0__A1_REG CYREG_B0_UDB15_A1
#define I2C_bI2C_UDB_Master_ClkGen_u0__D0_D1_REG CYREG_B0_UDB15_D0_D1
#define I2C_bI2C_UDB_Master_ClkGen_u0__D0_REG CYREG_B0_UDB15_D0
#define I2C_bI2C_UDB_Master_ClkGen_u0__D1_REG CYREG_B0_UDB15_D1
#define I2C_bI2C_UDB_Master_ClkGen_u0__DP_AUX_CTL_REG CYREG_B0_UDB15_ACTL
#define I2C_bI2C_UDB_Master_ClkGen_u0__F0_F1_REG CYREG_B0_UDB15_F0_F1
#define I2C_bI2C_UDB_Master_ClkGen_u0__F0_REG CYREG_B0_UDB15_F0
#define I2C_bI2C_UDB_Master_ClkGen_u0__F1_REG CYREG_B0_UDB15_F1
#define I2C_bI2C_UDB_Shifter_u0__16BIT_A0_REG CYREG_B0_UDB12_13_A0
#define I2C_bI2C_UDB_Shifter_u0__16BIT_A1_REG CYREG_B0_UDB12_13_A1
#define I2C_bI2C_UDB_Shifter_u0__16BIT_D0_REG CYREG_B0_UDB12_13_D0
#define I2C_bI2C_UDB_Shifter_u0__16BIT_D1_REG CYREG_B0_UDB12_13_D1
#define I2C_bI2C_UDB_Shifter_u0__16BIT_DP_AUX_CTL_REG CYREG_B0_UDB12_13_ACTL
#define I2C_bI2C_UDB_Shifter_u0__16BIT_F0_REG CYREG_B0_UDB12_13_F0
#define I2C_bI2C_UDB_Shifter_u0__16BIT_F1_REG CYREG_B0_UDB12_13_F1
#define I2C_bI2C_UDB_Shifter_u0__A0_A1_REG CYREG_B0_UDB12_A0_A1
#define I2C_bI2C_UDB_Shifter_u0__A0_REG CYREG_B0_UDB12_A0
#define I2C_bI2C_UDB_Shifter_u0__A1_REG CYREG_B0_UDB12_A1
#define I2C_bI2C_UDB_Shifter_u0__D0_D1_REG CYREG_B0_UDB12_D0_D1
#define I2C_bI2C_UDB_Shifter_u0__D0_REG CYREG_B0_UDB12_D0
#define I2C_bI2C_UDB_Shifter_u0__D1_REG CYREG_B0_UDB12_D1
#define I2C_bI2C_UDB_Shifter_u0__DP_AUX_CTL_REG CYREG_B0_UDB12_ACTL
#define I2C_bI2C_UDB_Shifter_u0__F0_F1_REG CYREG_B0_UDB12_F0_F1
#define I2C_bI2C_UDB_Shifter_u0__F0_REG CYREG_B0_UDB12_F0
#define I2C_bI2C_UDB_Shifter_u0__F1_REG CYREG_B0_UDB12_F1
#define I2C_bI2C_UDB_StsReg__0__MASK 0x01u
#define I2C_bI2C_UDB_StsReg__0__POS 0
#define I2C_bI2C_UDB_StsReg__16BIT_STATUS_AUX_CTL_REG CYREG_B0_UDB11_12_ACTL
#define I2C_bI2C_UDB_StsReg__16BIT_STATUS_REG CYREG_B0_UDB11_12_ST
#define I2C_bI2C_UDB_StsReg__1__MASK 0x02u
#define I2C_bI2C_UDB_StsReg__1__POS 1
#define I2C_bI2C_UDB_StsReg__2__MASK 0x04u
#define I2C_bI2C_UDB_StsReg__2__POS 2
#define I2C_bI2C_UDB_StsReg__3__MASK 0x08u
#define I2C_bI2C_UDB_StsReg__3__POS 3
#define I2C_bI2C_UDB_StsReg__4__MASK 0x10u
#define I2C_bI2C_UDB_StsReg__4__POS 4
#define I2C_bI2C_UDB_StsReg__5__MASK 0x20u
#define I2C_bI2C_UDB_StsReg__5__POS 5
#define I2C_bI2C_UDB_StsReg__MASK 0x3Fu
#define I2C_bI2C_UDB_StsReg__MASK_REG CYREG_B0_UDB11_MSK
#define I2C_bI2C_UDB_StsReg__MASK_ST_AUX_CTL_REG CYREG_B0_UDB11_MSK_ACTL
#define I2C_bI2C_UDB_StsReg__PER_ST_AUX_CTL_REG CYREG_B0_UDB11_MSK_ACTL
#define I2C_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG CYREG_B0_UDB11_ACTL
#define I2C_bI2C_UDB_StsReg__STATUS_CNT_REG CYREG_B0_UDB11_ST_CTL
#define I2C_bI2C_UDB_StsReg__STATUS_CONTROL_REG CYREG_B0_UDB11_ST_CTL
#define I2C_bI2C_UDB_StsReg__STATUS_REG CYREG_B0_UDB11_ST
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_CONTROL_AUX_CTL_REG CYREG_B0_UDB11_12_ACTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_CONTROL_CONTROL_REG CYREG_B0_UDB11_12_CTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_CONTROL_COUNT_REG CYREG_B0_UDB11_12_CTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_COUNT_CONTROL_REG CYREG_B0_UDB11_12_CTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_COUNT_COUNT_REG CYREG_B0_UDB11_12_CTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_MASK_MASK_REG CYREG_B0_UDB11_12_MSK
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_MASK_PERIOD_REG CYREG_B0_UDB11_12_MSK
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_PERIOD_MASK_REG CYREG_B0_UDB11_12_MSK
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__16BIT_PERIOD_PERIOD_REG CYREG_B0_UDB11_12_MSK
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__1__MASK 0x02u
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__1__POS 1
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__2__MASK 0x04u
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__2__POS 2
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__4__MASK 0x10u
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__4__POS 4
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__5__MASK 0x20u
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__5__POS 5
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__6__MASK 0x40u
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__6__POS 6
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__7__MASK 0x80u
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__7__POS 7
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_AUX_CTL_REG CYREG_B0_UDB11_ACTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG CYREG_B0_UDB11_CTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_ST_REG CYREG_B0_UDB11_ST_CTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__COUNT_REG CYREG_B0_UDB11_CTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__COUNT_ST_REG CYREG_B0_UDB11_ST_CTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__MASK 0xF6u
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__MASK_CTL_AUX_CTL_REG CYREG_B0_UDB11_MSK_ACTL
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__PERIOD_REG CYREG_B0_UDB11_MSK
#define I2C_bI2C_UDB_SyncCtl_CtrlReg__PER_CTL_AUX_CTL_REG CYREG_B0_UDB11_MSK_ACTL

/* I2C_IntClock */
#define I2C_IntClock__CFG0 CYREG_CLKDIST_DCFG0_CFG0
#define I2C_IntClock__CFG1 CYREG_CLKDIST_DCFG0_CFG1
#define I2C_IntClock__CFG2 CYREG_CLKDIST_DCFG0_CFG2
#define I2C_IntClock__CFG2_SRC_SEL_MASK 0x07u
#define I2C_IntClock__INDEX 0x00u
#define I2C_IntClock__PM_ACT_CFG CYREG_PM_ACT_CFG2
#define I2C_IntClock__PM_ACT_MSK 0x01u
#define I2C_IntClock__PM_STBY_CFG CYREG_PM_STBY_CFG2
#define I2C_IntClock__PM_STBY_MSK 0x01u

/* I2C_I2C_IRQ */
#define I2C_I2C_IRQ__INTC_CLR_EN_REG CYREG_NVIC_CLRENA0
#define I2C_I2C_IRQ__INTC_CLR_PD_REG CYREG_NVIC_CLRPEND0
#define I2C_I2C_IRQ__INTC_MASK 0x01u
#define I2C_I2C_IRQ__INTC_NUMBER 0u
#define I2C_I2C_IRQ__INTC_PRIOR_NUM 7u
#define I2C_I2C_IRQ__INTC_PRIOR_REG CYREG_NVIC_PRI_0
#define I2C_I2C_IRQ__INTC_SET_EN_REG CYREG_NVIC_SETENA0
#define I2C_I2C_IRQ__INTC_SET_PD_REG CYREG_NVIC_SETPEND0

/* LED_Out */
#define LED_Out_Sync_ctrl_reg__0__MASK 0x01u
#define LED_Out_Sync_ctrl_reg__0__POS 0
#define LED_Out_Sync_ctrl_reg__16BIT_CONTROL_AUX_CTL_REG CYREG_B0_UDB10_11_ACTL
#define LED_Out_Sync_ctrl_reg__16BIT_CONTROL_CONTROL_REG CYREG_B0_UDB10_11_CTL
#define LED_Out_Sync_ctrl_reg__16BIT_CONTROL_COUNT_REG CYREG_B0_UDB10_11_CTL
#define LED_Out_Sync_ctrl_reg__16BIT_COUNT_CONTROL_REG CYREG_B0_UDB10_11_CTL
#define LED_Out_Sync_ctrl_reg__16BIT_COUNT_COUNT_REG CYREG_B0_UDB10_11_CTL
#define LED_Out_Sync_ctrl_reg__16BIT_MASK_MASK_REG CYREG_B0_UDB10_11_MSK
#define LED_Out_Sync_ctrl_reg__16BIT_MASK_PERIOD_REG CYREG_B0_UDB10_11_MSK
#define LED_Out_Sync_ctrl_reg__16BIT_PERIOD_MASK_REG CYREG_B0_UDB10_11_MSK
#define LED_Out_Sync_ctrl_reg__16BIT_PERIOD_PERIOD_REG CYREG_B0_UDB10_11_MSK
#define LED_Out_Sync_ctrl_reg__1__MASK 0x02u
#define LED_Out_Sync_ctrl_reg__1__POS 1
#define LED_Out_Sync_ctrl_reg__CONTROL_AUX_CTL_REG CYREG_B0_UDB10_ACTL
#define LED_Out_Sync_ctrl_reg__CONTROL_REG CYREG_B0_UDB10_CTL
#define LED_Out_Sync_ctrl_reg__CONTROL_ST_REG CYREG_B0_UDB10_ST_CTL
#define LED_Out_Sync_ctrl_reg__COUNT_REG CYREG_B0_UDB10_CTL
#define LED_Out_Sync_ctrl_reg__COUNT_ST_REG CYREG_B0_UDB10_ST_CTL
#define LED_Out_Sync_ctrl_reg__MASK 0x03u
#define LED_Out_Sync_ctrl_reg__MASK_CTL_AUX_CTL_REG CYREG_B0_UDB10_MSK_ACTL
#define LED_Out_Sync_ctrl_reg__PERIOD_REG CYREG_B0_UDB10_MSK
#define LED_Out_Sync_ctrl_reg__PER_CTL_AUX_CTL_REG CYREG_B0_UDB10_MSK_ACTL

/* LED_1 */
#define LED_1__0__MASK 0x20u
#define LED_1__0__PC CYREG_IO_PC_PRT15_PC5
#define LED_1__0__PORT 15u
#define LED_1__0__SHIFT 5
#define LED_1__AG CYREG_PRT15_AG
#define LED_1__AMUX CYREG_PRT15_AMUX
#define LED_1__BIE CYREG_PRT15_BIE
#define LED_1__BIT_MASK CYREG_PRT15_BIT_MASK
#define LED_1__BYP CYREG_PRT15_BYP
#define LED_1__CTL CYREG_PRT15_CTL
#define LED_1__DM0 CYREG_PRT15_DM0
#define LED_1__DM1 CYREG_PRT15_DM1
#define LED_1__DM2 CYREG_PRT15_DM2
#define LED_1__DR CYREG_PRT15_DR
#define LED_1__INP_DIS CYREG_PRT15_INP_DIS
#define LED_1__LCD_COM_SEG CYREG_PRT15_LCD_COM_SEG
#define LED_1__LCD_EN CYREG_PRT15_LCD_EN
#define LED_1__MASK 0x20u
#define LED_1__PORT 15u
#define LED_1__PRT CYREG_PRT15_PRT
#define LED_1__PRTDSI__CAPS_SEL CYREG_PRT15_CAPS_SEL
#define LED_1__PRTDSI__DBL_SYNC_IN CYREG_PRT15_DBL_SYNC_IN
#define LED_1__PRTDSI__OE_SEL0 CYREG_PRT15_OE_SEL0
#define LED_1__PRTDSI__OE_SEL1 CYREG_PRT15_OE_SEL1
#define LED_1__PRTDSI__OUT_SEL0 CYREG_PRT15_OUT_SEL0
#define LED_1__PRTDSI__OUT_SEL1 CYREG_PRT15_OUT_SEL1
#define LED_1__PRTDSI__SYNC_OUT CYREG_PRT15_SYNC_OUT
#define LED_1__PS CYREG_PRT15_PS
#define LED_1__SHIFT 5
#define LED_1__SLW CYREG_PRT15_SLW

/* LED_2 */
#define LED_2__0__MASK 0x10u
#define LED_2__0__PC CYREG_IO_PC_PRT15_PC4
#define LED_2__0__PORT 15u
#define LED_2__0__SHIFT 4
#define LED_2__AG CYREG_PRT15_AG
#define LED_2__AMUX CYREG_PRT15_AMUX
#define LED_2__BIE CYREG_PRT15_BIE
#define LED_2__BIT_MASK CYREG_PRT15_BIT_MASK
#define LED_2__BYP CYREG_PRT15_BYP
#define LED_2__CTL CYREG_PRT15_CTL
#define LED_2__DM0 CYREG_PRT15_DM0
#define LED_2__DM1 CYREG_PRT15_DM1
#define LED_2__DM2 CYREG_PRT15_DM2
#define LED_2__DR CYREG_PRT15_DR
#define LED_2__INP_DIS CYREG_PRT15_INP_DIS
#define LED_2__LCD_COM_SEG CYREG_PRT15_LCD_COM_SEG
#define LED_2__LCD_EN CYREG_PRT15_LCD_EN
#define LED_2__MASK 0x10u
#define LED_2__PORT 15u
#define LED_2__PRT CYREG_PRT15_PRT
#define LED_2__PRTDSI__CAPS_SEL CYREG_PRT15_CAPS_SEL
#define LED_2__PRTDSI__DBL_SYNC_IN CYREG_PRT15_DBL_SYNC_IN
#define LED_2__PRTDSI__OE_SEL0 CYREG_PRT15_OE_SEL0
#define LED_2__PRTDSI__OE_SEL1 CYREG_PRT15_OE_SEL1
#define LED_2__PRTDSI__OUT_SEL0 CYREG_PRT15_OUT_SEL0
#define LED_2__PRTDSI__OUT_SEL1 CYREG_PRT15_OUT_SEL1
#define LED_2__PRTDSI__SYNC_OUT CYREG_PRT15_SYNC_OUT
#define LED_2__PS CYREG_PRT15_PS
#define LED_2__SHIFT 4
#define LED_2__SLW CYREG_PRT15_SLW

/* SCL */
#define SCL__0__MASK 0x01u
#define SCL__0__PC CYREG_PRT6_PC0
#define SCL__0__PORT 6u
#define SCL__0__SHIFT 0
#define SCL__AG CYREG_PRT6_AG
#define SCL__AMUX CYREG_PRT6_AMUX
#define SCL__BIE CYREG_PRT6_BIE
#define SCL__BIT_MASK CYREG_PRT6_BIT_MASK
#define SCL__BYP CYREG_PRT6_BYP
#define SCL__CTL CYREG_PRT6_CTL
#define SCL__DM0 CYREG_PRT6_DM0
#define SCL__DM1 CYREG_PRT6_DM1
#define SCL__DM2 CYREG_PRT6_DM2
#define SCL__DR CYREG_PRT6_DR
#define SCL__INP_DIS CYREG_PRT6_INP_DIS
#define SCL__LCD_COM_SEG CYREG_PRT6_LCD_COM_SEG
#define SCL__LCD_EN CYREG_PRT6_LCD_EN
#define SCL__MASK 0x01u
#define SCL__PORT 6u
#define SCL__PRT CYREG_PRT6_PRT
#define SCL__PRTDSI__CAPS_SEL CYREG_PRT6_CAPS_SEL
#define SCL__PRTDSI__DBL_SYNC_IN CYREG_PRT6_DBL_SYNC_IN
#define SCL__PRTDSI__OE_SEL0 CYREG_PRT6_OE_SEL0
#define SCL__PRTDSI__OE_SEL1 CYREG_PRT6_OE_SEL1
#define SCL__PRTDSI__OUT_SEL0 CYREG_PRT6_OUT_SEL0
#define SCL__PRTDSI__OUT_SEL1 CYREG_PRT6_OUT_SEL1
#define SCL__PRTDSI__SYNC_OUT CYREG_PRT6_SYNC_OUT
#define SCL__PS CYREG_PRT6_PS
#define SCL__SHIFT 0
#define SCL__SLW CYREG_PRT6_SLW

/* SDA */
#define SDA__0__MASK 0x02u
#define SDA__0__PC CYREG_PRT6_PC1
#define SDA__0__PORT 6u
#define SDA__0__SHIFT 1
#define SDA__AG CYREG_PRT6_AG
#define SDA__AMUX CYREG_PRT6_AMUX
#define SDA__BIE CYREG_PRT6_BIE
#define SDA__BIT_MASK CYREG_PRT6_BIT_MASK
#define SDA__BYP CYREG_PRT6_BYP
#define SDA__CTL CYREG_PRT6_CTL
#define SDA__DM0 CYREG_PRT6_DM0
#define SDA__DM1 CYREG_PRT6_DM1
#define SDA__DM2 CYREG_PRT6_DM2
#define SDA__DR CYREG_PRT6_DR
#define SDA__INP_DIS CYREG_PRT6_INP_DIS
#define SDA__LCD_COM_SEG CYREG_PRT6_LCD_COM_SEG
#define SDA__LCD_EN CYREG_PRT6_LCD_EN
#define SDA__MASK 0x02u
#define SDA__PORT 6u
#define SDA__PRT CYREG_PRT6_PRT
#define SDA__PRTDSI__CAPS_SEL CYREG_PRT6_CAPS_SEL
#define SDA__PRTDSI__DBL_SYNC_IN CYREG_PRT6_DBL_SYNC_IN
#define SDA__PRTDSI__OE_SEL0 CYREG_PRT6_OE_SEL0
#define SDA__PRTDSI__OE_SEL1 CYREG_PRT6_OE_SEL1
#define SDA__PRTDSI__OUT_SEL0 CYREG_PRT6_OUT_SEL0
#define SDA__PRTDSI__OUT_SEL1 CYREG_PRT6_OUT_SEL1
#define SDA__PRTDSI__SYNC_OUT CYREG_PRT6_SYNC_OUT
#define SDA__PS CYREG_PRT6_PS
#define SDA__SHIFT 1
#define SDA__SLW CYREG_PRT6_SLW

/* Miscellaneous */
/* -- WARNING: define names containing LEOPARD or PANTHER are deprecated and will be removed in a future release */
#define CYDEV_DEBUGGING_DPS_SWD_SWV 6
#define CYDEV_CONFIG_UNUSED_IO_AllowButWarn 0
#define CYDEV_CONFIGURATION_MODE_COMPRESSED 0
#define CYDEV_CONFIG_FASTBOOT_ENABLED 1
#define CYDEV_CHIP_REV_PSOC5LP_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_5B_PRODUCTION 0u
#define CYDEV_CHIP_MEMBER_5B 4u
#define CYDEV_CHIP_FAMILY_PSOC5 3u
#define CYDEV_CHIP_DIE_PSOC5LP 4u
#define CYDEV_CHIP_DIE_EXPECT CYDEV_CHIP_DIE_PSOC5LP
#define BCLK__BUS_CLK__HZ 24000000U
#define BCLK__BUS_CLK__KHZ 24000U
#define BCLK__BUS_CLK__MHZ 24U
#define CYDEV_CHIP_DIE_ACTUAL CYDEV_CHIP_DIE_EXPECT
#define CYDEV_CHIP_DIE_LEOPARD 1u
#define CYDEV_CHIP_DIE_PANTHER 3u
#define CYDEV_CHIP_DIE_PSOC4A 2u
#define CYDEV_CHIP_DIE_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_PSOC3 1u
#define CYDEV_CHIP_FAMILY_PSOC4 2u
#define CYDEV_CHIP_FAMILY_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_USED CYDEV_CHIP_FAMILY_PSOC5
#define CYDEV_CHIP_JTAG_ID 0x2E123069u
#define CYDEV_CHIP_MEMBER_3A 1u
#define CYDEV_CHIP_MEMBER_4A 2u
#define CYDEV_CHIP_MEMBER_5A 3u
#define CYDEV_CHIP_MEMBER_UNKNOWN 0u
#define CYDEV_CHIP_MEMBER_USED CYDEV_CHIP_MEMBER_5B
#define CYDEV_CHIP_REVISION_3A_ES1 0u
#define CYDEV_CHIP_REVISION_3A_ES2 1u
#define CYDEV_CHIP_REVISION_3A_ES3 3u
#define CYDEV_CHIP_REVISION_3A_PRODUCTION 3u
#define CYDEV_CHIP_REVISION_4A_ES0 17u
#define CYDEV_CHIP_REVISION_4A_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_5A_ES0 0u
#define CYDEV_CHIP_REVISION_5A_ES1 1u
#define CYDEV_CHIP_REVISION_5A_PRODUCTION 1u
#define CYDEV_CHIP_REVISION_5B_ES0 0u
#define CYDEV_CHIP_REVISION_USED CYDEV_CHIP_REVISION_5B_PRODUCTION
#define CYDEV_CHIP_REV_EXPECT CYDEV_CHIP_REV_PSOC5LP_PRODUCTION
#define CYDEV_CHIP_REV_LEOPARD_ES1 0u
#define CYDEV_CHIP_REV_LEOPARD_ES2 1u
#define CYDEV_CHIP_REV_LEOPARD_ES3 3u
#define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3u
#define CYDEV_CHIP_REV_PANTHER_ES0 0u
#define CYDEV_CHIP_REV_PANTHER_ES1 1u
#define CYDEV_CHIP_REV_PANTHER_PRODUCTION 1u
#define CYDEV_CHIP_REV_PSOC4A_ES0 17u
#define CYDEV_CHIP_REV_PSOC4A_PRODUCTION 17u
#define CYDEV_CHIP_REV_PSOC5LP_ES0 0u
#define CYDEV_CONFIGURATION_COMPRESSED 1
#define CYDEV_CONFIGURATION_DMA 0
#define CYDEV_CONFIGURATION_ECC 1
#define CYDEV_CONFIGURATION_IMOENABLED CYDEV_CONFIG_FASTBOOT_ENABLED
#define CYDEV_CONFIGURATION_MODE CYDEV_CONFIGURATION_MODE_COMPRESSED
#define CYDEV_CONFIGURATION_MODE_DMA 2
#define CYDEV_CONFIGURATION_MODE_UNCOMPRESSED 1
#define CYDEV_CONFIG_UNUSED_IO CYDEV_CONFIG_UNUSED_IO_AllowButWarn
#define CYDEV_CONFIG_UNUSED_IO_AllowWithInfo 1
#define CYDEV_CONFIG_UNUSED_IO_Disallowed 2
#define CYDEV_DEBUGGING_DPS CYDEV_DEBUGGING_DPS_SWD_SWV
#define CYDEV_DEBUGGING_DPS_Disable 3
#define CYDEV_DEBUGGING_DPS_JTAG_4 1
#define CYDEV_DEBUGGING_DPS_JTAG_5 0
#define CYDEV_DEBUGGING_DPS_SWD 2
#define CYDEV_DEBUGGING_ENABLE 1
#define CYDEV_DEBUGGING_REQXRES 1
#define CYDEV_DEBUGGING_XRES 0
#define CYDEV_DEBUG_ENABLE_MASK 0x20u
#define CYDEV_DEBUG_ENABLE_REGISTER CYREG_MLOGIC_DEBUG
#define CYDEV_DMA_CHANNELS_AVAILABLE 24u
#define CYDEV_ECC_ENABLE 0
#define CYDEV_HEAP_SIZE 0x1000
#define CYDEV_INSTRUCT_CACHE_ENABLED 1
#define CYDEV_INTR_RISING 0x00000001u
#define CYDEV_PROJ_TYPE 0
#define CYDEV_PROJ_TYPE_BOOTLOADER 1
#define CYDEV_PROJ_TYPE_LOADABLE 2
#define CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER 3
#define CYDEV_PROJ_TYPE_STANDARD 0
#define CYDEV_PROTECTION_ENABLE 0
#define CYDEV_STACK_SIZE 0x4000
#define CYDEV_USE_BUNDLED_CMSIS 1
#define CYDEV_VARIABLE_VDDA 0
#define CYDEV_VDDA 5.0
#define CYDEV_VDDA_MV 5000
#define CYDEV_VDDD 5.0
#define CYDEV_VDDD_MV 5000
#define CYDEV_VDDIO0 5.0
#define CYDEV_VDDIO0_MV 5000
#define CYDEV_VDDIO1 5.0
#define CYDEV_VDDIO1_MV 5000
#define CYDEV_VDDIO2 5.0
#define CYDEV_VDDIO2_MV 5000
#define CYDEV_VDDIO3 5.0
#define CYDEV_VDDIO3_MV 5000
#define CYDEV_VIO0 5
#define CYDEV_VIO0_MV 5000
#define CYDEV_VIO1 5
#define CYDEV_VIO1_MV 5000
#define CYDEV_VIO2 5
#define CYDEV_VIO2_MV 5000
#define CYDEV_VIO3 5
#define CYDEV_VIO3_MV 5000
#define DMA_CHANNELS_USED__MASK0 0x00000000u
#define CYDEV_BOOTLOADER_ENABLE 0

#endif /* INCLUDED_CYFITTER_H */
