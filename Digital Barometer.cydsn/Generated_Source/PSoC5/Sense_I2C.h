/*******************************************************************************
* File Name: Sense_I2C.h
* Version 3.30
*
* Description:
*  This file provides constants and parameter values for the I2C component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_I2C_Sense_I2C_H)
#define CY_I2C_Sense_I2C_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h"

/* Check if required defines such as CY_PSOC5LP are available in cy_boot */
#if !defined (CY_PSOC5LP)
    #error Component I2C_v3_30 requires cy_boot v3.10 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define Sense_I2C_IMPLEMENTATION     (0u)
#define Sense_I2C_MODE               (2u)
#define Sense_I2C_ENABLE_WAKEUP      (0u)
#define Sense_I2C_ADDR_DECODE        (1u)
#define Sense_I2C_UDB_INTRN_CLOCK    (1u)


/* I2C implementation enum */
#define Sense_I2C_UDB    (0x00u)
#define Sense_I2C_FF     (0x01u)

#define Sense_I2C_FF_IMPLEMENTED     (Sense_I2C_FF  == Sense_I2C_IMPLEMENTATION)
#define Sense_I2C_UDB_IMPLEMENTED    (Sense_I2C_UDB == Sense_I2C_IMPLEMENTATION)

#define Sense_I2C_UDB_INTRN_CLOCK_ENABLED    (Sense_I2C_UDB_IMPLEMENTED && \
                                                     (0u != Sense_I2C_UDB_INTRN_CLOCK))
/* I2C modes enum */
#define Sense_I2C_MODE_SLAVE                 (0x01u)
#define Sense_I2C_MODE_MASTER                (0x02u)
#define Sense_I2C_MODE_MULTI_MASTER          (0x06u)
#define Sense_I2C_MODE_MULTI_MASTER_SLAVE    (0x07u)
#define Sense_I2C_MODE_MULTI_MASTER_MASK     (0x04u)

#define Sense_I2C_MODE_SLAVE_ENABLED         (0u != (Sense_I2C_MODE_SLAVE  & Sense_I2C_MODE))
#define Sense_I2C_MODE_MASTER_ENABLED        (0u != (Sense_I2C_MODE_MASTER & Sense_I2C_MODE))
#define Sense_I2C_MODE_MULTI_MASTER_ENABLED  (0u != (Sense_I2C_MODE_MULTI_MASTER_MASK & \
                                                            Sense_I2C_MODE))
#define Sense_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED    (Sense_I2C_MODE_MULTI_MASTER_SLAVE == \
                                                             Sense_I2C_MODE)

/* Address detection enum */
#define Sense_I2C_SW_DECODE      (0x00u)
#define Sense_I2C_HW_DECODE      (0x01u)

#define Sense_I2C_SW_ADRR_DECODE             (Sense_I2C_SW_DECODE == Sense_I2C_ADDR_DECODE)
#define Sense_I2C_HW_ADRR_DECODE             (Sense_I2C_HW_DECODE == Sense_I2C_ADDR_DECODE)

/* Wakeup enabled */
#define Sense_I2C_WAKEUP_ENABLED             (0u != Sense_I2C_ENABLE_WAKEUP)

/* Adds bootloader APIs to component */
#define Sense_I2C_BOOTLOADER_INTERFACE_ENABLED   (Sense_I2C_MODE_SLAVE_ENABLED && \
                                                            ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Sense_I2C) || \
                                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)))

/* Timeout functionality */
#define Sense_I2C_TIMEOUT_ENABLE             (0u)
#define Sense_I2C_TIMEOUT_SCL_TMOUT_ENABLE   (0u)
#define Sense_I2C_TIMEOUT_SDA_TMOUT_ENABLE   (0u)
#define Sense_I2C_TIMEOUT_PRESCALER_ENABLE   (0u)
#define Sense_I2C_TIMEOUT_IMPLEMENTATION     (0u)

/* Convert to boolean */
#define Sense_I2C_TIMEOUT_ENABLED            (0u != Sense_I2C_TIMEOUT_ENABLE)
#define Sense_I2C_TIMEOUT_SCL_TMOUT_ENABLED  (0u != Sense_I2C_TIMEOUT_SCL_TMOUT_ENABLE)
#define Sense_I2C_TIMEOUT_SDA_TMOUT_ENABLED  (0u != Sense_I2C_TIMEOUT_SDA_TMOUT_ENABLE)
#define Sense_I2C_TIMEOUT_PRESCALER_ENABLED  (0u != Sense_I2C_TIMEOUT_PRESCALER_ENABLE)

/* Timeout implementation enum. */
#define Sense_I2C_TIMEOUT_UDB    (0x00u)
#define Sense_I2C_TIMEOUT_FF     (0x01u)

#define Sense_I2C_TIMEOUT_FF_IMPLEMENTED     (Sense_I2C_TIMEOUT_FF  == \
                                                        Sense_I2C_TIMEOUT_IMPLEMENTATION)
#define Sense_I2C_TIMEOUT_UDB_IMPLEMENTED    (Sense_I2C_TIMEOUT_UDB == \
                                                        Sense_I2C_TIMEOUT_IMPLEMENTATION)

#define Sense_I2C_TIMEOUT_FF_ENABLED         (Sense_I2C_TIMEOUT_ENABLED && \
                                                     Sense_I2C_TIMEOUT_FF_IMPLEMENTED && \
                                                     CY_PSOC5LP)

#define Sense_I2C_TIMEOUT_UDB_ENABLED        (Sense_I2C_TIMEOUT_ENABLED && \
                                                     Sense_I2C_TIMEOUT_UDB_IMPLEMENTED)

#define Sense_I2C_EXTERN_I2C_INTR_HANDLER    (0u)
#define Sense_I2C_EXTERN_TMOUT_INTR_HANDLER  (0u)

#define Sense_I2C_INTERN_I2C_INTR_HANDLER    (0u == Sense_I2C_EXTERN_I2C_INTR_HANDLER)
#define Sense_I2C_INTERN_TMOUT_INTR_HANDLER  (0u == Sense_I2C_EXTERN_TMOUT_INTR_HANDLER)


/***************************************
*       Type defines
***************************************/

/* Structure to save registers before go to sleep */
typedef struct
{
    uint8 enableState;

    #if(Sense_I2C_FF_IMPLEMENTED)
        uint8 xcfg;
        uint8 cfg;

        #if(Sense_I2C_MODE_SLAVE_ENABLED)
            uint8 addr;
        #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            uint8 clkDiv;
        #else
            uint8 clkDiv1;
            uint8 clkDiv2;
        #endif /* (CY_PSOC5A) */

    #else
        uint8 control;

        #if(CY_UDB_V0)
            uint8 intMask;

            #if(Sense_I2C_MODE_SLAVE_ENABLED)
                uint8 addr;
            #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */
        #endif     /* (CY_UDB_V0) */

    #endif /* (Sense_I2C_FF_IMPLEMENTED) */

    #if(Sense_I2C_TIMEOUT_ENABLED)
        uint16 tmoutCfg;
        uint8  tmoutIntr;

        #if(Sense_I2C_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0)
            uint8 tmoutPrd;
        #endif /* (Sense_I2C_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0) */

    #endif /* (Sense_I2C_TIMEOUT_ENABLED) */

} Sense_I2C_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void Sense_I2C_Init(void)                            ;
void Sense_I2C_Enable(void)                          ;

void Sense_I2C_Start(void)                           ;
void Sense_I2C_Stop(void)                            ;

#define Sense_I2C_EnableInt()        CyIntEnable      (Sense_I2C_ISR_NUMBER)
#define Sense_I2C_DisableInt()       CyIntDisable     (Sense_I2C_ISR_NUMBER)
#define Sense_I2C_ClearPendingInt()  CyIntClearPending(Sense_I2C_ISR_NUMBER)
#define Sense_I2C_SetPendingInt()    CyIntSetPending  (Sense_I2C_ISR_NUMBER)

void Sense_I2C_SaveConfig(void)                      ;
void Sense_I2C_Sleep(void)                           ;
void Sense_I2C_RestoreConfig(void)                   ;
void Sense_I2C_Wakeup(void)                          ;

/* I2C Master functions prototypes */
#if(Sense_I2C_MODE_MASTER_ENABLED)
    /* Read and Clear status functions */
    uint8 Sense_I2C_MasterStatus(void)                ;
    uint8 Sense_I2C_MasterClearStatus(void)           ;

    /* Interrupt based operation functions */
    uint8 Sense_I2C_MasterWriteBuf(uint8 slaveAddress, uint8 * wrData, uint8 cnt, uint8 mode) \
                                                            ;
    uint8 Sense_I2C_MasterReadBuf(uint8 slaveAddress, uint8 * rdData, uint8 cnt, uint8 mode) \
                                                            ;
    uint8 Sense_I2C_MasterGetReadBufSize(void)       ;
    uint8 Sense_I2C_MasterGetWriteBufSize(void)      ;
    void  Sense_I2C_MasterClearReadBuf(void)         ;
    void  Sense_I2C_MasterClearWriteBuf(void)        ;

    /* Manual operation functions */
    uint8 Sense_I2C_MasterSendStart(uint8 slaveAddress, uint8 R_nW) \
                                                            ;
    uint8 Sense_I2C_MasterSendRestart(uint8 slaveAddress, uint8 R_nW) \
                                                            ;
    uint8 Sense_I2C_MasterSendStop(void)             ;
    uint8 Sense_I2C_MasterWriteByte(uint8 theByte)   ;
    uint8 Sense_I2C_MasterReadByte(uint8 acknNak)    ;

    /* This fake function use as workaround */
    void  Sense_I2C_Workaround(void)                 ;

#endif /* (Sense_I2C_MODE_MASTER_ENABLED) */

/* I2C Slave functions prototypes */
#if(Sense_I2C_MODE_SLAVE_ENABLED)
    /* Read and Clear status functions */
    uint8 Sense_I2C_SlaveStatus(void)                ;
    uint8 Sense_I2C_SlaveClearReadStatus(void)       ;
    uint8 Sense_I2C_SlaveClearWriteStatus(void)      ;

    void  Sense_I2C_SlaveSetAddress(uint8 address)   ;

    /* Interrupt based operation functions */
    void  Sense_I2C_SlaveInitReadBuf(uint8 * rdBuf, uint8 bufSize) \
                                                            ;
    void  Sense_I2C_SlaveInitWriteBuf(uint8 * wrBuf, uint8 bufSize) \
                                                            ;
    uint8 Sense_I2C_SlaveGetReadBufSize(void)        ;
    uint8 Sense_I2C_SlaveGetWriteBufSize(void)       ;
    void  Sense_I2C_SlaveClearReadBuf(void)          ;
    void  Sense_I2C_SlaveClearWriteBuf(void)         ;

    /* Communication bootloader I2C Slave APIs */
    #if defined(CYDEV_BOOTLOADER_IO_COMP) && (Sense_I2C_BOOTLOADER_INTERFACE_ENABLED)
        /* Physical layer functions */
        void     Sense_I2C_CyBtldrCommStart(void) CYSMALL \
                                                            ;
        void     Sense_I2C_CyBtldrCommStop(void)  CYSMALL \
                                                            ;
        void     Sense_I2C_CyBtldrCommReset(void) CYSMALL \
                                                            ;
        cystatus Sense_I2C_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) \
                                                        CYSMALL ;
        cystatus Sense_I2C_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)  CYSMALL \
                                                            ;

        #if(CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Sense_I2C)
            #define CyBtldrCommStart    Sense_I2C_CyBtldrCommStart
            #define CyBtldrCommStop     Sense_I2C_CyBtldrCommStop
            #define CyBtldrCommReset    Sense_I2C_CyBtldrCommReset
            #define CyBtldrCommWrite    Sense_I2C_CyBtldrCommWrite
            #define CyBtldrCommRead     Sense_I2C_CyBtldrCommRead
        #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Sense_I2C) */

        /* Size of Read/Write buffers for I2C bootloader  */
        #define Sense_I2C_BTLDR_SIZEOF_READ_BUFFER   (0x80u)
        #define Sense_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (0x80u)
        #define Sense_I2C_MIN_UNT16(a, b)            ( ((uint16)(a) < (b)) ? ((uint16) (a)) : ((uint16) (b)) )
        #define Sense_I2C_WAIT_1_MS                  (1u)

    #endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (Sense_I2C_BOOTLOADER_INTERFACE_ENABLED) */

#endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

/* I2C interrupt handler */
CY_ISR_PROTO(Sense_I2C_ISR);
#if((Sense_I2C_FF_IMPLEMENTED) || (Sense_I2C_WAKEUP_ENABLED))
    CY_ISR_PROTO(Sense_I2C_WAKEUP_ISR);
#endif /* ((Sense_I2C_FF_IMPLEMENTED) || (Sense_I2C_WAKEUP_ENABLED)) */


/**********************************
*   Variable with external linkage
**********************************/

extern uint8 Sense_I2C_initVar;


/***************************************
*   Initial Parameter Constants
***************************************/

#define Sense_I2C_DATA_RATE          (400u)
#define Sense_I2C_DEFAULT_ADDR       (8u)
#define Sense_I2C_I2C_PAIR_SELECTED  (0u)

/* I2C pair enum */
#define Sense_I2C_I2C_PAIR_ANY   (0x01u) /* Any pins for I2C */
#define Sense_I2C_I2C_PAIR0      (0x01u) /* I2C0: (SCL = P12[4]) && (SCL = P12[5]) */
#define Sense_I2C_I2C_PAIR1      (0x02u) /* I2C1: (SCL = P12[0]) && (SDA = P12[1]) */

#define Sense_I2C_I2C1_SIO_PAIR  (Sense_I2C_I2C_PAIR1 == Sense_I2C_I2C_PAIR_SELECTED)
#define Sense_I2C_I2C0_SIO_PAIR  (Sense_I2C_I2C_PAIR0 == Sense_I2C_I2C_PAIR_SELECTED)


/***************************************
*            API Constants
***************************************/

/* Master/Slave control constants */
#define Sense_I2C_READ_XFER_MODE     (0x01u) /* Read */
#define Sense_I2C_WRITE_XFER_MODE    (0x00u) /* Write */
#define Sense_I2C_ACK_DATA           (0x01u) /* Send ACK */
#define Sense_I2C_NAK_DATA           (0x00u) /* Send NAK */
#define Sense_I2C_OVERFLOW_RETURN    (0xFFu) /* Senf on bus in case of overflow */

#if(Sense_I2C_MODE_MASTER_ENABLED)
    /* "Mode" constants for MasterWriteBuf() or MasterReadBuf() function */
    #define Sense_I2C_MODE_COMPLETE_XFER     (0x00u) /* Full transfer with Start and Stop */
    #define Sense_I2C_MODE_REPEAT_START      (0x01u) /* Begin with a ReStart instead of a Start */
    #define Sense_I2C_MODE_NO_STOP           (0x02u) /* Complete the transfer without a Stop */

    /* Master status */
    #define Sense_I2C_MSTAT_CLEAR            (0x00u) /* Clear (init) status value */

    #define Sense_I2C_MSTAT_RD_CMPLT         (0x01u) /* Read complete */
    #define Sense_I2C_MSTAT_WR_CMPLT         (0x02u) /* Write complete */
    #define Sense_I2C_MSTAT_XFER_INP         (0x04u) /* Master transfer in progress */
    #define Sense_I2C_MSTAT_XFER_HALT        (0x08u) /* Transfer is halted */

    #define Sense_I2C_MSTAT_ERR_MASK         (0xF0u) /* Mask for all errors */
    #define Sense_I2C_MSTAT_ERR_SHORT_XFER   (0x10u) /* Master NAKed before end of packet */
    #define Sense_I2C_MSTAT_ERR_ADDR_NAK     (0x20u) /* Slave did not ACK */
    #define Sense_I2C_MSTAT_ERR_ARB_LOST     (0x40u) /* Master lost arbitration during communication */
    #define Sense_I2C_MSTAT_ERR_XFER         (0x80u) /* Error during transfer */

    /* Master API returns */
    #define Sense_I2C_MSTR_NO_ERROR          (0x00u) /* Function complete without error */
    #define Sense_I2C_MSTR_BUS_BUSY          (0x01u) /* Bus is busy, process not started */
    #define Sense_I2C_MSTR_NOT_READY         (0x02u) /* Master not Master on the bus or */
                                                            /*  Slave operation in progress */
    #define Sense_I2C_MSTR_ERR_LB_NAK        (0x03u) /* Last Byte Naked */
    #define Sense_I2C_MSTR_ERR_ARB_LOST      (0x04u) /* Master lost arbitration during communication */
    #define Sense_I2C_MSTR_ERR_ABORT_START_GEN  (0x05u) /* Master did not generate Start, the Slave */
                                                               /* was addressed before */

#endif /* (Sense_I2C_MODE_MASTER_ENABLED) */

#if(Sense_I2C_MODE_SLAVE_ENABLED)
    /* Slave Status Constants */
    #define Sense_I2C_SSTAT_RD_CMPLT     (0x01u) /* Read transfer complete */
    #define Sense_I2C_SSTAT_RD_BUSY      (0x02u) /* Read transfer in progress */
    #define Sense_I2C_SSTAT_RD_ERR_OVFL  (0x04u) /* Read overflow Error */
    #define Sense_I2C_SSTAT_RD_MASK      (0x0Fu) /* Read Status Mask */
    #define Sense_I2C_SSTAT_RD_NO_ERR    (0x00u) /* Read no Error */

    #define Sense_I2C_SSTAT_WR_CMPLT     (0x10u) /* Write transfer complete */
    #define Sense_I2C_SSTAT_WR_BUSY      (0x20u) /* Write transfer in progress */
    #define Sense_I2C_SSTAT_WR_ERR_OVFL  (0x40u) /* Write overflow Error */
    #define Sense_I2C_SSTAT_WR_MASK      (0xF0u) /* Write Status Mask  */
    #define Sense_I2C_SSTAT_WR_NO_ERR    (0x00u) /* Write no Error */

    #define Sense_I2C_SSTAT_RD_CLEAR     (0x0Du) /* Read Status clear */
    #define Sense_I2C_SSTAT_WR_CLEAR     (0xD0u) /* Write Status Clear */

#endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */


/***************************************
*       I2C state machine constants
***************************************/

/* Default slave address states */
#define  Sense_I2C_SM_IDLE           (0x10u) /* Default state - IDLE */
#define  Sense_I2C_SM_EXIT_IDLE      (0x00u) /* Pass master and slave processing and go to IDLE */

/* Slave mode states */
#define  Sense_I2C_SM_SLAVE          (Sense_I2C_SM_IDLE) /* Any Slave state */
#define  Sense_I2C_SM_SL_WR_DATA     (0x11u) /* Master writes data to slzve  */
#define  Sense_I2C_SM_SL_RD_DATA     (0x12u) /* Master reads data from slave */

/* Master mode states */
#define  Sense_I2C_SM_MASTER         (0x40u) /* Any master state */

#define  Sense_I2C_SM_MSTR_RD        (0x08u) /* Any master read state          */
#define  Sense_I2C_SM_MSTR_RD_ADDR   (0x49u) /* Master sends address with read */
#define  Sense_I2C_SM_MSTR_RD_DATA   (0x4Au) /* Master reads data              */

#define  Sense_I2C_SM_MSTR_WR        (0x04u) /* Any master read state           */
#define  Sense_I2C_SM_MSTR_WR_ADDR   (0x45u) /* Master sends address with write */
#define  Sense_I2C_SM_MSTR_WR_DATA   (0x46u) /* Master writes data              */

#define  Sense_I2C_SM_MSTR_HALT      (0x60u) /* Master waits for ReStart */

#define Sense_I2C_CHECK_SM_MASTER    (0u != (Sense_I2C_SM_MASTER & Sense_I2C_state))
#define Sense_I2C_CHECK_SM_SLAVE     (0u != (Sense_I2C_SM_SLAVE  & Sense_I2C_state))


/***************************************
*              Registers
***************************************/

#if(Sense_I2C_FF_IMPLEMENTED)
    /* Fixed Function registers */
    #define Sense_I2C_XCFG_REG           (* (reg8 *) Sense_I2C_I2C_FF__XCFG)
    #define Sense_I2C_XCFG_PTR           (  (reg8 *) Sense_I2C_I2C_FF__XCFG)

    #define Sense_I2C_ADDR_REG           (* (reg8 *) Sense_I2C_I2C_FF__ADR)
    #define Sense_I2C_ADDR_PTR           (  (reg8 *) Sense_I2C_I2C_FF__ADR)

    #define Sense_I2C_CFG_REG            (* (reg8 *) Sense_I2C_I2C_FF__CFG)
    #define Sense_I2C_CFG_PTR            (  (reg8 *) Sense_I2C_I2C_FF__CFG)

    #define Sense_I2C_CSR_REG            (* (reg8 *) Sense_I2C_I2C_FF__CSR)
    #define Sense_I2C_CSR_PTR            (  (reg8 *) Sense_I2C_I2C_FF__CSR)

    #define Sense_I2C_DATA_REG           (* (reg8 *) Sense_I2C_I2C_FF__D)
    #define Sense_I2C_DATA_PTR           (  (reg8 *) Sense_I2C_I2C_FF__D)

    #define Sense_I2C_MCSR_REG           (* (reg8 *) Sense_I2C_I2C_FF__MCSR)
    #define Sense_I2C_MCSR_PTR           (  (reg8 *) Sense_I2C_I2C_FF__MCSR)

    #define Sense_I2C_ACT_PWRMGR_REG     (* (reg8 *) Sense_I2C_I2C_FF__PM_ACT_CFG)
    #define Sense_I2C_ACT_PWRMGR_PTR     (  (reg8 *) Sense_I2C_I2C_FF__PM_ACT_CFG)
    #define Sense_I2C_ACT_PWR_EN         (  (uint8)  Sense_I2C_I2C_FF__PM_ACT_MSK)

    #define Sense_I2C_STBY_PWRMGR_REG    (* (reg8 *) Sense_I2C_I2C_FF__PM_STBY_CFG)
    #define Sense_I2C_STBY_PWRMGR_PTR    (  (reg8 *) Sense_I2C_I2C_FF__PM_STBY_CFG)
    #define Sense_I2C_STBY_PWR_EN        (  (uint8)  Sense_I2C_I2C_FF__PM_STBY_MSK)

    #define Sense_I2C_PWRSYS_CR1_REG     (* (reg8 *) CYREG_PWRSYS_CR1)
    #define Sense_I2C_PWRSYS_CR1_PTR     (  (reg8 *) CYREG_PWRSYS_CR1)

    /* Clock divider register depends on silicon */
    #if(CY_PSOC5A)
        #define Sense_I2C_CLKDIV_REG     (* (reg8 *) Sense_I2C_I2C_FF__CLK_DIV)
        #define Sense_I2C_CLKDIV_PTR     (  (reg8 *) Sense_I2C_I2C_FF__CLK_DIV)

    #else
        #define Sense_I2C_CLKDIV1_REG    (* (reg8 *) Sense_I2C_I2C_FF__CLK_DIV1)
        #define Sense_I2C_CLKDIV1_PTR    (  (reg8 *) Sense_I2C_I2C_FF__CLK_DIV1)

        #define Sense_I2C_CLKDIV2_REG    (* (reg8 *) Sense_I2C_I2C_FF__CLK_DIV2)
        #define Sense_I2C_CLKDIV2_PTR    (  (reg8 *) Sense_I2C_I2C_FF__CLK_DIV2)

    #endif /* (CY_PSOC5A) */

#else
    /* UDB implementation registers */
    #define Sense_I2C_CFG_REG    (* (reg8 *) \
                                           Sense_I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)
    #define Sense_I2C_CFG_PTR    (  (reg8 *) \
                                           Sense_I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)

    #define Sense_I2C_CSR_REG        (* (reg8 *) Sense_I2C_bI2C_UDB_StsReg__STATUS_REG)
    #define Sense_I2C_CSR_PTR        (  (reg8 *) Sense_I2C_bI2C_UDB_StsReg__STATUS_REG)

    #define Sense_I2C_INT_MASK_REG   (* (reg8 *) Sense_I2C_bI2C_UDB_StsReg__MASK_REG)
    #define Sense_I2C_INT_MASK_PTR   (  (reg8 *) Sense_I2C_bI2C_UDB_StsReg__MASK_REG)

    #define Sense_I2C_INT_ENABLE_REG (* (reg8 *) Sense_I2C_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)
    #define Sense_I2C_INT_ENABLE_PTR (  (reg8 *) Sense_I2C_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)

    #define Sense_I2C_DATA_REG       (* (reg8 *) Sense_I2C_bI2C_UDB_Shifter_u0__A0_REG)
    #define Sense_I2C_DATA_PTR       (  (reg8 *) Sense_I2C_bI2C_UDB_Shifter_u0__A0_REG)

    #define Sense_I2C_GO_REG         (* (reg8 *) Sense_I2C_bI2C_UDB_Shifter_u0__F1_REG)
    #define Sense_I2C_GO_PTR         (  (reg8 *) Sense_I2C_bI2C_UDB_Shifter_u0__F1_REG)

    #define Sense_I2C_MCLK_PRD_REG   (* (reg8 *) Sense_I2C_bI2C_UDB_Master_ClkGen_u0__D0_REG)
    #define Sense_I2C_MCLK_PRD_PTR   (  (reg8 *) Sense_I2C_bI2C_UDB_Master_ClkGen_u0__D0_REG)

    #define Sense_I2C_MCLK_CMP_REG   (* (reg8 *) Sense_I2C_bI2C_UDB_Master_ClkGen_u0__D1_REG)
    #define Sense_I2C_MCLK_CMP_PTR   (  (reg8 *) Sense_I2C_bI2C_UDB_Master_ClkGen_u0__D1_REG)

    #if(Sense_I2C_MODE_SLAVE_ENABLED)
        #define Sense_I2C_ADDR_REG       (* (reg8 *) Sense_I2C_bI2C_UDB_Shifter_u0__D0_REG)
        #define Sense_I2C_ADDR_PTR       (  (reg8 *) Sense_I2C_bI2C_UDB_Shifter_u0__D0_REG)

        #define Sense_I2C_PERIOD_REG     (* (reg8 *) Sense_I2C_bI2C_UDB_Slave_BitCounter__PERIOD_REG)
        #define Sense_I2C_PERIOD_PTR     (  (reg8 *) Sense_I2C_bI2C_UDB_Slave_BitCounter__PERIOD_REG)

        #define Sense_I2C_COUNTER_REG    (* (reg8 *) Sense_I2C_bI2C_UDB_Slave_BitCounter__COUNT_REG)
        #define Sense_I2C_COUNTER_PTR    (  (reg8 *) Sense_I2C_bI2C_UDB_Slave_BitCounter__COUNT_REG)

        #define Sense_I2C_COUNTER_AUX_CTL_REG  (* (reg8 *) \
                                                        Sense_I2C_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG)
        #define Sense_I2C_COUNTER_AUX_CTL_PTR  (  (reg8 *) \
                                                        Sense_I2C_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG)

    #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

#endif /* (Sense_I2C_FF_IMPLEMENTED) */


/***************************************
*        Registers Constants
***************************************/

/* Sense_I2C_I2C_IRQ */
#define Sense_I2C_ISR_NUMBER     ((uint8) Sense_I2C_I2C_IRQ__INTC_NUMBER)
#define Sense_I2C_ISR_PRIORITY   ((uint8) Sense_I2C_I2C_IRQ__INTC_PRIOR_NUM)

/* I2C Slave Data Register */
#define Sense_I2C_SLAVE_ADDR_MASK    (0x7Fu)
#define Sense_I2C_SLAVE_ADDR_SHIFT   (0x01u)
#define Sense_I2C_DATA_MASK          (0xFFu)
#define Sense_I2C_READ_FLAG          (0x01u)

#define Sense_I2C_FF_RESET_DELAY     (0x02u)

#if(Sense_I2C_FF_IMPLEMENTED)
    /* XCFG I2C Extended Configuration Register */
    #define Sense_I2C_XCFG_CLK_EN        (0x80u) /* Enable gated clock to block */
    #define Sense_I2C_XCFG_I2C_ON        (0x40u) /* Enable I2C as wake up source*/
    #define Sense_I2C_XCFG_RDY_TO_SLEEP  (0x20u) /* I2C ready go to sleep */
    #define Sense_I2C_XCFG_FORCE_NACK    (0x10u) /* Force NACK all incomming transactions */
    #define Sense_I2C_XCFG_NO_BC_INT     (0x08u) /* No interrupt on byte complete */
    #define Sense_I2C_XCFG_BUF_MODE      (0x02u) /* Enable buffer mode */
    #define Sense_I2C_XCFG_HDWR_ADDR_EN  (0x01u) /* Enable Hardware address match */

    /* CFG I2C Configuration Register */
    #define Sense_I2C_CFG_SIO_SELECT     (0x80u) /* Pin Select for SCL/SDA lines */
    #define Sense_I2C_CFG_PSELECT        (0x40u) /* Pin Select */
    #define Sense_I2C_CFG_BUS_ERR_IE     (0x20u) /* Bus Error Interrupt Enable */
    #define Sense_I2C_CFG_STOP_IE        (0x10u) /* Enable Interrupt on STOP condition */
    #define Sense_I2C_CFG_CLK_RATE_MSK   (0x0Cu) /* Clock rate select  **CHECK**  */
    #define Sense_I2C_CFG_CLK_RATE_100   (0x00u) /* Clock rate select 100K */
    #define Sense_I2C_CFG_CLK_RATE_400   (0x04u) /* Clock rate select 400K */
    #define Sense_I2C_CFG_CLK_RATE_050   (0x08u) /* Clock rate select 50K  */
    #define Sense_I2C_CFG_CLK_RATE_RSVD  (0x0Cu) /* Clock rate select Invalid */
    #define Sense_I2C_CFG_EN_MSTR        (0x02u) /* Enable Master operation */
    #define Sense_I2C_CFG_EN_SLAVE       (0x01u) /* Enable Slave operation */

    #define Sense_I2C_CFG_CLK_RATE_LESS_EQUAL_50 (0x04u) /* Clock rate select <= 50kHz */
    #define Sense_I2C_CFG_CLK_RATE_GRATER_50     (0x00u) /* Clock rate select > 50kHz */

    /* CSR I2C Control and Status Register */
    #define Sense_I2C_CSR_BUS_ERROR      (0x80u) /* Active high when bus error has occured */
    #define Sense_I2C_CSR_LOST_ARB       (0x40u) /* Set to 1 if lost arbitration in host mode */
    #define Sense_I2C_CSR_STOP_STATUS    (0x20u) /* Set if Stop has been detected */
    #define Sense_I2C_CSR_ACK            (0x10u) /* ACK response */
    #define Sense_I2C_CSR_NAK            (0x00u) /* NAK response */
    #define Sense_I2C_CSR_ADDRESS        (0x08u) /* Set in firmware 0 = status bit, 1 Address is slave */
    #define Sense_I2C_CSR_TRANSMIT       (0x04u) /* Set in firmware 1 = transmit, 0 = receive */
    #define Sense_I2C_CSR_LRB            (0x02u) /* Last received bit */
    #define Sense_I2C_CSR_LRB_ACK        (0x00u) /* Last received bit was an ACK */
    #define Sense_I2C_CSR_LRB_NAK        (0x02u) /* Last received bit was an NAK */
    #define Sense_I2C_CSR_BYTE_COMPLETE  (0x01u) /* Informs that last byte has been sent */
    #define Sense_I2C_CSR_STOP_GEN       (0x00u) /* Generate a stop condition */
    #define Sense_I2C_CSR_RDY_TO_RD      (0x00u) /* Set to recieve mode */

    /* MCSR I2C Master Control and Status Register */
    #define Sense_I2C_MCSR_STOP_GEN      (0x10u) /* Firmware sets this bit to initiate a Stop condition */
    #define Sense_I2C_MCSR_BUS_BUSY      (0x08u) /* Status bit, Set at Start and cleared at Stop condition */
    #define Sense_I2C_MCSR_MSTR_MODE     (0x04u) /* Status bit, Set at Start and cleared at Stop condition */
    #define Sense_I2C_MCSR_RESTART_GEN   (0x02u) /* Firmware sets this bit to initiate a ReStart condition */
    #define Sense_I2C_MCSR_START_GEN     (0x01u) /* Firmware sets this bit to initiate a Start condition */

    /* CLK_DIV I2C Clock Divide Factor Register */
    #define Sense_I2C_CLK_DIV_MSK    (0x07u) /* Status bit, Set at Start and cleared at Stop condition */
    #define Sense_I2C_CLK_DIV_1      (0x00u) /* Divide input clock by  1 */
    #define Sense_I2C_CLK_DIV_2      (0x01u) /* Divide input clock by  2 */
    #define Sense_I2C_CLK_DIV_4      (0x02u) /* Divide input clock by  4 */
    #define Sense_I2C_CLK_DIV_8      (0x03u) /* Divide input clock by  8 */
    #define Sense_I2C_CLK_DIV_16     (0x04u) /* Divide input clock by 16 */
    #define Sense_I2C_CLK_DIV_32     (0x05u) /* Divide input clock by 32 */
    #define Sense_I2C_CLK_DIV_64     (0x06u) /* Divide input clock by 64 */

    /* PWRSYS_CR1 to handle Sleep */
    #define Sense_I2C_PWRSYS_CR1_I2C_REG_BACKUP  (0x04u) /* Enables, power to I2C regs while sleep */

#else
    /* CONTROL REG bits location */
    #define Sense_I2C_CTRL_START_SHIFT           (7u)
    #define Sense_I2C_CTRL_STOP_SHIFT            (6u)
    #define Sense_I2C_CTRL_RESTART_SHIFT         (5u)
    #define Sense_I2C_CTRL_NACK_SHIFT            (4u)
    #define Sense_I2C_CTRL_ANY_ADDRESS_SHIFT     (3u)
    #define Sense_I2C_CTRL_TRANSMIT_SHIFT        (2u)
    #define Sense_I2C_CTRL_ENABLE_MASTER_SHIFT   (1u)
    #define Sense_I2C_CTRL_ENABLE_SLAVE_SHIFT    (0u)
    #define Sense_I2C_CTRL_START_MASK            ((uint8) (0x01u << Sense_I2C_CTRL_START_SHIFT))
    #define Sense_I2C_CTRL_STOP_MASK             ((uint8) (0x01u << Sense_I2C_CTRL_STOP_SHIFT))
    #define Sense_I2C_CTRL_RESTART_MASK          ((uint8) (0x01u << Sense_I2C_CTRL_RESTART_SHIFT))
    #define Sense_I2C_CTRL_NACK_MASK             ((uint8) (0x01u << Sense_I2C_CTRL_NACK_SHIFT))
    #define Sense_I2C_CTRL_ANY_ADDRESS_MASK      ((uint8) (0x01u << Sense_I2C_CTRL_ANY_ADDRESS_SHIFT))
    #define Sense_I2C_CTRL_TRANSMIT_MASK         ((uint8) (0x01u << Sense_I2C_CTRL_TRANSMIT_SHIFT))
    #define Sense_I2C_CTRL_ENABLE_MASTER_MASK    ((uint8) (0x01u << Sense_I2C_CTRL_ENABLE_MASTER_SHIFT))
    #define Sense_I2C_CTRL_ENABLE_SLAVE_MASK     ((uint8) (0x01u << Sense_I2C_CTRL_ENABLE_SLAVE_SHIFT))

    /* STATUS REG bits location */
    #define Sense_I2C_STS_LOST_ARB_SHIFT         (6u)
    #define Sense_I2C_STS_STOP_SHIFT             (5u)
    #define Sense_I2C_STS_BUSY_SHIFT             (4u)
    #define Sense_I2C_STS_ADDR_SHIFT             (3u)
    #define Sense_I2C_STS_MASTER_MODE_SHIFT      (2u)
    #define Sense_I2C_STS_LRB_SHIFT              (1u)
    #define Sense_I2C_STS_BYTE_COMPLETE_SHIFT    (0u)
    #define Sense_I2C_STS_LOST_ARB_MASK          ((uint8) (0x01u << Sense_I2C_STS_LOST_ARB_SHIFT))
    #define Sense_I2C_STS_STOP_MASK              ((uint8) (0x01u << Sense_I2C_STS_STOP_SHIFT))
    #define Sense_I2C_STS_BUSY_MASK              ((uint8) (0x01u << Sense_I2C_STS_BUSY_SHIFT))
    #define Sense_I2C_STS_ADDR_MASK              ((uint8) (0x01u << Sense_I2C_STS_ADDR_SHIFT))
    #define Sense_I2C_STS_MASTER_MODE_MASK       ((uint8) (0x01u << Sense_I2C_STS_MASTER_MODE_SHIFT))
    #define Sense_I2C_STS_LRB_MASK               ((uint8) (0x01u << Sense_I2C_STS_LRB_SHIFT))
    #define Sense_I2C_STS_BYTE_COMPLETE_MASK     ((uint8) (0x01u << Sense_I2C_STS_BYTE_COMPLETE_SHIFT))

    /* AUX_CTL bits definition */
    #define Sense_I2C_COUNTER_ENABLE_MASK        (0x20u) /* Enable 7-bit counter     */
    #define Sense_I2C_INT_ENABLE_MASK            (0x10u) /* Enable intr from statusi */
    #define Sense_I2C_CNT7_ENABLE                (Sense_I2C_COUNTER_ENABLE_MASK)
    #define Sense_I2C_INTR_ENABLE                (Sense_I2C_INT_ENABLE_MASK)

#endif /* (Sense_I2C_FF_IMPLEMENTED) */


/***************************************
*        Marco
***************************************/

/* ACK and NACK for data and address checks */
#define Sense_I2C_CHECK_ADDR_ACK(csr)    ((Sense_I2C_CSR_LRB_ACK | Sense_I2C_CSR_ADDRESS) == \
                                                 ((Sense_I2C_CSR_LRB    | Sense_I2C_CSR_ADDRESS) &  \
                                                  (csr)))


#define Sense_I2C_CHECK_ADDR_NAK(csr)    ((Sense_I2C_CSR_LRB_NAK | Sense_I2C_CSR_ADDRESS) == \
                                                 ((Sense_I2C_CSR_LRB    | Sense_I2C_CSR_ADDRESS) &  \
                                                  (csr)))

#define Sense_I2C_CHECK_DATA_ACK(csr)    (0u == ((csr) & Sense_I2C_CSR_LRB_NAK))

/* MCSR conditions check */
#define Sense_I2C_CHECK_BUS_FREE(mcsr)       (0u == ((mcsr) & Sense_I2C_MCSR_BUS_BUSY))
#define Sense_I2C_CHECK_MASTER_MODE(mcsr)    (0u != ((mcsr) & Sense_I2C_MCSR_MSTR_MODE))

/* CSR conditions check */
#define Sense_I2C_WAIT_BYTE_COMPLETE(csr)    (0u == ((csr) & Sense_I2C_CSR_BYTE_COMPLETE))
#define Sense_I2C_WAIT_STOP_COMPLETE(csr)    (0u == ((csr) & (Sense_I2C_CSR_BYTE_COMPLETE | \
                                                                     Sense_I2C_CSR_STOP_STATUS)))
#define Sense_I2C_CHECK_BYTE_COMPLETE(csr)   (0u != ((csr) & Sense_I2C_CSR_BYTE_COMPLETE))
#define Sense_I2C_CHECK_STOP_STS(csr)        (0u != ((csr) & Sense_I2C_CSR_STOP_STATUS))
#define Sense_I2C_CHECK_LOST_ARB(csr)        (0u != ((csr) & Sense_I2C_CSR_LOST_ARB))
#define Sense_I2C_CHECK_ADDRESS_STS(csr)     (0u != ((csr) & Sense_I2C_CSR_ADDRESS))

/* Software start and end of transaction check */
#define Sense_I2C_CHECK_RESTART(mstrCtrl)    (0u != ((mstrCtrl) & Sense_I2C_MODE_REPEAT_START))
#define Sense_I2C_CHECK_NO_STOP(mstrCtrl)    (0u != ((mstrCtrl) & Sense_I2C_MODE_NO_STOP))

/* Send read or write completion depends on state */
#define Sense_I2C_GET_MSTAT_CMPLT ((0u != (Sense_I2C_state & Sense_I2C_SM_MSTR_RD)) ? \
                                                 (Sense_I2C_MSTAT_RD_CMPLT) : (Sense_I2C_MSTAT_WR_CMPLT))

/* Returns 7-bit slave address and used for software address match */
#define Sense_I2C_GET_SLAVE_ADDR(dataReg)   (((dataReg) >> Sense_I2C_SLAVE_ADDR_SHIFT) & \
                                                                  Sense_I2C_SLAVE_ADDR_MASK)

#if(Sense_I2C_FF_IMPLEMENTED)
    /* Check enable of module */
    #define Sense_I2C_I2C_ENABLE_REG     (Sense_I2C_ACT_PWRMGR_REG)
    #define Sense_I2C_IS_I2C_ENABLE(reg) (0u != ((reg) & Sense_I2C_ACT_PWR_EN))
    #define Sense_I2C_IS_ENABLED         (0u != (Sense_I2C_ACT_PWRMGR_REG & Sense_I2C_ACT_PWR_EN))

    #define Sense_I2C_CHECK_PWRSYS_I2C_BACKUP    (0u != (Sense_I2C_PWRSYS_CR1_I2C_REG_BACKUP & \
                                                                Sense_I2C_PWRSYS_CR1_REG))

    /* Check start condition generation */
    #define Sense_I2C_CHECK_START_GEN(mcsr)  ((0u != ((mcsr) & Sense_I2C_MCSR_START_GEN)) && \
                                                     (0u == ((mcsr) & Sense_I2C_MCSR_MSTR_MODE)))

    #define Sense_I2C_CLEAR_START_GEN        do{ \
                                                        Sense_I2C_MCSR_REG &=                                   \
                                                                           ((uint8) ~Sense_I2C_MCSR_START_GEN); \
                                                    }while(0)

    /* Stop interrupt */
    #define Sense_I2C_ENABLE_INT_ON_STOP     do{ \
                                                        Sense_I2C_CFG_REG |= Sense_I2C_CFG_STOP_IE; \
                                                    }while(0)

    #define Sense_I2C_DISABLE_INT_ON_STOP    do{ \
                                                        Sense_I2C_CFG_REG &=                                 \
                                                                           ((uint8) ~Sense_I2C_CFG_STOP_IE); \
                                                    }while(0)

    /* Transmit data */
    #define Sense_I2C_TRANSMIT_DATA          do{ \
                                                        Sense_I2C_CSR_REG = Sense_I2C_CSR_TRANSMIT; \
                                                    }while(0)

    #define Sense_I2C_ACK_AND_TRANSMIT       do{ \
                                                        Sense_I2C_CSR_REG = (Sense_I2C_CSR_ACK |      \
                                                                                    Sense_I2C_CSR_TRANSMIT); \
                                                    }while(0)

    #define Sense_I2C_NAK_AND_TRANSMIT       do{ \
                                                        Sense_I2C_CSR_REG = Sense_I2C_CSR_NAK; \
                                                    }while(0)

    /* Special case: udb needs to ack, ff needs to nak */
    #define Sense_I2C_ACKNAK_AND_TRANSMIT    do{ \
                                                        Sense_I2C_CSR_REG  = (Sense_I2C_CSR_NAK |      \
                                                                                     Sense_I2C_CSR_TRANSMIT); \
                                                    }while(0)
    /* Receive data */
    #define Sense_I2C_ACK_AND_RECEIVE        do{ \
                                                        Sense_I2C_CSR_REG = Sense_I2C_CSR_ACK; \
                                                    }while(0)

    #define Sense_I2C_NAK_AND_RECEIVE        do{ \
                                                        Sense_I2C_CSR_REG = Sense_I2C_CSR_NAK; \
                                                    }while(0)

    #define Sense_I2C_READY_TO_READ          do{ \
                                                        Sense_I2C_CSR_REG = Sense_I2C_CSR_RDY_TO_RD; \
                                                    }while(0)

    /* Master condition generation */
    #define Sense_I2C_GENERATE_START         do{ \
                                                        Sense_I2C_MCSR_REG = Sense_I2C_MCSR_START_GEN; \
                                                    }while(0)

    #if(CY_PSOC5A)
        #define Sense_I2C_GENERATE_RESTART \
                        do{ \
                            Sense_I2C_MCSR_REG = Sense_I2C_MCSR_RESTART_GEN; \
                            Sense_I2C_CSR_REG  = Sense_I2C_CSR_NAK;          \
                        }while(0)

        #define Sense_I2C_GENERATE_STOP      do{ \
                                                        Sense_I2C_CSR_REG = Sense_I2C_CSR_STOP_GEN; \
                                                    }while(0)

    #else   /* PSoC3 ES3 handlees zero lenght packets */
        #define Sense_I2C_GENERATE_RESTART \
                        do{ \
                            Sense_I2C_MCSR_REG = (Sense_I2C_MCSR_RESTART_GEN | \
                                                         Sense_I2C_MCSR_STOP_GEN);    \
                            Sense_I2C_CSR_REG  = Sense_I2C_CSR_TRANSMIT;       \
                        }while(0)

        #define Sense_I2C_GENERATE_STOP \
                        do{ \
                            Sense_I2C_MCSR_REG = Sense_I2C_MCSR_STOP_GEN; \
                            Sense_I2C_CSR_REG  = Sense_I2C_CSR_TRANSMIT;  \
                        }while(0)
    #endif /* (CY_PSOC5A) */

    /* Master manual APIs compatible defines */
    #define Sense_I2C_GENERATE_RESTART_MANUAL    Sense_I2C_GENERATE_RESTART
    #define Sense_I2C_GENERATE_STOP_MANUAL       Sense_I2C_GENERATE_STOP
    #define Sense_I2C_TRANSMIT_DATA_MANUAL       Sense_I2C_TRANSMIT_DATA
    #define Sense_I2C_READY_TO_READ_MANUAL       Sense_I2C_READY_TO_READ
    #define Sense_I2C_ACK_AND_RECEIVE_MANUAL     Sense_I2C_ACK_AND_RECEIVE

#else

    /* Masks to enalbe interrupts from Status register */
    #define Sense_I2C_STOP_IE_MASK           (Sense_I2C_STS_STOP_MASK)
    #define Sense_I2C_BYTE_COMPLETE_IE_MASK  (Sense_I2C_STS_BYTE_COMPLETE_MASK)

    /* FF compatibility: CSR gegisters definitions */
    #define Sense_I2C_CSR_LOST_ARB       (Sense_I2C_STS_LOST_ARB_MASK)
    #define Sense_I2C_CSR_STOP_STATUS    (Sense_I2C_STS_STOP_MASK)
    #define Sense_I2C_CSR_BUS_ERROR      (0x00u)
    #define Sense_I2C_CSR_ADDRESS        (Sense_I2C_STS_ADDR_MASK)
    #define Sense_I2C_CSR_TRANSMIT       (Sense_I2C_CTRL_TRANSMIT_MASK)
    #define Sense_I2C_CSR_LRB            (Sense_I2C_STS_LRB_MASK)
    #define Sense_I2C_CSR_LRB_NAK        (Sense_I2C_STS_LRB_MASK)
    #define Sense_I2C_CSR_LRB_ACK        (0x00u)
    #define Sense_I2C_CSR_BYTE_COMPLETE  (Sense_I2C_STS_BYTE_COMPLETE_MASK)

    /* FF compatibility: MCSR gegisters definitions */
    #define Sense_I2C_MCSR_REG           (Sense_I2C_CSR_REG)   /* UDB incoporates master and slave regs */
    #define Sense_I2C_MCSR_BUS_BUSY      (Sense_I2C_STS_BUSY_MASK)       /* Is bus is busy              */
    #define Sense_I2C_MCSR_START_GEN     (Sense_I2C_CTRL_START_MASK)     /* Generate Sart condition     */
    #define Sense_I2C_MCSR_RESTART_GEN   (Sense_I2C_CTRL_RESTART_MASK)   /* Generates RESTART condition */
    #define Sense_I2C_MCSR_MSTR_MODE     (Sense_I2C_STS_MASTER_MODE_MASK)/* Define if active Master     */

    /* Data to write into TX FIFO to release FSM */
    #define Sense_I2C_RELEASE_FSM         (0x00u)
    
    /* Check enable of module */
    #define Sense_I2C_I2C_ENABLE_REG     (Sense_I2C_CFG_REG)
    #define Sense_I2C_IS_I2C_ENABLE(reg) ((0u != ((reg) & Sense_I2C_ENABLE_MASTER)) || \
                                                 (0u != ((reg) & Sense_I2C_ENABLE_SLAVE)))

    #define Sense_I2C_IS_ENABLED         (0u != (Sense_I2C_CFG_REG & Sense_I2C_ENABLE_MS))

    /* Check start condition generation */
    #define Sense_I2C_CHECK_START_GEN(mcsr)  ((0u != (Sense_I2C_CFG_REG &        \
                                                             Sense_I2C_MCSR_START_GEN)) \
                                                    &&                                         \
                                                    (0u == ((mcsr) & Sense_I2C_MCSR_MSTR_MODE)))

    #define Sense_I2C_CLEAR_START_GEN        do{ \
                                                        Sense_I2C_CFG_REG &=                 \
                                                        ((uint8) ~Sense_I2C_MCSR_START_GEN); \
                                                    }while(0)


    /* Stop interrupt */
    #define Sense_I2C_ENABLE_INT_ON_STOP     do{ \
                                                       Sense_I2C_INT_MASK_REG |= Sense_I2C_STOP_IE_MASK; \
                                                    }while(0)

    #define Sense_I2C_DISABLE_INT_ON_STOP    do{ \
                                                        Sense_I2C_INT_MASK_REG &=                               \
                                                                             ((uint8) ~Sense_I2C_STOP_IE_MASK); \
                                                    }while(0)


    /* Transmit data */
    #define Sense_I2C_TRANSMIT_DATA      do{ \
                                                    Sense_I2C_CFG_REG = (Sense_I2C_CTRL_TRANSMIT_MASK | \
                                                                                Sense_I2C_CTRL_DEFAULT);       \
                                                    Sense_I2C_GO_REG  = Sense_I2C_RELEASE_FSM;          \
                                                }while(0)

    #define Sense_I2C_ACK_AND_TRANSMIT   Sense_I2C_TRANSMIT_DATA


    #define Sense_I2C_NAK_AND_TRANSMIT   do{ \
                                                    Sense_I2C_CFG_REG = (Sense_I2C_CTRL_NACK_MASK     | \
                                                                                Sense_I2C_CTRL_TRANSMIT_MASK | \
                                                                                Sense_I2C_CTRL_DEFAULT);       \
                                                    Sense_I2C_GO_REG  =  Sense_I2C_RELEASE_FSM;         \
                                                }while(0)

    /* Receive data */
    #define Sense_I2C_READY_TO_READ      do{ \
                                                    Sense_I2C_CFG_REG = Sense_I2C_CTRL_DEFAULT; \
                                                    Sense_I2C_GO_REG  =  Sense_I2C_RELEASE_FSM; \
                                                }while(0)

    #define Sense_I2C_ACK_AND_RECEIVE    Sense_I2C_READY_TO_READ

    #define Sense_I2C_NAK_AND_RECEIVE    do{ \
                                                    Sense_I2C_CFG_REG = (Sense_I2C_CTRL_NACK_MASK | \
                                                                                Sense_I2C_CTRL_DEFAULT);   \
                                                    Sense_I2C_GO_REG  =  Sense_I2C_RELEASE_FSM;     \
                                                }while(0)

    /* Master condition generation */
    #define Sense_I2C_GENERATE_START     do{ \
                                                    Sense_I2C_CFG_REG = (Sense_I2C_CTRL_START_MASK | \
                                                                                 Sense_I2C_CTRL_DEFAULT);   \
                                                    Sense_I2C_GO_REG  =  Sense_I2C_RELEASE_FSM;      \
                                                }while(0)

    #define Sense_I2C_GENERATE_RESTART   do{ \
                                                    Sense_I2C_CFG_REG = (Sense_I2C_CTRL_RESTART_MASK | \
                                                                                Sense_I2C_CTRL_NACK_MASK    | \
                                                                                Sense_I2C_CTRL_DEFAULT);      \
                                                    Sense_I2C_GO_REG  =  Sense_I2C_RELEASE_FSM;        \
                                                }while(0)


    #define Sense_I2C_GENERATE_STOP      do{ \
                                                    Sense_I2C_CFG_REG = (Sense_I2C_CTRL_NACK_MASK | \
                                                                                Sense_I2C_CTRL_STOP_MASK | \
                                                                                Sense_I2C_CTRL_DEFAULT);   \
                                                    Sense_I2C_GO_REG  =  Sense_I2C_RELEASE_FSM;     \
                                                }while(0)

    /* Master manual APIs compatible defines */
    /* These defines wait while byte complete is cleared after command issued */
    #define Sense_I2C_GENERATE_RESTART_MANUAL    \
                                        do{             \
                                            Sense_I2C_GENERATE_RESTART;                                    \
                                            while(Sense_I2C_CHECK_BYTE_COMPLETE(Sense_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define Sense_I2C_GENERATE_STOP_MANUAL   \
                                        do{         \
                                            Sense_I2C_GENERATE_STOP;                                       \
                                            while(Sense_I2C_CHECK_BYTE_COMPLETE(Sense_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define Sense_I2C_TRANSMIT_DATA_MANUAL   \
                                        do{         \
                                            Sense_I2C_TRANSMIT_DATA;                                       \
                                            while(Sense_I2C_CHECK_BYTE_COMPLETE(Sense_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define Sense_I2C_READY_TO_READ_MANUAL   \
                                        do{         \
                                            Sense_I2C_READY_TO_READ;      \
                                            while(Sense_I2C_CHECK_BYTE_COMPLETE(Sense_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define Sense_I2C_ACK_AND_RECEIVE_MANUAL \
                                        do{         \
                                            Sense_I2C_ACK_AND_RECEIVE;                                     \
                                            while(Sense_I2C_CHECK_BYTE_COMPLETE(Sense_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)
#endif /* (Sense_I2C_FF_IMPLEMENTED) */

/* Comon for FF and UDB: used to release bus after lost arb */
#define Sense_I2C_BUS_RELEASE    Sense_I2C_READY_TO_READ


/***************************************
*     Default register init constants
***************************************/

#define Sense_I2C_DISABLE    (0u)
#define Sense_I2C_ENABLE     (1u)

#if(Sense_I2C_FF_IMPLEMENTED)
    /* Sense_I2C_XCFG_REG: bits definition */
    #define Sense_I2C_DEFAULT_XCFG_HW_ADDR_EN ((Sense_I2C_HW_ADRR_DECODE) ? \
                                                        (Sense_I2C_XCFG_HDWR_ADDR_EN) : (0u))

    #define Sense_I2C_DEFAULT_XCFG_I2C_ON    ((Sense_I2C_WAKEUP_ENABLED) ? \
                                                        (Sense_I2C_XCFG_I2C_ON) : (0u))


    #define Sense_I2C_DEFAULT_CFG_SIO_SELECT ((Sense_I2C_I2C1_SIO_PAIR) ? \
                                                        (Sense_I2C_CFG_SIO_SELECT) : (0u))


    /* Sense_I2C_CFG_REG: bits definition */
    #define Sense_I2C_DEFAULT_CFG_PSELECT    ((Sense_I2C_WAKEUP_ENABLED) ? \
                                                        (Sense_I2C_CFG_PSELECT) : (0u))

    #define Sense_I2C_DEFAULT_CLK_RATE0  ((Sense_I2C_DATA_RATE <= 50u) ?        \
                                                    (Sense_I2C_CFG_CLK_RATE_050) :     \
                                                    ((Sense_I2C_DATA_RATE <= 100u) ?   \
                                                        (Sense_I2C_CFG_CLK_RATE_100) : \
                                                        (Sense_I2C_CFG_CLK_RATE_400)))

    #define Sense_I2C_DEFAULT_CLK_RATE1  ((Sense_I2C_DATA_RATE <= 50u) ?           \
                                                 (Sense_I2C_CFG_CLK_RATE_LESS_EQUAL_50) : \
                                                 (Sense_I2C_CFG_CLK_RATE_GRATER_50))

    #define Sense_I2C_DEFAULT_CLK_RATE   ((CY_PSOC5A) ? (Sense_I2C_DEFAULT_CLK_RATE0) : \
                                                               (Sense_I2C_DEFAULT_CLK_RATE1))


    #define Sense_I2C_ENABLE_MASTER      ((Sense_I2C_MODE_MASTER_ENABLED) ? \
                                                 (Sense_I2C_CFG_EN_MSTR) : (0u))

    #define Sense_I2C_ENABLE_SLAVE       ((Sense_I2C_MODE_SLAVE_ENABLED) ? \
                                                 (Sense_I2C_CFG_EN_SLAVE) : (0u))

    #define Sense_I2C_ENABLE_MS      (Sense_I2C_ENABLE_MASTER | Sense_I2C_ENABLE_SLAVE)


    /* Sense_I2C_DEFAULT_XCFG_REG */
    #define Sense_I2C_DEFAULT_XCFG   (Sense_I2C_XCFG_CLK_EN         | \
                                             Sense_I2C_DEFAULT_XCFG_I2C_ON | \
                                             Sense_I2C_DEFAULT_XCFG_HW_ADDR_EN)

    /* Sense_I2C_DEFAULT_CFG_REG */
    #define Sense_I2C_DEFAULT_CFG    (Sense_I2C_DEFAULT_CFG_SIO_SELECT | \
                                             Sense_I2C_DEFAULT_CFG_PSELECT    | \
                                             Sense_I2C_DEFAULT_CLK_RATE       | \
                                             Sense_I2C_ENABLE_MASTER          | \
                                             Sense_I2C_ENABLE_SLAVE)

    /*Sense_I2C_DEFAULT_DIVIDE_FACTOR_REG */
    #define Sense_I2C_DEFAULT_DIVIDE_FACTOR  ((CY_PSOC5A) ? ((uint8) 0u) : ((uint16) 1u))

#else
    /* Sense_I2C_CFG_REG: bits definition  */
    #define Sense_I2C_ENABLE_MASTER  ((Sense_I2C_MODE_MASTER_ENABLED) ? \
                                             (Sense_I2C_CTRL_ENABLE_MASTER_MASK) : (0u))

    #define Sense_I2C_ENABLE_SLAVE   ((Sense_I2C_MODE_SLAVE_ENABLED) ? \
                                             (Sense_I2C_CTRL_ENABLE_SLAVE_MASK) : (0u))

    #define Sense_I2C_ENABLE_MS      (Sense_I2C_ENABLE_MASTER | Sense_I2C_ENABLE_SLAVE)


    #define Sense_I2C_DEFAULT_CTRL_ANY_ADDR   ((Sense_I2C_HW_ADRR_DECODE) ? \
                                                      (0u) : (Sense_I2C_CTRL_ANY_ADDRESS_MASK))

    /* Sense_I2C_DEFAULT_CFG_REG */
    #define Sense_I2C_DEFAULT_CFG    (Sense_I2C_DEFAULT_CTRL_ANY_ADDR)

    /* All CTRL default bits to be used in macro */
    #define Sense_I2C_CTRL_DEFAULT   (Sense_I2C_DEFAULT_CTRL_ANY_ADDR | Sense_I2C_ENABLE_MS)

    /* Master clock generator: d0 and d1 */
    #define Sense_I2C_MCLK_PERIOD_VALUE  (0x0Fu)
    #define Sense_I2C_MCLK_COMPARE_VALUE (0x08u)

    /* Slave bit-counter: contorol period */
    #define Sense_I2C_PERIOD_VALUE       (0x07u)

    /* Sense_I2C_DEFAULT_INT_MASK */
    #define Sense_I2C_DEFAULT_INT_MASK   (Sense_I2C_BYTE_COMPLETE_IE_MASK)

    /* Sense_I2C_DEFAULT_MCLK_PRD_REG */
    #define Sense_I2C_DEFAULT_MCLK_PRD   (Sense_I2C_MCLK_PERIOD_VALUE)

    /* Sense_I2C_DEFAULT_MCLK_CMP_REG */
    #define Sense_I2C_DEFAULT_MCLK_CMP   (Sense_I2C_MCLK_COMPARE_VALUE)

    /* Sense_I2C_DEFAULT_PERIOD_REG */
    #define Sense_I2C_DEFAULT_PERIOD     (Sense_I2C_PERIOD_VALUE)

#endif /* (Sense_I2C_FF_IMPLEMENTED) */


/***************************************
*       Obsolete
***************************************/

/* Following code are OBSOLETE and must not be used 
 * starting from I2C 3.20
 */
 
#define Sense_I2C_SSTAT_RD_ERR       (0x08u)
#define Sense_I2C_SSTAT_WR_ERR       (0x80u)
#define Sense_I2C_MSTR_SLAVE_BUSY    (Sense_I2C_MSTR_NOT_READY)
#define Sense_I2C_MSTAT_ERR_BUF_OVFL (0x80u)
#define Sense_I2C_SSTAT_RD_CMPT      (Sense_I2C_SSTAT_RD_CMPLT)
#define Sense_I2C_SSTAT_WR_CMPT      (Sense_I2C_SSTAT_WR_CMPLT)
#define Sense_I2C_MODE_MULTI_MASTER_ENABLE    (Sense_I2C_MODE_MULTI_MASTER_MASK)
#define Sense_I2C_DATA_RATE_50       (50u)
#define Sense_I2C_DATA_RATE_100      (100u)
#define Sense_I2C_DEV_MASK           (0xF0u)
#define Sense_I2C_SM_SL_STOP         (0x14u)
#define Sense_I2C_SM_MASTER_IDLE     (0x40u)
#define Sense_I2C_HDWR_DECODE        (0x01u)

#endif /* CY_I2C_Sense_I2C_H */


/* [] END OF FILE */
