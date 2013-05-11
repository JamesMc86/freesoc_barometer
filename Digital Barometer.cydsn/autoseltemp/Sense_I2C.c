/*******************************************************************************
* File Name: Sense_I2C.c
* Version 3.30
*
* Description:
*  This file provides the source code of APIs for the I2C component.
*  Actual protocol and operation code resides in the interrupt service routine
*  file.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Sense_I2C_PVT.h"


/**********************************
*      System variables
**********************************/

uint8 Sense_I2C_initVar = 0u;    /* Defines if component was initialized */

volatile uint8 Sense_I2C_state;  /* Current state of I2C FSM */


/*******************************************************************************
* Function Name: Sense_I2C_Init
********************************************************************************
*
* Summary:
*  Initializes I2C registers with initial values provided from customizer.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void Sense_I2C_Init(void) 
{
    #if(Sense_I2C_FF_IMPLEMENTED)
        Sense_I2C_CFG_REG  = Sense_I2C_DEFAULT_CFG;
        Sense_I2C_XCFG_REG = Sense_I2C_DEFAULT_XCFG;

        #if(CY_PSOC5A)
            Sense_I2C_CLKDIV_REG  = LO8(Sense_I2C_DEFAULT_DIVIDE_FACTOR);
        #else
            Sense_I2C_CLKDIV1_REG = LO8(Sense_I2C_DEFAULT_DIVIDE_FACTOR);
            Sense_I2C_CLKDIV2_REG = HI8(Sense_I2C_DEFAULT_DIVIDE_FACTOR);
        #endif /* (CY_PSOC5A) */

    #else
        uint8 enableInterrupts;

        Sense_I2C_CFG_REG      = Sense_I2C_DEFAULT_CFG;      /* control  */
        Sense_I2C_INT_MASK_REG = Sense_I2C_DEFAULT_INT_MASK; /* int_mask */

        /* Enable interrupts from block */
        enableInterrupts = CyEnterCriticalSection();
        Sense_I2C_INT_ENABLE_REG |= Sense_I2C_INTR_ENABLE; /* aux_ctl */
        CyExitCriticalSection(enableInterrupts);

        #if(Sense_I2C_MODE_MASTER_ENABLED)
            Sense_I2C_MCLK_PRD_REG = Sense_I2C_DEFAULT_MCLK_PRD;
            Sense_I2C_MCLK_CMP_REG = Sense_I2C_DEFAULT_MCLK_CMP;
         #endif /* (Sense_I2C_MODE_MASTER_ENABLED) */

        #if(Sense_I2C_MODE_SLAVE_ENABLED)
            Sense_I2C_PERIOD_REG = Sense_I2C_DEFAULT_PERIOD;
        #endif  /* (Sense_I2C_MODE_SLAVE_ENABLED) */

    #endif /* (Sense_I2C_FF_IMPLEMENTED) */

    #if(Sense_I2C_TIMEOUT_ENABLED)
        Sense_I2C_TimeoutInit();
    #endif /* (Sense_I2C_TIMEOUT_ENABLED) */

    /* Disable Interrupt and set vector and priority */
    CyIntDisable    (Sense_I2C_ISR_NUMBER);
    CyIntSetPriority(Sense_I2C_ISR_NUMBER, Sense_I2C_ISR_PRIORITY);
    #if(Sense_I2C_INTERN_I2C_INTR_HANDLER)
        (void) CyIntSetVector(Sense_I2C_ISR_NUMBER, &Sense_I2C_ISR);
    #endif /* (Sense_I2C_INTERN_I2C_INTR_HANDLER) */


    /* Put state machine in idle state */
    Sense_I2C_state = Sense_I2C_SM_IDLE;

    #if(Sense_I2C_MODE_SLAVE_ENABLED)
        /* Reset status and buffers index */
        Sense_I2C_SlaveClearReadBuf();
        Sense_I2C_SlaveClearWriteBuf();
        Sense_I2C_slStatus = 0u; /* Reset slave status */

        /* Set default address */
        Sense_I2C_SlaveSetAddress(Sense_I2C_DEFAULT_ADDR);
    #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

    #if(Sense_I2C_MODE_MASTER_ENABLED)
        /* Reset status and buffers index */
        Sense_I2C_MasterClearReadBuf();
        Sense_I2C_MasterClearWriteBuf();
        (void) Sense_I2C_MasterClearStatus();
    #endif /* (Sense_I2C_MODE_MASTER_ENABLED) */
}


/*******************************************************************************
* Function Name: Sense_I2C_Enable
********************************************************************************
*
* Summary:
*  Enables I2C operations.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  None
*
*******************************************************************************/
void Sense_I2C_Enable(void) 
{
    #if(Sense_I2C_FF_IMPLEMENTED)
        uint8 enableInterrupts;

        /* Enable power to I2C FF block */
        enableInterrupts = CyEnterCriticalSection();
        Sense_I2C_ACT_PWRMGR_REG  |= Sense_I2C_ACT_PWR_EN;
        Sense_I2C_STBY_PWRMGR_REG |= Sense_I2C_STBY_PWR_EN;
        CyExitCriticalSection(enableInterrupts);

    #else

        #if(Sense_I2C_MODE_SLAVE_ENABLED)
            uint8 enableInterrupts;
        #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

        #if(Sense_I2C_MODE_SLAVE_ENABLED)
            /* Enable slave bit counter */
            enableInterrupts = CyEnterCriticalSection();
            Sense_I2C_COUNTER_AUX_CTL_REG |= Sense_I2C_CNT7_ENABLE;   /* aux_ctl */
            CyExitCriticalSection(enableInterrupts);
        #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

        Sense_I2C_CFG_REG |= Sense_I2C_ENABLE_MS;

    #endif /* (Sense_I2C_FF_IMPLEMENTED) */

    #if(Sense_I2C_TIMEOUT_ENABLED)
        Sense_I2C_TimeoutEnable();
    #endif /* (Sense_I2C_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: Sense_I2C_Start
********************************************************************************
*
* Summary:
*  Starts the I2C hardware. Enables Active mode power template bits or clock
*  gating as appropriate. It is required to be executed before I2C bus
*  operation.
*  The I2C interrupt remains disabled after this function call.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  This component automatically enables it's interrupt.  If I2C is enabled
*  without the interrupt enabled, it could lock up the I2C bus.
*
* Global variables:
*  Sense_I2C_initVar - used to check initial configuration, modified
*  on first function call.
*
* Reentrant:
*  No
*
*******************************************************************************/
void Sense_I2C_Start(void) 
{
    /* Initialize I2C registers, reset I2C buffer index and clears status */
    if(0u == Sense_I2C_initVar)
    {
        Sense_I2C_Init();
        Sense_I2C_initVar = 1u; /* Component initialized */
    }

    Sense_I2C_Enable();
    Sense_I2C_EnableInt();
}


/*******************************************************************************
* Function Name: Sense_I2C_Stop
********************************************************************************
*
* Summary:
*  Disables I2C hardware and disables I2C interrupt. Disables Active mode power
*  template bits or clock gating as appropriate.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Sense_I2C_Stop(void) 
{
    #if((Sense_I2C_FF_IMPLEMENTED)  || \
        (Sense_I2C_UDB_IMPLEMENTED && Sense_I2C_MODE_SLAVE_ENABLED))
        uint8 enableInterrupts;
    #endif /* ((Sense_I2C_FF_IMPLEMENTED)  || \
               (Sense_I2C_UDB_IMPLEMENTED && Sense_I2C_MODE_SLAVE_ENABLED)) */

    Sense_I2C_DisableInt();

    Sense_I2C_DISABLE_INT_ON_STOP;   /* Interrupt on Stop can be enabled by write */
    (void) Sense_I2C_CSR_REG;        /* Clear CSR reg */
    
    #if(Sense_I2C_TIMEOUT_ENABLED)
        Sense_I2C_TimeoutStop();
    #endif  /* End (Sense_I2C_TIMEOUT_ENABLED) */

    #if(Sense_I2C_FF_IMPLEMENTED)
        #if(CY_PSOC3 || CY_PSOC5LP)
            /* Store registers which are held in reset when Master and Slave bits are cleared */
            #if(Sense_I2C_MODE_SLAVE_ENABLED)
                Sense_I2C_backup.addr = Sense_I2C_ADDR_REG;
            #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

            Sense_I2C_backup.clkDiv1  = Sense_I2C_CLKDIV1_REG;
            Sense_I2C_backup.clkDiv2  = Sense_I2C_CLKDIV2_REG;


            /* Reset FF block */
            Sense_I2C_CFG_REG &= ((uint8) ~Sense_I2C_ENABLE_MS);
            CyDelayUs(Sense_I2C_FF_RESET_DELAY);
            Sense_I2C_CFG_REG |= ((uint8)  Sense_I2C_ENABLE_MS);


            /* Restore registers */
            #if(Sense_I2C_MODE_SLAVE_ENABLED)
                Sense_I2C_ADDR_REG = Sense_I2C_backup.addr;
            #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

            Sense_I2C_CLKDIV1_REG = Sense_I2C_backup.clkDiv1;
            Sense_I2C_CLKDIV2_REG = Sense_I2C_backup.clkDiv2;

        #endif /* (CY_PSOC3 || CY_PSOC5LP) */

        /* Disable power to I2C block */
        enableInterrupts = CyEnterCriticalSection();
        Sense_I2C_ACT_PWRMGR_REG  &= ((uint8) ~Sense_I2C_ACT_PWR_EN);
        Sense_I2C_STBY_PWRMGR_REG &= ((uint8) ~Sense_I2C_STBY_PWR_EN);
        CyExitCriticalSection(enableInterrupts);

    #else

        #if(Sense_I2C_MODE_SLAVE_ENABLED)
            /* Disable slave bit counter */
            enableInterrupts = CyEnterCriticalSection();
            Sense_I2C_COUNTER_AUX_CTL_REG &= ((uint8) ~Sense_I2C_CNT7_ENABLE);
            CyExitCriticalSection(enableInterrupts);
        #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

        Sense_I2C_CFG_REG &= ((uint8) ~Sense_I2C_ENABLE_MS);

    #endif /* (Sense_I2C_FF_IMPLEMENTED) */

    Sense_I2C_ClearPendingInt();  /* Clear interrupt triggers on reset */

    Sense_I2C_state = Sense_I2C_SM_IDLE;  /* Reset software FSM */
}


/* [] END OF FILE */
