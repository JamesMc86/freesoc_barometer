/*******************************************************************************
* File Name: Sense_I2C_PM.c
* Version 3.30
*
* Description:
*  This file provides Low power mode APIs for I2C component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Sense_I2C_PVT.h"

Sense_I2C_BACKUP_STRUCT Sense_I2C_backup =
{
    Sense_I2C_DISABLE, /* enableState */

    #if(Sense_I2C_FF_IMPLEMENTED)
        Sense_I2C_DEFAULT_XCFG,  /* xcfg */
        Sense_I2C_DEFAULT_CFG,   /* cfg  */

        #if(Sense_I2C_MODE_SLAVE_ENABLED)
            Sense_I2C_DEFAULT_ADDR, /* addr */
        #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            LO8(Sense_I2C_DEFAULT_DIVIDE_FACTOR),  /* div */
        #else
            LO8(Sense_I2C_DEFAULT_DIVIDE_FACTOR), /* div1 */
            HI8(Sense_I2C_DEFAULT_DIVIDE_FACTOR), /* div2 */
        #endif /* (CY_PSOC5A) */

    #else  /* (Sense_I2C_UDB_IMPLEMENTED) */
        Sense_I2C_DEFAULT_CFG,    /* control */

        #if(CY_UDB_V0)
            Sense_I2C_INT_ENABLE_MASK, /* aux_ctl */

            #if(Sense_I2C_MODE_SLAVE_ENABLED)
                Sense_I2C_DEFAULT_ADDR, /* addr_d0 */
            #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */
        #endif /* (CY_UDB_V0) */
    #endif /* (Sense_I2C_FF_IMPLEMENTED) */

    #if(Sense_I2C_TIMEOUT_ENABLED)
        Sense_I2C_DEFAULT_TMOUT_PERIOD,
        Sense_I2C_DEFAULT_TMOUT_INTR_MASK,

        #if(Sense_I2C_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0)
            Sense_I2C_DEFAULT_TMOUT_PRESCALER_PRD,
        #endif /* (Sense_I2C_TIMEOUT_PRESCALER_ENABLED) */

    #endif /* (Sense_I2C_TIMEOUT_ENABLED) */
};

#if((Sense_I2C_FF_IMPLEMENTED) && (Sense_I2C_WAKEUP_ENABLED))
    volatile uint8 Sense_I2C_wakeupSource;
#endif /* ((Sense_I2C_FF_IMPLEMENTED) && (Sense_I2C_WAKEUP_ENABLED)) */


/*******************************************************************************
* Function Name: Sense_I2C_SaveConfig
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: disables I2C Master(if was enabled before go
*  to sleep), enables I2C backup regulator. Waits while on-going transaction be
*  will completed and I2C will be ready go to sleep. All incoming transaction
*  will be NACKed till power down will be asserted. The address match event
*  wakes up the chip.
*  Wakeup on address match disabled: saves I2C configuration and non-retention
*  register values.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global Variables:
*  Sense_I2C_backup - used to save component configuration and
*       none-retention registers before enter sleep mode.
*
* Reentrant:
*  No
*
*******************************************************************************/
void Sense_I2C_SaveConfig(void) 
{
    #if(Sense_I2C_FF_IMPLEMENTED)
        #if(Sense_I2C_WAKEUP_ENABLED)
            uint8 enableInterrupts;
        #endif /* (Sense_I2C_WAKEUP_ENABLED) */

        /* Store regiters in either Sleep mode */
        Sense_I2C_backup.cfg  = Sense_I2C_CFG_REG;
        Sense_I2C_backup.xcfg = Sense_I2C_XCFG_REG;

        #if(Sense_I2C_MODE_SLAVE_ENABLED)
            Sense_I2C_backup.addr = Sense_I2C_ADDR_REG;
        #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            Sense_I2C_backup.clkDiv   = Sense_I2C_CLKDIV_REG;
        #else
            Sense_I2C_backup.clkDiv1  = Sense_I2C_CLKDIV1_REG;
            Sense_I2C_backup.clkDiv2  = Sense_I2C_CLKDIV2_REG;
        #endif /* (CY_PSOC5A) */

        #if(Sense_I2C_WAKEUP_ENABLED)
            /* Need to disable Master */
            Sense_I2C_CFG_REG &= ((uint8) ~Sense_I2C_ENABLE_MASTER);

            /* Enable the I2C regulator backup */
            enableInterrupts = CyEnterCriticalSection();
            Sense_I2C_PWRSYS_CR1_REG |= Sense_I2C_PWRSYS_CR1_I2C_REG_BACKUP;
            CyExitCriticalSection(enableInterrupts);

            /* 1) Set force NACK to ignore I2C transactions
               2) Wait while I2C will be ready go to Sleep
               3) These bits are cleared on wake up */
            Sense_I2C_XCFG_REG |= Sense_I2C_XCFG_FORCE_NACK;
            while(0u == (Sense_I2C_XCFG_REG & Sense_I2C_XCFG_RDY_TO_SLEEP))
            {
                ; /* Wait when block is ready to Sleep */
            }

            /* Setup wakeup interrupt */
            Sense_I2C_DisableInt();
            (void) CyIntSetVector(Sense_I2C_ISR_NUMBER, &Sense_I2C_WAKEUP_ISR);
            Sense_I2C_wakeupSource = 0u;
            Sense_I2C_EnableInt();

        #endif /* (Sense_I2C_WAKEUP_ENABLED) */

    #else
        /* Store only address match bit */
        Sense_I2C_backup.control = (Sense_I2C_CFG_REG & Sense_I2C_CTRL_ANY_ADDRESS_MASK);

        #if(CY_UDB_V0)
            /* Store interrupt mask bits */
            Sense_I2C_backup.intMask = Sense_I2C_INT_MASK_REG;

            #if(Sense_I2C_MODE & Sense_I2C_MODE_SLAVE)
                Sense_I2C_backup.addr = Sense_I2C_ADDR_REG;
            #endif /* (Sense_I2C_MODE & Sense_I2C_MODE_SLAVE) */

        #endif /* (CY_UDB_V0) */

    #endif /* (Sense_I2C_FF_IMPLEMENTED) */

    #if(Sense_I2C_TIMEOUT_ENABLED)
        Sense_I2C_TimeoutSaveConfig();   /* Save Timeout config */
    #endif /* (Sense_I2C_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: Sense_I2C_Sleep
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: All incoming transaction will be NACKed till
*  power down will be asserted. The address match event wakes up the chip.
*  Wakeup on address match disabled: Disables active mode power template bits or
*  clock gating as appropriate. Saves I2C configuration and non-retention
*  register values.
*  Disables I2C interrupt.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void Sense_I2C_Sleep(void) 
{
    #if(Sense_I2C_WAKEUP_ENABLED)
        /* The I2C block should be always enabled if used as wakeup source */
        Sense_I2C_backup.enableState = Sense_I2C_DISABLE;

        #if(Sense_I2C_TIMEOUT_ENABLED)
            Sense_I2C_TimeoutStop();
        #endif /* (Sense_I2C_TIMEOUT_ENABLED) */

    #else

        Sense_I2C_backup.enableState = ((uint8) Sense_I2C_IS_ENABLED);

        if(Sense_I2C_IS_ENABLED)
        {
            Sense_I2C_Stop();
        }
    #endif /* (Sense_I2C_WAKEUP_ENABLED) */

    Sense_I2C_SaveConfig();
}


/*******************************************************************************
* Function Name: Sense_I2C_RestoreConfig
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: enables I2C Master (if was enabled before go
*  to sleep), disables I2C backup regulator.
*  Wakeup on address match disabled: Restores I2C configuration and
*  non-retention register values.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global Variables:
*  Sense_I2C_backup - used to save component configuration and
*  none-retention registers before exit sleep mode.
*
*******************************************************************************/
void Sense_I2C_RestoreConfig(void) 
{
    #if(Sense_I2C_FF_IMPLEMENTED)
        uint8 enableInterrupts;

        if(Sense_I2C_CHECK_PWRSYS_I2C_BACKUP)    /* Enabled if was in Sleep */
        {
            /* Disable back-up regulator */
            enableInterrupts = CyEnterCriticalSection();
            Sense_I2C_PWRSYS_CR1_REG &= ((uint8) ~Sense_I2C_PWRSYS_CR1_I2C_REG_BACKUP);
            CyExitCriticalSection(enableInterrupts);

            /* Re-enable Master */
            Sense_I2C_CFG_REG = Sense_I2C_backup.cfg;
        }
        else /* The I2C_REG_BACKUP was cleaned by PM API: it means Hibernate or wake-up not set */
        {
            #if(Sense_I2C_WAKEUP_ENABLED)
                /* Disable power to I2C block before register restore */
                enableInterrupts = CyEnterCriticalSection();
                Sense_I2C_ACT_PWRMGR_REG  &= ((uint8) ~Sense_I2C_ACT_PWR_EN);
                Sense_I2C_STBY_PWRMGR_REG &= ((uint8) ~Sense_I2C_STBY_PWR_EN);
                CyExitCriticalSection(enableInterrupts);

                /* Enable component after restore complete */
                Sense_I2C_backup.enableState = Sense_I2C_ENABLE;
            #endif /* (Sense_I2C_WAKEUP_ENABLED) */

            /* Restore component registers: Hibernate disable power */
            Sense_I2C_XCFG_REG = Sense_I2C_backup.xcfg;
            Sense_I2C_CFG_REG  = Sense_I2C_backup.cfg;

            #if(Sense_I2C_MODE_SLAVE_ENABLED)
                Sense_I2C_ADDR_REG = Sense_I2C_backup.addr;
            #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

            #if(CY_PSOC5A)
                Sense_I2C_CLKDIV_REG  = Sense_I2C_backup.clkDiv;
            #else
                Sense_I2C_CLKDIV1_REG = Sense_I2C_backup.clkDiv1;
                Sense_I2C_CLKDIV2_REG = Sense_I2C_backup.clkDiv2;
            #endif /* (CY_PSOC5A) */
        }

        #if(Sense_I2C_WAKEUP_ENABLED)
            Sense_I2C_DisableInt();
            (void) CyIntSetVector(Sense_I2C_ISR_NUMBER, &Sense_I2C_ISR);
            if(0u != Sense_I2C_wakeupSource)
            {
                Sense_I2C_SetPendingInt();   /* Generate interrupt to process incomming transcation */
            }
            Sense_I2C_EnableInt();
        #endif /* (Sense_I2C_WAKEUP_ENABLED) */

    #else

        #if(CY_UDB_V0)
            uint8 enableInterrupts;

            Sense_I2C_INT_MASK_REG |= Sense_I2C_backup.intMask;

            enableInterrupts = CyEnterCriticalSection();
            Sense_I2C_INT_ENABLE_REG |= Sense_I2C_INT_ENABLE_MASK;
            CyExitCriticalSection(enableInterrupts);

            #if(Sense_I2C_MODE_MASTER_ENABLED)
                /* Restore Master Clock generator */
                Sense_I2C_MCLK_PRD_REG = Sense_I2C_DEFAULT_MCLK_PRD;
                Sense_I2C_MCLK_CMP_REG = Sense_I2C_DEFAULT_MCLK_CMP;
            #endif /* (Sense_I2C_MODE_MASTER_ENABLED) */

            #if(Sense_I2C_MODE_SLAVE_ENABLED)
                Sense_I2C_ADDR_REG = Sense_I2C_backup.addr;

                /* Restore slave bit counter period */
                Sense_I2C_PERIOD_REG = Sense_I2C_DEFAULT_PERIOD;
            #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */

        #endif /* (CY_UDB_V0) */

        Sense_I2C_CFG_REG = Sense_I2C_backup.control;

    #endif /* (Sense_I2C_FF_IMPLEMENTED) */

    #if(Sense_I2C_TIMEOUT_ENABLED)
        Sense_I2C_TimeoutRestoreConfig();
    #endif /* (Sense_I2C_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: Sense_I2C_Wakeup
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: enables I2C Master (if was enabled before go
*  to sleep) and disables I2C backup regulator.
*  Wakeup on address match disabled: Restores I2C configuration and
*  non-retention register values. Restores Active mode power template bits or
*  clock gating as appropriate.
*  The I2C interrupt remains disabled after function call.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void Sense_I2C_Wakeup(void) 
{
    Sense_I2C_RestoreConfig();   /* Restore I2C register settings */

    /* Restore component enable state */
    if(0u != Sense_I2C_backup.enableState)
    {
        Sense_I2C_Enable();
        Sense_I2C_EnableInt();
    }
    else
    {
        #if(Sense_I2C_TIMEOUT_ENABLED)
            Sense_I2C_TimeoutEnable();
        #endif /* (Sense_I2C_TIMEOUT_ENABLED) */
    }
}


/* [] END OF FILE */
