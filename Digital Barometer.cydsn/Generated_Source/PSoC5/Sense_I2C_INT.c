/*******************************************************************************
* File Name: Sense_I2C_INT.c
* Version 3.30
*
* Description:
*  This file provides the source code of Interrupt Service Routine (ISR)
*  for I2C component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Sense_I2C_PVT.h"


/*******************************************************************************
*  Place your includes, defines and code here
********************************************************************************/
/* `#START Sense_I2C_ISR_intc` */

/* `#END` */


/*******************************************************************************
* Function Name: Sense_I2C_ISR
********************************************************************************
*
* Summary:
*  Handler for I2C interrupt. The Slave and Master operations are handled here.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Reentrant:
*  No
*
*******************************************************************************/
CY_ISR(Sense_I2C_ISR)
{
    #if(Sense_I2C_MODE_SLAVE_ENABLED)
       uint8  tmp8;
    #endif  /* (Sense_I2C_MODE_SLAVE_ENABLED) */

    uint8  tmpCsr;

    #if(Sense_I2C_TIMEOUT_FF_ENABLED)
        if(0u != Sense_I2C_TimeoutGetStatus())
        {
            Sense_I2C_TimeoutReset();
            Sense_I2C_state = Sense_I2C_SM_EXIT_IDLE;
            /* Sense_I2C_CSR_REG should be cleared after reset */
        }
    #endif /* (Sense_I2C_TIMEOUT_FF_ENABLED) */


    tmpCsr = Sense_I2C_CSR_REG;      /* Make copy as interrupts clear */

    #if(Sense_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED)
        if(Sense_I2C_CHECK_START_GEN(Sense_I2C_MCSR_REG))
        {
            Sense_I2C_CLEAR_START_GEN;

            /* Set READ complete, but was aborted */
            Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_ERR_XFER |
                                            Sense_I2C_GET_MSTAT_CMPLT);

            /* The slave was addressed */
            Sense_I2C_state = Sense_I2C_SM_SLAVE;
        }
    #endif /* (Sense_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED) */


    #if(Sense_I2C_MODE_MULTI_MASTER_ENABLED)
        if(Sense_I2C_CHECK_LOST_ARB(tmpCsr))
        {
            /* Set errors */
            Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_ERR_XFER     |
                                            Sense_I2C_MSTAT_ERR_ARB_LOST |
                                            Sense_I2C_GET_MSTAT_CMPLT);

            Sense_I2C_DISABLE_INT_ON_STOP; /* Interrupt on Stop is enabled by write */

            #if(Sense_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED)
                if(Sense_I2C_CHECK_ADDRESS_STS(tmpCsr))
                {
                    /* The slave was addressed */
                    Sense_I2C_state = Sense_I2C_SM_SLAVE;
                }
                else
                {
                    Sense_I2C_BUS_RELEASE;

                    Sense_I2C_state = Sense_I2C_SM_EXIT_IDLE;
                }
            #else
                Sense_I2C_BUS_RELEASE;

                Sense_I2C_state = Sense_I2C_SM_EXIT_IDLE;

            #endif /* (Sense_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED) */
        }
    #endif /* (Sense_I2C_MODE_MULTI_MASTER_ENABLED) */

    /* Check for Master operation mode */
    if(Sense_I2C_CHECK_SM_MASTER)
    {
        #if(Sense_I2C_MODE_MASTER_ENABLED)
            if(Sense_I2C_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                switch (Sense_I2C_state)
                {
                case Sense_I2C_SM_MSTR_WR_ADDR:  /* After address is sent, WRITE data */
                case Sense_I2C_SM_MSTR_RD_ADDR:  /* After address is sent, READ  data */

                    tmpCsr &= ((uint8) ~Sense_I2C_CSR_STOP_STATUS); /* Clear STOP bit history on address phase */
                    
                    if(Sense_I2C_CHECK_ADDR_ACK(tmpCsr))
                    {
                        /* Setup for transmit or receive of data */
                        if(Sense_I2C_state == Sense_I2C_SM_MSTR_WR_ADDR)   /* TRANSMIT data */
                        {
                            /* Check if at least one byte to transfer */
                            if(Sense_I2C_mstrWrBufSize > 0u)
                            {
                                /* Load the 1st data byte */
                                Sense_I2C_DATA_REG = Sense_I2C_mstrWrBufPtr[0u];
                                Sense_I2C_TRANSMIT_DATA;
                                Sense_I2C_mstrWrBufIndex = 1u;   /* Set index to 2nd element */

                                /* Set transmit state until done */
                                Sense_I2C_state = Sense_I2C_SM_MSTR_WR_DATA;
                            }
                            /* End of buffer: complete writing */
                            else if(Sense_I2C_CHECK_NO_STOP(Sense_I2C_mstrControl))
                            {
                                #if(CY_PSOC5A)
                                    /* Do not handles 0 bytes transfer - HALT is NOT allowed */
                                    Sense_I2C_ENABLE_INT_ON_STOP;
                                    Sense_I2C_GENERATE_STOP;
                                
                                #else
                                    /* Set WRITE complete and Master HALTED */
                                    Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_XFER_HALT |
                                                                    Sense_I2C_MSTAT_WR_CMPLT);

                                    Sense_I2C_state = Sense_I2C_SM_MSTR_HALT; /* Expect RESTART */
                                    Sense_I2C_DisableInt();
                                
                                #endif /* (CY_PSOC5A) */
                            }
                            else
                            {
                                Sense_I2C_ENABLE_INT_ON_STOP; /* Enable interrupt on STOP, to catch it */
                                Sense_I2C_GENERATE_STOP;
                            }
                        }
                        else  /* Master Receive data */
                        {
                            Sense_I2C_READY_TO_READ; /* Release bus to read data */

                            Sense_I2C_state  = Sense_I2C_SM_MSTR_RD_DATA;
                        }
                    }
                    /* Address is NACKed */
                    else if(Sense_I2C_CHECK_ADDR_NAK(tmpCsr))
                    {
                        /* Set Address NAK error */
                        Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_ERR_XFER |
                                                        Sense_I2C_MSTAT_ERR_ADDR_NAK);
                                                        
                        if(Sense_I2C_CHECK_NO_STOP(Sense_I2C_mstrControl))
                        {
                            Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_XFER_HALT | 
                                                            Sense_I2C_GET_MSTAT_CMPLT);

                            Sense_I2C_state = Sense_I2C_SM_MSTR_HALT; /* Expect RESTART */
                            Sense_I2C_DisableInt();
                        }
                        else  /* Do normal Stop */
                        {
                            Sense_I2C_ENABLE_INT_ON_STOP; /* Enable interrupt on STOP, to catch it */
                            Sense_I2C_GENERATE_STOP;
                        }
                    }
                    else
                    {
                        /* Address phase is not set for some reason: error */
                        #if(Sense_I2C_TIMEOUT_ENABLED)
                            /* Exit from interrupt to take a chance for timeout timer handle this case */
                            Sense_I2C_DisableInt();
                            Sense_I2C_ClearPendingInt();
                        #else
                            /* Block execution flow: unexpected condition */
                            CYASSERT(0u != 0u);
                        #endif /* (Sense_I2C_TIMEOUT_ENABLED) */
                    }
                    break;

                case Sense_I2C_SM_MSTR_WR_DATA:

                    if(Sense_I2C_CHECK_DATA_ACK(tmpCsr))
                    {
                        /* Check if end of buffer */
                        if(Sense_I2C_mstrWrBufIndex  < Sense_I2C_mstrWrBufSize)
                        {
                            Sense_I2C_DATA_REG =
                                                     Sense_I2C_mstrWrBufPtr[Sense_I2C_mstrWrBufIndex];
                            Sense_I2C_TRANSMIT_DATA;
                            Sense_I2C_mstrWrBufIndex++;
                        }
                        /* End of buffer: complete writing */
                        else if(Sense_I2C_CHECK_NO_STOP(Sense_I2C_mstrControl))
                        {
                            /* Set WRITE complete and Master HALTED */
                            Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_XFER_HALT |
                                                            Sense_I2C_MSTAT_WR_CMPLT);

                            Sense_I2C_state = Sense_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                            Sense_I2C_DisableInt();
                        }
                        else  /* Do normal STOP */
                        {
                            Sense_I2C_Workaround();          /* Workaround: empty function */
                            Sense_I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                            Sense_I2C_GENERATE_STOP;
                        }
                    }
                    /* Last byte NAKed: end writing */
                    else if(Sense_I2C_CHECK_NO_STOP(Sense_I2C_mstrControl))
                    {
                        /* Set WRITE complete, SHORT transfer and Master HALTED */
                        Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_ERR_XFER       |
                                                        Sense_I2C_MSTAT_ERR_SHORT_XFER |
                                                        Sense_I2C_MSTAT_XFER_HALT      |
                                                        Sense_I2C_MSTAT_WR_CMPLT);

                        Sense_I2C_state = Sense_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                        Sense_I2C_DisableInt();
                    }
                    else  /* Do normal STOP */
                    {
                        Sense_I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                        Sense_I2C_GENERATE_STOP;

                        /* Set SHORT and ERR transfer */
                        Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_ERR_SHORT_XFER |
                                                        Sense_I2C_MSTAT_ERR_XFER);
                    }
                    
                    break;

                case Sense_I2C_SM_MSTR_RD_DATA:

                    Sense_I2C_mstrRdBufPtr[Sense_I2C_mstrRdBufIndex] = Sense_I2C_DATA_REG;
                    Sense_I2C_mstrRdBufIndex++;

                    /* Check if end of buffer */
                    if(Sense_I2C_mstrRdBufIndex < Sense_I2C_mstrRdBufSize)
                    {
                        Sense_I2C_ACK_AND_RECEIVE;       /* ACK and receive byte */
                    }
                    /* End of buffer: complete reading */
                    else if(Sense_I2C_CHECK_NO_STOP(Sense_I2C_mstrControl))
                    {                        
                        /* Set READ complete and Master HALTED */
                        Sense_I2C_mstrStatus |= (Sense_I2C_MSTAT_XFER_HALT |
                                                        Sense_I2C_MSTAT_RD_CMPLT);
                        
                        Sense_I2C_state = Sense_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                        Sense_I2C_DisableInt();
                    }
                    else
                    {
                        Sense_I2C_ENABLE_INT_ON_STOP;
                        Sense_I2C_NAK_AND_RECEIVE;       /* NACK and TRY to generate STOP */
                    }
                    break;

                default: /* This is an invalid state and should not occur */

                    #if(Sense_I2C_TIMEOUT_ENABLED)
                        /* Exit from interrupt to take a chance for timeout timer handle this case */
                        Sense_I2C_DisableInt();
                        Sense_I2C_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (Sense_I2C_TIMEOUT_ENABLED) */

                    break;
                }
            }

            /* Catches the Stop: end of transaction */
            if(Sense_I2C_CHECK_STOP_STS(tmpCsr))
            {
                Sense_I2C_mstrStatus |= Sense_I2C_GET_MSTAT_CMPLT;

                Sense_I2C_DISABLE_INT_ON_STOP;
                Sense_I2C_state = Sense_I2C_SM_IDLE;
            }
        #endif /* (Sense_I2C_MODE_MASTER_ENABLED) */
    }
    else if(Sense_I2C_CHECK_SM_SLAVE)
    {
        #if(Sense_I2C_MODE_SLAVE_ENABLED)
            
            if((Sense_I2C_CHECK_STOP_STS(tmpCsr)) || /* Stop || Restart */
               (Sense_I2C_CHECK_BYTE_COMPLETE(tmpCsr) && Sense_I2C_CHECK_ADDRESS_STS(tmpCsr)))
            {
                /* Catch end of master write transcation: use interrupt on Stop */
                /* The STOP bit history on address phase does not have correct state */
                if(Sense_I2C_SM_SL_WR_DATA == Sense_I2C_state)
                {
                    Sense_I2C_DISABLE_INT_ON_STOP;

                    Sense_I2C_slStatus &= ((uint8) ~Sense_I2C_SSTAT_WR_BUSY);
                    Sense_I2C_slStatus |= ((uint8)  Sense_I2C_SSTAT_WR_CMPLT);

                    Sense_I2C_state = Sense_I2C_SM_IDLE;
                }
            }

            if(Sense_I2C_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                /* The address only issued after Start or ReStart: so check address
                   to catch this events:
                    FF : sets Addr phase with byte_complete interrupt trigger.
                    UDB: sets Addr phase immediately after Start or ReStart. */
                if(Sense_I2C_CHECK_ADDRESS_STS(tmpCsr))
                {
                    /* Check for software address detection */
                    #if(Sense_I2C_SW_ADRR_DECODE)
                        tmp8 = Sense_I2C_GET_SLAVE_ADDR(Sense_I2C_DATA_REG);

                        if(tmp8 == Sense_I2C_slAddress)   /* Check for address match */
                        {
                            if(0u != (Sense_I2C_DATA_REG & Sense_I2C_READ_FLAG))
                            {
                                /* Place code to prepare read buffer here                  */
                                /* `#START Sense_I2C_SW_PREPARE_READ_BUF_interrupt` */

                                /* `#END` */

                                /* Prepare next opeation to read, get data and place in data register */
                                if(Sense_I2C_slRdBufIndex < Sense_I2C_slRdBufSize)
                                {
                                    /* Load first data byte from array */
                                    Sense_I2C_DATA_REG = Sense_I2C_slRdBufPtr[Sense_I2C_slRdBufIndex];
                                    Sense_I2C_ACK_AND_TRANSMIT;
                                    Sense_I2C_slRdBufIndex++;

                                    Sense_I2C_slStatus |= Sense_I2C_SSTAT_RD_BUSY;
                                }
                                else    /* Overflow: provide 0xFF on the bus */
                                {
                                    Sense_I2C_DATA_REG = Sense_I2C_OVERFLOW_RETURN;
                                    Sense_I2C_ACK_AND_TRANSMIT;

                                    Sense_I2C_slStatus  |= (Sense_I2C_SSTAT_RD_BUSY |
                                                                   Sense_I2C_SSTAT_RD_ERR_OVFL);
                                }

                                Sense_I2C_state = Sense_I2C_SM_SL_RD_DATA;
                            }
                            else  /* Write transaction: receive 1st byte */
                            {
                                Sense_I2C_ACK_AND_RECEIVE;
                                Sense_I2C_state = Sense_I2C_SM_SL_WR_DATA;

                                Sense_I2C_slStatus |= Sense_I2C_SSTAT_WR_BUSY;
                                Sense_I2C_ENABLE_INT_ON_STOP;
                            }
                        }    
                        else
                        {
                            /*     Place code to compare for additional address here    */
                            /* `#START Sense_I2C_SW_ADDR_COMPARE_interruptStart` */

                            /* `#END` */
                            
                            Sense_I2C_NAK_AND_RECEIVE;   /* NACK address */

                            /* Place code to end of condition for NACK generation here */
                            /* `#START Sense_I2C_SW_ADDR_COMPARE_interruptEnd`  */

                            /* `#END` */
                        }
                        
                    #else /* (Sense_I2C_HW_ADRR_DECODE) */
                        
                        if(0u != (Sense_I2C_DATA_REG & Sense_I2C_READ_FLAG))
                        {
                            /* Place code to prepare read buffer here                  */
                            /* `#START Sense_I2C_HW_PREPARE_READ_BUF_interrupt` */

                            /* `#END` */

                            /* Prepare next opeation to read, get data and place in data register */
                            if(Sense_I2C_slRdBufIndex < Sense_I2C_slRdBufSize)
                            {
                                /* Load first data byte from array */
                                Sense_I2C_DATA_REG = Sense_I2C_slRdBufPtr[Sense_I2C_slRdBufIndex];
                                Sense_I2C_ACK_AND_TRANSMIT;
                                Sense_I2C_slRdBufIndex++;

                                Sense_I2C_slStatus |= Sense_I2C_SSTAT_RD_BUSY;
                            }
                            else    /* Overflow: provide 0xFF on the bus */
                            {
                                Sense_I2C_DATA_REG = Sense_I2C_OVERFLOW_RETURN;
                                Sense_I2C_ACK_AND_TRANSMIT;

                                Sense_I2C_slStatus  |= (Sense_I2C_SSTAT_RD_BUSY |
                                                               Sense_I2C_SSTAT_RD_ERR_OVFL);
                            }

                            Sense_I2C_state = Sense_I2C_SM_SL_RD_DATA;
                        }
                        else  /* Write transaction: receive 1st byte */
                        {
                            Sense_I2C_ACK_AND_RECEIVE;
                            Sense_I2C_state = Sense_I2C_SM_SL_WR_DATA;

                            Sense_I2C_slStatus |= Sense_I2C_SSTAT_WR_BUSY;
                            Sense_I2C_ENABLE_INT_ON_STOP;
                        }
                        
                    #endif /* (Sense_I2C_SW_ADRR_DECODE) */
                }
                /* Data states */
                /* Data master writes into slave */
                else if(Sense_I2C_state == Sense_I2C_SM_SL_WR_DATA)
                {
                    if(Sense_I2C_slWrBufIndex < Sense_I2C_slWrBufSize)
                    {
                        tmp8 = Sense_I2C_DATA_REG;
                        Sense_I2C_ACK_AND_RECEIVE;
                        Sense_I2C_slWrBufPtr[Sense_I2C_slWrBufIndex] = tmp8;
                        Sense_I2C_slWrBufIndex++;
                    }
                    else  /* of array: complete write, send NACK */
                    {
                        Sense_I2C_NAK_AND_RECEIVE;

                        Sense_I2C_slStatus |= Sense_I2C_SSTAT_WR_ERR_OVFL;
                    }
                }
                /* Data master reads from slave */
                else if(Sense_I2C_state == Sense_I2C_SM_SL_RD_DATA)
                {
                    if(Sense_I2C_CHECK_DATA_ACK(tmpCsr))
                    {
                        if(Sense_I2C_slRdBufIndex < Sense_I2C_slRdBufSize)
                        {
                             /* Get data from array */
                            Sense_I2C_DATA_REG = Sense_I2C_slRdBufPtr[Sense_I2C_slRdBufIndex];
                            Sense_I2C_TRANSMIT_DATA;
                            Sense_I2C_slRdBufIndex++;
                        }
                        else   /* Overflow: provide 0xFF on the bus */
                        {
                            Sense_I2C_DATA_REG = Sense_I2C_OVERFLOW_RETURN;
                            Sense_I2C_TRANSMIT_DATA;

                            Sense_I2C_slStatus |= Sense_I2C_SSTAT_RD_ERR_OVFL;
                        }
                    }
                    else  /* Last byte was NACKed: read complete */
                    {
                        /* Only NACK appears on the bus */
                        Sense_I2C_DATA_REG = Sense_I2C_OVERFLOW_RETURN;
                        Sense_I2C_NAK_AND_TRANSMIT;

                        Sense_I2C_slStatus &= ((uint8) ~Sense_I2C_SSTAT_RD_BUSY);
                        Sense_I2C_slStatus |= ((uint8)  Sense_I2C_SSTAT_RD_CMPLT);

                        Sense_I2C_state = Sense_I2C_SM_IDLE;
                    }
                }
                else
                {
                    #if(Sense_I2C_TIMEOUT_ENABLED)
                        /* Exit from interrupt to take a chance for timeout timer handle this case */
                        Sense_I2C_DisableInt();
                        Sense_I2C_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (Sense_I2C_TIMEOUT_ENABLED) */
                }
            }
        #endif /* (Sense_I2C_MODE_SLAVE_ENABLED) */
    }
    else
    {
        /* The FSM skips master and slave processing: return to IDLE */
        Sense_I2C_state = Sense_I2C_SM_IDLE;
    }
}


#if((Sense_I2C_FF_IMPLEMENTED) && (Sense_I2C_WAKEUP_ENABLED))
    /*******************************************************************************
    * Function Name: Sense_I2C_WAKEUP_ISR
    ********************************************************************************
    *
    * Summary:
    *  Empty interrupt handler to trigger after wakeup.
    *
    * Parameters:
    *  void
    *
    * Return:
    *  void
    *
    *******************************************************************************/
    CY_ISR(Sense_I2C_WAKEUP_ISR)
    {
        Sense_I2C_wakeupSource = 1u;  /* I2C was wakeup source */
        /* The SCL is stretched unitl the I2C_Wake() is called */
    }
#endif /* ((Sense_I2C_FF_IMPLEMENTED) && (Sense_I2C_WAKEUP_ENABLED))*/


/* [] END OF FILE */
