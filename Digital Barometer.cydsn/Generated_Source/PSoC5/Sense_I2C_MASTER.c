/*******************************************************************************
* File Name: Sense_I2C_MASTER.c
* Version 3.30
*
* Description:
*  This file provides the source code of APIs for the I2C component Master mode.
*
* Note:
*
*******************************************************************************
* Copyright 2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Sense_I2C_PVT.h"

#if(Sense_I2C_MODE_MASTER_ENABLED)

/**********************************
*      System variables
**********************************/

volatile uint8 Sense_I2C_mstrStatus;     /* Master Status byte  */
volatile uint8 Sense_I2C_mstrControl;    /* Master Control byte */

/* Transmit buffer variables */
volatile uint8 * Sense_I2C_mstrRdBufPtr;     /* Pointer to Master Read buffer */
volatile uint8   Sense_I2C_mstrRdBufSize;    /* Master Read buffer size       */
volatile uint8   Sense_I2C_mstrRdBufIndex;   /* Master Read buffer Index      */

/* Receive buffer variables */
volatile uint8 * Sense_I2C_mstrWrBufPtr;     /* Pointer to Master Write buffer */
volatile uint8   Sense_I2C_mstrWrBufSize;    /* Master Write buffer size       */
volatile uint8   Sense_I2C_mstrWrBufIndex;   /* Master Write buffer Index      */


/*******************************************************************************
* Function Name: Sense_I2C_MasterWriteBuf
********************************************************************************
*
* Summary:
*  Automatically writes an entire buffer of data to a slave device. Once the
*  data transfer is initiated by this function, further data transfer is handled
*  by the included ISR in byte by byte mode.
*
* Parameters:
*  slaveAddr: 7-bit slave address.
*  xferData:  Pointer to buffer of data to be sent.
*  cnt:       Size of buffer to send.
*  mode:      Transfer mode defines: start or restart condition generation at
*             begin of the transfer and complete the transfer or halt before
*             generating a stop.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  The included ISR will start transfer after start or restart condition will
*  be generated.
*
* Global variables:
*  Sense_I2C_mstrStatus  - used to store current status of I2C Master.
*  Sense_I2C_state       - used to store current state of software FSM.
*  Sense_I2C_mstrControl - used to control master end of transaction with
*  or without the Stop generation.
*  Sense_I2C_mstrWrBufPtr - used to store pointer to master write buffer.
*  Sense_I2C_mstrWrBufIndex - used to current index within master write
*  buffer.
*  Sense_I2C_mstrWrBufSize - used to store master write buffer size.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 Sense_I2C_MasterWriteBuf(uint8 slaveAddress, uint8 * wrData, uint8 cnt, uint8 mode)
      
{
    uint8 errStatus;

    errStatus = Sense_I2C_MSTR_NOT_READY;

    if(NULL != wrData)
    {
        /* Check I2C state before transfer: valid are IDLE or HALT */
        if(Sense_I2C_SM_IDLE == Sense_I2C_state)
        {
            /* Check if free: Master is ready to transaction */
            if(Sense_I2C_CHECK_BUS_FREE(Sense_I2C_MCSR_REG))
            {
                errStatus = Sense_I2C_MSTR_NO_ERROR;
            }
            else
            {
                errStatus = Sense_I2C_MSTR_BUS_BUSY;
            }
        }
        else if(Sense_I2C_SM_MSTR_HALT == Sense_I2C_state)
        {
            errStatus = Sense_I2C_MSTR_NO_ERROR;

            CyIntClearPending(Sense_I2C_ISR_NUMBER);
            Sense_I2C_mstrStatus &= ((uint8) ~Sense_I2C_MSTAT_XFER_HALT);
        }
        else
        {
            /* errStatus = Sense_I2C_MSTR_NOT_READY was send before */
        }

        if(Sense_I2C_MSTR_NO_ERROR == errStatus)
        {
            Sense_I2C_state    = Sense_I2C_SM_MSTR_WR_ADDR;
            Sense_I2C_DATA_REG = ((uint8) (slaveAddress << Sense_I2C_SLAVE_ADDR_SHIFT));

            Sense_I2C_mstrWrBufIndex = 0u;
            Sense_I2C_mstrWrBufSize  = cnt;
            Sense_I2C_mstrWrBufPtr   = (volatile uint8 *) wrData;

            Sense_I2C_mstrControl = mode;    /* Save transaction mode */

            /* Generate a Start or ReStart depends on mode */
            if(Sense_I2C_CHECK_RESTART(mode))
            {
                Sense_I2C_GENERATE_RESTART;
            }
            else
            {
                Sense_I2C_GENERATE_START;
            }

            Sense_I2C_mstrStatus &= ((uint8) ~Sense_I2C_MSTAT_WR_CMPLT);

            Sense_I2C_EnableInt();   /* Enable intr to complete transfer */
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterReadBuf
********************************************************************************
*
* Summary:
*  Automatically writes an entire buffer of data to a slave device. Once the
*  data transfer is initiated by this function, further data transfer is handled
*  by the included ISR in byte by byte mode.
*
* Parameters:
*  slaveAddr: 7-bit slave address.
*  xferData:  Pointer to buffer where to put data from slave.
*  cnt:       Size of buffer to read.
*  mode:      Transfer mode defines: start or restart condition generation at
*             begin of the transfer and complete the transfer or halt before
*             generating a stop.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  The included ISR will start transfer after start or restart condition will
*  be generated.
*
* Global variables:
*  Sense_I2C_mstrStatus  - used to store current status of I2C Master.
*  Sense_I2C_state       - used to store current state of software FSM.
*  Sense_I2C_mstrControl - used to control master end of transaction with
*  or without the Stop generation.
*  Sense_I2C_mstrRdBufPtr - used to store pointer to master write buffer.
*  Sense_I2C_mstrRdBufIndex - used to current index within master write
*  buffer.
*  Sense_I2C_mstrRdBufSize - used to store master write buffer size.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 Sense_I2C_MasterReadBuf(uint8 slaveAddress, uint8 * rdData, uint8 cnt, uint8 mode)
      
{
    uint8 errStatus;

    errStatus = Sense_I2C_MSTR_NOT_READY;

    if(NULL != rdData)
    {
        /* Check I2C state before transfer: valid are IDLE or HALT */
        if(Sense_I2C_SM_IDLE == Sense_I2C_state)
        {
            /* Check if free: Master is ready to transaction */
            if(Sense_I2C_CHECK_BUS_FREE(Sense_I2C_MCSR_REG))
            {
                errStatus = Sense_I2C_MSTR_NO_ERROR;
            }
            else
            {
                errStatus = Sense_I2C_MSTR_BUS_BUSY;
            }
        }
        else if(Sense_I2C_SM_MSTR_HALT == Sense_I2C_state)
        {
            errStatus = Sense_I2C_MSTR_NO_ERROR;

            CyIntClearPending(Sense_I2C_ISR_NUMBER);
            Sense_I2C_mstrStatus &= ((uint8) ~Sense_I2C_MSTAT_XFER_HALT);
        }
        else
        {
            /* errStatus = Sense_I2C_MSTR_NOT_READY was send before */
        }

        if(Sense_I2C_MSTR_NO_ERROR == errStatus)
        {
            Sense_I2C_state    = Sense_I2C_SM_MSTR_RD_ADDR;
            Sense_I2C_DATA_REG = (((uint8) (slaveAddress << Sense_I2C_SLAVE_ADDR_SHIFT)) |
                                                   Sense_I2C_READ_FLAG);

            Sense_I2C_mstrRdBufIndex  = 0u;
            Sense_I2C_mstrRdBufSize   = cnt;
            Sense_I2C_mstrRdBufPtr    = (volatile uint8 *) rdData;

            Sense_I2C_mstrControl = mode;    /* Save transaction mode */

            /* Generate a Start or ReStart depends on mode */
            if(Sense_I2C_CHECK_RESTART(mode))
            {
                Sense_I2C_GENERATE_RESTART;
            }
            else
            {
                Sense_I2C_GENERATE_START;
            }

            Sense_I2C_mstrStatus &= ((uint8) ~Sense_I2C_MSTAT_RD_CMPLT);

            Sense_I2C_EnableInt();   /* Enable intr to complete transfer */
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterSendStart
********************************************************************************
*
* Summary:
*  Generates Start condition and sends slave address with read/write bit.
*
* Parameters:
*  slaveAddress:  7-bit slave address.
*  R_nW:          Zero, send write command, non-zero send read command.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  This function is entered without a 'byte complete' bit set in the I2C_CSR
*  register. It does not exit until it will be set.
*
* Global variables:
*  Sense_I2C_state - used to store current state of software FSM.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 Sense_I2C_MasterSendStart(uint8 slaveAddress, uint8 R_nW)
      
{
    uint8 errStatus;

    errStatus = Sense_I2C_MSTR_NOT_READY;

    /* If IDLE, check if bus is free */
    if(Sense_I2C_SM_IDLE == Sense_I2C_state)
    {
        /* If bus is free, generate Start condition */
        if(Sense_I2C_CHECK_BUS_FREE(Sense_I2C_MCSR_REG))
        {
            Sense_I2C_DisableInt();  /* Disable ISR for Manual functions */

            slaveAddress = ((uint8) (slaveAddress << Sense_I2C_SLAVE_ADDR_SHIFT)); /* Set Address */
            if(0u != R_nW)                                      /* Set the Read/Write flag */
            {
                slaveAddress |= Sense_I2C_READ_FLAG;
                Sense_I2C_state = Sense_I2C_SM_MSTR_RD_ADDR;
            }
            else
            {
                Sense_I2C_state = Sense_I2C_SM_MSTR_WR_ADDR;
            }
            Sense_I2C_DATA_REG = slaveAddress;   /* Write address to data reg */


            Sense_I2C_GENERATE_START;
            while(Sense_I2C_WAIT_BYTE_COMPLETE(Sense_I2C_CSR_REG))
            {
                ; /* Wait for the address to be transfered */
            }

            #if(Sense_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED)
                if(Sense_I2C_CHECK_START_GEN(Sense_I2C_MCSR_REG))
                {
                    Sense_I2C_CLEAR_START_GEN;

                    /* Start condition was not generated: reset FSM to IDLE */
                    Sense_I2C_state = Sense_I2C_SM_IDLE;
                    errStatus = Sense_I2C_MSTR_ERR_ABORT_START_GEN;
                }
                else
            #endif /* (Sense_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED) */

            #if(Sense_I2C_MODE_MULTI_MASTER_ENABLED)

                if(Sense_I2C_CHECK_LOST_ARB(Sense_I2C_CSR_REG))
                {
                    Sense_I2C_BUS_RELEASE;

                    /* Master lost arbitrage: reset FSM to IDLE */
                    Sense_I2C_state = Sense_I2C_SM_IDLE;
                    errStatus = Sense_I2C_MSTR_ERR_ARB_LOST;
                }
                else
            #endif /* (Sense_I2C_MODE_MULTI_MASTER_ENABLED) */

                if(Sense_I2C_CHECK_ADDR_NAK(Sense_I2C_CSR_REG))
                {
                    /* Address has been NACKed: reset FSM to IDLE */
                    Sense_I2C_state = Sense_I2C_SM_IDLE;
                    errStatus = Sense_I2C_MSTR_ERR_LB_NAK;
                }
                else
                {
                    /* Start was sent witout errors */
                    errStatus = Sense_I2C_MSTR_NO_ERROR;
                }
        }
        else
        {
            errStatus = Sense_I2C_MSTR_BUS_BUSY; /* Bus is busy */
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterSendRestart
********************************************************************************
*
* Summary:
*  Generates ReStart condition and sends slave address with read/write bit.
*
* Parameters:
*  slaveAddress:  7-bit slave address.
*  R_nW:          Zero, send write command, non-zero send read command.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  This function is entered without a 'byte complete' bit set in the I2C_CSR
*  register. It does not exit until it will be set.
*
* Global variables:
*  Sense_I2C_state - used to store current state of software FSM.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 Sense_I2C_MasterSendRestart(uint8 slaveAddress, uint8 R_nW)
      
{
    uint8 errStatus;

    errStatus = Sense_I2C_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(Sense_I2C_CHECK_MASTER_MODE(Sense_I2C_MCSR_REG))
    {
        slaveAddress = ((uint8) (slaveAddress << Sense_I2C_SLAVE_ADDR_SHIFT)); /* Set Address */
        if(0u != R_nW)  /* Set the Read/Write flag */
        {
            slaveAddress |= Sense_I2C_READ_FLAG;
            Sense_I2C_state = Sense_I2C_SM_MSTR_RD_ADDR;
        }
        else
        {
            Sense_I2C_state = Sense_I2C_SM_MSTR_WR_ADDR;
        }
        Sense_I2C_DATA_REG = slaveAddress;    /* Write address to data reg */


        Sense_I2C_GENERATE_RESTART_MANUAL;
        while(Sense_I2C_WAIT_BYTE_COMPLETE(Sense_I2C_CSR_REG))
        {
            ; /* Wait for the address to be transfered */
        }

        #if(Sense_I2C_MODE_MULTI_MASTER_ENABLED)
            if(Sense_I2C_CHECK_LOST_ARB(Sense_I2C_CSR_REG))
            {
                Sense_I2C_BUS_RELEASE;

                /* Master lost arbitrage: reset FSM to IDLE */
                Sense_I2C_state = Sense_I2C_SM_IDLE;
                errStatus = Sense_I2C_MSTR_ERR_ARB_LOST;
            }
            else
        #endif /* (Sense_I2C_MODE_MULTI_MASTER_ENABLED) */

            if(Sense_I2C_CHECK_ADDR_NAK(Sense_I2C_CSR_REG))
            {
                /* Address has been NACKed: reset FSM to IDLE */
                Sense_I2C_state = Sense_I2C_SM_IDLE;
                errStatus = Sense_I2C_MSTR_ERR_LB_NAK;
            }
            else
            {
                /* ReStart was sent witout errors */
                errStatus = Sense_I2C_MSTR_NO_ERROR;
            }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterSendStop
********************************************************************************
*
* Summary:
*  Generates I2C Stop condition on bus. Function do nothing if Start or Restart
*  condition was failed before call this function.
*
* Parameters:
*  None
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  The Stop generation is required to complete transaction.
*  This function does not wait whileStop condition will be generated.
*
* Global variables:
*  Sense_I2C_state - used to store current state of software FSM.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 Sense_I2C_MasterSendStop(void) 
{
    uint8 errStatus;

    errStatus = Sense_I2C_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(Sense_I2C_CHECK_MASTER_MODE(Sense_I2C_MCSR_REG))
    {
        Sense_I2C_GENERATE_STOP_MANUAL;              /* Generate STOP */
        Sense_I2C_state = Sense_I2C_SM_IDLE;  /* Reset state to IDLE */

        while(Sense_I2C_WAIT_STOP_COMPLETE(Sense_I2C_CSR_REG))
        {
            ; /* Wait for Stop to be generated */
        }

        errStatus = Sense_I2C_MSTR_NO_ERROR;
        #if(Sense_I2C_MODE_MULTI_MASTER_ENABLED)
            if(Sense_I2C_CHECK_LOST_ARB(Sense_I2C_CSR_REG))
            {
                Sense_I2C_BUS_RELEASE;

                /* NACK was generated by enother instead Stop */
                errStatus = Sense_I2C_MSTR_ERR_ARB_LOST;
            }
        #endif /* (Sense_I2C_MODE_MULTI_MASTER_ENABLED) */
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterWriteByte
********************************************************************************
*
* Summary:
*  Sends one byte to a slave. A valid Start or ReStart condition must be
*  generated before this call this function. Function do nothing if Start or
*  Restart condition was failed before call this function.
*
* Parameters:
*  data:  The data byte to send to the slave.
*
* Return:
*  Status error - zero means no errors.
*
* Side Effects:
*  This function is entered without a 'byte complete' bit set in the I2C_CSR
*  register. It does not exit until it will be set.
*
* Global variables:
*  Sense_I2C_state - used to store current state of software FSM.
*
*******************************************************************************/
uint8 Sense_I2C_MasterWriteByte(uint8 theByte) 
{
    uint8 errStatus;

    errStatus = Sense_I2C_MSTR_NOT_READY;

    /* Check if START condition was generated */
    if(Sense_I2C_CHECK_MASTER_MODE(Sense_I2C_MCSR_REG))
    {
        Sense_I2C_DATA_REG = theByte;                        /* Write DATA register */
        Sense_I2C_TRANSMIT_DATA_MANUAL;                      /* Set transmit mode */
        Sense_I2C_state = Sense_I2C_SM_MSTR_WR_DATA;  /* Set state WR_DATA */

        /* Make sure the last byte has been transfered first */
        while(Sense_I2C_WAIT_BYTE_COMPLETE(Sense_I2C_CSR_REG))
        {
            ; /* Wait for byte to be written */
        }

        #if(Sense_I2C_MODE_MULTI_MASTER_ENABLED)
            if(Sense_I2C_CHECK_LOST_ARB(Sense_I2C_CSR_REG))
            {
                Sense_I2C_BUS_RELEASE;

                /* Master lost arbitrage: reset FSM to IDLE */
                Sense_I2C_state = Sense_I2C_SM_IDLE;
                errStatus = Sense_I2C_MSTR_ERR_ARB_LOST;
            }
            /* Check LRB bit */
            else
        #endif /* (Sense_I2C_MODE_MULTI_MASTER_ENABLED) */

            if(Sense_I2C_CHECK_DATA_ACK(Sense_I2C_CSR_REG))
            {
                Sense_I2C_state = Sense_I2C_SM_MSTR_HALT;     /* Set state to HALT */
                errStatus = Sense_I2C_MSTR_NO_ERROR;                 /* The LRB was ACKed */
            }
            else
            {
                Sense_I2C_state = Sense_I2C_SM_MSTR_HALT;     /* Set state to HALT */
                errStatus = Sense_I2C_MSTR_ERR_LB_NAK;               /* The LRB was NACKed */
            }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterReadByte
********************************************************************************
*
* Summary:
*  Reads one byte from a slave and ACK or NACK the transfer. A valid Start or
*  ReStart condition must be generated before this call this function. Function
*  do nothing if Start or Restart condition was failed before call this
*  function.
*
* Parameters:
*  acknNack:  Zero, response with NACK, if non-zero response with ACK.
*
* Return:
*  Byte read from slave.
*
* Side Effects:
*  This function is entered without a 'byte complete' bit set in the I2C_CSR
*  register. It does not exit until it will be set.
*
* Global variables:
*  Sense_I2C_state - used to store current state of software FSM.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 Sense_I2C_MasterReadByte(uint8 acknNak) 
{
    uint8 theByte;

    theByte = 0u;

    /* Check if START condition was generated */
    if(Sense_I2C_CHECK_MASTER_MODE(Sense_I2C_MCSR_REG))
    {
        /* When address phase need release the bus and receive the byte, then decide ACK or NACK */
        if(Sense_I2C_SM_MSTR_RD_ADDR == Sense_I2C_state)
        {
            Sense_I2C_state = Sense_I2C_SM_MSTR_RD_DATA;
            Sense_I2C_READY_TO_READ_MANUAL;
        }

        while(Sense_I2C_WAIT_BYTE_COMPLETE(Sense_I2C_CSR_REG))
        {
            ; /* Wait for byte to be read */
        }

        theByte = Sense_I2C_DATA_REG;

        /* Now if the ACK flag was set, ACK the data which will release the bus and
           start the next byte in otherwise do NOTHING to the CSR reg.
           This will allow the calling routine to generate a repeat start or
           stop depending on it's preference. */
        if(acknNak != 0u)   /* Do ACK */
        {
            Sense_I2C_ACK_AND_RECEIVE_MANUAL;
        }
        else                /* Do NACK */
        {
            /* Do nothing to be able work with ReStart */
            Sense_I2C_state = Sense_I2C_SM_MSTR_HALT;
        }
    }

    return(theByte);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterStatus
********************************************************************************
*
* Summary:
*  Returns the master's communication status.
*
* Parameters:
*  None
*
* Return:
*  Current status of I2C master.
*
* Global variables:
*  Sense_I2C_mstrStatus - used to store current status of I2C Master.
*
*******************************************************************************/
uint8 Sense_I2C_MasterStatus(void) 
{
    uint8 status;

    status = Sense_I2C_mstrStatus;

    /* When in Master state only transaction is in progress */
    if(Sense_I2C_CHECK_SM_MASTER)
    {
        /* Add transaction in progress activity to master status */
        status |= Sense_I2C_MSTAT_XFER_INP;
    }
    else
    {
        /* Current master status is valid */
    }

    return(status);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterClearStatus
********************************************************************************
*
* Summary:
*  Clears all status flags and returns the master status.
*
* Parameters:
*  None
*
* Return:
*  Current status of I2C master.
*
* Global variables:
*  Sense_I2C_mstrStatus - used to store current status of I2C Master.
*
* Reentrant:
*  No
*
*******************************************************************************/
uint8 Sense_I2C_MasterClearStatus(void) 
{
    /* Current master status */
    uint8 status;

    /* Read and clear master status */
    status = Sense_I2C_mstrStatus;
    Sense_I2C_mstrStatus = Sense_I2C_MSTAT_CLEAR;

    return(status);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterGetReadBufSize
********************************************************************************
*
* Summary:
*  Returns the amount of bytes that has been transferred with an
*  I2C_MasterReadBuf command.
*
* Parameters:
*  None
*
* Return:
*  Byte count of transfer. If the transfer is not yet complete, it will return
*  the byte count transferred so far.
*
* Global variables:
*  Sense_I2C_mstrRdBufIndex - used to current index within master read
*  buffer.
*
*******************************************************************************/
uint8 Sense_I2C_MasterGetReadBufSize(void) 
{
    return(Sense_I2C_mstrRdBufIndex);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterGetWriteBufSize
********************************************************************************
*
* Summary:
*  Returns the amount of bytes that has been transferred with an
*  I2C_MasterWriteBuf command.
*
* Parameters:
*  None
*
* Return:
*  Byte count of transfer. If the transfer is not yet complete, it will return
*  the byte count transferred so far.
*
* Global variables:
*  Sense_I2C_mstrWrBufIndex - used to current index within master write
*  buffer.
*
*******************************************************************************/
uint8 Sense_I2C_MasterGetWriteBufSize(void) 
{
    return(Sense_I2C_mstrWrBufIndex);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterClearReadBuf
********************************************************************************
*
* Summary:
*  Resets the read buffer pointer back to the first byte in the buffer.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  Sense_I2C_mstrRdBufIndex - used to current index within master read
*   buffer.
*  Sense_I2C_mstrStatus - used to store current status of I2C Master.
*
* Reentrant:
*  No
*
*******************************************************************************/
void Sense_I2C_MasterClearReadBuf(void) 
{
    Sense_I2C_mstrRdBufIndex = 0u;
    Sense_I2C_mstrStatus    &= ((uint8) ~Sense_I2C_MSTAT_RD_CMPLT);
}


/*******************************************************************************
* Function Name: Sense_I2C_MasterClearWriteBuf
********************************************************************************
*
* Summary:
*  Resets the write buffer pointer back to the first byte in the buffer.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  Sense_I2C_mstrRdBufIndex - used to current index within master read
*   buffer.
*  Sense_I2C_mstrStatus - used to store current status of I2C Master.
*
* Reentrant:
*  No
*
*******************************************************************************/
void Sense_I2C_MasterClearWriteBuf(void) 
{
    Sense_I2C_mstrWrBufIndex = 0u;
    Sense_I2C_mstrStatus    &= ((uint8) ~Sense_I2C_MSTAT_WR_CMPLT);
}


/*******************************************************************************
* Function Name: Sense_I2C_Workaround
********************************************************************************
*
* Summary:
*  Do nothing. This fake fuction use as workaround for CDT 78083.
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
void Sense_I2C_Workaround(void) 
{
    /* Does nothing */
}

#endif /* (Sense_I2C_MODE_MASTER_ENABLED) */


/* [] END OF FILE */
