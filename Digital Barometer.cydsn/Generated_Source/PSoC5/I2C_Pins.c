/*******************************************************************************
* File Name: I2C_Pins.c  
* Version 1.90
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "I2C_Pins.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 I2C_Pins__PORT == 15 && ((I2C_Pins__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: I2C_Pins_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None
*  
*******************************************************************************/
void I2C_Pins_Write(uint8 value) 
{
    uint8 staticBits = (I2C_Pins_DR & (uint8)(~I2C_Pins_MASK));
    I2C_Pins_DR = staticBits | ((uint8)(value << I2C_Pins_SHIFT) & I2C_Pins_MASK);
}


/*******************************************************************************
* Function Name: I2C_Pins_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to this drive mode.
*
* Return: 
*  None
*
*******************************************************************************/
void I2C_Pins_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(I2C_Pins_0, mode);
	CyPins_SetPinDriveMode(I2C_Pins_1, mode);
}


/*******************************************************************************
* Function Name: I2C_Pins_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro I2C_Pins_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 I2C_Pins_Read(void) 
{
    return (I2C_Pins_PS & I2C_Pins_MASK) >> I2C_Pins_SHIFT;
}


/*******************************************************************************
* Function Name: I2C_Pins_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 I2C_Pins_ReadDataReg(void) 
{
    return (I2C_Pins_DR & I2C_Pins_MASK) >> I2C_Pins_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(I2C_Pins_INTSTAT) 

    /*******************************************************************************
    * Function Name: I2C_Pins_ClearInterrupt
    ********************************************************************************
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 I2C_Pins_ClearInterrupt(void) 
    {
        return (I2C_Pins_INTSTAT & I2C_Pins_MASK) >> I2C_Pins_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
