/* ========================================
 *
 * Library for I2C comms with the MPL3115A2
 * Altitude/Pressure/Temperature Sensor by
 * Hobbytronics and Sparkfun.
 *
 * Copyright James McNally, 2013
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <MPL3115A2 Sensor.h>

uint8 MPL3115A2Start(uint8 mode)
{
	uint8 response,ModeFlag, OSFlag;
	
	//Set mode bit
	if (mode == MPL3115A2_ALT_MODE) {ModeFlag = 0x80;}
	else {ModeFlag = 0x00;}
	
	//Set oversample bits.
	OSFlag = 0x38; //Higest for now.
	
	I2C_Start();
	
	//Do basic read to check connection.
	response = MPL3115A2ReadReg(0x0C);
	//Write LED to show good I2C bus
	if (response == 196) {
	
		MPL3115A2SetStandby(); //Needed for reg changes.
		
		//May want to add to here:
		//Sea level calibration.
		//Interrupt setup or ready flags.
		
		MPL3115A2WriteReg(0x26,ModeFlag | OSFlag | 0x03); 
		
		return 0;
		}
	else {return 1;}
	
}

void MPL3115A2SetStandby() {

	uint8 CtrlRegValue;
	
	CtrlRegValue = MPL3115A2ReadReg(0x26);
	CtrlRegValue = CtrlRegValue & 0xFE;
	MPL3115A2WriteReg(0x26,CtrlRegValue);
}

uint8 MPL3115A2ReadReg(uint8 reg_addr)
{
	uint8 data_buffer,err_status;
	
	I2C_MasterClearStatus();
	err_status = I2C_MasterWriteBuf(TEMP_ADDR,&reg_addr,1,I2C_MODE_NO_STOP);
	
		//Wait for bus completion.
	for(;;)
	{
		if(0u != (I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT))
		{
			/* Transfer complete. Check Master status to make sure that transfer 
			completed without errors. */
			break;
		}
	}
	
	I2C_MasterClearStatus();
	err_status = I2C_MasterReadBuf(TEMP_ADDR,&data_buffer,1,I2C_MODE_REPEAT_START);
	
	//Wait for bus completion.
	for(;;)
	{
		if(0u != (I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT))
		{
			/* Transfer complete. Check Master status to make sure that transfer 
			completed without errors. */
			break;
		}
	}
	
	
	return data_buffer;
}



//Writes the values to the registers and returns the master status response.
uint8 MPL3115A2WriteReg(uint8 regAddr, uint8 regValue)
{
	uint8 err_status;
	uint8 dataBuffer[2];
	
	//Put values in a single write buffer.
	dataBuffer[0] = regAddr;
	dataBuffer[1] = regValue;
	
	I2C_MasterClearStatus();
	err_status = I2C_MasterWriteBuf(TEMP_ADDR,dataBuffer,2,I2C_MODE_COMPLETE_XFER);
	
		//Wait for bus completion.
	for(;;)
	{
		if(0u != (I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT))
		{
			/* Transfer complete. Check Master status to make sure that transfer 
			completed without errors. */
			break;
		}
	}
	
	return I2C_MasterStatus();
}

//Reads the temperature from the device registers
//and returns in degC
float MPL3115A2ReadTemp() {

	uint8 IntegerTemp, FractionalTemp;
	float FractionalFloat;
	
	IntegerTemp = MPL3115A2ReadReg(0x04);
	FractionalTemp = MPL3115A2ReadReg(0x05);
	FractionalFloat = (float)(FractionalTemp>>4)/16;
	return FractionalFloat + (float)IntegerTemp;
}

float MPL3115A2ReadPressure() {
	
	uint8 p_reg[3], FractionalPressure;
	int IntegerPressure;
	float Pressure;
	
	//Load the registervalues MSB first.
	p_reg[0] = MPL3115A2ReadReg(0x01);
	p_reg[1] = MPL3115A2ReadReg(0x02);
	p_reg[2] = MPL3115A2ReadReg(0x03);
	
	//Presure is 20bit left aligned with 2 fractional bits.
	IntegerPressure = (int)p_reg[0];
	IntegerPressure = IntegerPressure << 8; //Shift up MSB
	IntegerPressure = IntegerPressure | (int)p_reg[1];
	IntegerPressure = IntegerPressure << 2; //Shift for last 2 bits
	IntegerPressure = IntegerPressure | (p_reg[2] >> 6);
	
	//Now calc fractional part.
	FractionalPressure = (p_reg[2] >> 4) & 0x03;
	
	Pressure = (float)IntegerPressure + ((float)FractionalPressure/4);
	return Pressure;
}
/* [] END OF FILE */
