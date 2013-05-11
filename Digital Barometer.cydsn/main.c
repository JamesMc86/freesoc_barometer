/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <device.h>

float temp, pressure;


void main()
{
    uint8 SensorActive;
	
	CyGlobalIntEnable; /* Uncomment this line to enable global interrupts. */
	
	/* Place your initialization/startup code here (e.g. MyInst_Start()) */
	SensorActive = MPL3115A2Start(MPL3115A2_BAR_MODE);
	if (SensorActive == 0) {LED_Out_Write(1);}
	else {LED_Out_Write(0);}


    
    for(;;)
    {
        /* Place your application code here. */
		temp =  MPL3115A2ReadTemp();
		pressure = MPL3115A2ReadPressure();
		CyDelay(1000);
		
		
		
    }
}




/* [] END OF FILE */
