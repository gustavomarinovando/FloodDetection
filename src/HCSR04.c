/*
 * HCSR04.c
 *
 *  Created on: Dec 3, 2018
 *      Author: Gustavo
 */

#include "HCSR04.h"
#include "US_Delay.h"
#include "Delay.h"
#include "stm32l1xx_hal.h"


float get_distance();


float get_distance(){
	//US_Init();
	uint32_t time, time_copy;
	float Distance;
	GPIOC -> BRR = TR_Pin;
	//US_Delay(2);						//Delay of 2 microseconds
	Delay(2);						//Delay of 2 microseconds
	GPIOC -> BSRR = TR_Pin;
	//US_Delay(10);						//Delay of 10 microseconds
	Delay(10);						//Delay of 10 microseconds
	GPIOC -> BRR = TR_Pin;
		while ((GPIOC -> IDR & EC_Pin) == 0x00)
			{

			}
		time = 0;

		while ((GPIOC -> IDR & EC_Pin) != 0x00)
			{
				time++;
				//US_Delay(1);
				Delay(1);
			}
		time_copy = time;
		//Distance = (float)time_copy*0.0171821;
		Distance = (float)time_copy*0.0171821*2.3214;

		return Distance;
}



