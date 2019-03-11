/*
 * Delay.c
 *
 *  Created on: Dec 9, 2018
 *      Author: gustavo
 */

#include "Delay.h"
#include "stm32l1xx_hal.h"

void Delay(uint16_t i)
{
	static uint32_t j = 0, ij = 0;
	for (ij = 0; ij < i; ij++)
		for (j = 0; j < 1; j++);
}
