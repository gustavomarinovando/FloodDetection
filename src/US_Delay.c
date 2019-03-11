/*
 * US_Delay.c
 *
 *  Created on: Dec 8, 2018
 *      Author: gustavo
 */
#include "stm32l1xx_hal.h"          // change to whatever MCU you use
#include "US_Delay.h"


void US_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

/**
 * Delay routine itself.
 * Time is in microseconds (1/1000000th of a second), not to be
 * confused with millisecond (1/1000th).
 *
 * @param uint32_t us  Number of microseconds to delay for
 */
void US_Delay(uint32_t us) // microseconds
{
  int32_t targetTick = DWT->CYCCNT + us * (SystemCoreClock/1000000);
  while (DWT->CYCCNT <= targetTick);
}
