/**
 * Version 1.0 : 2022-07-21
 */


#include <at_plaformAbstraction_V1_1.h>
#include "stm32wbxx.h"
#include "main.h"



uint32_t atGetSysTick_ms()
{
	return HAL_GetTick();
}



