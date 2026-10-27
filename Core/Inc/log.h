#ifndef __LOG_H__
#define __LOG_H__

#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

void ST_Once_log(uint8_t *str);

#endif 
