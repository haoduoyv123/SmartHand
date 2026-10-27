#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern volatile uint16_t tim_log;
extern TIM_HandleTypeDef htim1;

uint16_t Sensor_Read(void);
float Convert_TO_Val(uint16_t num);
void Sensor_Log(void);


#endif
