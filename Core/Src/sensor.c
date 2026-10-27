#include "main.h"
#include "sensor.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_adc.h"
#include <stdint.h>
#include <stdio.h>

volatile uint16_t tim_log = 0;
static uint16_t adc_value = 0;

/* 连续转换模式：直接读取 ADC 数据寄存器的最新转换结果 */
uint16_t Sensor_Read(void)
{
    return (uint16_t)HAL_ADC_GetValue(&hadc1);
}

/* 16 位精度，参考电压 3.3V */
float Convert_TO_Val(uint16_t num)
{
    return (float)num * 3.3f / 65535.0f;
}

/* 打印 ADC 原始值与对应电压 */
void Sensor_Log(void)
{
    float voltage = Convert_TO_Val(adc_value);
    printf("ADC: %u, Voltage: %.3f V\r\n", adc_value, voltage);
}

/* 定时器更新中断回调：定时采样 ADC */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
    adc_value = Sensor_Read();
    tim_log = 1;
}
