#include "log.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_it.h"
#include <string.h>



void ST_Once_log(uint8_t *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen((const char *)str), 1000);
}
