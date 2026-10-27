#include "led.h"
#include "stm32h750xx.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_it.h"

void LED_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {

        .Alternate = 0,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pin = GPIO_PIN_13,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };


    HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,  GPIO_PIN_RESET);

}

void LED_Tringle(void)
{

    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    HAL_Delay(300);
    
}





