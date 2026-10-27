#include "main.h"
#include <stdio.h>
#include "led.h"
#include "sensor.h"
#include "peripheral.h"
#include "log.h"
#include "mot_drv.h"


float val;
uint8_t a = 'a';

/* ==================== 单舵机角度调试 ==================== */
#define SERVO_DEBUG_CH      SERVO_CH1      /* 调试对象：TIM2_CH1         */
#define SERVO_DEBUG_HOLD_MS 1500U          /* 每个角度保持时间(ms)       */

/* 对应 0.5ms/1.0ms/1.5ms/2.0ms/2.5ms 五个标定点 */
static const float g_servo_debug_angle[] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f};
#define SERVO_DEBUG_POINT_NUM (sizeof(g_servo_debug_angle) / sizeof(g_servo_debug_angle[0]))

static uint8_t  g_servo_debug_idx  = 0U;
static int8_t   g_servo_debug_dir  = 1;    /* 往返扫描，避免 180° 直接跳回 0° */
static uint8_t  g_servo_debug_busy = 0U;   /* 当前角度指令是否已下发 */
static uint32_t g_servo_debug_tick = 0U;

/* 依次转到 0/45/90/135/180° 并往返，到位后串口打印实际角度、脉宽与 CCR */
static void Servo_Debug_Task(void)
{
    if (g_servo_debug_busy == 0U)
    {
        /* 上一个动作已到位且保持时间到，下发下一个角度 */
        if (Servo_IsMoving(SERVO_DEBUG_CH) != 0U)
        {
            return;
        }
        if ((HAL_GetTick() - g_servo_debug_tick) < SERVO_DEBUG_HOLD_MS)
        {
            return;
        }

        Servo_SetAngleSmooth(SERVO_DEBUG_CH, g_servo_debug_angle[g_servo_debug_idx]);
        g_servo_debug_busy = 1U;
        return;
    }

    /* 等待本次动作到位后打印结果 */
    if (Servo_IsMoving(SERVO_DEBUG_CH) != 0U)
    {
        return;
    }

    printf("CH%d -> %5.1f deg | pulse %4u us | ccr %5lu\r\n",
           (int)SERVO_DEBUG_CH + 1,
           (double)Servo_GetAngle(SERVO_DEBUG_CH),
           (unsigned int)Servo_GetPulseUs(SERVO_DEBUG_CH),
           (unsigned long)Servo_GetCCR(SERVO_DEBUG_CH));

    g_servo_debug_busy = 0U;
    g_servo_debug_tick = HAL_GetTick();

    /* 计算下一个标定点（到两端折返） */
    if (g_servo_debug_dir > 0)
    {
        if ((size_t)g_servo_debug_idx >= (SERVO_DEBUG_POINT_NUM - 1U))
        {
            g_servo_debug_dir = -1;
            g_servo_debug_idx = (uint8_t)(SERVO_DEBUG_POINT_NUM - 2U);
        }
        else
        {
            g_servo_debug_idx++;
        }
    }
    else
    {
        if (g_servo_debug_idx == 0U)
        {
            g_servo_debug_dir = 1;
            g_servo_debug_idx = 1U;
        }
        else
        {
            g_servo_debug_idx--;
        }
    }
}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


  HAL_Init();

  peripheral_Init();
  /* USER CODE BEGIN 2 */
  Servo_Init();                 /* 配置 TIM2 为 50Hz 并启动 4 路 PWM */
  Servo_Enable(SERVO_CH2, 0U);  /* 单舵机调试：关闭其余 3 路输出 */
  Servo_Enable(SERVO_CH3, 0U);
  Servo_Enable(SERVO_CH4, 0U);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Starting application...\n");
  ST_Once_log(&a);
  while (1)
  {
    /* USER CODE END WHILE */
    // printf("hello\r\n");
    Servo_Update();             /* 舵机平滑运动规划，需周期调用 */
    Servo_Debug_Task();         /* 单舵机角度调试 */

    if(tim_log == 1){
      
        Sensor_Log();
        tim_log = 0;
    }
    
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}






