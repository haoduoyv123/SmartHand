/**
  ******************************************************************************
  * @file    mot_drv.c
  * @brief   50Hz 舵机驱动(TIM2 CH1~CH4 PWM)
  *
  *  功能：
  *   1. Servo_Init()  将 TIM2 配置为精确的 50Hz(周期 20ms)，计数时钟 1MHz(1us/计数)，
  *      并启动 4 路 PWM；
  *   2. 角度 <-> 脉宽 <-> CCR 三级换算，脉宽/角度范围可按通道标定；
  *   3. 梯形速度规划(加速度 + 速度限幅)，Servo_Update() 周期调用即可实现平稳运行，
  *      不会因角度突变产生抖动或冲击。
  *
  *  用法：
  *       Servo_Init();                       // 初始化后舵机归位到 SERVO_HOME_ANGLE
  *       Servo_SetAngleSmooth(SERVO_CH1, 45.0f);
  *       while (1) { Servo_Update(); }       // 1~5ms 调用一次(主循环或定时中断)
  ******************************************************************************
  */

#include "mot_drv.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;

/* TIM2 4 个通道对应的 HAL 通道号 */
static const uint32_t kServoTimChannel[SERVO_NUM] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static Servo_Handle_t s_servo[SERVO_NUM];
static uint32_t s_tick_hz;      /* PWM 实际计数频率(Hz) */
static uint32_t s_arr;          /* 自动重装载值 */
static uint32_t s_period_us;    /* PWM 周期(us) */
static uint32_t s_last_tick;    /* 上次规划的 HAL_GetTick() */

/* ------------------------------ 内部工具 ------------------------------ */

/* 获取 TIM2 的定时器内核时钟(TIM2 挂在 APB1) */
static uint32_t Servo_GetTimClock(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t apb1_pre = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
    uint32_t mul = 1U;

    /* APB1 不分频时定时器时钟 = PCLK1；分频时再倍频(TIMPRE=0:×2，TIMPRE=1:×4) */
    if (apb1_pre != 0U)
    {
        mul = ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0U) ? 4U : 2U;
    }
    return pclk1 * mul;
}

/* 配置 TIM2 输出指定频率的 PWM，计数时钟尽量接近 SERVO_TIM_TICK_HZ(1MHz) */
static void Servo_ConfigPwmFreq(uint32_t freq_hz)
{
    uint32_t timclk = Servo_GetTimClock();
    uint32_t div    = timclk / SERVO_TIM_TICK_HZ;   /* 分频系数 = PSC + 1 */
    uint32_t tick, arr;

    if (div == 0U)
    {
        div = 1U;
    }

    /* 保证 ARR 落在 16 位以内，兼容 16 位定时器 */
    while (((timclk / div) / freq_hz) > 0xFFFFU)
    {
        div++;
    }

    tick = timclk / div;                            /* 实际计数频率 */
    arr  = (tick + (freq_hz / 2U)) / freq_hz;      /* 四舍五入 */
    if (arr < 2U)
    {
        arr = 2U;
    }

    htim2.Init.Prescaler = div - 1U;
    htim2.Init.Period    = arr - 1U;
    HAL_TIM_PWM_Init(&htim2);                       /* 写入 PSC/ARR 并产生更新事件 */

    s_tick_hz   = tick;
    s_arr       = arr - 1U;
    s_period_us = (uint32_t)(((uint64_t)arr * 1000000ULL) / tick);
}

/* 脉宽(us) -> CCR */
static uint32_t Servo_PulseToCCR(uint16_t us)
{
    uint32_t ccr = (uint32_t)(((uint64_t)us * s_tick_hz) / 1000000ULL);

    if (ccr > s_arr)
    {
        ccr = s_arr;
    }
    return ccr;
}

/* 角度 -> 脉宽(us)，带限幅 */
static uint16_t Servo_AngleToPulse(const Servo_Handle_t *s, float angle)
{
    float range, t, pulse;

    if (angle < s->angle_min) angle = s->angle_min;
    if (angle > s->angle_max) angle = s->angle_max;

    range = s->angle_max - s->angle_min;
    t = (range > 0.0f) ? ((angle - s->angle_min) / range) : 0.0f;
    pulse = (float)s->pulse_min_us + t * (float)(s->pulse_max_us - s->pulse_min_us);

    /* 限制在 0~周期 内，避免超出 20ms 帧 */
    if (pulse < 0.0f)              pulse = 0.0f;
    if (pulse > (float)s_period_us) pulse = (float)s_period_us;

    return (uint16_t)(pulse + 0.5f);
}

/* 把当前角度输出到 CCR */
static void Servo_Apply(Servo_Handle_t *s)
{
    uint32_t ccr = Servo_PulseToCCR(Servo_AngleToPulse(s, s->angle));

    s->ccr = ccr;
    if (s->enabled)
    {
        __HAL_TIM_SET_COMPARE(&htim2, s->tim_channel, ccr);
    }
}

/* 单路梯形速度规划：加速度/速度双限幅，保证启停无冲击 */
static void Servo_Step(Servo_Handle_t *s, float dt)
{
    float err   = s->target - s->angle;
    float dir   = (err >= 0.0f) ? 1.0f : -1.0f;
    float brake;

    if (fabsf(err) <= SERVO_ANGLE_EPS)
    {
        s->angle = s->target;
        s->vel   = 0.0f;
        return;
    }

    /* 以当前速度减速到 0 所需的角度 */
    brake = (s->vel * s->vel) / (2.0f * s->accel);

    if ((s->vel * dir > 0.0f) && (fabsf(err) <= brake))
    {
        /* 已进入制动区：减速 */
        s->vel -= dir * s->accel * dt;
        if (s->vel * dir < 0.0f)
        {
            s->vel = 0.0f;
        }
    }
    else
    {
        /* 加速或匀速：向目标方向加速并限幅 */
        s->vel += dir * s->accel * dt;
        if (fabsf(s->vel) > s->vel_max)
        {
            s->vel = dir * s->vel_max;
        }
    }

    if (fabsf(s->vel * dt) >= fabsf(err))
    {
        s->angle = s->target;
        s->vel   = 0.0f;
    }
    else
    {
        s->angle += s->vel * dt;
    }

    Servo_Apply(s);
}

/* ------------------------------ 对外接口 ------------------------------ */

/**
  * @brief  舵机初始化：配置 50Hz PWM、装载默认参数并启动 4 路输出
  */
void Servo_Init(void)
{
    uint8_t i;

    Servo_ConfigPwmFreq(SERVO_PWM_HZ);

    for (i = 0U; i < SERVO_NUM; i++)
    {
        Servo_Handle_t *s = &s_servo[i];

        s->tim_channel  = kServoTimChannel[i];
        s->pulse_min_us = SERVO_PULSE_MIN_US;
        s->pulse_max_us = SERVO_PULSE_MAX_US;
        s->angle_min    = SERVO_ANGLE_MIN;
        s->angle_max    = SERVO_ANGLE_MAX;
        s->vel_max      = SERVO_VEL_MAX_DFT;
        s->accel        = SERVO_ACC_DFT;
        s->angle        = SERVO_HOME_ANGLE;
        s->target       = SERVO_HOME_ANGLE;
        s->vel          = 0.0f;
        s->enabled      = 1U;

        /* 先装入 CCR 再启动输出，避免上电瞬间出现错误脉宽 */
        s->ccr = Servo_PulseToCCR(Servo_AngleToPulse(s, s->angle));
        __HAL_TIM_SET_COMPARE(&htim2, s->tim_channel, s->ccr);
        HAL_TIM_PWM_Start(&htim2, s->tim_channel);
    }

    s_last_tick = HAL_GetTick();
}

/**
  * @brief  平滑运动规划，需周期性调用(推荐 1~5ms)
  */
void Servo_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms;
    float    dt;
    uint8_t  i;

    if (now == s_last_tick)
    {
        return;
    }

    dt_ms = (now >= s_last_tick) ? (now - s_last_tick) : 1U;
    if (dt_ms > 100U)
    {
        dt_ms = 100U;                 /* 长时间未调用时限制步长，防止跳变 */
    }
    s_last_tick = now;
    dt = (float)dt_ms * 0.001f;

    for (i = 0U; i < SERVO_NUM; i++)
    {
        if (s_servo[i].enabled)
        {
            Servo_Step(&s_servo[i], dt);
        }
    }
}

/**
  * @brief  使能/关闭某路 PWM 输出
  */
void Servo_Enable(uint8_t ch, uint8_t en)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }

    if (en != 0U)
    {
        if (s_servo[ch].enabled == 0U)
        {
            s_servo[ch].enabled = 1U;
            __HAL_TIM_SET_COMPARE(&htim2, s_servo[ch].tim_channel, s_servo[ch].ccr);
            HAL_TIM_PWM_Start(&htim2, s_servo[ch].tim_channel);
        }
    }
    else
    {
        s_servo[ch].enabled = 0U;
        HAL_TIM_PWM_Stop(&htim2, s_servo[ch].tim_channel);
    }
}

/**
  * @brief  直接设置比较值(限幅到 0~ARR)
  */
void Servo_SetCCR(uint8_t ch, uint32_t ccr)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }

    if (ccr > s_arr)
    {
        ccr = s_arr;
    }

    s_servo[ch].ccr = ccr;
    if (s_servo[ch].enabled)
    {
        __HAL_TIM_SET_COMPARE(&htim2, s_servo[ch].tim_channel, ccr);
    }
}

/**
  * @brief  按脉宽(us)设置输出
  */
void Servo_SetPulseUs(uint8_t ch, uint16_t us)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }
    Servo_SetCCR(ch, Servo_PulseToCCR(us));
}

/**
  * @brief  读取当前 CCR
  */
uint32_t Servo_GetCCR(uint8_t ch)
{
    return (ch < SERVO_NUM) ? s_servo[ch].ccr : 0U;
}

/**
  * @brief  读取当前角度对应的脉宽(us)
  */
uint16_t Servo_GetPulseUs(uint8_t ch)
{
    return (ch < SERVO_NUM) ? Servo_AngleToPulse(&s_servo[ch], s_servo[ch].angle) : 0U;
}

/**
  * @brief  立即定位到指定角度(无插补，用于标定或上电初始化)
  */
void Servo_SetAngle(uint8_t ch, float angle)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }

    Servo_Handle_t *s = &s_servo[ch];

    if (angle < s->angle_min) angle = s->angle_min;
    if (angle > s->angle_max) angle = s->angle_max;

    s->angle  = angle;
    s->target = angle;
    s->vel    = 0.0f;
    Servo_Apply(s);
}

/**
  * @brief  平滑转动到指定角度，由 Servo_Update() 完成插补
  */
void Servo_SetAngleSmooth(uint8_t ch, float angle)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }

    Servo_Handle_t *s = &s_servo[ch];

    if (angle < s->angle_min) angle = s->angle_min;
    if (angle > s->angle_max) angle = s->angle_max;

    s->target = angle;
}

/**
  * @brief  4 路同步平滑转动，duration_ms 为期望运动时间(各路同时到达)
  */
void Servo_SetAnglesSmooth(const float *angles, uint16_t duration_ms)
{
    uint8_t i;
    float   t = (float)duration_ms * 0.001f;

    if (angles == NULL)
    {
        return;
    }

    for (i = 0U; i < SERVO_NUM; i++)
    {
        Servo_Handle_t *s = &s_servo[i];
        float angle = angles[i];
        float dist;

        if (angle < s->angle_min) angle = s->angle_min;
        if (angle > s->angle_max) angle = s->angle_max;

        s->target = angle;
        dist = fabsf(angle - s->angle);

        if ((t > 0.0f) && (dist > 0.0f))
        {
            /* 梯形速度规划：加速 T/4 + 匀速 T/2 + 减速 T/4 => D = 3/4 * vmax * T */
            s->vel_max = (4.0f * dist) / (3.0f * t);
            s->accel   = (4.0f * s->vel_max) / t;
        }
        else
        {
            s->vel_max = SERVO_VEL_MAX_DFT;
            s->accel   = SERVO_ACC_DFT;
        }
    }
}

/**
  * @brief  读取当前规划角度
  */
float Servo_GetAngle(uint8_t ch)
{
    return (ch < SERVO_NUM) ? s_servo[ch].angle : 0.0f;
}

/**
  * @brief  查询该通道是否仍在运动
  */
uint8_t Servo_IsMoving(uint8_t ch)
{
    if (ch >= SERVO_NUM)
    {
        return 0U;
    }
    return (fabsf(s_servo[ch].target - s_servo[ch].angle) > SERVO_ANGLE_EPS) ? 1U : 0U;
}

/**
  * @brief  设置单路最大角速度(°/s)与角加速度(°/s²)
  */
void Servo_SetSpeed(uint8_t ch, float vel_max, float accel)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }
    if (vel_max <= 0.0f) vel_max = SERVO_VEL_MAX_DFT;
    if (accel   <= 0.0f) accel   = SERVO_ACC_DFT;

    s_servo[ch].vel_max = vel_max;
    s_servo[ch].accel   = accel;
}

/**
  * @brief  标定脉宽范围
  */
void Servo_SetPulseRange(uint8_t ch, uint16_t min_us, uint16_t max_us)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }
    s_servo[ch].pulse_min_us = min_us;
    s_servo[ch].pulse_max_us = (max_us > min_us) ? max_us : (uint16_t)(min_us + 1U);
    Servo_Apply(&s_servo[ch]);
}

/**
  * @brief  标定角度范围
  */
void Servo_SetAngleRange(uint8_t ch, float angle_min, float angle_max)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }
    s_servo[ch].angle_min = angle_min;
    s_servo[ch].angle_max = (angle_max > angle_min) ? (angle_max) : (angle_min + 1.0f);

    if (s_servo[ch].target < s_servo[ch].angle_min) s_servo[ch].target = s_servo[ch].angle_min;
    if (s_servo[ch].target > s_servo[ch].angle_max) s_servo[ch].target = s_servo[ch].angle_max;
    Servo_Apply(&s_servo[ch]);
}
