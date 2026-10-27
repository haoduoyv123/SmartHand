#ifndef __MOT_DRV_H__
#define __MOT_DRV_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 用户可配置参数 ============================ */
#define SERVO_PWM_HZ        (50U)       /* 舵机 PWM 频率 (Hz)                        */
#define SERVO_TIM_TICK_HZ   (1000000U)  /* PWM 计数时钟目标值(Hz)，1MHz => 1计数=1us */
/* 180° 舵机脉宽标定：0.5ms/0° 1.0ms/45° 1.5ms/90° 2.0ms/135° 2.5ms/180° */
#define SERVO_PULSE_MIN_US  (500U)      /* 0°   对应脉宽(us) */
#define SERVO_PULSE_MID_US  (1500U)     /* 90°  对应脉宽(us) */
#define SERVO_PULSE_MAX_US  (2500U)     /* 180° 对应脉宽(us) */
#define SERVO_ANGLE_MIN     (0.0f)      /* 最小角度(°)                               */
#define SERVO_ANGLE_MAX     (180.0f)    /* 最大角度(°)                               */
#define SERVO_HOME_ANGLE    (90.0f)     /* 上电归位角度(°)                           */
#define SERVO_VEL_MAX_DFT   (120.0f)    /* 默认最大角速度 (°/s)                      */
#define SERVO_ACC_DFT       (240.0f)    /* 默认角加速度 (°/s²)                       */
#define SERVO_ANGLE_EPS     (0.05f)     /* 到位判定阈值(°)                           */

#define SERVO_NUM           (4U)        /* TIM2 的 4 路 PWM：CH1~CH4                */

/* 舵机通道编号，对应 TIM2_CH1 ~ TIM2_CH4 */
typedef enum
{
    SERVO_CH1 = 0U,
    SERVO_CH2,
    SERVO_CH3,
    SERVO_CH4
} Servo_Ch_t;

/* 单路舵机运行参数 */
typedef struct
{
    uint32_t tim_channel;   /* 对应的 TIM 通道 (TIM_CHANNEL_x) */
    uint16_t pulse_min_us;  /* 最小角度脉宽 */
    uint16_t pulse_max_us;  /* 最大角度脉宽 */
    float    angle_min;     /* 角度下限 */
    float    angle_max;     /* 角度上限 */
    float    vel_max;       /* 最大角速度 (°/s) */
    float    accel;         /* 角加速度 (°/s²) */
    float    angle;         /* 当前角度(°) —— 规划器输出 */
    float    target;        /* 目标角度(°) */
    float    vel;           /* 当前角速度(°/s) */
    uint32_t ccr;           /* 当前写入的比较值 */
    uint8_t  enabled;       /* 输出使能标志 */
} Servo_Handle_t;

/* ------------------------------ 基础接口 ------------------------------ */
void     Servo_Init(void);                                        /* 初始化：配置 50Hz 并启动 4 路 PWM */
void     Servo_Update(void);                                      /* 周期调用(1~5ms)的平滑运动规划     */
void     Servo_Enable(uint8_t ch, uint8_t en);                    /* 使能/关闭某路 PWM 输出            */

/* ---------------------------- CCR / 脉宽设置 ---------------------------- */
void     Servo_SetCCR(uint8_t ch, uint32_t ccr);                  /* 直接写 CCR(自动限幅到 ARR)        */
void     Servo_SetPulseUs(uint8_t ch, uint16_t us);               /* 按脉宽(us)设置输出                */
uint32_t Servo_GetCCR(uint8_t ch);
uint16_t Servo_GetPulseUs(uint8_t ch);                            /* 当前角度对应的脉宽(us)            */

/* ------------------------------ 角度控制 ------------------------------ */
void     Servo_SetAngle(uint8_t ch, float angle);                 /* 立即定位(不做插补)                */
void     Servo_SetAngleSmooth(uint8_t ch, float angle);           /* 平滑转动到目标角度                */
void     Servo_SetAnglesSmooth(const float *angles, uint16_t duration_ms); /* 4 路同步平滑转动      */
float    Servo_GetAngle(uint8_t ch);
uint8_t  Servo_IsMoving(uint8_t ch);

/* ------------------------------ 参数标定 ------------------------------ */
void     Servo_SetSpeed(uint8_t ch, float vel_max, float accel);  /* 单独设置速度/加速度               */
void     Servo_SetPulseRange(uint8_t ch, uint16_t min_us, uint16_t max_us);
void     Servo_SetAngleRange(uint8_t ch, float angle_min, float angle_max);

#ifdef __cplusplus
}
#endif

#endif /* __MOT_DRV_H__ */
