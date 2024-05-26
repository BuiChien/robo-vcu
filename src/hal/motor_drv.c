#include <stm32f1xx_hal.h>
#include <math.h>
#include <string.h>
#include "motor_drv.h"
#include "common.h"
#include "low_pass_filter.h"

#define WHEEL_RADIUS_MM                  (WHEEL_DIAMETER_MM / 2)
#define WHEEL_CIRCUMFERENCE              (WHEEL_DIAMETER_MM * M_PI)    //Chu vi của bánh xe
#define ENC_PULSES_TO_MM(x)              (float)((float)((float)x / (ENC_PULSES_PER_REVOLUTION * GEAR_RATIO)) * WHEEL_CIRCUMFERENCE)
#define ENC_PULSES_TO_RAD(cnt,dt)        ((float) 2.0 * M_PI * cnt / ENC_PULSES_PER_REVOLUTION / dt)
#define MM_PER_SECOND_TO_RAD(x)          (x / WHEEL_RADIUS_MM)

#define PID_KP                          2.0f
#define PID_KI                          0.5f
#define PID_KD                          0.25f
#define PID_TAU                         0.02f
#define PID_LIM_MIN                     -MOTOR_VOLTAGE_MAX
#define PID_LIM_MAX                      MOTOR_VOLTAGE_MAX
#define PID_LIM_MIN_INTEGRATOR          -6.0f
#define PID_LIM_MAX_INTEGRATOR          6.0f

#define MOTOR_DRV_LEFT_EN_PIN                       GPIO_PIN_0
#define MOTOR_DRV_LEFT_EN_PORT                      GPIOA
#define MOTOR_DRV_LEFT_DIRA_PIN                     GPIO_PIN_2
#define MOTOR_DRV_LEFT_DIRA_PORT                    GPIOA
#define MOTOR_DRV_LEFT_DIRB_PIN                     GPIO_PIN_3
#define MOTOR_DRV_LEFT_DIRB_PORT                    GPIOA
#define MOTOR_DRV_LEFT_ENCA_PIN                     GPIO_PIN_5
#define MOTOR_DRV_LEFT_ENCA_PORT                    GPIOB
#define MOTOR_DRV_LEFT_ENCB_PIN                     GPIO_PIN_4
#define MOTOR_DRV_LEFT_ENCB_PORT                    GPIOB
#define MOTOR_DRV_LEFT_ENC_TIMER                    TIM3
#define MOTOR_DRV_LEFT_PWM_TIMER                    TIM5
#define MOTOR_DRV_LEFT_PWM_TIMER_CHANNEL            TIM_CHANNEL_1
#define MOTOR_DRV_LEFT_ENC_TIMER_CHANNEL            TIM_CHANNEL_1 | TIM_CHANNEL_2

#define MOTOR_DRV_RIGHT_EN_PIN                      GPIO_PIN_1
#define MOTOR_DRV_RIGHT_EN_PORT                     GPIOA
#define MOTOR_DRV_RIGHT_DIRA_PIN                    GPIO_PIN_4
#define MOTOR_DRV_RIGHT_DIRA_PORT                   GPIOA
#define MOTOR_DRV_RIGHT_DIRB_PIN                    GPIO_PIN_5
#define MOTOR_DRV_RIGHT_DIRB_PORT                   GPIOA
#define MOTOR_DRV_RIGHT_ENCA_PIN                    GPIO_PIN_3
#define MOTOR_DRV_RIGHT_ENCA_PORT                   GPIOB
#define MOTOR_DRV_RIGHT_ENCB_PIN                    GPIO_PIN_15
#define MOTOR_DRV_RIGHT_ENCB_PORT                   GPIOA
#define MOTOR_DRV_RIGHT_ENC_TIMER                   TIM2
#define MOTOR_DRV_RIGHT_PWM_TIMER                   TIM5
#define MOTOR_DRV_RIGHT_PWM_TIMER_CHANNEL           TIM_CHANNEL_2
#define MOTOR_DRV_RIGHT_ENC_TIMER_CHANNEL          TIM_CHANNEL_1 | TIM_CHANNEL_2

#define MOTOR_DRV_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE(); \
                                                    __HAL_RCC_GPIOB_CLK_ENABLE();
#define MOTOR_DRV_GPIO_REMAP()                      __HAL_AFIO_REMAP_TIM2_PARTIAL_1(); \
                                                    __HAL_AFIO_REMAP_TIM3_PARTIAL();
#define MOTOR_DRV_TIMER_CLK_ENABLE()                __HAL_RCC_TIM2_CLK_ENABLE(); \
                                                    __HAL_RCC_TIM3_CLK_ENABLE(); \
                                                    __HAL_RCC_TIM5_CLK_ENABLE();

typedef struct
{
    uint32_t Pin;
    GPIO_TypeDef *Port;
} MotorDrv_IoTypeDef;

typedef struct
{
    MotorDrv_IoTypeDef IoEncoderA;
    MotorDrv_IoTypeDef IoEncoderB;
    uint32_t LastCounter;
    uint32_t Counter;
    TIM_TypeDef *Timer;
    TIM_HandleTypeDef *hTimer;
    uint32_t    TimerChannel;
} MotorDrv_EncoderTypeDef;

typedef struct
{
    MotorDrv_IoTypeDef IoEnable;
    MotorDrv_IoTypeDef IoDirA;
    MotorDrv_IoTypeDef IoDirB;
    TIM_TypeDef *Timer;
    TIM_HandleTypeDef *hTimer;
    uint32_t    TimerChannel;
    uint32_t    iLastCalculate;
    MotorDrv_EncoderTypeDef sEncoder;
} MotorDrv_InfoTypeDef;
TIM_HandleTypeDef hEncLeftTimer;
TIM_HandleTypeDef hEncRightTimer;
TIM_HandleTypeDef hPwmTimer;
MotorDrv_InfoTypeDef sMotorInfo[MOTOR_ID_NUM] = {
    {
        .IoEnable = 
        { 
            .Pin = MOTOR_DRV_LEFT_EN_PIN,
            .Port = MOTOR_DRV_LEFT_EN_PORT
        },
        .IoDirA = 
        { 
            .Pin = MOTOR_DRV_LEFT_DIRA_PIN,
            .Port = MOTOR_DRV_LEFT_DIRA_PORT
        },
        .IoDirB = 
        { 
            .Pin = MOTOR_DRV_LEFT_DIRB_PIN,
            .Port = MOTOR_DRV_LEFT_DIRB_PORT
        },
        .Timer = MOTOR_DRV_LEFT_PWM_TIMER,
        .TimerChannel = MOTOR_DRV_LEFT_PWM_TIMER_CHANNEL,
        .hTimer = &hPwmTimer,
        .sEncoder = 
        { 
            .IoEncoderA = { 
                .Pin = MOTOR_DRV_LEFT_ENCA_PIN,
                .Port = MOTOR_DRV_LEFT_ENCA_PORT
            },
            .IoEncoderB = {
                .Pin = MOTOR_DRV_LEFT_ENCB_PIN,
                .Port = MOTOR_DRV_LEFT_ENCB_PORT
            },
            .LastCounter = 0,
            .Counter = 0,
            .Timer = MOTOR_DRV_LEFT_ENC_TIMER,
            .hTimer = &hEncLeftTimer,
            .TimerChannel = MOTOR_DRV_LEFT_ENC_TIMER_CHANNEL
        }
    },
    {
        .IoEnable = 
        { 
            .Pin = MOTOR_DRV_RIGHT_EN_PIN,
            .Port = MOTOR_DRV_RIGHT_EN_PORT
        },
        .IoDirA = 
        { 
            .Pin = MOTOR_DRV_RIGHT_DIRA_PIN,
            .Port = MOTOR_DRV_RIGHT_DIRA_PORT
        },
        .IoDirB = 
        { 
            .Pin = MOTOR_DRV_RIGHT_DIRB_PIN,
            .Port = MOTOR_DRV_RIGHT_DIRB_PORT
        },
        .Timer = MOTOR_DRV_RIGHT_PWM_TIMER,
        .TimerChannel = MOTOR_DRV_RIGHT_PWM_TIMER_CHANNEL,
        .hTimer = &hPwmTimer,
        .sEncoder = { 
            .IoEncoderA = { 
                .Pin = MOTOR_DRV_RIGHT_ENCA_PIN,
                .Port = MOTOR_DRV_RIGHT_ENCA_PORT
            },
            .IoEncoderB = { 
                .Pin = MOTOR_DRV_RIGHT_ENCB_PIN,
                .Port = MOTOR_DRV_RIGHT_ENCB_PORT
            },
            .LastCounter = 0,
            .Counter = 0,
            .Timer = MOTOR_DRV_RIGHT_ENC_TIMER,
            .hTimer = &hEncRightTimer,
            .TimerChannel = MOTOR_DRV_RIGHT_ENC_TIMER_CHANNEL
        }
    }
};

void MotorDrv_Init() 
{
    MotorDrv_InfoTypeDef *pMotor;
    GPIO_InitTypeDef   gpio;
    MOTOR_DRV_GPIO_CLK_ENABLE();
    MOTOR_DRV_TIMER_CLK_ENABLE();
    TIM_OC_InitTypeDef sConfig;
    TIM_Encoder_InitTypeDef encoderConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    LowPassFilter_Init(LOW_PASS_FILTER_CUTOFF_HZ, LOW_PASS_FILTER_SAMPLE_HZ, true);
    for (size_t i = 0; i < MOTOR_ID_NUM; i++)
    {
        pMotor = &sMotorInfo[i];
        // Configure GPIO Enable
        gpio.Pin       = pMotor->IoEnable.Pin;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(pMotor->IoEnable.Port, &gpio);
        // Configure GPIO DIRA
        gpio.Pin       = pMotor->IoDirA.Pin;
        gpio.Mode      = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(pMotor->IoDirA.Port, &gpio);
         // Configure GPIO DIRB
        gpio.Pin       = pMotor->IoDirB.Pin;
        gpio.Mode      = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(pMotor->IoDirB.Port, &gpio);
        // Configure GPIO Encoder
        gpio.Pin       = pMotor->sEncoder.IoEncoderA.Pin;
        gpio.Mode      = GPIO_MODE_INPUT;
        HAL_GPIO_Init(pMotor->sEncoder.IoEncoderA.Port, &gpio);
        gpio.Pin       = pMotor->sEncoder.IoEncoderB.Pin;
        gpio.Mode      = GPIO_MODE_INPUT;
        HAL_GPIO_Init(pMotor->sEncoder.IoEncoderB.Port, &gpio);
        MOTOR_DRV_GPIO_REMAP();
        /* Initialize TIMx peripheral as follows:
             + Period = 0xFFFF
             + Prescaler = 0
             + ClockDivision = 0
             + Counter direction = Up
             + TIM2CLK frequency is set to SystemCoreClock (Hz) => 72MHz
           */
        pMotor->sEncoder.hTimer->Instance = pMotor->sEncoder.Timer;
        pMotor->sEncoder.hTimer->Init.Prescaler         = 0;
        pMotor->sEncoder.hTimer->Init.Period            = MAX_UINT16;
        pMotor->sEncoder.hTimer->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        pMotor->sEncoder.hTimer->Init.CounterMode       = TIM_COUNTERMODE_UP;
        pMotor->sEncoder.hTimer->Init.RepetitionCounter = 0;
        pMotor->sEncoder.hTimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        encoderConfig.EncoderMode = TIM_ENCODERMODE_TI1;
        encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        encoderConfig.IC1Filter = 0;
        encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        encoderConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(pMotor->sEncoder.hTimer, &encoderConfig) != HAL_OK)
        {
            Common_Assert(__LINE__, __FILE__);
        }
        if (HAL_TIM_Encoder_Start(pMotor->sEncoder.hTimer, pMotor->sEncoder.TimerChannel) != HAL_OK)
        {
            Common_Assert(__LINE__, __FILE__);
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(pMotor->sEncoder.hTimer, &sMasterConfig) != HAL_OK)
        {
            Common_Assert(__LINE__, __FILE__);
        }
        /* Initialize TIMx peripheral as follows:
         + Period = 0xFFFF
         + Prescaler = 0
         + ClockDivision = 0
         + Counter direction = Up
         + TIM2CLK frequency is set to SystemCoreClock (Hz) => 72MHz
        }
       */
        pMotor->hTimer->Instance = pMotor->Timer;
        pMotor->hTimer->Init.Prescaler         = 0;
        pMotor->hTimer->Init.Period            = MAX_UINT16;
        pMotor->hTimer->Init.ClockDivision     = 0;
        pMotor->hTimer->Init.CounterMode       = TIM_COUNTERMODE_UP;
        pMotor->hTimer->Init.RepetitionCounter = 0;
        pMotor->hTimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_PWM_Init(pMotor->hTimer) != HAL_OK)
        {
            /* Initialization Error */
            Common_Assert(__LINE__, __FILE__);
        }
        /*##-2- Configure the PWM channels #########################################*/
        /* Common configuration for all channels */
        sConfig.OCMode       = TIM_OCMODE_PWM1;
        sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
        sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
        sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
        sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
        /* Set the pulse value for channel 1 */
        sConfig.Pulse = 0;
        if (HAL_TIM_PWM_ConfigChannel(pMotor->hTimer, &sConfig, pMotor->TimerChannel) != HAL_OK)
        {
            /* Configuration Error */
            Common_Assert(__LINE__, __FILE__);
        }
        if (HAL_TIM_PWM_Start(pMotor->hTimer, pMotor->TimerChannel) != HAL_OK)
        {
            /* PWM Generation Error */
            Common_Assert(__LINE__, __FILE__);
        }
    }
}

void MotorDrv_Execute(const Planner_BlockTypeDef * const block, uint32_t diffTime)
{
    uint16_t pwmValue = 0;
    uint32_t iDiffCounter = 0;
    float moveLen = 0;
    for(uint8_t i = 0; i < MOTOR_ID_NUM; i++) 
    {
        MotorDrv_InfoTypeDef *motor = &sMotorInfo[i];
        if(block->Stage == PLAN_CTRL_STAGE_ACCEL) 
        {
            if (block->MotorOps[i].isCw)
            {
                HAL_GPIO_WritePin(motor->IoDirA.Port, motor->IoDirA.Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(motor->IoDirB.Port, motor->IoDirB.Pin, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(motor->IoDirA.Port, motor->IoDirA.Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(motor->IoDirB.Port, motor->IoDirB.Pin, GPIO_PIN_SET);
            }
        }
        motor->sEncoder.Counter = __HAL_TIM_GET_COUNTER(motor->sEncoder.hTimer);
        if (motor->sEncoder.Counter >= motor->sEncoder.LastCounter)
        {
            iDiffCounter = motor->sEncoder.Counter - motor->sEncoder.LastCounter;
        }
        else
        {
            iDiffCounter = ((MAX_UINT16 - motor->sEncoder.LastCounter) + motor->sEncoder.Counter) + 1;
        }
        moveLen = setting->PulsesPerMM * iDiffCounter;
        motorOps[i].Speed = moveLen / diffTime;
        motorOps[i].Millimeters += (uint32_t)moveLen;
        motor->sEncoder.LastCounter = motor->sEncoder.Counter;
        pwmValue = (uint16_t) fabs(block->MotorOps[i].Voltage * (float) MAX_UINT16 / 100.0);
        __HAL_TIM_SET_COMPARE(motor->hTimer, motor->TimerChannel, pwmValue);
    }
}

void MotorDrv_Stop()
{
    for(uint8_t i = 0; i < MOTOR_ID_NUM; i++) 
    {
        MotorDrv_InfoTypeDef *pMotor = &sMotorInfo[i];
        motorOps[i].Millimeters = 0;
        motorOps[i].Speed = 0;
        HAL_GPIO_WritePin(pMotor->IoDirA.Port, pMotor->IoDirA.Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pMotor->IoDirB.Port, pMotor->IoDirB.Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(pMotor->hTimer, pMotor->TimerChannel, 0);
    }
}
