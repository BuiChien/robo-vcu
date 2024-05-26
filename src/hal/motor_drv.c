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
#define MOTOR_DRV_LEFT_ENC_PIN                      GPIO_PIN_6
#define MOTOR_DRV_LEFT_ENC_PORT                     GPIOA
#define MOTOR_DRV_LEFT_ENC_TIMER                    TIM3
#define MOTOR_DRV_LEFT_PWM_TIMER                    TIM2
#define MOTOR_DRV_LEFT_PWM_TIMER_CHANNEL            TIM_CHANNEL_1
#define MOTOR_DRV_LEFT_ENC_TIMER_CHANNEL            TIM_CHANNEL_1

#define MOTOR_DRV_RIGHT_EN_PIN                      GPIO_PIN_1
#define MOTOR_DRV_RIGHT_EN_PORT                     GPIOA
#define MOTOR_DRV_RIGHT_DIRA_PIN                    GPIO_PIN_4
#define MOTOR_DRV_RIGHT_DIRA_PORT                   GPIOA
#define MOTOR_DRV_RIGHT_DIRB_PIN                    GPIO_PIN_5
#define MOTOR_DRV_RIGHT_DIRB_PORT                   GPIOA
#define MOTOR_DRV_RIGHT_ENC_PIN                     GPIO_PIN_6
#define MOTOR_DRV_RIGHT_ENC_PORT                    GPIOB
#define MOTOR_DRV_RIGHT_ENC_TIMER                   TIM4
#define MOTOR_DRV_RIGHT_PWM_TIMER                   TIM2
#define MOTOR_DRV_RIGHT_PWM_TIMER_CHANNEL           TIM_CHANNEL_2
#define MOTOR_DRV_RIGHT_ENC_TIMER_CHANNEL           TIM_CHANNEL_1

#define MOTOR_DRV_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE(); \
                                                    __HAL_RCC_GPIOB_CLK_ENABLE();
#define MOTOR_DRV_TIMER_CLK_ENABLE()                __HAL_RCC_TIM2_CLK_ENABLE(); \
                                                    __HAL_RCC_TIM3_CLK_ENABLE(); \
                                                    __HAL_RCC_TIM4_CLK_ENABLE();

typedef struct
{
    uint32_t Pin;
    GPIO_TypeDef *Port;
} MotorDrv_IoTypeDef;

typedef struct
{
    MotorDrv_IoTypeDef sIoEncoder;
    uint32_t iLastCounter;
    uint32_t iCounter;
    TIM_TypeDef *pTimer;
    TIM_HandleTypeDef *hTimer;
    uint32_t    iTimerChannel;
} MotorDrv_EncoderTypeDef;

typedef struct
{
    MotorDrv_IoTypeDef sIoEnable;
    MotorDrv_IoTypeDef sIoDirA;
    MotorDrv_IoTypeDef sIoDirB;
    TIM_TypeDef *pTimer;
    TIM_HandleTypeDef *hTimer;
    uint32_t    iTimerChannel;
    uint32_t    iLastCalculate;
    MotorDrv_EncoderTypeDef sEncoder;
} MotorDrv_InfoTypeDef;
TIM_HandleTypeDef hEncLeftTimer;
TIM_HandleTypeDef hEncRightTimer;
TIM_HandleTypeDef hPwmTimer;
MotorDrv_InfoTypeDef sMotorInfo[MOTOR_ID_NUM] = {
    {
        .sIoEnable = 
        { 
            .Pin = MOTOR_DRV_LEFT_EN_PIN,
            .Port = MOTOR_DRV_LEFT_EN_PORT
        },
        .sIoDirA = 
        { 
            .Pin = MOTOR_DRV_LEFT_DIRA_PIN,
            .Port = MOTOR_DRV_LEFT_DIRA_PORT
        },
        .sIoDirB = 
        { 
            .Pin = MOTOR_DRV_LEFT_DIRB_PIN,
            .Port = MOTOR_DRV_LEFT_DIRB_PORT
        },
        .pTimer = MOTOR_DRV_LEFT_PWM_TIMER,
        .iTimerChannel = MOTOR_DRV_LEFT_PWM_TIMER_CHANNEL,
        .hTimer = &hPwmTimer,
        .sEncoder = 
        { 
            .sIoEncoder = { 
            .Pin = MOTOR_DRV_LEFT_ENC_PIN,
            .Port = MOTOR_DRV_LEFT_ENC_PORT
        },
            .iLastCounter = 0,
            .iCounter = 0,
            .pTimer = MOTOR_DRV_LEFT_ENC_TIMER,
            .hTimer = &hEncLeftTimer,
            .iTimerChannel = MOTOR_DRV_LEFT_ENC_TIMER_CHANNEL,
        }
    },
    {
        .sIoEnable = 
        { 
            .Pin = MOTOR_DRV_RIGHT_EN_PIN,
            .Port = MOTOR_DRV_RIGHT_EN_PORT
        },
        .sIoDirA = 
        { 
            .Pin = MOTOR_DRV_RIGHT_DIRA_PIN,
            .Port = MOTOR_DRV_RIGHT_DIRA_PORT
        },
        .sIoDirB = 
        { 
            .Pin = MOTOR_DRV_RIGHT_DIRB_PIN,
            .Port = MOTOR_DRV_RIGHT_DIRB_PORT
        },
        .pTimer = MOTOR_DRV_RIGHT_PWM_TIMER,
        .iTimerChannel = MOTOR_DRV_RIGHT_PWM_TIMER_CHANNEL,
        .hTimer = &hPwmTimer,
        .sEncoder = { 
            .sIoEncoder = { 
                .Pin = MOTOR_DRV_RIGHT_ENC_PIN,
                .Port = MOTOR_DRV_RIGHT_ENC_PORT
            },
            .iLastCounter = 0,
            .iCounter = 0,
            .pTimer = MOTOR_DRV_RIGHT_ENC_TIMER,
            .hTimer = &hEncRightTimer,
            .iTimerChannel = MOTOR_DRV_RIGHT_ENC_TIMER_CHANNEL,
        }
    }
};

void MotorDrv_Init() 
{
    MotorDrv_InfoTypeDef *pMotor;
    GPIO_InitTypeDef   sGpio;
    MOTOR_DRV_GPIO_CLK_ENABLE();
    MOTOR_DRV_TIMER_CLK_ENABLE();
    TIM_OC_InitTypeDef sConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    LowPassFilter_Init(LOW_PASS_FILTER_CUTOFF_HZ, LOW_PASS_FILTER_SAMPLE_HZ, true);
    for (size_t i = 0; i < MOTOR_ID_NUM; i++)
    {
        pMotor = &sMotorInfo[i];
        // Configure GPIO Enable
        sGpio.Pin       = pMotor->sIoEnable.Pin;
        sGpio.Mode      = GPIO_MODE_AF_PP;
        sGpio.Pull      = GPIO_PULLUP;
        sGpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(pMotor->sIoEnable.Port, &sGpio);
        // Configure GPIO DIRA
        sGpio.Pin       = pMotor->sIoDirA.Pin;
        sGpio.Mode      = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(pMotor->sIoDirA.Port, &sGpio);
         // Configure GPIO DIRB
        sGpio.Pin       = pMotor->sIoDirB.Pin;
        sGpio.Mode      = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(pMotor->sIoDirB.Port, &sGpio);
        // Configure GPIO Encoder
        sGpio.Pin       = pMotor->sEncoder.sIoEncoder.Pin;
        sGpio.Mode      = GPIO_MODE_INPUT;
        HAL_GPIO_Init(pMotor->sEncoder.sIoEncoder.Port, &sGpio);
        /* Initialize TIMx peripheral as follows:
             + Period = 0xFFFF
             + Prescaler = 0
             + ClockDivision = 0
             + Counter direction = Up
             + TIM2CLK frequency is set to SystemCoreClock (Hz) => 72MHz
           */
        pMotor->sEncoder.hTimer->Instance = pMotor->sEncoder.pTimer;
        pMotor->sEncoder.hTimer->Init.Prescaler         = 0;
        pMotor->sEncoder.hTimer->Init.Period            = MAX_UINT16;
        pMotor->sEncoder.hTimer->Init.ClockDivision     = 0;
        pMotor->sEncoder.hTimer->Init.CounterMode       = TIM_COUNTERMODE_UP;
        pMotor->sEncoder.hTimer->Init.RepetitionCounter = 0;
        pMotor->sEncoder.hTimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_Base_Init(pMotor->sEncoder.hTimer) != HAL_OK)
        {
            /* Initialization Error */
            Common_Assert(__LINE__, __FILE__);
        }
        sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
        sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
        sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
        sSlaveConfig.TriggerFilter = 0;
        if (HAL_TIM_SlaveConfigSynchro(pMotor->sEncoder.hTimer, &sSlaveConfig) != HAL_OK)
        {
            Common_Assert(__LINE__, __FILE__);
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(pMotor->sEncoder.hTimer, &sMasterConfig) != HAL_OK)
        {
            Common_Assert(__LINE__, __FILE__);
        }
        if (HAL_TIM_Base_Start(pMotor->sEncoder.hTimer))
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
        pMotor->hTimer->Instance = pMotor->pTimer;
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
        if (HAL_TIM_PWM_ConfigChannel(pMotor->hTimer, &sConfig, pMotor->iTimerChannel) != HAL_OK)
        {
            /* Configuration Error */
            Common_Assert(__LINE__, __FILE__);
        }
        if (HAL_TIM_PWM_Start(pMotor->hTimer, pMotor->iTimerChannel) != HAL_OK)
        {
            /* PWM Generation Error */
            Common_Assert(__LINE__, __FILE__);
        }
    }
}

void MotorDrv_Execute(const Planner_BlockTypeDef * const block)
{
    uint16_t pwmValue = 0;
    for(uint8_t i = 0; i < MOTOR_ID_NUM; i++) 
    {
        MotorDrv_InfoTypeDef *motor = &sMotorInfo[i];
        if(block->Stage == PLAN_CTRL_STAGE_ACCEL) 
        {
            motor->sEncoder.iCounter = __HAL_TIM_GET_COUNTER(motor->sEncoder.hTimer);
            motor->sEncoder.iLastCounter = motor->sEncoder.iCounter;
            if (block->MotorOps[i].isCw)
            {
                HAL_GPIO_WritePin(motor->sIoDirA.Port, motor->sIoDirA.Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(motor->sIoDirB.Port, motor->sIoDirB.Pin, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(motor->sIoDirA.Port, motor->sIoDirA.Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(motor->sIoDirB.Port, motor->sIoDirB.Pin, GPIO_PIN_SET);
            }
        }
        pwmValue = (uint16_t) fabs(block->MotorOps[i].Voltage * (float) MAX_UINT16 / 100.0);
        __HAL_TIM_SET_COMPARE(motor->hTimer, motor->iTimerChannel, pwmValue);
    }
}

void MotorDrv_Stop()
{
    for(uint8_t i = 0; i < MOTOR_ID_NUM; i++) 
    {
        MotorDrv_InfoTypeDef *pMotor = &sMotorInfo[i];
        motorOps[i].Millimeters = 0;
        motorOps[i].Speed = 0;
        HAL_GPIO_WritePin(pMotor->sIoDirA.Port, pMotor->sIoDirA.Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pMotor->sIoDirB.Port, pMotor->sIoDirB.Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(pMotor->hTimer, pMotor->iTimerChannel, 0);
    }
}
