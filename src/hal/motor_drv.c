#include <stm32f1xx_hal.h>
#include <math.h>
#include <string.h>
#include "motor_drv.h"
#include "pid.h"
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
    PID_InfoTypeDef sPid;
} MotorDrv_InfoTypeDef;
TIM_HandleTypeDef hEncLeftTimer;
TIM_HandleTypeDef hEncRightTimer;
TIM_HandleTypeDef hPwmTimer;
MotorDrv_InfoTypeDef sMotorInfo[MOTOR_DRV_NUMBER] = {
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
        },
        .sPid = 
        {
            PID_KP, 
            PID_KI, 
            PID_KD,
            PID_TAU,
            PID_LIM_MIN,
            PID_LIM_MAX,
            PID_LIM_MIN_INTEGRATOR,
            PID_LIM_MAX_INTEGRATOR
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
        },
        .sPid = 
        {
            PID_KP, 
            PID_KI, 
            PID_KD,
            PID_TAU,
            PID_LIM_MIN,
            PID_LIM_MAX,
            PID_LIM_MIN_INTEGRATOR,
            PID_LIM_MAX_INTEGRATOR
        }
    }
};

MotorDrv_OpsTypeDef sMotorOps[MOTOR_DRV_NUMBER];
MotorDrv_OpsTypeDef *pMotorOps = sMotorOps;

void MotorDrv_Init() 
{
    MotorDrv_InfoTypeDef *pMotor;
    GPIO_InitTypeDef   sGpio;
    MOTOR_DRV_GPIO_CLK_ENABLE();
    MOTOR_DRV_TIMER_CLK_ENABLE();
    TIM_OC_InitTypeDef sConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    memset(sMotorOps, 0, sizeof(MotorDrv_OpsTypeDef) * MOTOR_DRV_NUMBER);
    LowPassFilter_Init(LOW_PASS_FILTER_CUTOFF_HZ, LOW_PASS_FILTER_SAMPLE_HZ, true);
    for (size_t i = 0; i < MOTOR_DRV_NUMBER; i++)
    {
        pMotor = &sMotorInfo[i];
        PID_Init(&pMotor->sPid);
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

void MotorDrv_SetSpeed(MotorDrv_IdTypeDef id, float target)
{
    sMotorOps[id].dTargetSpeed = MM_PER_SECOND_TO_RAD(target);
}

void MotorDrv_SetTarget(MotorDrv_IdTypeDef id, float target)
{
    MotorDrv_InfoTypeDef *pMotor = &sMotorInfo[id];
    pMotor->iLastCalculate = HAL_GetTick();
    pMotor->sEncoder.iCounter = __HAL_TIM_GET_COUNTER(pMotor->sEncoder.hTimer);
    pMotor->sEncoder.iLastCounter = pMotor->sEncoder.iCounter;
    if (target > 0)
    {
        HAL_GPIO_WritePin(pMotor->sIoDirA.Port, pMotor->sIoDirA.Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(pMotor->sIoDirB.Port, pMotor->sIoDirB.Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(pMotor->sIoDirA.Port, pMotor->sIoDirA.Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pMotor->sIoDirB.Port, pMotor->sIoDirB.Pin, GPIO_PIN_SET);
    }
    sMotorOps[id].dTarget = fabs(target);
}

void MotorDrv_Execute(MotorDrv_IdTypeDef id)
{
    MotorDrv_InfoTypeDef *pMotor = &sMotorInfo[id];
    MotorDrv_OpsTypeDef *pOps = &sMotorOps[id];
    if (pOps->dTarget > 0)
    {
        float iDiffCounter = 0;
        uint32_t iDiffTime = 0;
        uint32_t iTimeNow = HAL_GetTick();
        float dPidOut = 0;
        uint16_t iPwmValue = 0;
        pMotor->sEncoder.iCounter = __HAL_TIM_GET_COUNTER(pMotor->sEncoder.hTimer);
        if (pMotor->sEncoder.iCounter >= pMotor->sEncoder.iLastCounter)
        {
            iDiffCounter = pMotor->sEncoder.iCounter - pMotor->sEncoder.iLastCounter;
        }
        else
        {
            iDiffCounter = ((MAX_UINT16 - pMotor->sEncoder.iLastCounter) + pMotor->sEncoder.iCounter) + 1;
        }
        iDiffTime = iTimeNow - pMotor->iLastCalculate;
        pOps->dTarget -= ENC_PULSES_TO_MM(iDiffCounter);
        if (pOps->dTarget > 0)
        {
            pOps->dSpeed = ENC_PULSES_TO_RAD(iDiffCounter, iDiffTime);
            pOps->dSpeed = LowPassFilter_Filt(pOps->dSpeed);
            dPidOut = PID_Calculate(&pMotor->sPid, pOps->dTargetSpeed, pOps->dSpeed, ((float)iDiffTime / 1000.0f));
            // theese small voltages will only make the motors whine anyway
            if(fabs(dPidOut) < 0.6) 
            {
                dPidOut = 0; 
            }
            // convert voltage to pwm duty cycle
            dPidOut = 100.0 * (dPidOut / MOTOR_VOLTAGE_MAX);
            iPwmValue = (uint16_t) fabs(dPidOut * (float) MAX_UINT16 / 100.0);
            __HAL_TIM_SET_COMPARE(pMotor->hTimer, pMotor->iTimerChannel, iPwmValue);
        }
        else
        {
            pOps->dTarget = 0.0;
            pOps->dSpeed = 0.0;
            __HAL_TIM_SET_COMPARE(pMotor->hTimer, pMotor->iTimerChannel, 0);
        }
        // Update variable
        pMotor->sEncoder.iLastCounter = pMotor->sEncoder.iCounter;
        pMotor->iLastCalculate = iTimeNow;
    }
}

void MotorDrv_Stop(MotorDrv_IdTypeDef id)
{
    MotorDrv_InfoTypeDef *pMotor = &sMotorInfo[id];
    sMotorOps[id].dTarget = 0;
    HAL_GPIO_WritePin(pMotor->sIoDirA.Port, pMotor->sIoDirA.Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pMotor->sIoDirB.Port, pMotor->sIoDirB.Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(pMotor->hTimer, pMotor->iTimerChannel, 0);
}

bool MotorDrv_IsIdle(MotorDrv_IdTypeDef id)
{
    return sMotorOps[id].dTarget > 0 ? false : true;
}


void MotorDrv_SetPid(MotorDrv_IdTypeDef id,  float fKp, float fKi, float fKd)
{
    MotorDrv_InfoTypeDef *pMotor = &sMotorInfo[id];
    pMotor->sPid.fKp = fKp;
    pMotor->sPid.fKi = fKi;
    pMotor->sPid.fKd = fKd;
}
