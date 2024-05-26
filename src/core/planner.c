#include <stdlib.h>
#include <math.h>
#include "configs.h"
#include "pid.h"
#include "planner.h"

#define PLANNER_BLOCK_BUFFER_SIZE       8
#define DEFAULT_INDEX                   (-1)
#define PID_KP                          2.0f
#define PID_KI                          0.5f
#define PID_KD                          0.25f
#define PID_TAU                         0.02f
#define PID_LIM_MIN                     -MOTOR_VOLTAGE_MAX
#define PID_LIM_MAX                      MOTOR_VOLTAGE_MAX
#define PID_LIM_MIN_INTEGRATOR          -6.0f
#define PID_LIM_MAX_INTEGRATOR          6.0f

static Planner_BlockTypeDef blockBuffer[PLANNER_BLOCK_BUFFER_SIZE];
static int8_t blockBufferHeadIndex;
static int8_t blockBufferTailIndex;
static PID_InfoTypeDef pid[MOTOR_ID_NUM];

void Planner_Init()
{
    for(uint8_t i = 0; i < MOTOR_ID_NUM; i++) 
    {
        pid[i].fKp = PID_KP;
        pid[i].fKi  = PID_KI;
        pid[i].fKd  = PID_KD;
        pid[i].fTau = PID_TAU;
        pid[i].fOutputLimitMin = -setting->MotorMaxVoltage;
        pid[i].fOutputLimitMax = setting->MotorMaxVoltage;
        pid[i].fIntegratorLimitMin = -setting->MotorMinVoltage;
        pid[i].fIntegratorLimitMax = setting->MotorMinVoltage;
        PID_Init(&pid[i]);
    }
}

void Planner_Move(int32_t distance, uint16_t speed, bool isRotate)
{
    int8_t prevIndex = blockBufferHeadIndex;
    int8_t nextIndex = blockBufferHeadIndex + 1;
    if(prevIndex != DEFAULT_INDEX) 
    {
        blockBuffer[nextIndex].Stage = PLAN_CTRL_STAGE_NORMAL;
    } 
    else 
    {
        blockBuffer[nextIndex].Stage = PLAN_CTRL_STAGE_ACCEL;
    }
    int32_t targetDistance = distance;
    for(uint8_t i = 0; i < MOTOR_ID_NUM; i++) 
    {
        if(isRotate) 
        {
            targetDistance = (i % 2) > 0 ? targetDistance : -targetDistance;
        }
        blockBuffer[nextIndex].MotorOps[i].isCw = targetDistance > 0 ? true : false;
        blockBuffer[nextIndex].MotorOps[i].Speed = speed;
        blockBuffer[nextIndex].MotorOps[i].Target = abs(targetDistance);
    }
}

void Planner_Reset()
{
    blockBufferHeadIndex = DEFAULT_INDEX;
    blockBufferTailIndex = DEFAULT_INDEX;
}

Planner_BlockTypeDef *Planner_Recalculate(uint32_t diffTime)
{
    Planner_BlockTypeDef *block = NULL;
    if(blockBufferHeadIndex != blockBufferTailIndex) 
    {
        int planningIndex = blockBufferTailIndex + 1;
        block = &blockBuffer[planningIndex];
        uint8_t finishCount = 0;
        for(uint8_t i = 0; i < MOTOR_ID_NUM; i++) 
        {
            if(block->MotorOps[i].Target < motorOps[i].Millimeters) 
            {
                block->MotorOps[i].Voltage = PID_Calculate(&pid[i], block->MotorOps[i].Speed, motorOps->Speed, ((float)diffTime / 1000.0f));
                if(block->MotorOps[i].Voltage < setting->MotorMinVoltage)
                {
                    block->MotorOps[i].Voltage = setting->MotorMinVoltage;
                }
            } 
            else 
            {
                block->MotorOps[i].Voltage = 0;
                finishCount++;
            }
        }
        if(finishCount == MOTOR_ID_NUM) 
        {
            blockBufferTailIndex++;
            if(blockBufferTailIndex > (PLANNER_BLOCK_BUFFER_SIZE - 1))
            {
                blockBufferTailIndex = DEFAULT_INDEX;
            }
            block = Planner_Recalculate(diffTime);
        }
    }
    return block;
}

void Planner_SetPid(uint8_t motorId, float fKp, float fKi, float fKd)
{
}
