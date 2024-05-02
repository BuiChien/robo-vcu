#pragma once
#include <stm32f1xx_hal.h>
#include <stdbool.h>
#include "configs.h"

#define MOTOR_DRV_NUMBER   2

typedef enum
{
    MOTOR_DRV_LEFT,
    MOTOR_DRV_RIGHT
} MotorDrv_IdTypeDef;

typedef struct
{
    float dTarget;
    float dTargetSpeed; // rpm/s
    float dSpeed;       // rmp/s
    int iPwmOut;
} MotorDrv_OpsTypeDef;

void MotorDrv_Init();
void MotorDrv_SetSpeed(MotorDrv_IdTypeDef id, float target);
void MotorDrv_SetTarget(MotorDrv_IdTypeDef id, float target);
void MotorDrv_Execute(MotorDrv_IdTypeDef id);
void MotorDrv_Stop(MotorDrv_IdTypeDef id);
bool MotorDrv_IsIdle(MotorDrv_IdTypeDef id);
void MotorDrv_SetPid(MotorDrv_IdTypeDef id, float fKp, float fKi, float fKd);



