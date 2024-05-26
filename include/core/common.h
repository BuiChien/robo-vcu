#pragma once
#include <stddef.h>
#include "configs.h"
#include "command.h"

#define CONSTRAIN(amt,low,high)             ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct
{
    void (*Init)();
    void(*ProcessCommand)(const Command_InfoTypeDef * const cmd);
} Unit_InterfaceTypeDef;

typedef enum
{
    MOTOR_ID_LEFT,
    MOTOR_ID_RIGHT,
    MOTOR_ID_NUM
} Motor_IdTypeDef;

typedef struct
{
    uint16_t Speed;
    uint32_t Millimeters;   // Distance has been run
} Motor_OpsTypeDef;

typedef struct
{
    uint16_t Speed;
    uint32_t Target; //mm
    int16_t Voltage;
    bool isCw;
} Planner_MotorPlanTypeDef;

typedef enum
{
  PLAN_CTRL_STAGE_ACCEL      = 0x00U,
  PLAN_CTRL_STAGE_NORMAL     = 0x01U,
  PLAN_CTRL_STAGE_DECEL      = 0x02U
} Planner_CtrlStageTypeDef;

typedef struct {
    Planner_CtrlStageTypeDef Stage;
    Planner_MotorPlanTypeDef MotorOps[MOTOR_ID_NUM];
} Planner_BlockTypeDef;

typedef struct {
    uint8_t MotorMaxVoltage;    // Maximum voltage can drive of motor driver
    uint8_t MotorMinVoltage;    // Miximum voltage which motor can start moving
    uint8_t PulsePerRevolution; // Number pulse per revolution of wheel
    float PulsesPerMM;          // Number pulses per MM
    uint16_t WheelDiameterMM;
    uint16_t RobotDiameterMM;
} Setting_InfoTypeDef;

extern Setting_InfoTypeDef* setting;
extern Motor_OpsTypeDef *motorOps;
extern void Common_Assert(uint32_t ulLine, const char *pcFile);