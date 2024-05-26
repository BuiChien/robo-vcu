#pragma once
#include "stdint.h"
#include "stdbool.h"
#define PACKED __attribute__ ((packed))

#define CMD_PID             0x01
#define CMD_ROTATE          0x02
#define CMD_RUN             0x03
#define CMD_STOP            0x04
#define CMD_STATUS          0xFC

typedef struct PACKED
{
    uint8_t Id;
} Command_InfoTypeDef;

typedef struct PACKED
{
    uint8_t Id;
    int16_t Degree;
    uint16_t Speed;
} RotateCommand_InfoTypeDef;

typedef struct PACKED
{
    uint8_t Id;
    int32_t Distance;
    uint16_t Speed;
} RunCommand_InfoTypeDef;

typedef struct PACKED
{
    uint8_t Id;
    float Kp;
    float Ki;
    float Kd;
} PidCommand_InfoTypeDef;

typedef struct PACKED
{
    uint8_t PlanId;
    uint16_t MotorLeftSpeed;
    uint16_t MotorRightSpeed;
} RealtimeCommand_InfoTypeDef;