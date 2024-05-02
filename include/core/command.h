#pragma once
#include "stdint.h"
#include "stdbool.h"

#define CMD_PID             0x01
#define CMD_ROTATE          0x02
#define CMD_RUN             0x03
#define CMD_STOP            0x04
#define CMD_STATUS          0xFC

struct Command_InfoTypeDef;
typedef struct 
{
    uint8_t iId;
    bool(*Parse)(struct Command_InfoTypeDef *cmd, char *data, uint16_t length);
} Command_InfoTypeDef;

typedef struct
{
    Command_InfoTypeDef info;
    float fDegree;
    float fSpeed;
} RotateCommand_InfoTypeDef;

typedef struct
{
    Command_InfoTypeDef info;
    float fTarget;
    float fSpeed;
} RunCommand_InfoTypeDef;

typedef struct
{
    Command_InfoTypeDef info;
    float fKp;
    float fKi;
    float fKd;
} PidCommand_InfoTypeDef;

typedef struct
{
    Command_InfoTypeDef info;
} CommonCommand_InfoTypeDef;

extern RunCommand_InfoTypeDef runCommandParser;
extern PidCommand_InfoTypeDef pidCommandParser;
extern RotateCommand_InfoTypeDef rotateCommandParser;
extern CommonCommand_InfoTypeDef stopCommandParser;
extern CommonCommand_InfoTypeDef statusCommandParser;
extern uint8_t Command_GetId(char *data);