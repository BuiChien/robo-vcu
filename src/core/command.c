#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "command.h"

#define COMMA_DELIMITER ","

bool RunCommand_Parse(struct Command_InfoTypeDef *cmd, char *data, uint16_t length);
bool RotateCommand_Parse(struct Command_InfoTypeDef *cmd, char *data, uint16_t length);
bool PidCommand_Parse(struct Command_InfoTypeDef *cmd, char *data, uint16_t length);
bool Command_Parse(struct Command_InfoTypeDef *cmd, char *data, uint16_t length);

RunCommand_InfoTypeDef runCommandParser = { 
    {
        CMD_RUN,
        RunCommand_Parse
    },
    0.0,
    0.0
};

RotateCommand_InfoTypeDef rotateCommandParser = { 
    {
        CMD_ROTATE,
        RotateCommand_Parse
    },
    0.0,
    0.0
};

PidCommand_InfoTypeDef pidCommandParser = { 
    {
        CMD_PID,
        PidCommand_Parse
    },
    0.0,
    0.0,
    0.0
};

CommonCommand_InfoTypeDef stopCommandParser = { 
    {
        CMD_STOP,
        Command_Parse
    }
};

CommonCommand_InfoTypeDef statusCommandParser = { 
    {
        CMD_STATUS,
        Command_Parse
    }
};


uint8_t Command_GetId(char *data)
{
    char *str = strtok(data, COMMA_DELIMITER);
    return atoi(str);
}

bool Command_Parse(struct Command_InfoTypeDef *cmd, char *data, uint16_t length)
{
    (void)cmd;
    (void)data;
    (void)length;
    return true;
}

bool RunCommand_Parse(struct Command_InfoTypeDef *cmd, char *data, uint16_t length)
{
    char *str = strtok(data, COMMA_DELIMITER);
    RunCommand_InfoTypeDef* run = (RunCommand_InfoTypeDef*)cmd;
    bool bRet = false;
    if (str != NULL) 
    {
        run->fTarget = atof(str);
    }
    str = strtok(NULL, COMMA_DELIMITER);
    if (str != NULL)
    {
        run->fSpeed = atof(str);
        bRet = true;
    }
    return bRet;
}

bool RotateCommand_Parse(struct Command_InfoTypeDef *cmd, char *data, uint16_t length)
{
    char *str = strtok(data, COMMA_DELIMITER);
    RotateCommand_InfoTypeDef* rotate = (RotateCommand_InfoTypeDef*)cmd;
    bool bRet = false;
    if (str != NULL) 
    {
        rotate->fDegree = atof(str);
    }
    str = strtok(NULL, COMMA_DELIMITER);
    if (str != NULL)
    {
        rotate->fSpeed = atof(str);
        bRet = true;
    }
    return bRet;
}

bool PidCommand_Parse(struct Command_InfoTypeDef *cmd, char *data, uint16_t length)
{
    char *str = strtok(data, COMMA_DELIMITER);
    PidCommand_InfoTypeDef* pid = (PidCommand_InfoTypeDef*)cmd;
    bool bRet = false;
    if (str != NULL) 
    {
        pid->fKp = atof(str);
    }
    str = strtok(NULL, COMMA_DELIMITER);
    if (str != NULL)
    {
        pid->fKi = atof(str);
        bRet = true;
    }
    str = strtok(NULL, COMMA_DELIMITER);
    if (str != NULL)
    {
        pid->fKd = atof(str);
        bRet = true;
    }
    return bRet;
}