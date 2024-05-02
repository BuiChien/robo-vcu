#pragma once
#include "configs.h"
#include "command.h"

#define CONSTRAIN(amt,low,high)             ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct
{
    void (*Init)();
    void(*ProcessCommand)(Command_InfoTypeDef *cmd);
} Unit_InterfaceTypeDef;

extern void Common_Assert(uint32_t ulLine, const char *pcFile);