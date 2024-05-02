#pragma once
#include <stm32f1xx_hal.h>
#include <stdbool.h>
#include "configs.h"

typedef struct
{
    uint16_t iLength;
    uint8_t  data[SERIAL_RECEIVE_BUFFER];
} SerialDrv_DataTypeDef;

extern void SerialDrv_Init();
extern size_t SerialDrv_Send(char *buffer, uint16_t length, int timeout);
extern bool SerialDrv_Receive(SerialDrv_DataTypeDef *buffer, int timeout);
