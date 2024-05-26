#pragma once
#include <stm32f1xx_hal.h>
#include <stdbool.h>

extern void CanBusDrv_Init();
extern bool CanBusDrv_Receive(uint8_t *buffer, int timeout);
extern void CanBusDrv_Send(uint32_t stdId, const uint8_t * const buffer, const uint8_t len);

