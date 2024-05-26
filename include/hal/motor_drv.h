#pragma once
#include <stm32f1xx_hal.h>
#include <stdbool.h>
#include "configs.h"
#include "common.h"

extern void MotorDrv_Init();
extern void MotorDrv_Execute(const Planner_BlockTypeDef * const block, uint32_t diffTime);
extern void MotorDrv_Stop();



