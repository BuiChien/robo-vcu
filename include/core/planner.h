#pragma once
#include <stdint.h>
#include "configs.h"
#include "common.h"

extern void Planner_Init();
extern void Planner_Move(int32_t distance, uint16_t speed, bool isRotate);
extern void Planner_Reset();
extern Planner_BlockTypeDef *Planner_Recalculate(uint32_t diffTime);
extern void Planner_SetPid(uint8_t motorId, float fKp, float fKi, float fKd);