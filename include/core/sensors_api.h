#pragma once

typedef struct
{
    void(*Init)();
    void(*Sensing)();
} Sensors_ApiTypeDef;
