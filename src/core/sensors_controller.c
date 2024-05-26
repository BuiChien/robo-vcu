#include <FreeRTOS.h>
#include <task.h>
#include "sensors_controller.h"
#include "vl53l0x_drv.h"

#define     NUMBER_SENSOR       1
TaskHandle_t hSensorsCtrlHandle;
Sensors_ApiTypeDef* sensors[NUMBER_SENSOR] = 
{ 
    &vl53l0x
};

void SensorsCtrl_Init();
void SensorsCtrl_ProcessCommand(const Command_InfoTypeDef * const cmd);
void SendorsCtrl_Task(void *args);

Unit_InterfaceTypeDef sensorsCtrl = 
{
    SensorsCtrl_Init,
    SensorsCtrl_ProcessCommand
};

void  SensorsCtrl_Init()
{
    for (int i = 0; i < NUMBER_SENSOR; i++)
    {
        sensors[i]->Init();
    }
    xTaskCreate(SendorsCtrl_Task, "SensorsTask", 256, NULL, 1, &hSensorsCtrlHandle);
}

void SensorsCtrl_ProcessCommand(const Command_InfoTypeDef * const cmd)
{
    
}

void SendorsCtrl_Task(void *args)
{
    for (;;)
    {
        for (int i = 0; i < NUMBER_SENSOR; i++)
        {
            sensors[i]->Sensing();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
