#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "comunicator.h"
#include "motion_controller.h"
#include "sensors_controller.h"
#include "serial_drv.h"

#define UNIT_NUMBER         2
#define TASK_STACK_SIZE     (SERIAL_RECEIVE_BUFFER * 4)
Unit_InterfaceTypeDef* units[UNIT_NUMBER] =
{
    &motionCtrl,
    &sensorsCtrl
};
char buffer[SERIAL_RECEIVE_BUFFER];
TaskHandle_t hCommHandle;
void Comunicator_Init();
void Comunicator_Task(void *args);
Command_InfoTypeDef* Comunicator_GetCommandParser(uint8_t id); 
void Communicator_SendStatusResponse();

Unit_InterfaceTypeDef comunicator = 
{
    Comunicator_Init,
    NULL
};
Unit_InterfaceTypeDef *pComunicator = &comunicator;

void  Comunicator_Init()
{
    for (int i = 0; i < UNIT_NUMBER; i++)
    {
        units[i]->Init();
    }
    xTaskCreate(Comunicator_Task, "ComunicatorTask", TASK_STACK_SIZE, NULL, 2, &hCommHandle);
}

void Comunicator_Task(void *args)
{
    SerialDrv_Init();
    SerialDrv_DataTypeDef buffer;
    bool isOk;
    uint8_t iId;
    Command_InfoTypeDef *pCmd;
    char *pPayload;
    for (;;)
    {
        isOk = SerialDrv_Receive(&buffer, portMAX_DELAY);
        if (isOk)
        {
            iId = Command_GetId((char *)buffer.data);
            pCmd = Comunicator_GetCommandParser(iId);
            if (pCmd != NULL && buffer.iLength > 1)
            {
                pPayload = (char *)&buffer.data[3];
                isOk = pCmd->Parse((struct Command_InfoTypeDef *)pCmd, pPayload, buffer.iLength - 3);
                if (isOk)
                {
                    if(pCmd->iId == CMD_STATUS)
                    {
                        Communicator_SendStatusResponse();
                    } 
                    else 
                    {
                        for (int i = 0; i < UNIT_NUMBER; i++)
                        {
                            units[i]->ProcessCommand(pCmd);
                        }
                    }
                }
            }
        }
    }
}

Command_InfoTypeDef* Comunicator_GetCommandParser(uint8_t id)
{
    Command_InfoTypeDef *pCmd;
    switch (id)
    {
    case CMD_PID:
        pCmd = (Command_InfoTypeDef *)&pidCommandParser;
        break;
    case CMD_ROTATE:
        pCmd = (Command_InfoTypeDef *)&rotateCommandParser;
        break;
    case CMD_RUN:
        pCmd = (Command_InfoTypeDef *)&runCommandParser;
        break;
    case CMD_STOP:
        pCmd = (Command_InfoTypeDef *)&stopCommandParser;
        break;
    case CMD_STATUS:
        pCmd = (Command_InfoTypeDef *)&statusCommandParser;
        break;
    default:
        pCmd = NULL;
        break;
    }
    return pCmd;
}

void Communicator_SendStatusResponse()
{
    uint16_t totalLenght = sprintf(buffer,
        "99,%0.3f,%0.3f,%d,%0.3f,%0.3f,%d,%d\n", \
        pMotorOps[MOTOR_DRV_LEFT].dTarget,
        pMotorOps[MOTOR_DRV_LEFT].dSpeed,
        pMotorOps[MOTOR_DRV_LEFT].iPwmOut,
        pMotorOps[MOTOR_DRV_RIGHT].dTarget,
        pMotorOps[MOTOR_DRV_RIGHT].dSpeed,
        pMotorOps[MOTOR_DRV_RIGHT].iPwmOut,
        rangingData.RangeMilliMeter);
    buffer[totalLenght + 1] = 0;
    SerialDrv_Send(buffer, totalLenght, 1000);
}