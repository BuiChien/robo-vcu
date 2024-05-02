#include <FreeRTOS.h>
#include <task.h>
#include <math.h>
#include "configs.h"
#include "motion_controller.h"

TaskHandle_t hMotionCtrlHandle;
void MotionCtrl_Init();
void MotionCtrl_ProcessCommand(Command_InfoTypeDef *cmd);
void MotionCtrl_Task(void *args);

Unit_InterfaceTypeDef motionCtrl = 
{
    MotionCtrl_Init,
    MotionCtrl_ProcessCommand
};

void  MotionCtrl_Init()
{
    xTaskCreate(MotionCtrl_Task, "MotionTask", 256, NULL, 1, &hMotionCtrlHandle);
}

void MotionCtrl_ProcessCommand(Command_InfoTypeDef *cmd)
{
    switch (cmd->iId) {
    case CMD_PID: {
            PidCommand_InfoTypeDef* pid = (PidCommand_InfoTypeDef*)cmd;
            for (int i = 0; i < MOTOR_DRV_NUMBER; i++) 
            {
                MotorDrv_SetPid(i, pid->fKp, pid->fKi, pid->fKd);
            }
            break;
        }
    case CMD_ROTATE: {
            RotateCommand_InfoTypeDef* rotate = (RotateCommand_InfoTypeDef*)cmd;
            float target = (rotate->fDegree * M_PI * ROBOT_RADIUS_MM) / 180.0;
            MotorDrv_SetSpeed(MOTOR_DRV_LEFT, rotate->fSpeed);
            MotorDrv_SetTarget(MOTOR_DRV_LEFT, -target);
            MotorDrv_SetSpeed(MOTOR_DRV_RIGHT, rotate->fSpeed);
            MotorDrv_SetTarget(MOTOR_DRV_RIGHT, target);
            break;
        }
    case CMD_RUN: {
            RunCommand_InfoTypeDef* run = (RunCommand_InfoTypeDef*)cmd;
            MotorDrv_SetSpeed(MOTOR_DRV_LEFT, run->fSpeed);
            MotorDrv_SetTarget(MOTOR_DRV_LEFT, run->fTarget);
            MotorDrv_SetSpeed(MOTOR_DRV_RIGHT, run->fSpeed);
            MotorDrv_SetTarget(MOTOR_DRV_RIGHT, run->fTarget);
            break;
        }
    case CMD_STOP: {
            CommonCommand_InfoTypeDef* stop = (CommonCommand_InfoTypeDef*)cmd;
            MotorDrv_Stop(MOTOR_DRV_LEFT);
            MotorDrv_Stop(MOTOR_DRV_RIGHT);
            break;
        }
    default:
        break;
    }
}

void MotionCtrl_Task(void *args)
{
    MotorDrv_Init();
    for (;;)
    {
        for (int i = 0; i < MOTOR_DRV_NUMBER; i++) 
        {
            MotorDrv_Execute(i);
        }
        vTaskDelay(pdMS_TO_TICKS(MOTOR_SPEED_SAMPLE_RATE_MS));
    }
}