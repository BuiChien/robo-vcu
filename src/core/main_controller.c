#include <FreeRTOS.h>
#include <task.h>
#include <math.h>
#include <string.h>
#include "main_controller.h"
#include "can_bus_drv.h"
#include "sensors_controller.h"
#include "planner.h"
#include "motor_drv.h"

#define UNIT_NUMBER             (1)
#define MAIN_TASK_STACK_SIZE    (4 *128)

static Setting_InfoTypeDef st;
Setting_InfoTypeDef* setting = &st;
static Motor_OpsTypeDef mt[MOTOR_ID_NUM];
Motor_OpsTypeDef *motorOps = &mt[0];

static TaskHandle_t hHandle;
static void MainController_Task();
static void MainController_LoadSetting();
static void MainController_ReportStatus();
static void MainController_UnitInit();
static void MainController_ProcessCommand(const Command_InfoTypeDef * const cmd);

static Unit_InterfaceTypeDef* units[UNIT_NUMBER] =
{

};

void MainController_Init()
{
    memset(motorOps, 0, sizeof(Motor_OpsTypeDef) * MOTOR_ID_NUM);
    MainController_LoadSetting();
    xTaskCreate(&MainController_Task, "MainTask", MAIN_TASK_STACK_SIZE, NULL, 2, &hHandle);
}

static void MainController_Task() {
    uint8_t buffer[RECEIVE_CMD_BUFFER_SIZE];
    CanBusDrv_Init();
//    MainController_UnitInit();
    MotorDrv_Init();
    Planner_Init();
    uint32_t cTick = 0;
    uint32_t pTickReport = 0;
    uint32_t pTickSampling = 0;
    uint32_t diffTime = 0;
    for (;;)
    {
        if(CanBusDrv_Receive(buffer, pdMS_TO_TICKS(MOTOR_SPEED_SAMPLE_RATE_MS))) 
        {
            MainController_ProcessCommand((Command_InfoTypeDef *)buffer);
        }
        cTick = HAL_GetTick();
        diffTime = cTick - pTickSampling;
        if(diffTime >= MOTOR_SPEED_SAMPLE_RATE_MS) 
        {
            pTickSampling = cTick;
            Planner_BlockTypeDef *block = Planner_Recalculate(diffTime);
            if(block != NULL) 
            {
                MotorDrv_Execute(block);
            } 
            else 
            {
                MotorDrv_Stop();
            }
        }
        if((cTick - pTickReport) >= NOTIFY_STATUS_INTERVAL_MS) 
        {
            pTickReport = cTick;
            MainController_ReportStatus();
        }
    }
}

void MainController_ReportStatus() 
{
    RealtimeCommand_InfoTypeDef realtime;
    realtime.PlanId = 1;
    realtime.MotorLeftSpeed = motorOps[MOTOR_ID_LEFT].Speed;
    realtime.MotorLeftSpeed = motorOps[MOTOR_ID_RIGHT].Speed;
    CanBusDrv_Send(0x1, (uint8_t *)&realtime, sizeof(realtime));
}

void MainController_UnitInit() 
{
    for (int i = 0; i < UNIT_NUMBER; i++)
    {
        units[i]->Init();
    }
}

void MainController_ProcessCommand(const Command_InfoTypeDef * const cmd)
{
    switch (cmd->Id) {
    case CMD_PID: {
            PidCommand_InfoTypeDef* pid = (PidCommand_InfoTypeDef*)cmd;
            break;
        }
    case CMD_ROTATE: {
            RotateCommand_InfoTypeDef* rotate = (RotateCommand_InfoTypeDef*)cmd;
            int32_t target = (abs(rotate->Degree) * M_PI * setting->WheelDiameterMM) / 360.0;
            Planner_Move(target, rotate->Speed, true);
            break;
        }
    case CMD_RUN: {
            RunCommand_InfoTypeDef* run = (RunCommand_InfoTypeDef*)cmd;
            Planner_Move(run->Distance, run->Speed, false);
            break;
        }
    case CMD_STOP: {
            Planner_Reset();
            break;
        }
    default: {
            for (int i = 0; i < UNIT_NUMBER; i++)
            {
                units[i]->ProcessCommand(cmd);
            }
            break;
        }
    }
}


void MainController_LoadSetting()
{
    setting->MotorMaxVoltage = MOTOR_VOLTAGE_MAX;
    setting->MotorMinVoltage = MOTOR_VOLTAGE_MIN;
    setting->RobotDiameterMM = ROBOT_DIAMETER_MM;
    setting->WheelDiameterMM = WHEEL_DIAMETER_MM;
    setting->PulsePerRevolution = PULSE_PER_REVOLUTION;
    setting->PulsesPerMM = (M_PI * setting->WheelDiameterMM) / setting->PulsePerRevolution;
}