#include <stm32f1xx_hal.h>
#include <vl53l0x_api.h>
#include <vl53l0x_tof.h>
#include "common.h"
#include "vl53l0x_drv.h"

#define I2Cx                            I2C2
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C2_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 
/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_10
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_11
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2C_ADDRESS                     ((uint16_t)0x0052)
/* I2C SPEEDCLOCK define to max value: 400 KHz on STM32F1xx*/
#define I2C_SPEEDCLOCK                  400000
#define I2C_DUTYCYCLE                   I2C_DUTYCYCLE_2

#define VL53L0X_ID                      ((uint16_t)0xEEAA)
#define VL53L0X_XSHUT_Pin               GPIO_PIN_1
#define VL53L0X_XSHUT_GPIO_Port         GPIOB

void VL53L0XDrv_Init();
void VL53L0XDrv_Sensing();
Sensors_ApiTypeDef vl53l0x = 
{ 
    VL53L0XDrv_Init,
    VL53L0XDrv_Sensing
};
I2C_HandleTypeDef hI2cVl53l0x;
VL53L0X_Dev_t sVl53l0xDev =
{
    .I2cHandle = &hI2cVl53l0x,
    .I2cDevAddr = I2C_ADDRESS
};
VL53L0X_RangingMeasurementData_t rangingData;

void VL53L0XDrv_Init()
{
    GPIO_InitTypeDef  sGpio;
    VL53L0X_DeviceInfo_t sDeviceInfo;
    uint16_t vl53l0xId = 0;
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    I2Cx_SCL_GPIO_CLK_ENABLE();
    I2Cx_SDA_GPIO_CLK_ENABLE();
    /* Enable I2Cx clock */
    I2Cx_CLK_ENABLE(); 
    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* I2C TX GPIO pin configuration  */
    sGpio.Pin       = I2Cx_SCL_PIN;
    sGpio.Mode      = GPIO_MODE_AF_OD;
    sGpio.Pull      = GPIO_PULLUP;
    sGpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &sGpio);
    /* I2C RX GPIO pin configuration  */
    sGpio.Pin       = I2Cx_SDA_PIN;
    HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &sGpio);
    /*##-1- Configure the I2C peripheral ######################################*/
    hI2cVl53l0x.Instance             = I2Cx;
    hI2cVl53l0x.Init.ClockSpeed      = I2C_SPEEDCLOCK;
    hI2cVl53l0x.Init.DutyCycle       = I2C_DUTYCYCLE;
    hI2cVl53l0x.Init.OwnAddress1     = I2C_ADDRESS;
    hI2cVl53l0x.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hI2cVl53l0x.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hI2cVl53l0x.Init.OwnAddress2     = 0xFF;
    hI2cVl53l0x.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hI2cVl53l0x.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;    
    if (HAL_I2C_Init(&hI2cVl53l0x) != HAL_OK)
    {
        /* Initialization Error */
        Common_Assert(__LINE__, __FILE__);
    }
    sGpio.Pin = VL53L0X_XSHUT_Pin;
    sGpio.Mode = GPIO_MODE_OUTPUT_PP;
    sGpio.Pull = GPIO_PULLUP;
    sGpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(VL53L0X_XSHUT_GPIO_Port, &sGpio);
    HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    memset(&sDeviceInfo, 0, sizeof(VL53L0X_DeviceInfo_t));
    if (VL53L0X_ERROR_NONE == VL53L0X_GetDeviceInfo(&sVl53l0xDev, &sDeviceInfo))
    {  
        if (VL53L0X_ERROR_NONE == VL53L0X_RdWord(&sVl53l0xDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, (uint16_t *) &vl53l0xId))
        {
            if (vl53l0xId == VL53L0X_ID)
            {
                if (VL53L0X_ERROR_NONE == VL53L0X_DataInit(&sVl53l0xDev))
                {
                    sVl53l0xDev.Present = 1;
                    SetupSingleShot(sVl53l0xDev);
                }
                else
                { 
                    Common_Assert(__LINE__, __FILE__);
                }
            }
        }
        else
        {
            Common_Assert(__LINE__, __FILE__);
        }
    }
    else
    {
        Common_Assert(__LINE__, __FILE__);
    }
}

void VL53L0XDrv_Sensing()
{
    VL53L0X_PerformSingleRangingMeasurement(&sVl53l0xDev, &rangingData);
}