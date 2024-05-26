/* Includes ------------------------------------------------------------------*/
#include <FreeRTOS.h>
#include <task.h>
#include <stm32f1xx_hal.h>
#include "main_controller.h"
/* Private function prototypes -----------------------------------------------*/
void Common_Assert(uint32_t ulLine, const char *pcFile);

int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    MainController_Init();
    /* Start scheduler */
    vTaskStartScheduler();

    while (1)
    {

    }
}

void Common_Assert(uint32_t ulLine, const char *pcFile)
{
    volatile unsigned long ul = 0;

    (void) pcFile;
    (void) ulLine;

    taskENTER_CRITICAL();
    {
        /* Set ul to a non-zero value using the debugger to step out of this function. */
        while (ul == 0)
        {
            __NOP();
        }
    }
    taskEXIT_CRITICAL();
}
/**
  * @brief  Function called to read the current micro second
  * @param  None
  * @retval None
  */
uint32_t getCurrentMicros(void)
{
    uint32_t m0 = HAL_GetTick();
    __IO uint32_t u0 = SysTick->VAL;
    uint32_t m1 = HAL_GetTick();
    __IO uint32_t u1 = SysTick->VAL;
    const uint32_t tms = SysTick->LOAD + 1;

    if (m1 != m0) {
        return (m1 * 1000 + ((tms - u1) * 1000) / tms);
    } else {
        return (m0 * 1000 + ((tms - u0) * 1000) / tms);
    }
}
// Interrupt-compatible version of micros
uint32_t micros(void) 
{
  // ToDo: ensure no interrupts
    return getCurrentMicros();
}

